package edu.berkeley.cs.uciedigital.loopback

import chisel3._
import edu.berkeley.cs.uciedigital.d2dadapter.D2DAdapter
import edu.berkeley.cs.uciedigital.interfaces._
import edu.berkeley.cs.uciedigital.logphy._
import edu.berkeley.cs.uciedigital.sideband._

/**
  * Full-stack two-die loopback harness: on each die a real D2DAdapter sits on
  * a real LogicalPhy over the verified UcieDigitalTop RDI wiring
  * (UcieDigitalTop.scala:79, `d2dAdapter.io.rdi <> logicalPhy.io.rdi`), and
  * the two dies are cross-wired at the LogicalPhy analog boundary exactly
  * like LogPhyLoopbackHarness (sideband serial bits+fwClock, mainband AFE
  * lanes straight).
  *
  * Unlike AdapterLoopbackHarness (which pokes plInbandPres/plStateSts stubs
  * and crosses the RDI cfg lanes directly), every pl* signal the adapter sees
  * here is produced by real PHY logic, and the adapters' sideband messages
  * (ADV_CAP and friends) travel lpCfg/plCfg -> LogPhySidebandChannel -> the
  * 1-bit serial link -> the remote die (LogicalPhy.scala:138-144), i.e. the
  * RDIController/AdapterSM contact surface is exercised on real wiring for
  * the first time.
  *
  * The testbench drives only the protocol-facing FDI subset (same tie-off
  * list as AdapterLoopbackHarness) plus the LogicalPhy software training
  * trigger. Training must be started by swStartLinkTraining: the adapter only
  * raises its RDI active request after plInbandPres, which itself only rises
  * once the LTSM reaches LINKINIT (RDIController.scala:150-154) -- the
  * adapter cannot bootstrap the PHY.
  *
  * Index convention: die 0 and die 1; die i's RX comes from die 1-i.
  */
class FullStackLoopbackHarness(
  val fdiParams: FdiParams = new FdiParams(64, 32),
  val rdiParams: RdiParams = RdiParams(64, 32),
  val afeParams: AfeParams = new AfeParams(),
  val sbParams: SidebandParams = new SidebandParams(),
  val ltsmTimeoutCycles: Int = 16384,
) extends Module {
  val io = IO(new Bundle {
    // PHY training trigger (poked by the testbench after the RESET wait).
    val swStartLinkTraining = Input(Vec(2, Bool()))

    // Protocol-layer inputs, one per die (poked by the testbench).
    val lpStateReq = Input(Vec(2, FDIStateReq()))
    val lpRxActiveSts = Input(Vec(2, Bool()))
    val lpStallAck = Input(Vec(2, Bool()))

    // FDI observations, one per die (peeked by the testbench).
    val fdiPlStateSts = Output(Vec(2, FDIState()))
    val fdiPlInbandPres = Output(Vec(2, Bool()))
    val fdiPlRxActiveReq = Output(Vec(2, Bool()))
    val fdiPlProtocolVld = Output(Vec(2, Bool()))
    val fdiPlStallReq = Output(Vec(2, Bool()))

    // RDI observation: the state request each adapter drives to its real PHY.
    val rdiLpStateReq = Output(Vec(2, RDIStateReq()))

    // PHY observations.
    val ltsmState = Output(Vec(2, LTSMState()))
    val phyPlStateSts = Output(Vec(2, RDIState()))
    val phyPlInbandPres = Output(Vec(2, Bool()))
    val trainingTimedout = Output(Vec(2, Bool()))
    val plTrainError = Output(Vec(2, Bool()))
    val sbFaultSeen = Output(Vec(2, Bool()))
  })

  val phys = Seq.fill(2)(Module(new LogicalPhy(
    afeParams = afeParams,
    sbParams = sbParams,
    rdiParams = rdiParams,
    // WIP: ltsmTimeoutCycles / phyTrainerAutoRespond were params on the
    // (reverted) instrumented LogicalPhy; restore when observability hooks land.
  )))
  val adapters = Seq.fill(2)(Module(new D2DAdapter(fdiParams, rdiParams, sbParams)))

  for (i <- 0 until 2) {
    val phy = phys(i).io
    val peerPhy = phys(1 - i).io
    val adapter = adapters(i).io

    // Adapter <-> PHY: the verified top-level RDI wiring.
    adapter.rdi <> phy.rdi

    // Sideband serial crossover (bits and fwClock together, zero skew).
    phy.analog.sidebandLink.in.bits := peerPhy.analog.sidebandLink.out.bits
    phy.analog.sidebandLink.in.fwClock := peerPhy.analog.sidebandLink.out.fwClock

    // Mainband AFE crossover, lanes straight (0 <-> 0).
    phy.analog.mainband.rx.bits := peerPhy.analog.mainband.tx.bits
    phy.analog.mainband.rx.valid := peerPhy.analog.mainband.tx.valid
    phy.analog.mainband.tx.ready := peerPhy.analog.mainband.rx.ready

    // Analog macro status: PLL locked and clocks stable at all times.
    phy.analog.status.pllLock := true.B
    phy.analog.status.clocksUngatedAndStable := true.B

    // PHY control (same settings as LogPhyLoopbackHarness: both dies
    // identical, speed4, no retries).
    phy.ctrl.pwrGood := true.B
    phy.ctrl.retryTrainingAmt := 0.U
    phy.ctrl.swStartLinkTraining := io.swStartLinkTraining(i)
    phy.ctrl.maxErrorThresholdPerLane := 0.U
    phy.ctrl.changeInRuntimeLinkCtrlRegsDetected := false.B
    phy.ctrl.runtimeLinkCtrlBusyBit := false.B
    phy.ctrl.runtimeRequestForRepair := false.B

    phy.ctrl.localPhyParamSettings.valid := true.B
    phy.ctrl.localPhyParamSettings.bits.voltageSwing := 0.U
    phy.ctrl.localPhyParamSettings.bits.maxDataRate := 0.U
    phy.ctrl.localPhyParamSettings.bits.clockMode := 0.U
    phy.ctrl.localPhyParamSettings.bits.clockPhase := 0.U
    phy.ctrl.localPhyParamSettings.bits.ucieSx8 := 0.U
    phy.ctrl.localPhyParamSettings.bits.sbFeatExt := 0.U
    phy.ctrl.localPhyParamSettings.bits.txAdjRuntime := 0.U
    phy.ctrl.localPhyParamSettings.bits.moduleId := 0.U

    phy.ctrl.linkTrainingParameters.clockPhase := 0.U
    phy.ctrl.linkTrainingParameters.dataPattern := 0.U
    phy.ctrl.linkTrainingParameters.validPattern := 0.U
    phy.ctrl.linkTrainingParameters.patternMode := 0.U
    phy.ctrl.linkTrainingParameters.iterationCount := 0.U
    phy.ctrl.linkTrainingParameters.idleCount := 0.U
    phy.ctrl.linkTrainingParameters.burstCount := 0.U
    phy.ctrl.linkTrainingParameters.maxErrorThreshold := 0.U
    phy.ctrl.linkTrainingParameters.comparisonMode := 0.U

    // Protocol-layer drive (testbench-controlled state-management subset).
    adapter.fdi.lpStateReq := io.lpStateReq(i)
    adapter.fdi.lpRxActiveSts := io.lpRxActiveSts(i)
    adapter.fdi.lpStallAck := io.lpStallAck(i)

    // Remaining FDI inputs tied off to a quiet protocol layer
    // (AdapterLoopbackHarness.scala:88-97 tie-off list).
    adapter.fdi.lpIrdy := false.B
    adapter.fdi.lpValid := false.B
    adapter.fdi.lpData := 0.U
    adapter.fdi.lpLinkError := false.B
    adapter.fdi.lpClkAck := false.B
    adapter.fdi.lpWakeReq := false.B
    adapter.fdi.lpCfg := 0.U
    adapter.fdi.lpCfgVld := false.B
    adapter.fdi.plCfgCrd := false.B

    // Observations.
    io.fdiPlStateSts(i) := adapter.fdi.plStateSts
    io.fdiPlInbandPres(i) := adapter.fdi.plInbandPres
    io.fdiPlRxActiveReq(i) := adapter.fdi.plRxActiveReq
    io.fdiPlProtocolVld(i) := adapter.fdi.plProtocolVld
    io.fdiPlStallReq(i) := adapter.fdi.plStallReq
    io.rdiLpStateReq(i) := adapter.rdi.lpStateReq

    io.ltsmState(i) := phy.status.currentState
    io.phyPlStateSts(i) := phy.rdi.plStateSts
    io.phyPlInbandPres(i) := phy.rdi.plInbandPres
    io.trainingTimedout(i) := phy.status.trainingTimedout
    io.plTrainError(i) := phy.rdi.plTrainError
    io.sbFaultSeen(i) :=
      phy.status.sideband.sbParityErrSeen ||
      phy.status.sideband.sbRxPriorityQueuesFullSeen ||
      phy.status.sideband.sbDeserializerTimedoutSeen ||
      phy.status.sideband.sbInvalidRouteUpperSeen ||
      phy.status.sideband.sbInvalidRouteCurrSeen ||
      phy.status.sideband.sbInvalidRouteLowerSeen ||
      phy.status.sideband.sbUnhandledCurrentLayerMsgSeen
  }
}
