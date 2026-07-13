package edu.berkeley.cs.uciedigital.logphy

import chisel3._
import edu.berkeley.cs.uciedigital.interfaces._
import edu.berkeley.cs.uciedigital.sideband._

/**
  * Two-LogicalPhy loopback harness for full link-training tests.
  *
  * Two complete LogicalPhy instances are cross-wired at the analog boundary,
  * so every training exchange travels the real digital path:
  *   - Sideband serial: die i's 1-bit serial RX (bits + forwarded clock) is
  *     die 1-i's serializer output (LogicalPhy.scala:251-254). The receiving
  *     deserializer reclocks on (!fwClock).asClock and crosses back through an
  *     AsyncQueue (SidebandLinkSerdes.scala:228-235), so bits and fwClock must
  *     never see different delays; this harness adds none.
  *   - Mainband AFE: die i's RX lanes (data/valid/clkP/clkN/trk) are die 1-i's
  *     TX lanes, straight-wired (lane 0 <-> lane 0), so MBINIT.REVERSALMB
  *     resolves to "no reversal". clkP/clkN/trk are included because
  *     MBTRAIN.RXCLKCAL drives the fwd-clock pattern on them
  *     (LogicalPhy.scala:406-411).
  *
  * The analog macro itself is not modeled: LogicalPhy only consumes two status
  * bits from it (pllLock, clocksUngatedAndStable, Bundles.scala:77-80), which
  * are tied high here.
  *
  * The testbench drives only what sits above the RDI: lpStateReq (the
  * nop->active edge is the training trigger, LinkTrainingSM.scala:223-235) and
  * swStartLinkTraining. Clock/stall RDI handshakes are auto-acked so the PHY
  * never waits on an absent adapter.
  *
  * Both dies advertise identical PHY parameters (maxDataRate=speed4, x16), so
  * MBINIT.PARAM interoperability passes and MBTRAIN.SPEEDIDLE performs no real
  * frequency change.
  *
  * `ltsmTimeoutCycles` rescales the hardcoded 8 ms residency timeout; its half
  * is the RESET minimum wait that otherwise costs 3.2M cycles per bring-up.
  * `phyTrainerAutoRespond=false` keeps the default PhyLaneTrainer stub whose
  * constant-false responses stall MBINIT.CAL forever (PhyLaneTrainer.scala,
  * MBInitSM.scala sCAL) -- used to pin the stub-blocked behavior.
  *
  * Index convention: die 0 and die 1; die i's RX comes from die 1-i.
  */
class LogPhyLoopbackHarness(
  val afeParams: AfeParams = new AfeParams(),
  val sbParams: SidebandParams = new SidebandParams(),
  val rdiParams: RdiParams = RdiParams(64, 32),
  val ltsmTimeoutCycles: Int = 16384,
  val phyTrainerAutoRespond: Boolean = true,
) extends Module {
  val io = IO(new Bundle {
    // Per-die drive (poked by the testbench).
    val lpStateReq = Input(Vec(2, RDIStateReq()))
    val swStartLinkTraining = Input(Vec(2, Bool()))

    // Primary observation: the 26-state debug LTSM state and the coarse state.
    val ltsmState = Output(Vec(2, LTSMState()))
    val ltState = Output(Vec(2, LTState()))

    // Transition evidence and failure early-detection.
    val trainingTimedout = Output(Vec(2, Bool()))
    val negotiatedParamsValid = Output(Vec(2, Bool()))
    val plStateSts = Output(Vec(2, RDIState()))
    val plInbandPres = Output(Vec(2, Bool()))
    val plTrainError = Output(Vec(2, Bool()))
    val plPhyInRecenter = Output(Vec(2, Bool()))
    val plSpeedmode = Output(Vec(2, SpeedMode()))
    val sbFaultSeen = Output(Vec(2, Bool()))

    // Waveform-level activity indicators.
    val sbFwClock = Output(Vec(2, Bool()))
    val mbTxValid = Output(Vec(2, Bool()))
  })

  val duts = Seq.fill(2)(Module(new LogicalPhy(
    afeParams = afeParams,
    sbParams = sbParams,
    rdiParams = rdiParams,
    ltsmTimeoutCycles = ltsmTimeoutCycles,
    phyTrainerAutoRespond = phyTrainerAutoRespond,
  )))

  for (i <- 0 until 2) {
    val dut = duts(i).io
    val peer = duts(1 - i).io

    // Sideband serial crossover (bits and fwClock together, zero skew).
    dut.analog.sidebandLink.in.bits := peer.analog.sidebandLink.out.bits
    dut.analog.sidebandLink.in.fwClock := peer.analog.sidebandLink.out.fwClock

    // Mainband AFE crossover, lanes straight (0 <-> 0).
    dut.analog.mainband.rx.bits := peer.analog.mainband.tx.bits
    dut.analog.mainband.rx.valid := peer.analog.mainband.tx.valid
    dut.analog.mainband.tx.ready := peer.analog.mainband.rx.ready

    // Analog macro status: PLL locked and clocks stable at all times.
    // (RESET exit and MBTRAIN.SPEEDIDLE's speedChanged both require pllLock.)
    dut.analog.status.pllLock := true.B
    dut.analog.status.clocksUngatedAndStable := true.B

    // PHY control: power good, no retries (retry would amplify any failure by
    // another full timeout), identical local parameters on both dies
    // (MBINIT.PARAM interoperability requires it, MBInitSM.scala:136-145).
    dut.ctrl.pwrGood := true.B
    dut.ctrl.retryTrainingAmt := 0.U
    dut.ctrl.swStartLinkTraining := io.swStartLinkTraining(i)
    dut.ctrl.maxErrorThresholdPerLane := 0.U
    dut.ctrl.changeInRuntimeLinkCtrlRegsDetected := false.B
    dut.ctrl.runtimeLinkCtrlBusyBit := false.B
    dut.ctrl.runtimeRequestForRepair := false.B

    dut.ctrl.localPhyParamSettings.valid := true.B
    dut.ctrl.localPhyParamSettings.bits.voltageSwing := 0.U
    dut.ctrl.localPhyParamSettings.bits.maxDataRate := 0.U // speed4: no real freq change
    dut.ctrl.localPhyParamSettings.bits.clockMode := 0.U
    dut.ctrl.localPhyParamSettings.bits.clockPhase := 0.U
    dut.ctrl.localPhyParamSettings.bits.ucieSx8 := 0.U
    dut.ctrl.localPhyParamSettings.bits.sbFeatExt := 0.U
    dut.ctrl.localPhyParamSettings.bits.txAdjRuntime := 0.U
    dut.ctrl.localPhyParamSettings.bits.moduleId := 0.U

    dut.ctrl.linkTrainingParameters.clockPhase := 0.U
    dut.ctrl.linkTrainingParameters.dataPattern := 0.U
    dut.ctrl.linkTrainingParameters.validPattern := 0.U
    dut.ctrl.linkTrainingParameters.patternMode := 0.U
    dut.ctrl.linkTrainingParameters.iterationCount := 0.U
    dut.ctrl.linkTrainingParameters.idleCount := 0.U
    dut.ctrl.linkTrainingParameters.burstCount := 0.U
    dut.ctrl.linkTrainingParameters.maxErrorThreshold := 0.U
    dut.ctrl.linkTrainingParameters.comparisonMode := 0.U

    // RDI: only lpStateReq is testbench-controlled. The clock handshake is
    // auto-acked (RDI active entry requires it, RDIController.scala:71-73);
    // stall requests are auto-acked; everything else is a quiet adapter.
    dut.rdi.lclk := false.B
    dut.rdi.lpStateReq := io.lpStateReq(i)
    dut.rdi.lpClkAck := dut.rdi.plClkReq
    dut.rdi.lpStallAck := dut.rdi.plStallReq
    dut.rdi.lpIrdy := false.B
    dut.rdi.lpValid := false.B
    dut.rdi.lpData := 0.U
    dut.rdi.lpLinkError := false.B
    dut.rdi.lpWakeReq := false.B
    dut.rdi.lpCfg := 0.U
    dut.rdi.lpCfgVld := false.B
    dut.rdi.lpCfgCrd := false.B

    // Observations.
    io.ltsmState(i) := dut.status.currentState
    io.ltState(i) := dut.status.ltState
    io.trainingTimedout(i) := dut.status.trainingTimedout
    io.negotiatedParamsValid(i) := dut.status.negotiatedPhyParamSettings.valid
    io.plStateSts(i) := dut.rdi.plStateSts
    io.plInbandPres(i) := dut.rdi.plInbandPres
    io.plTrainError(i) := dut.rdi.plTrainError
    io.plPhyInRecenter(i) := dut.rdi.plPhyInRecenter
    io.plSpeedmode(i) := dut.rdi.plSpeedmode
    io.sbFaultSeen(i) :=
      dut.status.sideband.sbParityErrSeen ||
      dut.status.sideband.sbRxPriorityQueuesFullSeen ||
      dut.status.sideband.sbDeserializerTimedoutSeen ||
      dut.status.sideband.sbInvalidRouteUpperSeen ||
      dut.status.sideband.sbInvalidRouteCurrSeen ||
      dut.status.sideband.sbInvalidRouteLowerSeen ||
      dut.status.sideband.sbUnhandledCurrentLayerMsgSeen
    io.sbFwClock(i) := dut.analog.sidebandLink.out.fwClock.asBool
    io.mbTxValid(i) := dut.analog.mainband.tx.valid
  }
}
