package edu.berkeley.cs.uciedigital.loopback

import edu.berkeley.cs.uciedigital.logphy._

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
  * never waits on an absent adapter. pwrGood is testbench-driven so
  * mid-training power-loss behavior can be characterized (the LTSM samples it
  * only on the RESET exit arc, LinkTrainingSM.scala:1142).
  *
  * Both dies advertise identical PHY parameters (maxDataRate=speed4, x16), so
  * MBINIT.PARAM interoperability passes and MBTRAIN.SPEEDIDLE performs no real
  * frequency change.
  *
  * Pristine-RTL timing: the RESET minimum wait is timeoutCycles/2 =
  * 3,200,000 cycles (LinkTrainingSM.scala:108-116, 800 MHz * 4 ms) and every
  * substate has a 6,400,000-cycle residency timeout. There are no
  * instrumentation hooks; bring-up tests must simply pay the 3.2M-cycle wait.
  *
  * Index convention: die 0 and die 1; die i's RX comes from die 1-i.
  */
class LogPhyLoopbackHarness(
  val afeParams: AfeParams = new AfeParams(),
  val sbParams: SidebandParams = new SidebandParams(),
  val rdiParams: RdiParams = RdiParams(64, 32),
  val exposeDataPath: Boolean = false,
) extends Module {
  val io = IO(new Bundle {
    // Per-die drive (poked by the testbench).
    val lpStateReq = Input(Vec(2, RDIStateReq()))
    val swStartLinkTraining = Input(Vec(2, Bool()))
    val pwrGood = Input(Vec(2, Bool()))

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

    // NOTE: adding further observation ports (e.g. plCfg / per-class fault
    // bits) makes the generated Verilator model segfault at time zero in
    // VerilatedScope registration with this toolchain (verilator 5.048
    // custom build) -- keep the port list as-is and observe finer detail via
    // waveforms if needed. The RDI data ports below are the one deliberate
    // exception: a data round trip cannot be observed from a waveform assertion.

    // RDI mainband data path. One RDI word is nBytes*8 bits and, at x16 with a
    // serialiser ratio of 32, occupies exactly one mainband beat, so a transfer
    // completes in the cycle plTrdy is high with lpValid and lpIrdy asserted
    // (MainbandLaneController.scala:124,:186). The receive side has no
    // backpressure -- plValid is a one-cycle pulse and plData is only valid
    // during it (:239-241).
    //
    // Present only for a rung that actually moves data, because the tie-offs
    // they replace are what let the simulator drop both packing paths. With
    // lpValid and lpIrdy constant false the transmit pack folds away, and with
    // plData unobserved so does the receive unpack -- 16 lanes x 4 dynamically
    // indexed byte writes into a 64-entry accumulator
    // (MainbandLaneController.scala:174-182, :209-236). Left permanently
    // connected they cost every rung several times its wall clock, paid on all
    // 3.2M reset cycles.
    val lpData =
      Option.when(exposeDataPath)(Input(Vec(2, UInt((rdiParams.nBytes * 8).W))))
    val lpValid = Option.when(exposeDataPath)(Input(Vec(2, Bool())))
    val lpIrdy = Option.when(exposeDataPath)(Input(Vec(2, Bool())))
    val plTrdy = Output(Vec(2, Bool()))
    val plValid = Output(Vec(2, Bool()))
    val plData =
      Option.when(exposeDataPath)(Output(Vec(2, UInt((rdiParams.nBytes * 8).W))))

    // Waveform-level activity indicators. sbTxBits is the raw 1-bit serial
    // sideband TX line (SBINIT's 64-bit clock pattern is visible here as
    // alternating 0/1 while sbFwClock toggles).
    val sbFwClock = Output(Vec(2, Bool()))
    val sbTxBits = Output(Vec(2, UInt(1.W)))
    val mbTxValid = Output(Vec(2, Bool()))
  })

  val duts = Seq.fill(2)(Module(new LogicalPhy(
    afeParams = afeParams,
    sbParams = sbParams,
    rdiParams = rdiParams,
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

    // PHY control: power good is testbench-driven, no retries (retry would
    // amplify any failure by another full timeout), identical local parameters
    // on both dies (MBINIT.PARAM interoperability, MBInitSM.scala:136-145).
    dut.ctrl.pwrGood := io.pwrGood(i)
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
    dut.rdi.lpIrdy := io.lpIrdy.map(_(i)).getOrElse(false.B)
    dut.rdi.lpValid := io.lpValid.map(_(i)).getOrElse(false.B)
    dut.rdi.lpData := io.lpData.map(_(i)).getOrElse(0.U)
    dut.rdi.lpLinkError := false.B
    dut.rdi.lpWakeReq := false.B
    dut.rdi.lpCfg := 0.U
    dut.rdi.lpCfgVld := false.B
    // No cfg credits are returned (quiet adapter). Note for SBINIT: the
    // sideband switch misroutes the raw 64-bit clock-pattern words up the
    // D2D cfg path (SidebandSwitch.scala:56 routes on msg[57:56], and
    // 0x5555...5555 gives 0b01 = D2D), so they exhaust the 32 cfg credits
    // here and overflow the RX priority queues -- measured as
    // sbRxPriorityQueuesFullSeen (part of sbFaultSeen) latching ~6.6k cycles
    // after the training trigger.
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
    io.plTrdy(i) := dut.rdi.plTrdy
    io.plValid(i) := dut.rdi.plValid
    io.plData.foreach(p => p(i) := dut.rdi.plData)
    io.sbFwClock(i) := dut.analog.sidebandLink.out.fwClock.asBool
    io.sbTxBits(i) := dut.analog.sidebandLink.out.bits
    io.mbTxValid(i) := dut.analog.mainband.tx.valid
  }
}
