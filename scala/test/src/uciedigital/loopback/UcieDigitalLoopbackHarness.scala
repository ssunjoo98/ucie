package edu.berkeley.cs.uciedigital.loopback

import chisel3._
import chisel3.util.experimental.BoringUtils
import edu.berkeley.cs.uciedigital.d2dadapter.{D2DAdapter, LinkInitState}
import edu.berkeley.cs.uciedigital.interfaces._
import edu.berkeley.cs.uciedigital.logphy._
import edu.berkeley.cs.uciedigital.protocol._
import edu.berkeley.cs.uciedigital.sideband._

/** Bit positions inside `io.protoCtrl` (one 4-bit word per die).
  *
  * Packed rather than four `Input(Vec(2, Bool()))` ports for the same reason as
  * `io.flags`: this toolchain has been observed to segfault the generated
  * Verilator model at time zero during VerilatedScope registration when the
  * harness port list grows (LogPhyLoopbackHarness.scala:73-78), and the full
  * stack registers strictly more scopes than the LogPhy harness does.
  */
object ProtoCtrl {
  val active = 0
  val retrain = 1
  val linkReset = 2
  val disable = 3

  def word(
      reqActive: Boolean = false,
      reqRetrain: Boolean = false,
      reqLinkReset: Boolean = false,
      reqDisable: Boolean = false,
  ): BigInt =
    (if (reqActive) BigInt(1) << active else BigInt(0)) |
      (if (reqRetrain) BigInt(1) << retrain else BigInt(0)) |
      (if (reqLinkReset) BigInt(1) << linkReset else BigInt(0)) |
      (if (reqDisable) BigInt(1) << disable else BigInt(0))
}

/** Bit positions inside `io.flags` (one 64-bit word per die).
  *
  * Forty-two single-bit observations would be forty-two `Vec(2, Bool())` ports.
  * One packed word costs one port and peeks just as cheaply, and the decode
  * table below turns it back into names for the failure messages.
  *
  * The seven sideband fault bits are carried INDIVIDUALLY, unlike
  * LogPhyLoopbackHarness.scala:200-207 which ORs them into a single
  * `sbFaultSeen`. At this level the adapters put their own traffic (ADV_CAP,
  * REQ/RSP_ACTIVE) on the RDI cfg path, and a fault latched there has to be
  * attributed to a specific bit before it can be triaged -- an OR only tells
  * you to go open a waveform.
  */
object DieFlag {
  // --- PHY / RDI contact surface ---------------------------------------------
  val rdiInbandPres = 0
  val rdiLpReqActive = 1 // adapter -> PHY lp_state_req == active
  val rdiStallReq = 2
  val rdiStallAck = 3
  val rdiPlTrdy = 4
  val rdiPlValid = 5
  val rdiPlWakeAck = 6 // the wake handshake the LogPhy harness never exercised
  val phyParamsVld = 7
  val phyRecenter = 8
  val phyTrainError = 9
  val phyTimedout = 10
  val rdiPlError = 11 // valid-framing error; invisible above the FDI

  // --- Sideband faults, one bit each -----------------------------------------
  val sbParityErr = 12
  val sbRxQueuesFull = 13
  val sbDeserTimedout = 14
  val sbBadRouteUpper = 15
  val sbBadRouteCurr = 16
  val sbBadRouteLower = 17
  val sbUnhandledMsg = 18

  // --- Adapter / FDI contact surface -----------------------------------------
  val fdiInbandPres = 19
  val fdiRxActiveReq = 20
  val fdiRxActiveSts = 21
  val fdiProtocolVld = 22
  val fdiStallReq = 23
  val fdiStallAck = 24
  val fdiLpReqActive = 25 // protocol -> adapter lp_state_req == active
  val fdiPlTrdy = 26
  val fdiPlValid = 27 // MUST be a one-cycle pulse per delivered beat
  val fdiLpValid = 28 // protocol is presenting a beat

  // --- Protocol layer ---------------------------------------------------------
  val negotiatedProto = 29
  val protoStalled = 30
  val rxOverflow = 31
  val chipTxReady = 32
  val chipRxValid = 33

  // --- AdapterSM link-init probes (zero when exposeAdapterProbes = false) -----
  val advCapSent = 34
  val advCapRcvd = 35
  val actReqSent = 36
  val actReqRcvd = 37
  val actRspSent = 38
  val actRspRcvd = 39
  val transitionToActive = 40

  val width = 64

  /** All seven latched sideband fault bits, for the rung that triages them. */
  val sbFaults: Seq[(Int, String)] = Seq(
    sbParityErr -> "sbParityErr",
    sbRxQueuesFull -> "sbRxQueuesFull",
    sbDeserTimedout -> "sbDeserTimedout",
    sbBadRouteUpper -> "sbBadRouteUpper",
    sbBadRouteCurr -> "sbBadRouteCurr",
    sbBadRouteLower -> "sbBadRouteLower",
    sbUnhandledMsg -> "sbUnhandledMsg",
  )

  val names: Seq[(Int, String)] = Seq(
    rdiInbandPres -> "rdiPres",
    rdiLpReqActive -> "rdiReqAct",
    rdiStallReq -> "rdiStallReq",
    rdiStallAck -> "rdiStallAck",
    rdiPlTrdy -> "rdiTrdy",
    rdiPlValid -> "rdiVld",
    rdiPlWakeAck -> "wakeAck",
    phyParamsVld -> "phyParams",
    phyRecenter -> "recenter",
    phyTrainError -> "trainErr",
    phyTimedout -> "timedout",
    rdiPlError -> "plError",
  ) ++ sbFaults ++ Seq(
    fdiInbandPres -> "fdiPres",
    fdiRxActiveReq -> "rxActReq",
    fdiRxActiveSts -> "rxActSts",
    fdiProtocolVld -> "protoVld",
    fdiStallReq -> "fdiStallReq",
    fdiStallAck -> "fdiStallAck",
    fdiLpReqActive -> "fdiReqAct",
    fdiPlTrdy -> "fdiTrdy",
    fdiPlValid -> "fdiVld",
    fdiLpValid -> "fdiLpVld",
    negotiatedProto -> "negProto",
    protoStalled -> "stalled",
    rxOverflow -> "rxOvf",
    chipTxReady -> "txRdy",
    chipRxValid -> "rxVld",
    advCapSent -> "advSnt",
    advCapRcvd -> "advRcv",
    actReqSent -> "reqSnt",
    actReqRcvd -> "reqRcv",
    actRspSent -> "rspSnt",
    actRspRcvd -> "rspRcv",
    transitionToActive -> "toActive",
  )
}

/**
  * Two-die UcieDigital loopback harness: a full {ProtocolLayer + D2DAdapter +
  * LogicalPhy} stack on each die, wired exactly as UcieDigitalTop wires them
  * (UcieDigitalTop.scala:89-90), with the two dies cross-wired at the LogicalPhy
  * analog boundary exactly as LogPhyLoopbackHarness does
  * (LogPhyLoopbackHarness.scala:122-134). That crossover is proven by the S0..S8
  * ladder and is copied verbatim: sideband bits and fwClock see zero relative
  * skew, and the mainband lanes are straight (0 <-> 0) so MBINIT.REVERSALMB
  * resolves to "no reversal".
  *
  * WHAT IS MODELLED. Everything on the bring-up path, in real RTL, on both dies:
  * link training, the RDI, the D2D adapter link-init choreography over the real
  * serial sideband, the FDI, and the protocol layer's own state machines. No
  * pl_* / lp_* signal on either interface is stubbed by the testbench any more:
  *   - the RDI is no longer a testbench stub. lp_clk_ack now arrives one cycle
  *     late (D2DAdapter.scala:59-60) where the stub answered combinationally
  *     (LogPhyLoopbackHarness.scala:172); lp_stall_ack now runs through
  *     RDIStallHandler (StallHandler.scala:62-93) where the stub auto-acked;
  *     lp_cfg_crd returns real credits (D2DAdapter.scala:90) where the stub
  *     returned none (LogPhyLoopbackHarness.scala:188); lp_wake_req is now a
  *     constant true (D2DAdapter.scala:61) where the stub held it false, so the
  *     PHY's wake responder is on the critical path for the first time; and
  *     lp_state_req comes from AdapterSM instead of a poke.
  *   - the FDI is not a stub either: lp_state_req / lp_rx_active_sts /
  *     lp_stall_ack / lp_irdy / lp_valid / lp_data / lp_cfg* are outputs of a
  *     real ProtocolLayer.
  *
  * WHAT IS FAKED. Three things, all of them deliberate:
  *   - the analog macro. LogicalPhy consumes exactly two bits from it (pllLock,
  *     clocksUngatedAndStable, Bundles.scala:77-80) and both are tied high.
  *     There is no hardware source for either in this design (BRINGUP_SEQUENCE
  *     D-19), so tying them high is the only option and it is also what makes
  *     the RDI wake responder able to leave sUNGATE at all
  *     (RDIWakeHandshakeResponder.scala:46-52).
  *   - the register block. UcieDigitalTop instantiates one, but it drives
  *     nothing on the bring-up path (UcieDigitalTop.scala:109-126 ties its
  *     inputs off and dontTouches its outputs), and instantiating it here would
  *     drag an unbound TileLink RegisterNode and IntNode diplomacy graph into a
  *     plain Module. Software control is `io.protoCtrl` instead.
  *   - the chip-facing protocol traffic, which is a Decoupled beat source/sink
  *     driven by the testbench (ProtocolRawBeat has one 512-bit `data` field and
  *     no framing at all, ProtocolTypes.scala:17-19).
  *
  * WHY NOT UcieDigitalTop. It ties `protocolLayer.io.ctrl`,
  * `logicalPhy.io.ctrl` and `logicalPhy.io.analog.status` to DontCare
  * (UcieDigitalTop.scala:102-104), so at that top there is no
  * swStartLinkTraining, no pwrGood, no pllLock and no clocksUngatedAndStable:
  * the LTSM's RESET exit gate `pwrGood && pllLock && resetMinWait &&
  * (freshTrainingTrigger || autoRetrain)` (LinkTrainingSM.scala:1141-1142) can
  * never be satisfied and the shipping top-level cannot bring its own link up.
  * That is D-16/D-19 and it is a separate defect, not something this harness
  * should paper over. UcieDigitalTop also cannot be instantiated twice in a
  * plain Module without its diplomacy graph. Finally D-15 (RxIO has no
  * clkp/clkn, so TileLink.scala hard-codes the RX clock lanes) and D-17
  * (Phy.SerdesRatio vs AfeParams.mbLanes are unrelated by any `require`) are why
  * the ladder cross-wires the full `LogicalPhy.io.analog` bundle here rather
  * than moving up to the chip top -- do not "improve" this harness by doing so.
  *
  * Index convention: die 0 and die 1; die i's RX comes from die 1-i.
  *
  * DEPENDENCY: `proto.ctrl.requestActive` does not exist in the current tree.
  * This harness does not elaborate until the ProtocolLayerCtrlIO /
  * ProtocolStateController patch lands (see the U4 rung's blocker string in
  * UcieDigitalStagedBringupTest.scala).
  */
class UcieDigitalLoopbackHarness(
  val protocolParams: ProtocolLayerParams = ProtocolLayerParams(),
  val fdiParams: FdiParams = FdiParams(64, 32),
  val rdiParams: RdiParams = RdiParams(64, 32),
  val afeParams: AfeParams = new AfeParams(),
  val sbParams: SidebandParams = new SidebandParams(),
  val exposeDataPath: Boolean = false,
  val exposeAdapterProbes: Boolean = true,
) extends Module {
  require(fdiParams.nBytes == rdiParams.nBytes,
    "D2DAdapter requires equal FDI/RDI data widths (D2DAdapter.scala:19)")
  require(fdiParams.ncWidth == rdiParams.ncWidth,
    "D2DAdapter requires equal FDI/RDI cfg widths (D2DAdapter.scala:20)")
  // Nothing in UcieDigitalTopParams.validate() relates the FDI word to the AFE
  // geometry, so state the ladder's own assumption here: one FDI/RDI word must
  // be exactly one mainband beat, or the byte-exact data rungs stop meaning what
  // they say (MainbandLaneController.scala:83-96 would split a word into beats).
  require(fdiParams.nBytes * 8 == afeParams.mbLanes * afeParams.mbSerializerRatio,
    s"ladder assumes 1 FDI word == 1 mainband beat: ${fdiParams.nBytes * 8} bits vs " +
      s"${afeParams.mbLanes} lanes x ${afeParams.mbSerializerRatio}")

  val beatBits: Int = fdiParams.nBytes * 8

  val io = IO(new Bundle {
    // ---- Per-die drive (poked by the testbench) -----------------------------
    val swStartLinkTraining = Input(Vec(2, Bool()))
    /** Testbench-driven like LogPhyLoopbackHarness.scala:139, not tied true like
      * FullStackLoopbackHarness.scala:98, so mid-training power loss stays
      * characterizable. The LTSM samples it only on the RESET exit arc. */
    val pwrGood = Input(Vec(2, Bool()))
    /** Software requests into ProtocolLayer.io.ctrl, packed; see ProtoCtrl. */
    val protoCtrl = Input(Vec(2, UInt(4.W)))

    // ---- Observation: the five state machines that matter -------------------
    val ltState = Output(Vec(2, LTState()))
    val ltsmState = Output(Vec(2, LTSMState()))
    val rdiState = Output(Vec(2, RDIState()))
    val fdiState = Output(Vec(2, FDIState()))
    /** AdapterSM.linkInitStateReg, tapped. The single most useful diagnostic on
      * this path: INIT_START / RDI_BRINGUP / PARAM_EXCH / FDI_BRINGUP /
      * INIT_DONE separates "the PHY never came up" from "the ADV_CAP was
      * dropped" from "the protocol never asked for Active". Constant INIT_START
      * when exposeAdapterProbes = false. */
    val adapterLinkInit = Output(Vec(2, LinkInitState()))

    /** Everything else, packed one bit per observation; see DieFlag. */
    val flags = Output(Vec(2, UInt(DieFlag.width.W)))

    // ---- Protocol-level (chip-facing) data path -----------------------------
    // Behind `exposeDataPath` for the same reason as the RDI ports one level
    // down (LogPhyLoopbackHarness.scala:87-94). With mainbandTx.valid tied false
    // and mainbandRx.bits unobserved, the whole 512-bit chain folds away: the
    // protocol TX/RX queues, the adapter's dataBuff registers, and the
    // expensive part -- the 16-lane x 4-byte dynamically indexed pack/unpack in
    // MainbandLaneController.scala:174-182 and :209-236. MEASURED on this
    // harness, 1M idle cycles: 4.7 s with exposeDataPath=false, 33.1 s with it
    // true, i.e. 7.1x, and that multiplier is paid on all 3.2M RESET cycles of
    // every rung. The handshake bits (chipTxReady / chipRxValid / fdiPlValid /
    // rdiPlValid) stay in `flags` unconditionally: one bit each, no datapath
    // kept alive behind them.
    val txValid = Option.when(exposeDataPath)(Input(Vec(2, Bool())))
    val txData = Option.when(exposeDataPath)(Input(Vec(2, UInt(beatBits.W))))
    val rxReady = Option.when(exposeDataPath)(Input(Vec(2, Bool())))
    val rxData = Option.when(exposeDataPath)(Output(Vec(2, UInt(beatBits.W))))
  })

  val phys = Seq.fill(2)(Module(new LogicalPhy(
    afeParams = afeParams,
    sbParams = sbParams,
    rdiParams = rdiParams,
  )))
  val adapters = Seq.fill(2)(Module(new D2DAdapter(fdiParams, rdiParams, sbParams)))
  val protocols = Seq.fill(2)(Module(new ProtocolLayer(protocolParams, fdiParams, sbParams)))

  for (i <- 0 until 2) {
    val phy = phys(i).io
    val peer = phys(1 - i).io
    val adapter = adapters(i).io
    val proto = protocols(i).io

    // ========================================================================
    // Analog crossover -- copied verbatim from LogPhyLoopbackHarness:122-134.
    // Bits and fwClock must never see different delays; this harness adds none.
    // ========================================================================
    phy.analog.sidebandLink.in.bits := peer.analog.sidebandLink.out.bits
    phy.analog.sidebandLink.in.fwClock := peer.analog.sidebandLink.out.fwClock

    phy.analog.mainband.rx.bits := peer.analog.mainband.tx.bits
    phy.analog.mainband.rx.valid := peer.analog.mainband.tx.valid
    phy.analog.mainband.tx.ready := peer.analog.mainband.rx.ready

    // RESET exit and MBTRAIN.SPEEDIDLE both need pllLock; the RDI wake
    // responder needs clocksUngatedAndStable to leave sUNGATE, and with a real
    // adapter holding lp_wake_req high forever that is now a hard gate on RDI
    // ACTIVE entry (RDIController.scala:71-73).
    phy.analog.status.pllLock := true.B
    phy.analog.status.clocksUngatedAndStable := true.B

    // ========================================================================
    // The stack, wired as UcieDigitalTop.scala:89-90 does it. These two lines
    // replace all fourteen FDI drive statements the old FullStackLoopbackHarness
    // carried (FullStackLoopbackHarness.scala:127-141): if any of them survived
    // alongside a real ProtocolLayer, Chisel last-connect would silently make
    // the tie-off win and the protocol layer would be a decoration.
    // ========================================================================
    proto.fdi <> adapter.fdi
    adapter.rdi <> phy.rdi

    // ========================================================================
    // PHY control -- same settings as LogPhyLoopbackHarness:139-165: identical
    // parameters on both dies so MBINIT.PARAM interoperability passes, speed4 so
    // MBTRAIN.SPEEDIDLE performs no real frequency change, no retries (a retry
    // would amplify any failure by another full timeout).
    // ========================================================================
    phy.ctrl.pwrGood := io.pwrGood(i)
    phy.ctrl.retryTrainingAmt := 0.U
    phy.ctrl.swStartLinkTraining := io.swStartLinkTraining(i)
    phy.ctrl.maxErrorThresholdPerLane := 0.U
    phy.ctrl.changeInRuntimeLinkCtrlRegsDetected := false.B
    phy.ctrl.runtimeLinkCtrlBusyBit := false.B
    phy.ctrl.runtimeRequestForRepair := false.B

    phy.ctrl.localPhyParamSettings.valid := true.B
    phy.ctrl.localPhyParamSettings.bits.voltageSwing := 0.U
    phy.ctrl.localPhyParamSettings.bits.maxDataRate := 0.U // speed4
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

    // ========================================================================
    // Protocol-layer software control. `requestActive` is the new ctrl bit; the
    // protocol layer, not the testbench, decides WHEN to present Active on the
    // FDI, because the adapter edge-detects nop->active and only while it is in
    // FDI_BRINGUP (AdapterSM.scala:281-286, blanket-cleared everywhere else by
    // AdapterSM.scala:226). A raw level held from reset consumes the edge before
    // the window opens and hangs both dies with a signature indistinguishable
    // from having no requestActive bit at all.
    // ========================================================================
    proto.ctrl.requestActive := io.protoCtrl(i)(ProtoCtrl.active)
    proto.ctrl.requestRetrain := io.protoCtrl(i)(ProtoCtrl.retrain)
    proto.ctrl.requestLinkReset := io.protoCtrl(i)(ProtoCtrl.linkReset)
    proto.ctrl.requestDisable := io.protoCtrl(i)(ProtoCtrl.disable)

    // ========================================================================
    // Chip-facing data path.
    // ========================================================================
    proto.mainbandTx.valid := io.txValid.map(_(i)).getOrElse(false.B)
    proto.mainbandTx.bits.data := io.txData.map(_(i)).getOrElse(0.U)
    proto.mainbandRx.ready := io.rxReady.map(_(i)).getOrElse(false.B)
    io.rxData.foreach(p => p(i) := proto.mainbandRx.bits.data)

    // ========================================================================
    // Observations.
    // ========================================================================
    io.ltState(i) := phy.status.ltState
    io.ltsmState(i) := phy.status.currentState
    io.rdiState(i) := phy.rdi.plStateSts
    io.fdiState(i) := adapter.fdi.plStateSts

    val f = Wire(Vec(DieFlag.width, Bool()))
    f.foreach(_ := false.B)

    f(DieFlag.rdiInbandPres) := phy.rdi.plInbandPres
    f(DieFlag.rdiLpReqActive) := adapter.rdi.lpStateReq === RDIStateReq.active
    f(DieFlag.rdiStallReq) := phy.rdi.plStallReq
    f(DieFlag.rdiStallAck) := adapter.rdi.lpStallAck
    f(DieFlag.rdiPlTrdy) := phy.rdi.plTrdy
    f(DieFlag.rdiPlValid) := phy.rdi.plValid
    f(DieFlag.rdiPlWakeAck) := phy.rdi.plWakeAck
    f(DieFlag.phyParamsVld) := phy.status.negotiatedPhyParamSettings.valid
    f(DieFlag.phyRecenter) := phy.rdi.plPhyInRecenter
    f(DieFlag.phyTrainError) := phy.rdi.plTrainError
    f(DieFlag.phyTimedout) := phy.status.trainingTimedout
    f(DieFlag.rdiPlError) := phy.rdi.plError

    f(DieFlag.sbParityErr) := phy.status.sideband.sbParityErrSeen
    f(DieFlag.sbRxQueuesFull) := phy.status.sideband.sbRxPriorityQueuesFullSeen
    f(DieFlag.sbDeserTimedout) := phy.status.sideband.sbDeserializerTimedoutSeen
    f(DieFlag.sbBadRouteUpper) := phy.status.sideband.sbInvalidRouteUpperSeen
    f(DieFlag.sbBadRouteCurr) := phy.status.sideband.sbInvalidRouteCurrSeen
    f(DieFlag.sbBadRouteLower) := phy.status.sideband.sbInvalidRouteLowerSeen
    f(DieFlag.sbUnhandledMsg) := phy.status.sideband.sbUnhandledCurrentLayerMsgSeen

    f(DieFlag.fdiInbandPres) := adapter.fdi.plInbandPres
    f(DieFlag.fdiRxActiveReq) := adapter.fdi.plRxActiveReq
    f(DieFlag.fdiRxActiveSts) := proto.fdi.lpRxActiveSts
    f(DieFlag.fdiProtocolVld) := adapter.fdi.plProtocolVld
    f(DieFlag.fdiStallReq) := adapter.fdi.plStallReq
    f(DieFlag.fdiStallAck) := proto.fdi.lpStallAck
    f(DieFlag.fdiLpReqActive) := proto.fdi.lpStateReq === FDIStateReq.active
    f(DieFlag.fdiPlTrdy) := adapter.fdi.plTrdy
    f(DieFlag.fdiPlValid) := adapter.fdi.plValid
    f(DieFlag.fdiLpValid) := proto.fdi.lpValid

    f(DieFlag.negotiatedProto) := proto.status.negotiatedProtocolValid
    f(DieFlag.protoStalled) := proto.status.stalled
    f(DieFlag.rxOverflow) := proto.status.rxOverflow
    f(DieFlag.chipTxReady) := proto.mainbandTx.ready
    f(DieFlag.chipRxValid) := proto.mainbandRx.valid

    // AdapterSM link-init probes. BoringUtils taps read the registers in place,
    // so no production RTL gains a port: firtool lowers these to `wire
    // <reg>_probe = <reg>;` aliases inside AdapterSM plus an absolute XMR from
    // here, and the emitted D2DAdapter port list is byte-identical with the
    // probes on and off. XMR targets are public signals, and public-signal scope
    // registration is the same VerilatedScope machinery blamed for the
    // time-zero segfault at LogPhyLoopbackHarness.scala:73-78 -- so
    // `exposeAdapterProbes = false` is a load-bearing escape hatch, not
    // decoration: turn it off and read the same registers from a waveform.
    //
    // CAUTION on advCapSent/advCapRcvd: AdapterSM.scala:220-221 clears both
    // unconditionally at the top of the `linkStateReg === reset` block and only
    // the PARAM_EXCH arm self-holds them (:242-250). They are a LIVE VIEW of
    // PARAM_EXCH, not a record that PARAM_EXCH happened, so they read false once
    // the adapter is in FDI_BRINGUP. Do not conclude from advSnt/advRcv being
    // clear at FDI_BRINGUP that ADV_CAP never crossed.
    if (exposeAdapterProbes) {
      val sm = adapters(i).linkManager
      io.adapterLinkInit(i) := BoringUtils.tapAndRead(sm.linkInitStateReg)
      f(DieFlag.advCapSent) := BoringUtils.tapAndRead(sm.paramExchSbMsgSntFlag)
      f(DieFlag.advCapRcvd) := BoringUtils.tapAndRead(sm.paramExchSbMsgRcvFlag)
      f(DieFlag.actReqSent) := BoringUtils.tapAndRead(sm.activeSbMsgExtReqReg)
      f(DieFlag.actReqRcvd) := BoringUtils.tapAndRead(sm.activeSbMsgReqRcvFlag)
      f(DieFlag.actRspSent) := BoringUtils.tapAndRead(sm.activeSbMsgExtRspReg)
      f(DieFlag.actRspRcvd) := BoringUtils.tapAndRead(sm.activeSbMsgRspRcvFlag)
      f(DieFlag.transitionToActive) := BoringUtils.tapAndRead(sm.transitionToActiveReg)
    } else {
      io.adapterLinkInit(i) := LinkInitState.INIT_START
    }

    io.flags(i) := f.asUInt
  }
}
