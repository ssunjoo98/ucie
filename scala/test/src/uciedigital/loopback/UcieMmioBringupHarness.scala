package edu.berkeley.cs.uciedigital.loopback

import chisel3._
import chisel3.util.experimental.BoringUtils
import edu.berkeley.cs.chippy.{TLTester, TLTesterIO, TLTesterParams}
import edu.berkeley.cs.uciedigital.d2dadapter.LinkInitState
import edu.berkeley.cs.uciedigital.interfaces._
import edu.berkeley.cs.uciedigital.logphy.{LTSMState, LTState}
import edu.berkeley.cs.uciedigital.top.{UcieDigitalTop, UcieDigitalTopParams}
import freechips.rocketchip.tilelink._
import org.chipsalliance.cde.config.Parameters
import org.chipsalliance.diplomacy.lazymodule._

/** Bit positions inside `io.flags` (one word per die); packed for the same reason as
  * UcieDigitalLoopbackHarness.scala:37-49 -- one port instead of a dozen. */
object MmioFlag {
  val rdiPlWakeAck = 0
  val rdiInbandPres = 1
  val rdiLpReqActive = 2
  val fdiInbandPres = 3
  val fdiProtocolVld = 4
  val fdiLpReqActive = 5
  val fdiRxActiveReq = 6
  val fdiRxActiveSts = 7
  val negotiatedProto = 8
  val chipTxReady = 9
  val phyTrainError = 10
  val phyTimedout = 11
  val phyRecenter = 12
  val rdiPlError = 13
  val rxOverflow = 14
  val protoReqActive = 15 // ProtocolLayer.io.ctrl.requestActive, i.e. what the REGISTER holds

  // --- Chip/FDI data handshakes (M6, M7). One bit each; no datapath kept alive behind them.
  val chipRxValid = 16
  val fdiPlValid = 17 // MUST be a one-cycle pulse per delivered beat
  val fdiLpValid = 18
  val fdiPlTrdy = 19
  val fdiStallReq = 20
  val protoStalled = 21

  // --- The seven latched sideband fault bits (M8) ----------------------------
  val sbParityErr = 22
  val sbRxQueuesFull = 23
  val sbDeserTimedout = 24
  val sbBadRouteUpper = 25
  val sbBadRouteCurr = 26
  val sbBadRouteLower = 27
  val sbUnhandledMsg = 28

  // 64, not 33: matches DieFlag.width, which is proven under this Verilator, and leaves room.
  // The VerilatedScope time-zero segfault (LogPhyLoopbackHarness.scala:73-78) is about the NUMBER
  // of ports, not their width -- which is why both harnesses pack flags into one word.
  val width = 64

  /** The seven latched sideband fault bits, for the rung that triages them. Carried INDIVIDUALLY
    * like UcieDigitalLoopbackHarness.scala:104-113 and unlike LogPhyLoopbackHarness's single OR:
    * an OR only tells you to go open a waveform. sbFirstFaultValid is deliberately NOT here -- it
    * is set by the same seven terms (LogicalPhy.scala:224-244), so it adds no detection, and
    * sbFirstFaultOpcode/Header are captured payload that would make `flags.orR` lie. */
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
    rdiPlWakeAck -> "wakeAck",
    rdiInbandPres -> "rdiPres",
    rdiLpReqActive -> "rdiReqAct",
    fdiInbandPres -> "fdiPres",
    fdiProtocolVld -> "protoVld",
    fdiLpReqActive -> "fdiReqAct",
    fdiRxActiveReq -> "rxActReq",
    fdiRxActiveSts -> "rxActSts",
    negotiatedProto -> "negProto",
    chipTxReady -> "txRdy",
    phyTrainError -> "trainErr",
    phyTimedout -> "timedout",
    phyRecenter -> "recenter",
    rdiPlError -> "plError",
    rxOverflow -> "rxOvf",
    protoReqActive -> "reqActive",
    chipRxValid -> "rxVld",
    fdiPlValid -> "fdiVld",
    fdiLpValid -> "fdiLpVld",
    fdiPlTrdy -> "fdiTrdy",
    fdiStallReq -> "fdiStallReq",
    protoStalled -> "stalled",
  ) ++ sbFaults
}

object UcieMmioBringupHarness {
  /** UcieRegTop's node is 4 bytes wide (UcieRegTop.scala:18); a mismatched TLTester dataWidth
    * would turn every access into a multi-beat burst the tester never completes. */
  val beatBytes = 4
  val tlParams = TLTesterParams(addrWidth = 32, dataWidth = 32)

  /** includeRegNode = true so each die owns a real TLRegisterNode, includeInterruptNode = FALSE
    * because an unbound IntSourceNode fails diplomacy elaboration (UcieRegTop.scala:54). Both
    * dies sit at base 0: each has its own tester and therefore its own address space. */
  def topParams(): UcieDigitalTopParams = {
    val d = UcieDigitalTopParams.default()
    d.copy(regs = d.regs.copy(
      baseAddress = 0,
      includeRegNode = true,
      includeInterruptNode = false,
    ))
  }
}

/**
  * Two-die UcieDigitalTop loopback harness, controlled by MMIO ONLY.
  *
  * The sibling harness (UcieDigitalLoopbackHarness) instantiates the three layers by hand and
  * drives ProtocolLayer.io.ctrl / LogicalPhy.io.ctrl from testbench ports. This one instantiates
  * the SHIPPING top twice and gives the testbench exactly one control surface per die: a
  * TileLink master on that die's register node. Nothing here pokes a layer control bundle.
  *
  * The analog crossover is copied verbatim from UcieDigitalLoopbackHarness.scala:303-315, which
  * is proven by the S0..S8 and U0..U10 ladders: zero relative skew between sideband bits and
  * fwClock, straight mainband lanes so MBINIT.REVERSALMB resolves to "no reversal", and both
  * analog status bits tied high because no hardware source for them exists (D-19). Those two
  * bits are now top-level PINS (UcieDigitalTop.scala:29-35) rather than DontCare, which is what
  * makes bring-up at this top possible at all.
  *
  * Observation is by BoringUtils tap, never by new production ports -- the same call
  * UcieDigitalLoopbackHarness.scala:448-457 made. Control by MMIO, observation by taps, and (when
  * exposeDataPath is set) payload by testbench port at io.chipFacingIo, which is data, not control.
  */
class UcieMmioBringupHarness(val exposeDataPath: Boolean = false)(implicit p: Parameters)
    extends LazyModule {
  val tops = Seq.fill(2)(LazyModule(new UcieDigitalTop(UcieMmioBringupHarness.topParams())))
  val testers = Seq.fill(2)(LazyModule(
    new TLTester(UcieMmioBringupHarness.tlParams, UcieMmioBringupHarness.beatBytes)))
  (tops zip testers).foreach { case (top, tester) => top.regNode.get := tester.node }

  override lazy val module = new UcieMmioBringupHarnessImp(this)
}

class UcieMmioBringupHarnessImp(outer: UcieMmioBringupHarness) extends LazyModuleImp(outer) {
  private val tp = UcieMmioBringupHarness.topParams()
  // Nothing in UcieDigitalTopParams.validate() relates the FDI word to the AFE geometry, so state
  // the ladder's own assumption: one FDI word must be exactly one mainband beat, or the byte-exact
  // rungs stop meaning what they say (MainbandLaneController.scala:83-95 would split a word).
  require(tp.protocol.fdi.nBytes * 8 == tp.logPhy.afe.mbLanes * tp.logPhy.afe.mbSerializerRatio,
    s"ladder assumes 1 FDI word == 1 mainband beat: ${tp.protocol.fdi.nBytes * 8} bits vs " +
      s"${tp.logPhy.afe.mbLanes} lanes x ${tp.logPhy.afe.mbSerializerRatio}")

  val beatBits: Int = tp.protocol.fdi.nBytes * 8

  val io = IO(new Bundle {
    /** The ONLY drive surface: one TileLink master per die. */
    val reg = Vec(2, new TLTesterIO(UcieMmioBringupHarness.tlParams))

    val ltState = Output(Vec(2, LTState()))
    val ltsmState = Output(Vec(2, LTSMState()))
    val rdiState = Output(Vec(2, RDIState()))
    val fdiState = Output(Vec(2, FDIState()))
    val adapterLinkInit = Output(Vec(2, LinkInitState()))
    val flags = Output(Vec(2, UInt(MmioFlag.width.W)))

    // ---- Chip-facing data path (M6, M7) -------------------------------------
    // DATA, not control: the ladder's MMIO-only rule is about io.ctrl, and driving payload from
    // the testbench is exactly what U8/U9 do (UcieDigitalLoopbackHarness.scala:378-381).
    //
    // Behind `exposeDataPath` because being a port of the shipping top does NOT keep the datapath
    // alive: UcieDigitalTop is not the simulation's top, this harness is, and with identical
    // constants into both instances and mainbandRx.bits observed nowhere, firtool constant-folds
    // the TX pack and DCEs the RX unpack. The same fold already crosses a module boundary with two
    // instances one level up -- measured 7.1x on 1M idle cycles
    // (UcieDigitalLoopbackHarness.scala:268-278) and 6x one level down (FIX_LOG.md:561-564). The
    // M ladder gets that fold for free today; gating it keeps M0..M5 at their current cost and
    // confines the 7x to the two rungs that actually move beats.
    val txValid = Option.when(outer.exposeDataPath)(Input(Vec(2, Bool())))
    val txData = Option.when(outer.exposeDataPath)(Input(Vec(2, UInt(beatBits.W))))
    val rxReady = Option.when(outer.exposeDataPath)(Input(Vec(2, Bool())))
    val rxData = Option.when(outer.exposeDataPath)(Output(Vec(2, UInt(beatBits.W))))
  })

  for (i <- 0 until 2) {
    val me = outer.tops(i).module
    val peer = outer.tops(1 - i).module

    io.reg(i) <> outer.testers(i).module.io

    // ========================================================================
    // Analog crossover -- UcieDigitalLoopbackHarness.scala:303-315, verbatim.
    // ========================================================================
    me.io.phyFacingIo.sidebandLink.in.bits := peer.io.phyFacingIo.sidebandLink.out.bits
    me.io.phyFacingIo.sidebandLink.in.fwClock := peer.io.phyFacingIo.sidebandLink.out.fwClock

    me.io.phyFacingIo.mainbandLink.rx.bits := peer.io.phyFacingIo.mainbandLink.tx.bits
    me.io.phyFacingIo.mainbandLink.rx.valid := peer.io.phyFacingIo.mainbandLink.tx.valid
    me.io.phyFacingIo.mainbandLink.tx.ready := peer.io.phyFacingIo.mainbandLink.rx.ready

    me.io.phyFacingIo.status.pllLock := true.B
    me.io.phyFacingIo.status.clocksUngatedAndStable := true.B

    // Chip-facing data path. Quiet constants when exposeDataPath = false, which is what lets the
    // whole 512-bit chain fold away on the rungs that never move a beat.
    me.io.chipFacingIo.mainbandTx.valid := io.txValid.map(_(i)).getOrElse(false.B)
    me.io.chipFacingIo.mainbandTx.bits.data := io.txData.map(_(i)).getOrElse(0.U)
    me.io.chipFacingIo.mainbandRx.ready := io.rxReady.map(_(i)).getOrElse(false.B)
    io.rxData.foreach(p => p(i) := me.io.chipFacingIo.mainbandRx.bits.data)

    // ========================================================================
    // Observation. Every source is an output port of a grandchild module or a register inside
    // one, so nothing here adds a port to production RTL.
    // ========================================================================
    io.ltState(i) := BoringUtils.tapAndRead(me.logicalPhy.io.status.ltState)
    io.ltsmState(i) := BoringUtils.tapAndRead(me.logicalPhy.io.status.currentState)
    io.rdiState(i) := BoringUtils.tapAndRead(me.logicalPhy.io.rdi.plStateSts)
    io.fdiState(i) := BoringUtils.tapAndRead(me.d2dAdapter.io.fdi.plStateSts)
    io.adapterLinkInit(i) := BoringUtils.tapAndRead(me.d2dAdapter.linkManager.linkInitStateReg)

    val f = Wire(Vec(MmioFlag.width, Bool()))
    f.foreach(_ := false.B)

    f(MmioFlag.rdiPlWakeAck) := BoringUtils.tapAndRead(me.logicalPhy.io.rdi.plWakeAck)
    f(MmioFlag.rdiInbandPres) := BoringUtils.tapAndRead(me.logicalPhy.io.rdi.plInbandPres)
    f(MmioFlag.rdiLpReqActive) :=
      BoringUtils.tapAndRead(me.d2dAdapter.io.rdi.lpStateReq) === RDIStateReq.active
    f(MmioFlag.fdiInbandPres) := BoringUtils.tapAndRead(me.d2dAdapter.io.fdi.plInbandPres)
    f(MmioFlag.fdiProtocolVld) := BoringUtils.tapAndRead(me.d2dAdapter.io.fdi.plProtocolVld)
    f(MmioFlag.fdiLpReqActive) :=
      BoringUtils.tapAndRead(me.protocolLayer.io.fdi.lpStateReq) === FDIStateReq.active
    f(MmioFlag.fdiRxActiveReq) := BoringUtils.tapAndRead(me.d2dAdapter.io.fdi.plRxActiveReq)
    f(MmioFlag.fdiRxActiveSts) := BoringUtils.tapAndRead(me.protocolLayer.io.fdi.lpRxActiveSts)
    f(MmioFlag.negotiatedProto) :=
      BoringUtils.tapAndRead(me.protocolLayer.io.status.negotiatedProtocolValid)
    f(MmioFlag.chipTxReady) := me.io.chipFacingIo.mainbandTx.ready
    f(MmioFlag.phyTrainError) := BoringUtils.tapAndRead(me.logicalPhy.io.rdi.plTrainError)
    f(MmioFlag.phyTimedout) := BoringUtils.tapAndRead(me.logicalPhy.io.status.trainingTimedout)
    f(MmioFlag.phyRecenter) := BoringUtils.tapAndRead(me.logicalPhy.io.rdi.plPhyInRecenter)
    f(MmioFlag.rdiPlError) := BoringUtils.tapAndRead(me.logicalPhy.io.rdi.plError)
    f(MmioFlag.rxOverflow) := BoringUtils.tapAndRead(me.protocolLayer.io.status.rxOverflow)
    // The whole point of the exercise: this level is held by a REGISTER bit, not a poke.
    f(MmioFlag.protoReqActive) := BoringUtils.tapAndRead(me.protocolLayer.io.ctrl.requestActive)

    // Data handshakes. chipRxValid is a direct port read like chipTxReady; the rest are taps.
    f(MmioFlag.chipRxValid) := me.io.chipFacingIo.mainbandRx.valid
    f(MmioFlag.fdiPlValid) := BoringUtils.tapAndRead(me.d2dAdapter.io.fdi.plValid)
    f(MmioFlag.fdiLpValid) := BoringUtils.tapAndRead(me.protocolLayer.io.fdi.lpValid)
    f(MmioFlag.fdiPlTrdy) := BoringUtils.tapAndRead(me.d2dAdapter.io.fdi.plTrdy)
    f(MmioFlag.fdiStallReq) := BoringUtils.tapAndRead(me.d2dAdapter.io.fdi.plStallReq)
    f(MmioFlag.protoStalled) := BoringUtils.tapAndRead(me.protocolLayer.io.status.stalled)

    // The seven latched sideband fault bits. Sticky RegInits in LogicalPhy's own scope
    // (LogicalPhy.scala:166-172), outside the sbReset block, so they survive a retrain and one
    // read at the milestone captures everything latched on the way up.
    val sb = me.logicalPhy.io.status.sideband
    f(MmioFlag.sbParityErr) := BoringUtils.tapAndRead(sb.sbParityErrSeen)
    f(MmioFlag.sbRxQueuesFull) := BoringUtils.tapAndRead(sb.sbRxPriorityQueuesFullSeen)
    f(MmioFlag.sbDeserTimedout) := BoringUtils.tapAndRead(sb.sbDeserializerTimedoutSeen)
    f(MmioFlag.sbBadRouteUpper) := BoringUtils.tapAndRead(sb.sbInvalidRouteUpperSeen)
    f(MmioFlag.sbBadRouteCurr) := BoringUtils.tapAndRead(sb.sbInvalidRouteCurrSeen)
    f(MmioFlag.sbBadRouteLower) := BoringUtils.tapAndRead(sb.sbInvalidRouteLowerSeen)
    f(MmioFlag.sbUnhandledMsg) := BoringUtils.tapAndRead(sb.sbUnhandledCurrentLayerMsgSeen)

    io.flags(i) := f.asUInt
  }
}
