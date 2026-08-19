package edu.berkeley.cs.uciedigital.loopback

import chisel3._
import chisel3.simulator.HasSimulator
import chisel3.simulator.scalatest.ChiselSim
import edu.berkeley.cs.uciedigital.UcieSimBackend
import edu.berkeley.cs.uciedigital.d2dadapter.LinkInitState
import edu.berkeley.cs.uciedigital.interfaces._
import edu.berkeley.cs.uciedigital.logphy._
import edu.berkeley.cs.uciedigital.regs.{D2DAdapterOffsets, DvsecOffsets, PhyOffsets, UcieRegMap}
import org.chipsalliance.cde.config.Parameters
import org.chipsalliance.diplomacy.lazymodule._
import org.scalatest.funspec.AnyFunSpec

/*
  Run:
    rm -rf build/chiselsim/UcieMmioBringupTest   # stale svsim workdirs produce broken binaries
    ./mill --no-daemon test.testOnly edu.berkeley.cs.uciedigital.loopback.UcieMmioBringupTest
    # one rung only:
    ./mill --no-daemon test.testOnly edu.berkeley.cs.uciedigital.loopback.UcieMmioBringupTest -- -z "M2"

  Purpose:
    The SOFTWARE ladder. UcieDigitalStagedBringupTest (U0..U10) proves the stack works when the
    testbench pokes ProtocolLayer.io.ctrl and LogicalPhy.io.ctrl directly. This one proves the
    same bring-up happens through the SHIPPING top, driven by nothing but TileLink register
    writes: two UcieDigitalTop instances, one TL master each, no control poke anywhere.

  Relationship to the U ladder -- read before adding rungs:
    U0..U10 OWN the layer-level milestones. This ladder does NOT re-derive them; it asserts the
    subset that the register path is responsible for, and every rung above M2 exists only to say
    "the register-driven start reached the same place the poked start reaches". A rung red HERE
    and green THERE is a fact about the register wiring, not about the layers.

  The bring-up ladder (M0..M8), one describe block:
    M0  MMIO liveness        -> both register nodes answer reads and hold writes (no reset wait)
    M1  RDI wake handshake   -> pl_wake_ack on both dies, no training  (no reset wait)
    M2  one write trains     -> write Link Control b10 on die 0 ONLY; both dies reach sACTIVE
                                and RDI active, and software SEES link_training rise
    M3  ADV_CAP exchange     -> both dies raise FDI pl_inband_pres (still die 0's write only)
    M4  FDI Active           -> both dies pl_state_sts == active, and Link Status reads back up
    M5  chip interface opens -> mainbandTx.ready on both dies, link holds over a dwell
    M6  one beat each way    -> byte-exact protocol data over the register-trained link
    M7  bursts both ways     -> burstLength beats each way, simultaneously, in order
    M8  sideband stayed clean-> none of the seven latched fault bits set on the way up

  The contract rungs (C1..C4), their own describe block: they verify the software USAGE contract
  of the register interface -- what a boot ROM may rely on -- not the climb:
    C1  the early write      -> a start_link_training write issued before the RESET minimum is
                                DEFERRED to the end of it, not lost (measured; trap T0 corrected)
    C2  the deferred write   -> a die may be written AFTER it has trained; the drop window is
                                exactly "that die is training", not "any die has left RESET"
    C3  retrain_link is dead -> dropped while the link is down, latched at 1 forever while it is up
    C4  no teardown lever    -> cancelled by construction: there is no register that can drop,
                                retrain or disable a link that is up

  M4..M8 and C3 write BOTH dies (C2 writes the second one late), M2, M3 and C1 only die 0. That
  is not a testbench convenience: the FDI Active request has no register of its own and rides
  start_link_training's pending bit, so a die that was never written never asks for Active and
  INIT_DONE can never complete. It is the single strongest argument for giving the request its own
  bit -- see coldStart's note.

  Rung ORDER is load-bearing, because gateOnLowerRungs cancels everything ABOVE a failure and
  nothing below it. Genuine prerequisites go low; observational and documentary rungs go high.
  M8 (the fault surface) tops the bring-up group for the reason U10 sits at the top of the U
  ladder: a latched status bit blocks no milestone, so a red there cancels no climb rung -- it
  does gate the contract rungs, which prove nothing over a faulted link. The contract rungs run
  AFTER the whole climb and gate on it and on each other in declaration order: C1 and C2 are the
  write-timing contract, C3 the dead lever, and C4 always cancels, so it must be last or it
  would poison every rung above it.

  What is NOT here, deliberately:
    - MMIO as a data path. M6/M7 drive payload through TESTBENCH ports, exactly as U8/U9 do.
      io.chipFacingIo is data, not control, and must not become an MMIO path.
    - anything driven by target_link_speed. UcieDigitalTop ties maxDataRate to 0 on purpose --
      see the comment at UcieDigitalTop.scala and the note in trap T1 below.

  Cost: M0 and M1 need no reset wait. M2..M8 and C1..C3 each pay the real 3.2M-cycle RESET
  minimum wait (LinkTrainingSM.scala:108-116) from a cold start. M6 and M7 additionally pay for
  the 512-bit chip datapath being LIVE -- they are the only two rungs that pass exposeDataPath,
  and the fold is worth 7.1x on 1M idle cycles (UcieDigitalLoopbackHarness.scala:268-278), i.e.
  roughly a minute each. Rungs above a blocker CANCEL before they elaborate.
*/
class UcieMmioBringupTest extends AnyFunSpec with ChiselSim {

  private implicit val p: Parameters = Parameters.empty

  // Hardware constants (LinkTrainingSM.scala:87-116), not scalable from outside.
  private val hwTimeoutCycles = 6400000
  private val hwResetWait = hwTimeoutCycles / 2

  // Mandatory, for the same reasons as UcieDigitalStagedBringupTest.scala:105-106: the SBINIT
  // pattern flood trips SidebandDeserializerDoesNotDropWords, and the layer-stripped build then
  // needs Verilator's fast x-initial or it segfaults at time zero.
  private val noAssertFirtoolOpts = Array(
    "--disable-layers=Verification,Verification.Assert,Verification.Assume,Verification.Cover")

  implicit private val ucieSimulator: HasSimulator =
    if (sys.env.get("UCIE_SIM_BACKEND").contains("vcs")) UcieSimBackend.fromEnv
    else
      HasSimulator.simulators.verilator(
        svsim.CommonCompilationSettings(
          optimizationStyle =
            svsim.CommonCompilationSettings.OptimizationStyle.OptimizeForSimulationSpeed))

  private val debugTrace = false

  // ============================================================================
  // Guards. Every wait is bounded; a guard is only paid in full when a rung FAILS.
  // ============================================================================
  private val tlGuard = 64
  private val wakeGuard = 64
  private val phyFloorGuard = 2000000
  private val sbExchangeGuard = 400000
  private val flagGuard = 4096
  private val dwellCycles = 20000
  /** Chip-to-chip beat delivery: three pipeline stages plus the mainband beat. */
  private val dataGuard = 8192
  private val burstLength = 4

  /** MMIO reads spent waiting for a status bit. Each read is a handful of cycles, so this is a
    * few thousand cycles of headroom -- and it is bounded, like every other wait here. */
  private val mmioPollReads = 256

  // ============================================================================
  // The register map this ladder touches. Offsets are relative to the die's node base, which is
  // 0 here (UcieMmioBringupHarness.topParams). NOTE the production map is different: UcieTL
  // offsets the whole UCIe block by ucieTLRegionSize inside its own node (TileLink.scala:156).
  // ============================================================================
  private object Off {
    val ExtCapHeader = DvsecOffsets.ExtCapHeader // 0x00
    val LinkControl = DvsecOffsets.LinkControl // 0x10
    val LinkStatus = DvsecOffsets.LinkStatus // 0x14
    // The D2D-adapter and PHY halves share one page base (UcieRegParams.scala:121), and their
    // offset objects are already relative to it. Derived, never literal, so the UcieTL move only
    // has to change the addend.
    val D2dUncorrStatus = (UcieRegMap.DvsecPageSize + D2DAdapterOffsets.UncorrStatus).toInt // 0x1010
    val D2dCorrStatus = (UcieRegMap.DvsecPageSize + D2DAdapterOffsets.CorrStatus).toInt // 0x101c
    val PhyErrLog1 = (UcieRegMap.DvsecPageSize + PhyOffsets.ErrorLog1).toInt // 0x2090, module 0
  }

  /** Link Control reset word: raw_format_enable=0, target_link_width=x16 (enc 2, bits 5:2),
    * target_link_speed=24GT/s (enc 4, bits 9:6). UcieLinkDvsecRegs.scala:81-104. */
  private val linkCtrlReset = BigInt(0x108)
  private val startLinkTrainingBit = 10
  private val retrainLinkBit = 11
  private val rawFormatEnableBit = 0

  /** Link Status bit positions, UcieLinkDvsecRegs.scala:116-134. */
  private val stsRawFormatEnabled = 0
  private val stsLinkStatus = 15
  private val stsLinkTraining = 16
  /** The three error bits. Their only hardware set input is a literal 0 today -- see M8 part C. */
  private val stsCorrErr = 19
  private val stsUncorrNonFatal = 20
  private val stsUncorrFatal = 21

  private val extCapHeaderReset = BigInt(0x00010023) // id 0x0023, rev 1

  private def bit(word: BigInt, b: Int): Boolean = ((word >> b) & 1) == 1

  // ============================================================================
  // Decoding helpers. All `lazy`: building them touches ChiselEnum literals, and a strict val
  // would do that while ScalaTest is merely CONSTRUCTING the suite.
  // ============================================================================
  private lazy val ltNames: Map[BigInt, String] = Map(
    LTState.sRESET.litValue -> "sRESET",
    LTState.sSBINIT.litValue -> "sSBINIT",
    LTState.sMBINIT.litValue -> "sMBINIT",
    LTState.sMBTRAIN.litValue -> "sMBTRAIN",
    LTState.sLINKINIT.litValue -> "sLINKINIT",
    LTState.sACTIVE.litValue -> "sACTIVE",
    LTState.sPHYRETRAIN.litValue -> "sPHYRETRAIN",
    LTState.sTRAINERROR.litValue -> "sTRAINERROR",
    LTState.sL1_L2.litValue -> "sL1_L2",
  )

  /** Forward progress rank. Off-path states have NO rank: they are a derailment, and every
    * milestone is "rank >= n" so it can never be satisfied by a derailed FSM. */
  private lazy val ltRank: Map[BigInt, Int] = Map(
    LTState.sRESET.litValue -> 0,
    LTState.sSBINIT.litValue -> 1,
    LTState.sMBINIT.litValue -> 2,
    LTState.sMBTRAIN.litValue -> 3,
    LTState.sLINKINIT.litValue -> 4,
    LTState.sACTIVE.litValue -> 5,
  )

  private lazy val rdiNames: Map[BigInt, String] = Map(
    RDIState.reset.litValue -> "reset",
    RDIState.active.litValue -> "active",
    RDIState.activePmNak.litValue -> "activePmNak",
    RDIState.l1.litValue -> "l1",
    RDIState.l2.litValue -> "l2",
    RDIState.linkReset.litValue -> "linkReset",
    RDIState.linkError.litValue -> "linkError",
    RDIState.retrain.litValue -> "retrain",
    RDIState.disabled.litValue -> "disabled",
  )

  private lazy val fdiNames: Map[BigInt, String] = Map(
    FDIState.reset.litValue -> "reset",
    FDIState.active.litValue -> "active",
    FDIState.activePmNak.litValue -> "activePmNak",
    FDIState.l1.litValue -> "l1",
    FDIState.l2.litValue -> "l2",
    FDIState.linkReset.litValue -> "linkReset",
    FDIState.linkError.litValue -> "linkError",
    FDIState.retrain.litValue -> "retrain",
    FDIState.disabled.litValue -> "disabled",
  )

  private lazy val linkInitNames: Map[BigInt, String] = Map(
    LinkInitState.INIT_START.litValue -> "INIT_START",
    LinkInitState.RDI_BRINGUP.litValue -> "RDI_BRINGUP",
    LinkInitState.PARAM_EXCH.litValue -> "PARAM_EXCH",
    LinkInitState.FDI_BRINGUP.litValue -> "FDI_BRINGUP",
    LinkInitState.INIT_DONE.litValue -> "INIT_DONE",
  )

  /** FDI states that mean teardown, not bring-up. `retrain` is in the set ON PURPOSE: AdapterSM
    * has no retrain -> active exit (AdapterSM.scala:512-519), so entering it is terminal here. */
  private lazy val fdiTeardown: Set[BigInt] = Set(
    FDIState.linkError.litValue,
    FDIState.disabled.litValue,
    FDIState.linkReset.litValue,
    FDIState.retrain.litValue,
  )

  private type H = UcieMmioBringupHarnessImp

  private def ltCode(h: H, i: Int): BigInt = h.io.ltState(i).peek().litValue
  private def ltsmCode(h: H, i: Int): BigInt = h.io.ltsmState(i).peek().litValue
  private def rdiCode(h: H, i: Int): BigInt = h.io.rdiState(i).peek().litValue
  private def fdiCode(h: H, i: Int): BigInt = h.io.fdiState(i).peek().litValue
  private def linkInitCode(h: H, i: Int): BigInt = h.io.adapterLinkInit(i).peek().litValue

  private def flag(h: H, i: Int, b: Int): Boolean =
    ((h.io.flags(i).peek().litValue >> b) & 1) == 1

  private def rankOf(h: H, i: Int): Int = ltRank.getOrElse(ltCode(h, i), -1)
  private def bothDies(cond: Int => Boolean): Boolean = cond(0) && cond(1)
  private def rdiIs(h: H, i: Int, s: RDIState.Type): Boolean = rdiCode(h, i) == s.litValue
  private def fdiIs(h: H, i: Int, s: FDIState.Type): Boolean = fdiCode(h, i) == s.litValue

  private def derailed(h: H): Boolean =
    (0 until 2).exists(i => rankOf(h, i) < 0 || fdiTeardown.contains(fdiCode(h, i)))

  private def name(table: Map[BigInt, String], code: BigInt): String =
    table.getOrElse(code, s"?0x${code.toString(16)}")

  private def setFlags(h: H, i: Int): String = {
    val w = h.io.flags(i).peek().litValue
    val on = MmioFlag.names.collect { case (b, n) if ((w >> b) & 1) == 1 => n }
    if (on.isEmpty) "-" else on.mkString(",")
  }

  private def dieSummary(h: H, i: Int): String =
    s"die$i{lt=${name(ltNames, ltCode(h, i))} " +
      s"ltsm=0x${ltsmCode(h, i).toString(16)} " +
      s"rdi=${name(rdiNames, rdiCode(h, i))} " +
      s"fdi=${name(fdiNames, fdiCode(h, i))} " +
      s"linkInit=${name(linkInitNames, linkInitCode(h, i))} " +
      s"[${setFlags(h, i)}]}"

  private def stateSummary(h: H): String = s"${dieSummary(h, 0)} | ${dieSummary(h, 1)}"

  private def traceStates(h: H, tag: String): Unit =
    if (debugTrace) println(s"[mmio] $tag :: ${stateSummary(h)}")

  // ============================================================================
  // MMIO. The ONLY stimulus this ladder has.
  // ============================================================================

  /** Park the bus between accesses: no request outstanding, response side always accepting.
    *
    * resp.ready stays HIGH on purpose -- the tester drives TL d.ready from it and RegMapper only
    * raises a.ready when the response side can accept, so dropping it wedges the bus rather than
    * idling it. req.valid stays LOW on purpose: it is what stops a single write from being
    * re-issued on every cycle of the 3.2M-cycle reset wait. */
  private def idleBus(h: H, die: Int): Unit = {
    h.io.reg(die).req.valid.poke(false.B)
    h.io.reg(die).resp.ready.poke(true.B)
  }

  /** One TileLink access, exactly one beat.
    *
    * The register map is built with concurrency = 0, so RegMapper wires its response side
    * COMBINATIONALLY to its request side (RegMapper.scala:69-75: with depth 0, `back` IS
    * `front`). Request and response therefore fire on the SAME clock edge, and the data must be
    * peeked BEFORE that edge, not after it.
    *
    * Deliberately not chippy's TLTesterIO.write/read: those hold req.valid true across their
    * whole wait, so one call can issue several identical A beats, and they never drain the extra
    * D beats -- the next read then returns the PREVIOUS access's data. Measured on this harness:
    * reading ExtCapHeader then LinkControl with TLTesterIO.read returns 0x00010023 twice. */
  private def tlAccess(h: H, die: Int, off: Int, isWrite: Boolean, data: BigInt): BigInt = {
    val r = h.io.reg(die)
    r.resp.ready.poke(true.B)
    r.req.bits.addr.poke(off.U(32.W))
    r.req.bits.data.poke(data.U(32.W))
    r.req.bits.is_write.poke(isWrite.B)
    r.req.valid.poke(true.B)

    var n = 0
    def handshaking: Boolean =
      r.req.ready.peek().litToBoolean && r.resp.valid.peek().litToBoolean
    while (!handshaking && n < tlGuard) { h.clock.step(1); n += 1 }
    assert(handshaking,
      s"die $die: TileLink access to 0x${off.toHexString} did not handshake within $tlGuard " +
        s"cycles (req.ready=${r.req.ready.peek().litToBoolean}, " +
        s"resp.valid=${r.resp.valid.peek().litToBoolean}). M0 gates this; if M0 passed, the bus " +
        s"wedged mid-run.")

    val v = r.resp.bits.data.peek().litValue
    h.clock.step(1) // both beats fire on this edge
    idleBus(h, die)
    v
  }

  private def regWrite(h: H, die: Int, off: Int, data: BigInt): Unit = {
    tlAccess(h, die, off, isWrite = true, data)
  }

  private def regRead(h: H, die: Int, off: Int): BigInt =
    tlAccess(h, die, off, isWrite = false, 0)

  /** Poll a status register over MMIO until `cond`, or give up after `mmioPollReads` reads.
    * Returns the last word read so the failure message can print it. */
  private def readUntil(h: H, die: Int, off: Int)(cond: BigInt => Boolean): (Boolean, BigInt) = {
    var n = 0
    var w = regRead(h, die, off)
    while (!cond(w) && n < mmioPollReads) { w = regRead(h, die, off); n += 1 }
    (cond(w), w)
  }

  private def initHarness(h: H): Unit = for (i <- 0 until 2) {
    idleBus(h, i)
    // Present only when exposeDataPath; parking them keeps the 3.2M-cycle wait un-poked.
    h.io.txValid.foreach(_(i).poke(false.B))
    h.io.txData.foreach(_(i).poke(0.U))
    h.io.rxReady.foreach(_(i).poke(false.B))
  }

  /** The 3.2M-cycle wait itself, chunked so a hung simulator is diagnosable from the log. Split
    * out because C1 must run it with a start_link_training write already pending, which is exactly
    * what runResetWait's pre-checks forbid. */
  private def stepResetWait(h: H): Unit =
    for (chunk <- 1 to 4) {
      h.clock.step(hwResetWait / 4)
      if (debugTrace) println(s"[mmio] reset wait chunk $chunk/4 done")
    }

  /** Sit out the hardware RESET minimum wait (3.2M cycles) plus slack, checking RESET-residency
    * invariants before and after. */
  private def runResetWait(h: H): Unit = {
    h.clock.step(64)
    for (i <- 0 until 2) {
      h.io.ltsmState(i).expect(LTSMState.sRESET, "LTSM must start in RESET")
      h.io.rdiState(i).expect(RDIState.reset, "RDI must start in reset")
      h.io.fdiState(i).expect(FDIState.reset, "FDI must start in reset")
      assert(!flag(h, i, MmioFlag.protoReqActive),
        s"die $i: the protocol layer is being asked for Active before any register was written. " +
          s"requestActive comes from start_link_training's pending bit " +
          s"(UcieDigitalTop.scala, RegsToLink.startTrainingPending), so this means the bit is " +
          s"set out of reset -- check RegFieldTypes.RWautoClear's reset value. " +
          s"Observed: ${stateSummary(h)}")
    }
    stepResetWait(h)
    h.clock.step(64)
    for (i <- 0 until 2) {
      h.io.ltsmState(i).expect(LTSMState.sRESET, "no training without a register write")
    }
  }

  /** The whole software driver: wait out the RESET minimum, then set Link Control b10 on each
    * die in `dies`, in order.
    *
    * TRAP T0 IS NOT WHAT THIS FILE USED TO SAY, and C1 measured it. start_link_training is a
    * one-cycle fire against an edge detector (LinkTrainingSM.scala:238-246) whose STATE TRANSITION
    * is gated on `resetMinWait` (:1141) -- but the episode latch at :1017-1021 is not, so an early
    * write sets trainingEpisodeActive, autoRetrain (:1013-1015) holds the exit condition true, and
    * the die trains the instant the 3.2M-cycle wait expires. An early write is DEFERRED, not
    * burned. Writing after the wait, as below, is still the right order -- it is deterministic and
    * it is what a boot ROM should do -- but mistiming it is not fatal.
    *
    * WHICH DIES TO WRITE, and why it is not always just die 0:
    *   - `Seq(0)` reproduces UcieDigitalStagedBringupTest.scala:340-354's asymmetric start: die 1
    *     is never written and must wake on the SBINIT clock pattern die 0 transmits
    *     (LinkTrainingSM.scala:1128-1140). That is the arrival order real chiplets have and the
    *     one a simultaneous start hides. M2 and M3 use it.
    *   - `Seq(0, 1)` is required from M4 up, and this is a REAL LIMITATION of the current
    *     register set, not a testbench convenience. The FDI Active request has no register of
    *     its own; UcieDigitalTop derives ProtocolLayer.io.ctrl.requestActive from
    *     start_link_training's PENDING bit, so a die whose bit was never set never asks for
    *     Active and INIT_DONE -- which needs REQ_ACTIVE both ways (AdapterSM.scala:288-290) --
    *     can never complete. Measured: with `Seq(0)` both dies reach sACTIVE, RDI active and
    *     FDI_BRINGUP, then die 0 sits with fdiReqAct set and die 1 with rxActReq/rxActSts set and
    *     no reqActive of its own, forever, with no timeout and no error bit.
    *     The drop window is narrower than it looks, and C2 measures it: RWautoClear takes
    *     `ignoreWhen = linkIn.linkTraining` (UcieLinkDvsecRegs.scala:87), which is phyTraining,
    *     and sACTIVE is NOT in that set (UcieDigitalTop.scala:167-170). So a write to a die is
    *     dropped only while THAT die is training; a die parked in FDI_BRINGUP at sACTIVE can
    *     still be written. Writing both up front is simply the cheapest order here -- one TL
    *     access is a handful of cycles and the remote SBINIT trigger needs at least a 64-bit
    *     serial word, so both writes land before either die leaves RESET.
    */
  private def coldStart(h: H, dies: Seq[Int] = Seq(0)): Unit = {
    initHarness(h)
    runResetWait(h)
    dies.foreach(d =>
      regWrite(h, d, Off.LinkControl, linkCtrlReset | (BigInt(1) << startLinkTrainingBit)))
  }

  private case class WaitResult(ok: Boolean, cycles: Int, derailed: Boolean)

  private def stepUntil(h: H, guard: Int, stride: Int)(cond: => Boolean): WaitResult = {
    var n = 0
    var off = false
    while (!cond && !off && n < guard) {
      h.clock.step(stride)
      n += stride
      off = derailed(h)
    }
    WaitResult(cond, n, off && !cond)
  }

  // ============================================================================
  // The ladder
  // ============================================================================
  private case class Stage(
      id: String,
      what: String,
      guard: Int,
      stride: Int,
      blocker: String,
      reached: H => Boolean,
  )

  private val ladder: Seq[Stage] = Seq(
    Stage(
      id = "M0",
      what = "both dies' TileLink register nodes accept a request out of reset",
      guard = tlGuard,
      stride = 1,
      blocker =
        "this is the diplomacy/plumbing rung and it costs nothing, so a failure here is a build " +
          "problem, not a UCIe problem. Three candidates, in order: (a) beatBytes mismatch -- " +
          "UcieRegTop's node is 4 bytes (UcieRegTop.scala:18) and TLTester issues " +
          "edge.Put(id, addr, lgBeatBytes, data), so a TLTesterParams.dataWidth other than 32 " +
          "turns every access into a multi-beat burst the tester never completes and a.ready " +
          "never rises; (b) the register domain has no clock -- UcieDigitalTop.scala:63-67 " +
          "drives regClockSource from the implicit clock, and if that binding is removed the " +
          "whole block sits in reset; (c) includeInterruptNode was left true, in which case " +
          "elaboration fails earlier with an unbound IntSourceNode (UcieRegTop.scala:54) and " +
          "this rung never runs at all",
      reached = (h: H) => bothDies(i => h.io.reg(i).req.ready.peek().litToBoolean),
    ),
    Stage(
      id = "M1",
      what = "the PHY acknowledges the adapter's RDI wake request (pl_wake_ack) on both dies, " +
        "straight out of reset and with no training",
      guard = wakeGuard,
      stride = 1,
      blocker =
        "U0 asserts exactly this on the layer harness and passes, so a failure HERE and not " +
          "there is about the top, not the wake responder. The one thing that differs is where " +
          "clocksUngatedAndStable comes from: the layer harness ties it high inside the harness " +
          "(UcieDigitalLoopbackHarness.scala:315), and this ladder ties the top's new " +
          "phyFacingIo.status pin high instead (UcieMmioBringupHarness). If that pin is left " +
          "DontCare -- which is what UcieDigitalTop did before tonight -- the responder parks in " +
          "sUNGATE forever (RDIWakeHandshakeResponder.scala:46-52), activeBringupReady stays " +
          "false, and M2 would fail 3.2M cycles later at rdi=reset with no hint why. THIS rung " +
          "is the cheap version of that question. It does NOT prove clock gating works: the " +
          "responder's ungateClocks output still has no field in PhyControlToPhyIO (D-41), so " +
          "the request side is a dangling wire while the answer side is a harness constant",
      reached = (h: H) => bothDies(i => flag(h, i, MmioFlag.rdiPlWakeAck)),
    ),
    Stage(
      id = "M2",
      what = "ONE register write (Link Control b10 on die 0) trains both dies: sACTIVE and RDI " +
        "pl_state_sts == active, with no control signal poked anywhere",
      guard = phyFloorGuard,
      stride = 64,
      blocker =
        "FIRST run UcieDigitalStagedBringupTest U1. If U1 is green and M2 is red, training is " +
          "fine and the defect is the register path, which is three links long: " +
          "(a) the WRITE reached the block -- M0 already proved the bus works, and M2's body " +
          "re-reads Link Status to prove software can see link_training rise, so if that read " +
          "shows link_training=0 the fire pulse never left the register block: check " +
          "RWautoClear's ignoreWhen, which is linkIn.linkTraining (UcieLinkDvsecRegs.scala:87) " +
          "and would swallow the write if linkTraining were stuck high; " +
          "(b) the FIRE reached the LTSM -- UcieDigitalTop drives swStartLinkTraining from " +
          "regsToLink.startTraining, a ONE-CYCLE pulse, into an edge detector that is only " +
          "sampled while pwrGood && pllLock && resetMinWait all hold (LinkTrainingSM.scala:1141). " +
          "pwrGood is tied true at the top (no register, no pin -- D-19) and pllLock now comes " +
          "from phyFacingIo.status, which M1 proved is high, so the remaining suspect is write " +
          "ORDER: a write before the 3.2M-cycle reset wait is silently lost (trap T0); " +
          "(c) die 1 never woke -- die 1 is NOT written, by design. It must wake on die 0's " +
          "SBINIT clock pattern (LinkTrainingSM.scala:1128-1140). die0 at sACTIVE with die1 at " +
          "sRESET is that path failing, and it is a LogPhy-level fact, not a register fact",
      reached = (h: H) => bothDies(i => rankOf(h, i) >= 5 && rdiIs(h, i, RDIState.active)),
    ),
    Stage(
      id = "M3",
      what = "the adapters complete the ADV_CAP exchange and both raise FDI pl_inband_pres",
      guard = sbExchangeGuard,
      stride = 8,
      blocker =
        "identical to U2's territory -- the AdapterSM ADV_CAP receive-flag race, whose tolerance " +
          "D2DAdapterBringupTest.scala:193-236 measured at 6 cycles of inter-die skew. NOTHING " +
          "on this rung is register-driven: the adapters run on their own once the RDI is " +
          "active. If U2 passes and M3 fails, the difference is the skew between the two dies' " +
          "RDI-active reports, which here is produced by a register write on die 0 plus die 1's " +
          "remote wake instead of a poke, so read U2's measured skew first. Signature: " +
          "linkInit=PARAM_EXCH on one die and FDI_BRINGUP on the other",
      reached = (h: H) => bothDies(i => flag(h, i, MmioFlag.fdiInbandPres)),
    ),
    Stage(
      id = "M4",
      what = "both dies reach FDI pl_state_sts == active, entirely as a consequence of the one " +
        "register write, and Link Status reads back link_status = 1",
      guard = sbExchangeGuard,
      stride = 8,
      blocker =
        "this is the rung that tests the HELD request. The FDI Active choreography needs " +
          "ProtocolLayer.io.ctrl.requestActive to be a LEVEL across ADV_CAP -> REQ_ACTIVE -> " +
          "RSP_ACTIVE, and UcieDigitalTop derives that level from start_link_training's PENDING " +
          "bit (RegsToLink.startTrainingPending), not from its one-cycle fire. FIRST check that " +
          "BOTH dies were written -- requestActive has no register of its own, so a die whose " +
          "start_link_training was never set never asks for Active, and the signature is exactly " +
          "reqActive+fdiReqAct on one die and rxActReq+rxActSts on the other, both parked in " +
          "FDI_BRINGUP forever (see coldStart's note). Then, two ways it breaks: " +
          "(a) the pending bit cleared too early -- it is cleared by " +
          "linkIn.trainingDone (RegFieldTypes.scala:110), which UcieDigitalTop deliberately " +
          "defines as FDI-active rather than LTSM-sACTIVE for exactly this reason; if someone " +
          "redefines trainingDone as sACTIVE, requestActive drops at M2 and both dies park in " +
          "FDI_BRINGUP forever with no timeout and no error bit. The reqActive flag in the " +
          "summary is that level, read straight off ProtocolLayer.io.ctrl -- check it first. " +
          "(b) the level was there and the EDGE was not: the adapter edge-detects nop->active " +
          "and only while in FDI_BRINGUP (AdapterSM.scala:281-286), so fdiReqAct set on both " +
          "dies with the FDI still in reset means the window moved. If reqActive is CLEAR, the " +
          "register bit is the fault, not the layers -- read Link Control b10 back",
      reached = (h: H) => bothDies(i => fdiIs(h, i, FDIState.active)),
    ),
    Stage(
      id = "M5",
      what = "the chip-facing TX interface opens on both dies (mainbandTx.ready) and the link " +
        "holds over a dwell",
      guard = flagGuard,
      stride = 8,
      blocker =
        "same predicate as U7, and U7's blocker applies verbatim: mainbandTx.ready is " +
          "queue.enq.ready && !(stallRequested || !active) (ProtocolMainbandTx.scala:34-35), so " +
          "with M4 green the only way this stays low is a held pl_stallreq. Nothing on this rung " +
          "is register-driven. If U7 passes and M5 does not, suspect the one thing this top does " +
          "that the layer harness does not: it drives maxErrorThresholdPerLane and the whole " +
          "linkTrainingParameters bundle from the Training Setup registers " +
          "(UcieDigitalTop.scala) instead of tying them to zero. Their reset values are " +
          "burst=4/idle=4/iters=4 (LogPhyRegsPerModule.scala) where the layer harness pokes 0, " +
          "which is inert only because the LTSM DontCare-overrides those requester interfaces " +
          "(LinkTrainingSM.scala:460,:499,:538,:578) -- if that stops being true, this is where " +
          "it shows up",
      reached = (h: H) => bothDies(i => flag(h, i, MmioFlag.chipTxReady)),
    ),
    Stage(
      id = "M6",
      what = "the receive path is armed on both dies (chip TX open, FDI rx_active_sts held) -- " +
        "the beat exchange itself is asserted in the rung BODY",
      guard = flagGuard,
      stride = 8,
      blocker =
        "identical gate to U8, and U8 passes, so a red HERE with U8 green is the register path or " +
          "the elaboration, not the layers -- start by checking that exposeDataPath actually " +
          "reached the harness (io.txValid is None otherwise and exchange throws on .get). Two " +
          "live RTL hazards can hold this low with M5 green: a held pl_stallreq, which " +
          "ProtocolMainbandTx.scala:46-48 turns into a QUEUE FLUSH rather than backpressure, so " +
          "beats vanish instead of stalling; and MainbandLaneController.scala:246-250's " +
          "stickyError, which is never cleared once a valid-framing error is seen and permanently " +
          "holds RDIController's holdUpperLayerStall (RDIController.scala:75-85), wedging plTrdy. " +
          "rx_active_sts comes from ProtocolStateController and drops if rxReadyForActive drops " +
          "(ProtocolMainbandRx.scala:48), i.e. after an rxOverflow -- check the rxOvf flag",
      reached = (h: H) =>
        bothDies(i => flag(h, i, MmioFlag.chipTxReady) && flag(h, i, MmioFlag.fdiRxActiveSts)),
    ),
    Stage(
      id = "M7",
      what = "same gate as M6 -- the burst and simultaneous-transmit cases are asserted in the " +
        "rung BODY",
      guard = flagGuard,
      stride = 8,
      blocker =
        "a burst that loses or reorders beats while M6's single beat passes is FLOW CONTROL, not " +
          "framing: the protocol TX queue is 2 deep (ProtocolTypes.scala:14) and drains one beat " +
          "per cycle through the adapter's single 1-deep buffer (D2DMainbandModule.scala:99-121) " +
          "into a single-beat PHY transfer, with NO end-to-end credit anywhere on the mainband. " +
          "A FIRST beat that matches followed by a CORRUPTED second is scrambler/descrambler " +
          "lockstep (LogicalPhy.scala:302,:313), which is LogPhyStagedBringupTest S8's territory " +
          "one level down. A beat that is byte-exact but PERMUTED is lane reversal " +
          "(LogicalPhy.scala:419-421, no inverse on the RX unpack); it cannot assert in this " +
          "harness because lanes are cross-wired straight, so MBINIT.REVERSALMB resolves to " +
          "no-reversal",
      reached = (h: H) =>
        bothDies(i => flag(h, i, MmioFlag.chipTxReady) && flag(h, i, MmioFlag.fdiRxActiveSts)),
    ),
    Stage(
      id = "M8",
      what = "the D2D sideband carried the adapters' own link-init traffic without latching a " +
        "fault bit -- asserted in the rung BODY, per fault bit",
      guard = flagGuard,
      stride = 8,
      blocker =
        "PREDICTED GREEN, because U10 asserts the SAME seven bits over the SAME analog crossover " +
          "at the same milestone and passes -- UcieMmioBringupHarness.scala:124-132 copies " +
          "UcieDigitalLoopbackHarness.scala:303-315 verbatim. Deliberately the TOP rung of the " +
          "bring-up group so it cancels no milestone below it: a latched status bit stops no " +
          "milestone. U10's per-bit " +
          "triage applies verbatim. Two M-SPECIFIC suspects on top of it, and they are the reason " +
          "this rung exists at the MMIO level at all: (a) sbDeserTimedout alone, six clean, is " +
          "not a link fault -- the forwarded sideband clock STOPS between packets " +
          "(SidebandLinkSerdes.scala:86-90,:115), idleStatus can then never go true again " +
          "(:263), and the core-domain timer latches ~514 cycles after the last word " +
          "(:270-279, desTimeoutCycles = 512). FIX_LOG.md:503 already had to shorten a training " +
          "dwell for exactly this. (b) this top drives the whole linkTrainingParameters bundle " +
          "from the Training Setup registers (UcieDigitalTop.scala:141-153) whose reset values " +
          "are burst=4/idle=4/iters=4 (LogPhyRegsPerModule.scala:35,:42,:43) where the U harness " +
          "pokes 0. That is inert only while every requester interface re-defaults the bundle to " +
          "DontCare (LinkTrainingSM.scala:460,:499,:538,:578); the day it is not, a pattern dwell " +
          "grows and sbDeserTimedout is what reports it",
      reached = (h: H) => bothDies(i => flag(h, i, MmioFlag.chipTxReady)),
    ),
    Stage(
      id = "C1",
      what = "an early start_link_training write is DEFERRED to the end of the RESET minimum -- " +
        "C1 climbs only to M3 and asserts the rest in its BODY; this predicate exists so rungs " +
        "above it can climb THROUGH index 9",
      guard = flagGuard,
      stride = 8,
      blocker =
        "C1 is a MEASURED result, not a prediction: the first draft asserted the opposite (that " +
          "the die stays in sRESET) and the run returned sSBINIT. The trigger survives because " +
          "LinkTrainingSM.scala:1017-1021 latches trainingEpisodeActive on freshTrainingTrigger " +
          "WITHOUT the resetMinWait term that gates the state transition at :1141, and " +
          "autoRetrain (:1013-1015) then holds the exit condition true -- with retryAmtMax = " +
          "io.retryTrainingAmt = 0 (UcieDigitalTop.scala:121) the test is `0 <= 0`, which passes. " +
          "A red here means one of those three lines changed. If the `<=` becomes `<`, or the " +
          "episode latch gains a resetMinWait term, the early write really is lost and this rung " +
          "must be re-inverted -- and coldStart's note, which this rung already corrected once, " +
          "has to change back with it",
      reached = (h: H) => bothDies(i => flag(h, i, MmioFlag.chipTxReady)),
    ),
    Stage(
      id = "C2",
      what = "same gate as M4 -- the deferred second-die write is asserted in the rung BODY",
      guard = sbExchangeGuard,
      stride = 8,
      blocker =
        "the ONE rung here whose outcome was unknown when it was written -- and it came back " +
          "GREEN, so a red now is a regression, not an " +
          "open question. It measures that RWautoClear's ignoreWhen window is exactly \"THIS die " +
          "is training\" and not \"any die has left RESET\": linkTraining is phyTraining " +
          "(UcieDigitalTop.scala:167-170) and sACTIVE is NOT in that set, so a die parked in " +
          "FDI_BRINGUP at sACTIVE still has ignoreWhen=false, hwDone=false and pending=0, and a " +
          "late write is accepted. If this rung goes RED, the pessimistic claim at coldStart's " +
          "note stands -- both writes really must land before either die leaves RESET -- and the " +
          "note must be restored rather than the rung deleted, because that is a much harsher " +
          "constraint on a boot ROM and needs to be written down. Signature to check first: " +
          "read Link Control b10 back on die 1 after the deferred write. Reading 0 means the " +
          "write was swallowed, i.e. ignoreWhen was still true; reading 1 with no reqActive means " +
          "the pending bit is set but UcieDigitalTop.scala:158 is not deriving the level from it",
      reached = (h: H) => bothDies(i => fdiIs(h, i, FDIState.active)),
    ),
    Stage(
      id = "C3",
      what = "same gate as M5 -- retrain_link's two behaviours are asserted in the rung BODY",
      guard = flagGuard,
      stride = 8,
      blocker =
        "C3 predicts GREEN and pins a DEFECT: retrain_link's fire reaches nothing. " +
          "regsToLink.retrain (UcieLinkDvsecRegs.scala:112) has no consumer in scala/src -- " +
          "UcieDigitalTop.scala:159 hard-ties protocolLayer.io.ctrl.requestRetrain to false, and " +
          "even wired it would change nothing because AdapterSM never compares fdi_lp_state_req " +
          "against retrain and has no retrain exit (AdapterSM.scala:512-519), which is also why " +
          "fdiTeardown treats FDI retrain as terminal. Meanwhile linkStatus.retrainDone is never " +
          "assigned (UcieDigitalTop.scala:174-184 zeroes the bundle and drives seven fields), so " +
          "retrain_link's hwDone is a constant 0 and its pending bit can never auto-clear. A red " +
          "here means one of those two facts changed: if requestRetrain got wired, this rung is " +
          "now testing a real path and needs rewriting; if retrainDone got a source, the " +
          "latch-forever half inverts",
      reached = (h: H) => bothDies(i => flag(h, i, MmioFlag.chipTxReady)),
    ),
    Stage(
      id = "C4",
      what = "there is no register that can drop, retrain, disable or reset a link that is up, " +
        "so teardown and second bring-up cannot be stimulated from this ladder at all",
      guard = flagGuard,
      stride = 8,
      blocker =
        "not a defect in the ladder: a missing hardware lever. This rung is CANCELLED by " +
          "construction and goes live the day a teardown register exists. Kept last because a " +
          "cancelled rung never enters stagePassed and would otherwise cancel everything above it",
      reached = (h: H) => bothDies(i => flag(h, i, MmioFlag.chipTxReady)),
    ),
  )

  private def stageFailure(h: H, stuckIdx: Int, targetIdx: Int, res: WaitResult): String = {
    val st = ladder(stuckIdx)
    val target = ladder(targetIdx)
    val why =
      if (res.derailed)
        s"gave up after ${res.cycles} cycles because a die derailed off the bring-up path (an " +
          "LT state with no forward rank, or an FDI teardown state -- linkError / disabled / " +
          "linkReset / retrain -- none of which can ever satisfy a forward milestone)"
      else
        s"not reached within the ${st.guard}-cycle guard (waited ${res.cycles}, polled every " +
          s"${st.stride})"
    val prereq =
      if (stuckIdx < targetIdx)
        s" This rung is a PREREQUISITE for ${target.id} (${target.what}), so ${target.id} cannot " +
          s"even be evaluated yet: the ladder is blocked lower down, at ${st.id}."
      else " This is the rung under test."
    s"[${st.id}] MISSED MILESTONE: ${st.what} -- $why.$prereq " +
      s"Observed at give-up: ${stateSummary(h)}. Known blocker: ${st.blocker}"
  }

  /** Walk the ladder up to and including `upTo`, failing at the LOWEST unmet rung. */
  private def climbTo(h: H, upTo: Int): Unit = {
    for (idx <- 0 to upTo) {
      val st = ladder(idx)
      val res = stepUntil(h, st.guard, st.stride)(st.reached(h))
      if (res.ok) traceStates(h, s"${st.id} reached after ${res.cycles} cycles")
      else traceStates(h, s"${st.id} GAVE UP after ${res.cycles} cycles")
      assert(res.ok, stageFailure(h, idx, upTo, res))
    }
  }

  // ============================================================================
  // Skip-ahead gate. Sequential, declaration-order execution (AnyFunSpec default, and
  // build.mill sets testParallelism = false). Do NOT add -P.
  // ============================================================================
  private val stageAttempted = scala.collection.mutable.Set.empty[Int]
  private val stagePassed = scala.collection.mutable.Set.empty[Int]

  private def gateOnLowerRungs(stage: Int): Unit = {
    val culprit = (0 until stage).find(s => stageAttempted(s) && !stagePassed(s))
    culprit.foreach { s =>
      cancel(
        s"${ladder(stage).id} NOT ATTEMPTED: ${ladder(s).id} ran earlier in this suite and did " +
          s"not pass, and ${ladder(s).id} (${ladder(s).what}) is a prerequisite -- " +
          s"${ladder(stage).id} could only reproduce the same failure. Read ${ladder(s).id}'s " +
          s"message above for the blocking defect and fix that first. Skipping here avoids a " +
          s"pointless $hwResetWait-cycle RESET wait plus a full verilate pass.")
    }
    stageAttempted += stage
  }

  private def harness(exposeDataPath: Boolean = false): UcieMmioBringupHarnessImp =
    LazyModule(new UcieMmioBringupHarness(exposeDataPath)).module

  // ============================================================================
  // Chip-level data traffic (M6, M7). Ported verbatim from
  // UcieDigitalStagedBringupTest.scala:744-850 -- same three layers, same chip edge, so every
  // diagnostic string in them is still accurate here. Only the flag enum differs.
  //
  // This is the DATA path, not the control path. The ladder's MMIO-only rule is about
  // ProtocolLayer.io.ctrl and LogicalPhy.io.ctrl; payload arrives at io.chipFacingIo, which is
  // where a protocol IP would drive it, and driving it from the testbench is what U8/U9 do.
  // ============================================================================

  /** Distinct, non-repeating payloads: a per-die tag, a sequence number and a walking pattern, so
    * a swapped beat, a stale beat and a lane permutation all fail differently. Every 16-bit word
    * is non-zero by construction, so an unwired link delivering zeros cannot pass. */
  private def payload(bits: Int, die: Int, seq: Int): BigInt =
    (0 until bits / 16).foldLeft(BigInt(0)) { (acc, i) =>
      val w = ((die + 1) << 12) | ((seq + 1) << 8) | ((i * 7 + die * 3 + seq) & 0xff)
      acc | (BigInt(w & 0xffff) << (i * 16))
    }

  /** Drive `words(i)` out of die i's chip-facing TX while draining BOTH dies' chip-facing RX, then
    * keep draining through `drainCycles` of quiet. Returns what each die received, in order.
    *
    * Draining concurrently is NOT optional: the FDI receive direction has no backpressure and the
    * protocol RX queue is 2 deep, so an undrained receiver drops beats and blames the link.
    * Draining through the quiet tail is what makes "exactly once" mean anything -- a sticky
    * pl_valid keeps delivering there.
    *
    * Handshake timing: mainbandTx.ready is combinational from queue.enq.ready and the active/stall
    * gates (ProtocolMainbandTx.scala:34-35), so valid+data are poked and ready is peeked in the
    * SAME cycle, before stepping. The RX queue output is exposed straight through
    * (ProtocolMainbandRx.scala:40), so valid/bits hold until ready pops them.
    *
    * Do NOT interleave MMIO here: tlAccess steps the clock internally, and a poked txValid would
    * be re-accepted on every one of those steps. Reads go before or after, never during. */
  private def exchange(h: H, words: Seq[Seq[BigInt]], drainCycles: Int = 64): Seq[Seq[BigInt]] = {
    val beatBits = h.beatBits
    val pending = words.map(_.to(scala.collection.mutable.ArrayBuffer))
    val got = Seq.fill(2)(scala.collection.mutable.ArrayBuffer.empty[BigInt])
    for (i <- 0 until 2) h.io.rxReady.get(i).poke(true.B)

    var quiet = 0
    var n = 0
    while ((pending.exists(_.nonEmpty) || quiet < drainCycles) && n < dataGuard) {
      for (i <- 0 until 2) {
        h.io.txValid.get(i).poke(pending(i).nonEmpty.B)
        if (pending(i).nonEmpty) h.io.txData.get(i).poke(pending(i).head.U(beatBits.W))
      }
      // Sample the handshakes completing on THIS edge, before stepping.
      val accepted = (0 until 2).map(i => pending(i).nonEmpty && flag(h, i, MmioFlag.chipTxReady))
      val delivered = (0 until 2).map(i =>
        Option.when(flag(h, i, MmioFlag.chipRxValid))(h.io.rxData.get(i).peek().litValue))

      h.clock.step(1)
      n += 1

      for (i <- 0 until 2) {
        if (accepted(i)) pending(i).remove(0)
        delivered(i).foreach(got(i) += _)
      }
      if (pending.forall(_.isEmpty)) quiet += 1
    }

    for (i <- 0 until 2) {
      h.io.txValid.get(i).poke(false.B)
      h.io.rxReady.get(i).poke(false.B)
    }
    assert(pending.forall(_.isEmpty),
      s"the chip-facing TX interface never accepted all beats within $dataGuard cycles " +
        s"(die0 left ${pending(0).size}, die1 left ${pending(1).size}). mainbandTx.ready is " +
        s"queue.enq.ready && active && !stall (ProtocolMainbandTx.scala:34-35), and the queue " +
        s"only drains on fdi.plTrdy (:44), which the adapter holds low while its 1-deep TX " +
        s"buffer is full (D2DMainbandModule.scala:99-101). A held pl_stallreq is worse than " +
        s"backpressure here: ProtocolMainbandTx.scala:46-48 FLUSHES the queue on stallRequested, " +
        s"so beats already accepted are discarded. Check the fdiStallReq/stalled flags. " +
        s"Observed: ${stateSummary(h)}")
    got.map(_.toSeq)
  }

  private def show(ws: Seq[BigInt]): String =
    if (ws.isEmpty) "<nothing>"
    else
      ws.take(3).map(w => s"0x${w.toString(16).take(16)}...").mkString(", ") +
        (if (ws.size > 3) s" (+${ws.size - 3} more)" else "")

  private def checkDelivery(h: H, from: Int, sent: Seq[BigInt], received: Seq[BigInt]): Unit = {
    val to = 1 - from
    val dup = received.size > sent.size && received.distinct.size <= sent.size
    // Compared through local Ints on purpose: `assert(received.size == sent.size)` makes
    // ScalaTest's macro pretty-print BOTH collections, which for a stuck pl_valid means sixty
    // 155-digit numbers ahead of the actual message.
    val nGot = received.size
    val nSent = sent.size
    assert(nGot == nSent,
      s"die $from -> die $to delivered $nGot beats, expected $nSent." +
        (if (dup)
           " Every extra beat is a DUPLICATE of one already delivered, which is the sticky FDI " +
             "pl_valid. D2DMainbandModule.scala:144 must stay `dataBuffRcvFillReg := " +
             "rxBeatAcceptedFromRdi` -- a one-cycle pulse, exactly as long as dataBuffRcvReg " +
             "holds the data (:135-137). If it went back to a level, pl_valid (:132) latches and " +
             "ProtocolMainbandRx re-enqueues the same beat every cycle (:38)."
         else if (received.size < sent.size)
           " Beats were LOST. The FDI receive direction has no backpressure and the protocol RX " +
             "queue is 2 deep (ProtocolMainbandRx.scala:38-46), so check the rxOvf flag first, " +
             "then whether the adapter's rxCaptureEnabled gate went low " +
             "(D2DMainbandModule.scala:124-127 needs d2dState===active && rxActiveReq && " +
             "rxActiveSts all three)."
         else "") +
        s"\n  sent     ${show(sent)}\n  received ${show(received)}\n  Observed: ${stateSummary(h)}")
    for (((s, r), k) <- sent.zip(received).zipWithIndex) {
      assert(s == r,
        s"die $from -> die $to corrupted beat #$k.\n  sent     0x${s.toString(16)}\n" +
          s"  received 0x${r.toString(16)}\n" +
          "The 512-bit beat crosses protocol -> adapter -> RDI -> lanes -> RDI -> adapter -> " +
          "protocol unchanged: ProtocolRawBeat has no header, no last/sop/eop and no byte " +
          "enables (ProtocolTypes.scala:17-19), lane pack and unpack must be exact inverses " +
          "(MainbandLaneController.scala:174-182 vs :225-235), and the sender's scrambler must " +
          "track the receiver's descrambler beat for beat (LogicalPhy.scala:302,:313). A first " +
          "beat that matches followed by a corrupted second means the two LFSRs are not " +
          "advancing together -- that is LogPhyStagedBringupTest S8, one level down. A beat that " +
          "is byte-exact but PERMUTED is lane reversal (LogicalPhy.scala:419-421, no inverse on " +
          "the RX unpack), which cannot assert here because the harness cross-wires lanes " +
          "straight (UcieMmioBringupHarness.scala:127-129).")
    }
  }

  private def assertLinkClean(h: H, where: String): Unit =
    for (i <- 0 until 2) {
      h.io.ltsmState(i).expect(LTSMState.sACTIVE, s"$where: the LTSM must stay in sACTIVE")
      h.io.rdiState(i).expect(RDIState.active, s"$where: the RDI must stay active")
      h.io.fdiState(i).expect(FDIState.active, s"$where: the FDI must stay active")
      assert(!flag(h, i, MmioFlag.phyTrainError),
        s"$where: die $i raised plTrainError. ${stateSummary(h)}")
      assert(!flag(h, i, MmioFlag.rdiPlError),
        s"$where: die $i raised RDI plError (a valid-framing error). stickyError is never " +
          s"cleared (MainbandLaneController.scala:244-250), so this permanently holds " +
          s"RDIController's holdUpperLayerStall and will wedge the TX path. ${stateSummary(h)}")
      assert(!flag(h, i, MmioFlag.rxOverflow),
        s"$where: die $i overflowed its protocol RX queue. ${stateSummary(h)}")
    }

  // ============================================================================
  // The rungs, two groups in declaration order: the bring-up climb (M0..M8), then the
  // software contract (C1..C4). One test per milestone; M2..M5 are each a cold start.
  // ============================================================================
  describe("UcieDigitalTop bring-up driven by register writes only (MMIO in, link up)") {

    it("M0: both register nodes answer MMIO reads and hold writes, out of reset") {
      gateOnLowerRungs(0)
      simulate(harness(), firtoolOpts = noAssertFirtoolOpts) { h =>
        initHarness(h)
        climbTo(h, 0)

        for (i <- 0 until 2) {
          assert(regRead(h, i, Off.ExtCapHeader) == extCapHeaderReset,
            s"die $i: DVSEC Extended Capability Header read back " +
              s"0x${regRead(h, i, Off.ExtCapHeader).toString(16)}, expected " +
              s"0x${extCapHeaderReset.toString(16)} (UcieLinkDvsecRegs.scala:37-41). A wrong " +
              s"value here means the map is at a different offset than this test believes, not " +
              s"that a register is broken -- the block base is params.baseAddress and the DVSEC " +
              s"sits at allocation.dvsecBase within it (UcieRegParams.scala:119-121)")

          val ctrl = regRead(h, i, Off.LinkControl)
          assert(ctrl == linkCtrlReset,
            s"die $i: Link Control read back 0x${ctrl.toString(16)}, expected reset value " +
              s"0x${linkCtrlReset.toString(16)} (raw_format_enable=0, target_link_width=x16, " +
              s"target_link_speed=24GT/s, start/retrain clear). NOTE target_link_speed resets to " +
              s"0x4 and NOT to speed4 -- UcieDigitalTop deliberately does not wire it to the " +
              s"PHY's maxDataRate for that reason, so a change here is a spec question, not a " +
              s"bring-up one")

          val sts = regRead(h, i, Off.LinkStatus)
          assert(!bit(sts, stsLinkStatus) && !bit(sts, stsLinkTraining),
            s"die $i: Link Status reads 0x${sts.toString(16)} out of reset with link_status or " +
              s"link_training already set. Both come from UcieDigitalTop's LinkToRegs wiring " +
              s"(linkUp = FDI active, linkTraining = LTSM in a training state), so this means " +
              s"the status path is inverted or tied. ${stateSummary(h)}")

          // A write to a plain RW field must stick, and must NOT start training: bit 10 is clear
          // in this word, and RWautoClear only fires on a written 1 (RegFieldTypes.scala:106).
          regWrite(h, i, Off.LinkControl, linkCtrlReset | (BigInt(1) << rawFormatEnableBit))
          val back = regRead(h, i, Off.LinkControl)
          assert(back == (linkCtrlReset | (BigInt(1) << rawFormatEnableBit)),
            s"die $i: wrote Link Control raw_format_enable=1 and read back 0x${back.toString(16)}. " +
              s"RegMapper only applies a write when EVERY mask bit over the field is set " +
              s"(RegMapper.scala:151), so a partial-word Put silently does nothing -- check that " +
              s"TLTester is issuing a full 4-byte Put")
          h.io.ltsmState(i).expect(LTSMState.sRESET,
            "a Link Control write with start_link_training CLEAR must not start training")
        }
      }
      stagePassed += 0
    }

    it("M1: the PHY acknowledges the adapter's RDI wake request out of reset") {
      gateOnLowerRungs(1)
      // No RESET wait: the wake responder runs on the plain module reset, independent of
      // training. Costs about a second.
      simulate(harness(), firtoolOpts = noAssertFirtoolOpts) { h =>
        initHarness(h)
        climbTo(h, 1)

        for (i <- 0 until 2) {
          h.io.ltsmState(i).expect(LTSMState.sRESET,
            "the wake handshake must complete without any training")
          h.io.rdiState(i).expect(RDIState.reset, "the RDI must still be in reset")
          assert(!flag(h, i, MmioFlag.rdiInbandPres),
            s"die $i reports RDI inband presence before training. ${stateSummary(h)}")
        }
        // It must HOLD: the adapter never drops lp_wake_req (D2DAdapter.scala:61 is a constant).
        h.clock.step(256)
        for (i <- 0 until 2) {
          assert(flag(h, i, MmioFlag.rdiPlWakeAck),
            s"die $i dropped pl_wake_ack while lp_wake_req is still a hardwired true. " +
              s"${stateSummary(h)}")
        }
      }
      stagePassed += 1
    }

    it("M2: one register write on die 0 trains both dies to sACTIVE") {
      gateOnLowerRungs(2)
      simulate(harness(), firtoolOpts = noAssertFirtoolOpts) { h =>
        initHarness(h)
        runResetWait(h)

        for (i <- 0 until 2) {
          val sts = regRead(h, i, Off.LinkStatus)
          assert(!bit(sts, stsLinkTraining),
            s"die $i already reports link_training after the RESET wait and before any write " +
              s"(Link Status 0x${sts.toString(16)}). ${stateSummary(h)}")
        }

        regWrite(h, 0, Off.LinkControl, linkCtrlReset | (BigInt(1) << startLinkTrainingBit))

        // Software must be able to SEE that its write took effect, over the same bus it wrote.
        val (sawTraining, w) = readUntil(h, 0, Off.LinkStatus)(bit(_, stsLinkTraining))
        assert(sawTraining,
          s"[M2] die 0's Link Status never reported link_training within $mmioPollReads reads " +
            s"of the start_link_training write (last read 0x${w.toString(16)}). The write is a " +
            s"ONE-CYCLE fire (RegFieldTypes.scala:104-114) into an edge detector that is only " +
            s"sampled while pwrGood && pllLock && resetMinWait hold (LinkTrainingSM.scala:1141). " +
            s"The reset wait already ran, and M1 proved pllLock reaches the LTSM, so the pulse " +
            s"was either swallowed in the register block (RWautoClear's ignoreWhen is " +
            s"linkIn.linkTraining, UcieLinkDvsecRegs.scala:87) or never left it. " +
            s"Observed: ${stateSummary(h)}")

        // die 1 was never written; it must be waking on die 0's SBINIT pattern, not on a write.
        val sts1 = regRead(h, 1, Off.LinkStatus)
        assert(!bit(sts1, stsLinkStatus),
          s"[M2] die 1 reports link_status up before training finished, and die 1 was never " +
            s"written. Link Status 0x${sts1.toString(16)}. ${stateSummary(h)}")

        climbTo(h, 2)

        for (i <- 0 until 2) {
          h.io.ltsmState(i).expect(LTSMState.sACTIVE, "the debug LTSM state must show ACTIVE")
          h.io.rdiState(i).expect(RDIState.active, "the RDI must be active in sACTIVE")
          assert(!flag(h, i, MmioFlag.phyTimedout),
            s"die $i hit a residency timeout. ${stateSummary(h)}")
          assert(!flag(h, i, MmioFlag.phyTrainError),
            s"die $i raised plTrainError. ${stateSummary(h)}")
          assert(!flag(h, i, MmioFlag.phyRecenter),
            s"die $i still reports recentering in sACTIVE. ${stateSummary(h)}")
        }
        // The register bit must still be HOLDING the Active request: it is cleared by
        // trainingDone, which UcieDigitalTop defines as FDI-active, not LTSM-sACTIVE.
        assert(flag(h, 0, MmioFlag.protoReqActive),
          s"[M2] die 0's start_link_training pending bit cleared at sACTIVE, so requestActive " +
            s"has already dropped and the FDI Active choreography can never start. That is the " +
            s"trainingDone definition (UcieDigitalTop's LinkToRegs wiring) going back to " +
            s"LTSM-sACTIVE. ${stateSummary(h)}")
        traceStates(h, "M2 at sACTIVE")
      }
      stagePassed += 2
    }

    it("M3: the adapters exchange ADV_CAP and raise FDI inband presence") {
      gateOnLowerRungs(3)
      simulate(harness(), firtoolOpts = noAssertFirtoolOpts) { h =>
        coldStart(h)
        climbTo(h, 3)

        for (i <- 0 until 2) {
          h.io.fdiState(i).expect(FDIState.reset,
            "FDI_BRINGUP is still inside RDIState.reset; pl_state_sts must not have moved yet")
          assert(flag(h, i, MmioFlag.fdiProtocolVld),
            s"die $i raised FDI inband presence without pl_protocol_vld, which the adapter " +
              s"derives from the same signal (D2DAdapter.scala:49-52). ${stateSummary(h)}")
          assert(flag(h, i, MmioFlag.negotiatedProto),
            s"die $i is in FDI_BRINGUP with no negotiated protocol. The capture window is " +
              s"narrow and closes behind you (ProtocolStateController.scala:48-56 captures only " +
              s"while pl_state_sts == reset && pl_inband_pres && pl_protocol_vld). " +
              s"${stateSummary(h)}")
        }
      }
      stagePassed += 3
    }

    it("M4: both dies reach FDI active and software can read the link up") {
      gateOnLowerRungs(4)
      simulate(harness(), firtoolOpts = noAssertFirtoolOpts) { h =>
        // BOTH dies from here up: requestActive rides start_link_training's pending bit, so a
        // die that was never written never asks for Active. See coldStart's note.
        coldStart(h, Seq(0, 1))
        climbTo(h, 4)

        for (i <- 0 until 2) {
          h.io.fdiState(i).expect(FDIState.active, "the FDI must reach active")
          assert(!flag(h, i, MmioFlag.fdiLpReqActive),
            s"die $i is in FDI active but the protocol layer is STILL presenting " +
              s"lp_state_req = active. The request must fall back to nop once pl_state_sts " +
              s"leaves reset, or the edge can never be re-armed. ${stateSummary(h)}")

          // The whole point: the link is up AND software can see it, on the same bus that
          // started it. link_training must be back to 0 -- sACTIVE is not a training state.
          val sts = regRead(h, i, Off.LinkStatus)
          assert(bit(sts, stsLinkStatus),
            s"die $i is in FDI active but Link Status reads 0x${sts.toString(16)} with " +
              s"link_status clear. linkUp is driven from ProtocolLayer.io.status.linkState in " +
              s"UcieDigitalTop; a mismatch here is the status path, not the link. " +
              s"${stateSummary(h)}")
          assert(!bit(sts, stsLinkTraining),
            s"die $i still reports link_training with the FDI active (Link Status " +
              s"0x${sts.toString(16)}). ${stateSummary(h)}")
          assert(bit(sts, stsRawFormatEnabled),
            s"die $i reports the link up without raw_format_enabled (Link Status " +
              s"0x${sts.toString(16)}). The adapter advertises Streaming/RAW unconditionally " +
              s"(D2DAdapter.scala:47-48). ${stateSummary(h)}")

          // start_link_training must have auto-cleared: its hwDone is trainingDone, which is
          // FDI-active. This is the only test of the register block's HARDWARE side.
          val ctrl = regRead(h, i, Off.LinkControl)
          assert(!bit(ctrl, startLinkTrainingBit),
            s"die $i: start_link_training is still pending with the link up (Link Control " +
              s"0x${ctrl.toString(16)}). RWautoClear clears pending on hwDone " +
              s"(RegFieldTypes.scala:110) and hwDone is LinkToRegs.trainingDone, so this means " +
              s"trainingDone never asserted -- software could never re-arm a second bring-up. " +
              s"${stateSummary(h)}")
        }
      }
      stagePassed += 4
    }

    it("M5: the chip-facing interface opens and the link holds over a dwell") {
      gateOnLowerRungs(5)
      simulate(harness(), firtoolOpts = noAssertFirtoolOpts) { h =>
        coldStart(h, Seq(0, 1))
        climbTo(h, 5)

        assertLinkClean(h, "M5 at entry")
        h.clock.step(dwellCycles)
        traceStates(h, s"M5 dwell +$dwellCycles cycles")
        assertLinkClean(h, s"M5 after a $dwellCycles-cycle dwell")
        for (i <- 0 until 2) {
          assert(flag(h, i, MmioFlag.chipTxReady),
            s"die $i closed its chip-facing TX interface while idling in active. " +
              s"${stateSummary(h)}")
          val sts = regRead(h, i, Off.LinkStatus)
          assert(bit(sts, stsLinkStatus),
            s"die $i dropped link_status over the dwell (Link Status 0x${sts.toString(16)}). " +
              s"${stateSummary(h)}")
        }
      }
      stagePassed += 5
    }

    it("M6: carries one protocol beat in each direction, exactly once") {
      gateOnLowerRungs(6)
      // The first rung that moves data, and so the first to pay for the 512-bit pack/unpack being
      // live -- see the exposeDataPath note in UcieMmioBringupHarness.
      simulate(harness(exposeDataPath = true), firtoolOpts = noAssertFirtoolOpts) { h =>
        coldStart(h, Seq(0, 1))
        climbTo(h, 6)

        // Pre-conditions, so every post-condition below is a real transition and not a constant.
        // All MMIO happens here, before exchange: tlAccess steps the clock internally.
        for (i <- 0 until 2) {
          assert(!flag(h, i, MmioFlag.fdiPlValid),
            s"die $i presents FDI pl_valid before any beat has been sent, so the post-delivery " +
              s"check below could not tell a one-cycle pulse from a stuck level. " +
              s"${stateSummary(h)}")
          assert(!flag(h, i, MmioFlag.fdiStallReq),
            s"die $i has FDI pl_stallreq asserted while idling in active. That is worse than " +
              s"backpressure: ProtocolMainbandTx.scala:46-48 turns it into a queue FLUSH, so " +
              s"accepted beats are discarded rather than delayed. ${stateSummary(h)}")
          val sts = regRead(h, i, Off.LinkStatus)
          assert(bit(sts, stsLinkStatus) && !bit(sts, stsLinkTraining),
            s"die $i: software does not see a trained link before the beat (Link Status " +
              s"0x${sts.toString(16)}). ${stateSummary(h)}")
          // The MMIO-specific claim, and the one thing the U ladder structurally cannot test:
          // UcieDigitalStagedBringupTest.scala:299 holds reqActive true for its whole run, while
          // here trainingDone (= fdiActive, UcieDigitalTop.scala:179) has already cleared
          // start_link_training's pending bit and with it requestActive (:158). So M6 is the
          // first place data moves with requestActive LOW, and that must be asserted, not assumed.
          assert(!flag(h, i, MmioFlag.protoReqActive),
            s"die $i still holds requestActive with the link up, so this rung would be moving " +
              s"data under the SAME condition the U ladder does and would prove nothing new. " +
              s"M4 asserts the pending bit auto-clears; if that passed and this failed, " +
              s"trainingDone is glitching. ${stateSummary(h)}")
        }

        val sent = Seq(Seq(payload(h.beatBits, 0, 0)), Seq(payload(h.beatBits, 1, 0)))
        val got = exchange(h, sent)
        checkDelivery(h, from = 0, sent = sent(0), received = got(1))
        checkDelivery(h, from = 1, sent = sent(1), received = got(0))

        for (i <- 0 until 2) {
          assert(!flag(h, i, MmioFlag.fdiPlValid),
            s"die $i still holds FDI pl_valid after its beat was delivered. It must be a " +
              s"one-cycle pulse per beat, i.e. D2DMainbandModule.scala:144 must stay " +
              s"`dataBuffRcvFillReg := rxBeatAcceptedFromRdi`. A level latches pl_valid (:132) " +
              s"and the protocol layer re-enqueues the same beat every cycle. ${stateSummary(h)}")
        }
        assertLinkClean(h, "M6 after one beat each way")

        // Traffic must not change software's view, and must not re-arm the pending bit. This is
        // the clause that makes M6 an MMIO rung rather than a copy of U8.
        for (i <- 0 until 2) {
          val sts = regRead(h, i, Off.LinkStatus)
          assert(bit(sts, stsLinkStatus) && !bit(sts, stsLinkTraining),
            s"die $i: carrying a beat changed software's view of the link (Link Status " +
              s"0x${sts.toString(16)}). ${stateSummary(h)}")
          val ctrl = regRead(h, i, Off.LinkControl)
          assert(!bit(ctrl, startLinkTrainingBit),
            s"die $i: start_link_training is pending again after data moved (Link Control " +
              s"0x${ctrl.toString(16)}). Nothing on the data path may set it -- its only write " +
              s"path is the bus (RegFieldTypes.scala:106). ${stateSummary(h)}")
        }
      }
      stagePassed += 6
    }

    it("M7: carries bursts in both directions simultaneously, in order and byte exact") {
      gateOnLowerRungs(7)
      simulate(harness(exposeDataPath = true), firtoolOpts = noAssertFirtoolOpts) { h =>
        coldStart(h, Seq(0, 1))
        climbTo(h, 7)

        // Both dies transmit on the same cycles: each die's scrambler and the peer's descrambler
        // advance together in BOTH directions at once, which a one-way test cannot reach, and the
        // queues see back-to-back beats instead of one isolated beat.
        val sent = Seq(
          (0 until burstLength).map(payload(h.beatBits, 0, _)),
          (0 until burstLength).map(payload(h.beatBits, 1, _)),
        )
        val got = exchange(h, sent)
        checkDelivery(h, from = 0, sent = sent(0), received = got(1))
        checkDelivery(h, from = 1, sent = sent(1), received = got(0))

        assertLinkClean(h, s"M7 after $burstLength beats each way")
        for (i <- 0 until 2) {
          val sts = regRead(h, i, Off.LinkStatus)
          assert(bit(sts, stsLinkStatus) && !bit(sts, stsLinkTraining),
            s"die $i: a $burstLength-beat burst changed software's view of the link (Link Status " +
              s"0x${sts.toString(16)}). ${stateSummary(h)}")
        }
      }
      stagePassed += 7
    }

    it("M8: the sideband carried the adapters' own link-init traffic without latching a fault") {
      gateOnLowerRungs(8)
      simulate(harness(), firtoolOpts = noAssertFirtoolOpts) { h =>
        coldStart(h, Seq(0, 1))
        climbTo(h, 8)

        // Part A -- all seven, AT the milestone, with NO stepping between climbTo and here.
        // The bits are sticky RegInits outside the sbReset block (LogicalPhy.scala:166-172), so
        // one read captures everything latched on the way up.
        for (i <- 0 until 2) {
          val latched = MmioFlag.sbFaults.collect { case (b, n) if flag(h, i, b) => n }
          assert(latched.isEmpty,
            s"[M8] die $i reached the milestone but latched sideband fault bit(s) " +
              s"${latched.mkString(", ")} while carrying the adapters' link-init traffic. " +
              s"${ladder(8).blocker}. Observed: ${stateSummary(h)}")
          assert(!flag(h, i, MmioFlag.phyTimedout),
            s"[M8] die $i hit a residency timeout on the way up. M2 asserts this at sACTIVE; a " +
              s"timeout that fires later (LINKINIT or a substate) is asserted nowhere else. " +
              s"${stateSummary(h)}")
          assert(ltCode(h, i) != LTState.sTRAINERROR.litValue,
            s"[M8] die $i is in sTRAINERROR. Note this only catches a RESIDENT visit: nothing " +
              s"latches 'we were here', ltRank has no entry for it so a transient visit is only " +
              s"caught if a stepUntil poll lands inside it, and giving it a sticky bit would be " +
              s"an RTL change. ${stateSummary(h)}")
        }

        // Part B -- six, after a dwell. sbDeserTimedout is an IDLE detector, not a fault detector:
        // the forwarded sideband clock STOPS between packets (SidebandLinkSerdes.scala:86-90,
        // :115), so idleStatus can never go true again (:263) and the core-domain timer latches
        // ~514 cycles after the last word (:270-279). Excluded here for the same reason the SV
        // ladder checks 6 of 7 after its dwell. Do NOT "fix" this back.
        h.clock.step(dwellCycles)
        val afterDwell = MmioFlag.sbFaults.filterNot(_._1 == MmioFlag.sbDeserTimedout)
        for (i <- 0 until 2) {
          val latched = afterDwell.collect { case (b, n) if flag(h, i, b) => n }
          assert(latched.isEmpty,
            s"[M8] die $i latched sideband fault bit(s) ${latched.mkString(", ")} over an idle " +
              s"$dwellCycles-cycle dwell, with the deserializer idle timeout already excluded. " +
              s"An idle link sends no sideband traffic, so nothing here should be able to move. " +
              s"${ladder(8).blocker}. Observed: ${stateSummary(h)}")
        }
        assertLinkClean(h, s"M8 after a $dwellCycles-cycle dwell")

        // Part C -- the register half, written as an IMPLICATION: the taps above are clean, so
        // every fault register software can read must agree. It is a TAUTOLOGY today, and saying
        // so is the point of the rung: no fault state reaches the register map at all. Four
        // independent severings, all in UcieDigitalTop: :193 ties phyToRegs to zero (killing
        // err_log1_flags, whose own description is "State Timeout / Sideband Timeout / Remote
        // LinkError / Internal"), :192 ties adapterToRegs to zero (killing uncorr/corr status),
        // :174 zeroes LinkToRegs and :175-184 drives only seven of its fourteen fields (killing
        // Link Status b19/b20/b21), and :195-201 leaves the remote register path unhandled.
        // On silicon there is nothing to read; M8 can only prove the RTL is clean, not that
        // bring-up would be debuggable. When that wiring lands, these become real checks.
        for (i <- 0 until 2) {
          val sts = regRead(h, i, Off.LinkStatus)
          assert(!bit(sts, stsCorrErr) && !bit(sts, stsUncorrNonFatal) && !bit(sts, stsUncorrFatal),
            s"die $i: Link Status 0x${sts.toString(16)} reports an error with all seven tapped " +
              s"fault bits clear. Either the taps and the register map disagree -- which is worse " +
              s"than a latched bit, because software's view and the RTL's view have diverged -- " +
              s"or someone wired linkStatus.corrErr/uncorrNonFatal/uncorrFatal and this rung is " +
              s"now testing a live path. ${stateSummary(h)}")
          val unc = regRead(h, i, Off.D2dUncorrStatus) & 0x3f
          val cor = regRead(h, i, Off.D2dCorrStatus) & 0x1f
          val el1 = (regRead(h, i, Off.PhyErrLog1) >> 8) & 0xf
          assert(unc == 0 && cor == 0 && el1 == 0,
            s"die $i: a D2D/PHY error register is set with all seven tapped fault bits clear " +
              s"(uncorr_err_status=0x${unc.toString(16)}, corr_err_status=0x${cor.toString(16)}, " +
              s"err_log1_flags=0x${el1.toString(16)}). Their hardware set inputs are literal " +
              s"zeros today (UcieDigitalTop.scala:192-193), so a non-zero read means either the " +
              s"wiring landed -- in which case compare it against the taps rather than against " +
              s"zero -- or this test is reading the wrong offset. ${stateSummary(h)}")
        }
      }
      stagePassed += 8
    }
  }

  describe("UcieDigitalTop register software contract (what a boot ROM may rely on)") {

    it("C1: a start_link_training write issued before the RESET minimum is deferred, not lost") {
      gateOnLowerRungs(9)
      // MEASURED, and it CONTRADICTS trap T0 as this file used to state it. The first draft of
      // this rung asserted the die stays in sRESET forever; the run said sSBINIT. The mechanism,
      // read back out of the RTL afterwards:
      //   LinkTrainingSM.scala:1017-1021  when(freshTrainingTrigger && currentState === sRESET)
      //                                   latches trainingEpisodeActive -- NOT gated on
      //                                   resetMinWait, unlike the state transition at :1141
      //   LinkTrainingSM.scala:1013-1015  autoRetrain := sRESET && trainingEpisodeActive &&
      //                                   (trainingRetryCounter <= retryAmtMax)
      //   LinkTrainingSM.scala:1141-1142  the sRESET exit accepts freshTrainingTrigger OR
      //                                   autoRetrain
      // retryAmtMax is io.retryTrainingAmt, which UcieDigitalTop.scala:121 ties to 0 -- and the
      // comparison is `<=`, so 0 <= 0 holds and autoRetrain is armed by the mistimed write and
      // stays armed. The instant resetMinWait latches 3.2M cycles later, the die trains on it.
      // So an early write is DEFERRED, not burned, and software cannot mistime it into a hang.
      simulate(harness(), firtoolOpts = noAssertFirtoolOpts) { h =>
        initHarness(h)
        h.clock.step(64)
        for (i <- 0 until 2) {
          h.io.ltsmState(i).expect(LTSMState.sRESET, "both dies must start in RESET")
        }

        // The mistimed write: 3.2M cycles too early, die 0 only.
        regWrite(h, 0, Off.LinkControl, linkCtrlReset | (BigInt(1) << startLinkTrainingBit))
        val early = regRead(h, 0, Off.LinkControl)
        assert(bit(early, startLinkTrainingBit),
          s"die 0: an early start_link_training write did not even latch the pending bit (Link " +
            s"Control 0x${early.toString(16)}). ignoreWhen is linkTraining and the LTSM is in " +
            s"sRESET, which is NOT in the phyTraining set (UcieDigitalTop.scala:167-170), so the " +
            s"write must be taken. ${stateSummary(h)}")
        assert(flag(h, 0, MmioFlag.protoReqActive),
          s"die 0: the pending bit is set but requestActive did not follow it " +
            s"(UcieDigitalTop.scala:158). ${stateSummary(h)}")
        h.io.ltsmState(0).expect(LTSMState.sRESET,
          "the state transition itself IS gated on resetMinWait (LinkTrainingSM.scala:1141), so " +
            "nothing may move yet -- only the episode latch arms early")

        // A second write while pending is 1 is REFUSED (RegFieldTypes.scala:106's !pending(0)),
        // and the readback is `pending` (:114), so it reads 1 either way: software cannot tell
        // "my write took" from "my write was refused". Harmless here only because the first write
        // is still going to fire; it is the reason a retry loop must not treat 1 as success.
        regWrite(h, 0, Off.LinkControl, linkCtrlReset | (BigInt(1) << startLinkTrainingBit))
        val retry = regRead(h, 0, Off.LinkControl)
        assert(bit(retry, startLinkTrainingBit),
          s"die 0: Link Control b10 reads 0x${retry.toString(16)} after a second write. " +
            s"${stateSummary(h)}")
        h.io.ltsmState(0).expect(LTSMState.sRESET, "still nothing may move before resetMinWait")

        stepResetWait(h)

        // THE FINDING: with no further write, the die trains anyway, and die 1 -- never written
        // at all -- wakes on its SBINIT pattern. climbTo re-uses M0..M3's own predicates and
        // messages, so a failure here is attributed to the rung that owns it.
        climbTo(h, 3)
        assert(flag(h, 0, MmioFlag.protoReqActive),
          s"die 0's pending bit cleared before the FDI came up, so the deferred trigger trained " +
            s"the PHY and then dropped the Active request. ${stateSummary(h)}")

        // End state: trained PHY, no link. die 1 was never written, so it never asks for Active
        // and INIT_DONE cannot complete either way -- see coldStart's note. That is what makes
        // this rung a statement about the WRITE TIMING and not about bring-up.
        for (i <- 0 until 2) {
          val sts = regRead(h, i, Off.LinkStatus)
          assert(!bit(sts, stsLinkStatus),
            s"die $i reports the link up with only ONE die ever written (Link Status " +
              s"0x${sts.toString(16)}). ${stateSummary(h)}")
        }
        val ctrl = regRead(h, 0, Off.LinkControl)
        assert(bit(ctrl, startLinkTrainingBit),
          s"die 0: start_link_training auto-cleared without the FDI reaching active (Link " +
            s"Control 0x${ctrl.toString(16)}). hwDone is LinkToRegs.trainingDone, which " +
            s"UcieDigitalTop.scala:179 defines as FDI-active and NOT LTSM-sACTIVE, precisely so " +
            s"the Active request survives the PHY coming up. ${stateSummary(h)}")
      }
      stagePassed += 9
    }

    it("C2: a die can be written after it has trained; only its OWN training window drops the write") {
      gateOnLowerRungs(10)
      // The boot-ROM-shaped sequence, and the one rung here whose result was unknown in advance.
      // It tests the exact scope of RWautoClear's ignoreWhen, which coldStart's note previously
      // described pessimistically as "both writes must land before either die leaves RESET".
      simulate(harness(), firtoolOpts = noAssertFirtoolOpts) { h =>
        coldStart(h, Seq(0)) // die 1 is deliberately NOT written up front

        // (a) The window that really is closed: die 1, mid-training, from its own point of view.
        val woke = stepUntil(h, phyFloorGuard, 8)(rankOf(h, 1) >= 1)
        assert(woke.ok,
          s"die 1 never left sRESET on die 0's SBINIT pattern within ${woke.cycles} cycles. That " +
            s"is M2's territory (remote wake, LinkTrainingSM.scala:1128-1140), not this rung's. " +
            s"${stateSummary(h)}")
        val training = regRead(h, 1, Off.LinkStatus)
        assert(bit(training, stsLinkTraining),
          s"die 1 left sRESET but Link Status 0x${training.toString(16)} does not report " +
            s"link_training, so the drop below could not be attributed to ignoreWhen. " +
            s"${stateSummary(h)}")
        regWrite(h, 1, Off.LinkControl, linkCtrlReset | (BigInt(1) << startLinkTrainingBit))
        val dropped = regRead(h, 1, Off.LinkControl)
        assert(!bit(dropped, startLinkTrainingBit),
          s"die 1: start_link_training latched while that die was training (Link Control " +
            s"0x${dropped.toString(16)}). ignoreWhen = linkIn.linkTraining " +
            s"(UcieLinkDvsecRegs.scala:87) must swallow it. ${stateSummary(h)}")

        // (b) die 1 trains anyway, on die 0's write alone, and parks in FDI_BRINGUP with no
        // request of its own -- exactly the hang M4's note describes.
        climbTo(h, 3)
        assert(!flag(h, 1, MmioFlag.protoReqActive),
          s"die 1 holds requestActive without ever having had a write accepted. " +
            s"${stateSummary(h)}")

        // (c) The claim under test: with die 1 in sACTIVE, linkTraining is false again -- sACTIVE
        // is NOT in the phyTraining set (UcieDigitalTop.scala:167-170) -- hwDone is false because
        // the FDI is not active, and pending is 0 because nothing was ever accepted. So the
        // deferred write must be taken. Software polls for this rather than racing RESET.
        val (idle, w) = readUntil(h, 1, Off.LinkStatus)(!bit(_, stsLinkTraining))
        assert(idle,
          s"die 1's link_training never cleared within $mmioPollReads reads (last read " +
            s"0x${w.toString(16)}), so a boot ROM has nothing to poll on. ${stateSummary(h)}")
        regWrite(h, 1, Off.LinkControl, linkCtrlReset | (BigInt(1) << startLinkTrainingBit))
        val accepted = regRead(h, 1, Off.LinkControl)
        assert(bit(accepted, startLinkTrainingBit),
          s"[C2] die 1: the DEFERRED start_link_training write was refused (Link Control " +
            s"0x${accepted.toString(16)}). ${ladder(10).blocker}. Observed: ${stateSummary(h)}")
        val armed = stepUntil(h, flagGuard, 1)(flag(h, 1, MmioFlag.protoReqActive))
        assert(armed.ok,
          s"[C2] die 1: the deferred write latched the pending bit but requestActive never " +
            s"followed within ${armed.cycles} cycles (UcieDigitalTop.scala:158). " +
            s"${stateSummary(h)}")

        climbTo(h, 10)
        for (i <- 0 until 2) {
          val sts = regRead(h, i, Off.LinkStatus)
          assert(bit(sts, stsLinkStatus) && !bit(sts, stsLinkTraining),
            s"die $i: the link came up on a deferred write but software cannot see it (Link " +
              s"Status 0x${sts.toString(16)}). ${stateSummary(h)}")
          val ctrl = regRead(h, i, Off.LinkControl)
          assert(!bit(ctrl, startLinkTrainingBit),
            s"die $i: start_link_training is still pending with the link up (Link Control " +
              s"0x${ctrl.toString(16)}). ${stateSummary(h)}")
        }
      }
      stagePassed += 10
    }

    it("C3: retrain_link is dropped while the link is down and latches at 1 forever while it is up") {
      gateOnLowerRungs(11)
      // Also PINS A DEFECT. retrain_link's fire reaches nothing (see the C3 blocker), and its
      // hwDone -- linkStatus.retrainDone -- has no driver at all, so once pending latches there is
      // no way back short of a hard reset.
      simulate(harness(), firtoolOpts = noAssertFirtoolOpts) { h =>
        initHarness(h)
        h.clock.step(64)

        // Half one, link DOWN. ignoreWhen is !linkUp (UcieLinkDvsecRegs.scala:89). RegFieldTypes'
        // write fn returns true.B unconditionally (:112), so the TL access COMPLETES normally and
        // the field silently discards it: no error, no retry signal, and the readback is unchanged.
        for (i <- 0 until 2) {
          regWrite(h, i, Off.LinkControl, linkCtrlReset | (BigInt(1) << retrainLinkBit))
          val ctrl = regRead(h, i, Off.LinkControl)
          assert(!bit(ctrl, retrainLinkBit),
            s"die $i: retrain_link latched with the link down (Link Control " +
              s"0x${ctrl.toString(16)}). ignoreWhen = !linkUp must swallow it. ${stateSummary(h)}")
          h.io.ltsmState(i).expect(LTSMState.sRESET,
            "a retrain_link write must not start training")
        }

        // Half two, link UP.
        coldStart(h, Seq(0, 1))
        climbTo(h, 11)
        assertLinkClean(h, "C3 before the retrain_link write")

        for (i <- 0 until 2) {
          regWrite(h, i, Off.LinkControl, linkCtrlReset | (BigInt(1) << retrainLinkBit))
        }
        h.clock.step(flagGuard)
        for (i <- 0 until 2) {
          val ctrl = regRead(h, i, Off.LinkControl)
          assert(bit(ctrl, retrainLinkBit),
            s"die $i: retrain_link auto-cleared (Link Control 0x${ctrl.toString(16)}). This rung " +
              s"asserts the OPPOSITE of what the field name promises: hwDone is " +
              s"linkStatus.retrainDone, which UcieDigitalTop.scala:174-184 never assigns, so the " +
              s"pending bit is expected to read 1 for the rest of the power cycle. A clear here " +
              s"means retrainDone got a source -- good news, and this half must be inverted. " +
              s"${stateSummary(h)}")
        }
        assertLinkClean(h, "C3 after the retrain_link write")
        for (i <- 0 until 2) {
          val sts = regRead(h, i, Off.LinkStatus)
          assert(bit(sts, stsLinkStatus) && !bit(sts, stsLinkTraining),
            s"die $i: a retrain_link write disturbed the link (Link Status " +
              s"0x${sts.toString(16)}). It must not: the fire pulse reaches nothing, because " +
              s"UcieDigitalTop.scala:159 ties requestRetrain false. ${stateSummary(h)}")
        }
      }
      stagePassed += 11
    }

    it("C4: teardown, retrain and a second bring-up cannot be stimulated from the registers") {
      gateOnLowerRungs(12)
      // CANCELLED BY CONSTRUCTION, in the idiom of LogPhyStagedBringupTest.scala:660-684. This is
      // the honest form of G3's third question: not "does re-bring-up work" but "there is no
      // register that can take a link down, so the question cannot be asked at this top".
      cancel(
        "C4 NOT ATTEMPTED: there is no register lever to test. UcieDigitalTop.scala:159, :160 " +
          "and :161 tie requestRetrain, requestLinkReset and requestDisable to false, and :190 " +
          "ties rb.linkReset false, so the register interface has NOTHING that can drop, retrain, " +
          "disable or reset a link that is up. retrain_link is separately dead in four places -- " +
          "C3 pins that -- and even if UcieDigitalTop.scala:159 were wired it would change " +
          "nothing observable, because AdapterSM has no fdi_lp_state_req == retrain consumer and " +
          "no retrain exit (AdapterSM.scala:512-519); wiring it would only create a reachable " +
          "TERMINAL trap, which is why fdiTeardown already treats FDI retrain as fatal. Nor can " +
          "the link fall over on its own at this top: sACTIVE has exactly two exits " +
          "(LinkTrainingSM.scala:1278-1282), the retrain one is unreachable and the trainError " +
          "one needs a timeout that is explicitly disabled in sACTIVE (:125-131). Recovery FROM " +
          "a drop is separately sound where it exists -- ProtocolStateController.scala:68-71 " +
          "presents lp_state_req = active as a LEVEL for linkError/disabled/linkReset and " +
          "AdapterSM.scala:522-543 consumes it -- but only from below: the layer-level " +
          "equivalents exist and pass today, D2DAdapterBringupTest.scala:358-431 'completes a " +
          "second bring-up' and D2DAdapterStateTest.scala:186,:251 'recovers to reset via both " +
          "triggers'. This rung goes live the day a teardown register exists.")
    }
  }
}
