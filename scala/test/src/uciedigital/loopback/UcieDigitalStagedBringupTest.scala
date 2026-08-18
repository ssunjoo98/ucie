package edu.berkeley.cs.uciedigital.loopback

import chisel3._
import chisel3.simulator.HasSimulator
import edu.berkeley.cs.uciedigital.UcieSimBackend
import chisel3.simulator.scalatest.ChiselSim
import edu.berkeley.cs.uciedigital.d2dadapter.LinkInitState
import edu.berkeley.cs.uciedigital.interfaces._
import edu.berkeley.cs.uciedigital.logphy._
import org.scalatest.funspec.AnyFunSpec

/*
  Run:
    rm -rf build/chiselsim/UcieDigitalStagedBringupTest   # stale svsim workdirs produce broken binaries
    ./mill --no-daemon test.testOnly edu.berkeley.cs.uciedigital.loopback.UcieDigitalStagedBringupTest
    # one rung only:
    ./mill --no-daemon test.testOnly edu.berkeley.cs.uciedigital.loopback.UcieDigitalStagedBringupTest -- -z "U4"
    # waveforms (one VCD per rung that RUNS; combine with -z, and read the same
    # notes in LogPhyStagedBringupTest.scala:18-48 first -- the dumps are multi-GB):
    ./mill --no-daemon test.testOnly edu.berkeley.cs.uciedigital.loopback.UcieDigitalStagedBringupTest -- -DemitVcd=1 -z "U2"

  Purpose:
    The SPECIFICATION ladder one level above LogPhyStagedBringupTest: two full
    {ProtocolLayer + D2DAdapter + LogicalPhy} stacks, cross-wired at the analog
    boundary, from RESET to protocol-level data end to end. Every rung asserts
    "the stack must have reached this milestone" and FAILS when it has not.

  Relationship to the LogPhy ladder (S0..S8) -- read before adding rungs:
    That ladder OWNS the training milestones. This one does NOT re-derive them
    rung by rung; it collapses all of S0..S7 into a single floor rung, U1.
    Reasons: (1) every rung that runs pays the 3.2M-cycle RESET wait from a cold
    start (~27 s each, measured on the LogPhy ladder), so re-deriving eight
    training milestones would double this ladder's cost to prove what another
    gate already proves; (2) a training milestone red HERE and green THERE is
    not a new fact about training, it is a fact about the adapter's RDI contact
    surface, which U1's blocker string enumerates; (3) one gate per fact -- two
    suites must not both assert "MBTRAIN completes".

    U1 is still a real rung, not a copy: four RDI signals change character when a
    real D2DAdapter replaces the testbench stub (lp_clk_ack becomes registered,
    lp_stall_ack runs through an FSM, lp_cfg_crd returns real credits,
    lp_wake_req becomes a constant TRUE so the PHY's wake responder is on the
    critical path for the first time).

  The ladder:
    U0   RDI wake handshake     -> pl_wake_ack on both dies (NO reset wait, ~1 s)
    U1   the PHY floor          -> both dies sACTIVE and RDI active, real adapter attached
    U2   ADV_CAP exchange       -> both dies raise FDI pl_inband_pres (adapter in FDI_BRINGUP)
    U3   protocol negotiated    -> negotiatedProtocolValid (Streaming/RAW) on both dies
    U4   Active requested       -> REQ_ACTIVE crosses, both dies raise pl_rx_active_req
    U5   RX-active handshake    -> both dies raise lp_rx_active_sts
    U6   FDI Active             -> both dies pl_state_sts == FDIState.active
    U7   chip interface opens   -> mainbandTx.ready on both dies, link holds over a dwell
    U8   one beat each way      -> chip TX on die i lands on die 1-i, byte exact, EXACTLY ONCE
    U9   bursts, both ways      -> 4 beats each way simultaneously, in order, byte exact
    U10  clean D2D sideband     -> no sideband fault bit latched carrying adapter traffic

  Why U10 is at the TOP and not folded into U1. When the ladder was written it
  was predicted red, and a latched status bit does not block anything below it:
  the RDI still reaches active and data still crosses. Rungs cancel everything
  ABOVE them, so a predicted-red cosmetic rung placed low would cancel the
  whole ladder for no reason. Order the rungs by dependency, not by where the
  signal lives.

  Status MEASURED 2026-08-13: ALL ELEVEN RUNGS PASS (11/11, 10 min 44 s wall
  clock), with the FDI pl_valid per-beat fix and the protocol requestActive
  input carried on this branch. Three rungs were predicted red when the ladder
  was written; what each one settles now:
    U2   the AdapterSM ADV_CAP receive-flag race (AdapterSM.scala:219-226,
         :239-254) is exposed to the RDI-active skew that real training
         produces instead of a poke -- measured at 10 cycles here, and the
         exchange completes. U2 MEASURES and PRINTS the skew each run so a
         tolerance regression is visible.
    U8   used to fail while the adapter's FDI pl_valid was a sticky level (one
         beat delivered N times); it now pins the per-beat fix in
         D2DMainbandModule.
    U10  used to latch a sideband fault bit on the way to sACTIVE; clean since
         the sideband and training fixes on this branch. The LogPhy ladder's
         S7 asserts the same thing one level down.

  Cost: every rung except U0 pays the real 3.2M-cycle RESET minimum wait
  (LinkTrainingSM.scala:108-116) plus the full training climb, from a cold start.
  Rungs above a blocker CANCEL before they elaborate, so a blocked run is
  cheaper, not dearer. U0 is deliberately free: it needs no reset wait at all.
*/
class UcieDigitalStagedBringupTest extends AnyFunSpec with ChiselSim {

  // Hardware constants (LinkTrainingSM.scala:87-116): 800 MHz * 8 ms residency
  // timeout; the RESET minimum wait is half of it. Not scalable from outside.
  private val hwTimeoutCycles = 6400000
  private val hwResetWait = hwTimeoutCycles / 2

  // Mandatory, both of them, for the same reasons as
  // LogPhyStagedBringupTest.scala:151-157: the SBINIT pattern flood trips
  // SidebandDeserializerDoesNotDropWords, and the layer-stripped build then
  // needs Verilator's fast x-initial or it segfaults at time zero.
  //
  // Stripping the layers ALSO masks three things that are live on this path and
  // are NOT defects the ladder should chase:
  //   - RDIController.scala:114-117 asserts the RDI clock prerequisites, but the
  //     adapter's lp_clk_ack is registered (D2DAdapter.scala:59-60) where the
  //     LogPhy stub answered combinationally, so there is a legal one-cycle
  //     window with plClkReq=1, lpClkAck=0. A 4-phase ack is allowed to take
  //     cycles; the assertion is what is wrong.
  //   - ProtocolMainbandRx.scala:51-58 FATALs on rx overflow and on pl_valid
  //     outside ACTIVE, both of which the sticky-pl_valid defect used to
  //     trigger before the per-beat fix (see U8).
  //   - ProtocolStateController.scala:134-147.
  // Turn the layers back on only when chasing one of those deliberately.
  private val noAssertFirtoolOpts = Array(
    "--disable-layers=Verification,Verification.Assert,Verification.Assume,Verification.Cover")

  // Verilator by default, tuned for speed: most rungs pay a 3.2M-cycle reset
  // from a cold start. UCIE_SIM_BACKEND=vcs switches to the coverage backend --
  // without this the ladder would stay on Verilator during a coverage run and
  // contribute no vdb at all, which is easy to miss because the run still passes.
  implicit private val ucieSimulator: HasSimulator =
    if (sys.env.get("UCIE_SIM_BACKEND").contains("vcs")) UcieSimBackend.fromEnv
    else
      HasSimulator.simulators.verilator(
        svsim.CommonCompilationSettings(
          optimizationStyle =
            svsim.CommonCompilationSettings.OptimizationStyle.OptimizeForSimulationSpeed))

  /** Print a two-die state dump as each rung is reached or given up on.
    * Terminal output only, always safe to turn on. */
  private val debugTrace = false

  /** Trim the VCD to the interesting window. MUST stay false unless the run also
    * passes -DemitVcd=1; see LogPhyStagedBringupTest.scala:173-184 for why the
    * two cannot be tied together automatically. */
  private val windowWaves = false

  // ============================================================================
  // Guards. Every wait is bounded; nothing may hang the simulator. A guard is
  // only paid in full when its rung FAILS.
  // ============================================================================
  /** Handshake that completes ~3 cycles out of reset. */
  private val wakeGuard = 64
  /** The whole training climb: well above the LogPhy ladder's measured times and
    * well below the 6.4M-cycle substate residency timeout, so a miss here means
    * "stuck", not "it would have made it eventually". */
  private val phyFloorGuard = 2000000
  /** A handful of adapter sideband exchanges over the 1-bit serial link.
    * ADV_CAP measures 7 cycles from PARAM_EXCH entry to the peer's sb_rcv
    * pulse; this is four orders of
    * magnitude of headroom. */
  private val sbExchangeGuard = 400000
  /** Plain registers, one or two cycles. */
  private val flagGuard = 4096
  /** Chip-to-chip beat delivery: three pipeline stages plus the mainband beat. */
  private val dataGuard = 8192

  private val dwellCycles = 20000
  private val burstLength = 4

  // ============================================================================
  // Decoding helpers (names for the log; ranks for monotone milestone checks).
  //
  // All `lazy` on purpose: building them touches ChiselEnum literals, and a
  // strict val would do that while ScalaTest is merely CONSTRUCTING the suite,
  // outside any Chisel elaboration or simulation context.
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

  /** Forward progress rank along the bring-up path. Off-path states have NO
    * rank: they are not "further along", they are a derailment, and every
    * milestone is expressed as "rank >= n" so it can never be satisfied by a
    * derailed FSM. Ranks also make milestones monotone, so a coarse polling
    * stride cannot race past one. */
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

  /** FDI states that mean "this link is being torn down, not brought up".
    * `retrain` is in the set ON PURPOSE: AdapterSM has no retrain -> active exit
    * (AdapterSM.scala:512-519),
    * so entering it is terminal for this ladder. */
  private lazy val fdiTeardown: Set[BigInt] = Set(
    FDIState.linkError.litValue,
    FDIState.disabled.litValue,
    FDIState.linkReset.litValue,
    FDIState.retrain.litValue,
  )

  private type H = UcieDigitalLoopbackHarness

  private def ltCode(h: H, i: Int): BigInt = h.io.ltState(i).peek().litValue
  private def ltsmCode(h: H, i: Int): BigInt = h.io.ltsmState(i).peek().litValue
  private def rdiCode(h: H, i: Int): BigInt = h.io.rdiState(i).peek().litValue
  private def fdiCode(h: H, i: Int): BigInt = h.io.fdiState(i).peek().litValue
  private def linkInitCode(h: H, i: Int): BigInt = h.io.adapterLinkInit(i).peek().litValue

  private def flag(h: H, i: Int, bit: Int): Boolean =
    ((h.io.flags(i).peek().litValue >> bit) & 1) == 1

  private def rankOf(h: H, i: Int): Int = ltRank.getOrElse(ltCode(h, i), -1)
  private def bothDies(cond: Int => Boolean): Boolean = cond(0) && cond(1)
  private def reachedAtLeast(h: H, r: Int): Boolean = bothDies(i => rankOf(h, i) >= r)
  private def rdiIs(h: H, i: Int, s: RDIState.Type): Boolean = rdiCode(h, i) == s.litValue
  private def fdiIs(h: H, i: Int, s: FDIState.Type): Boolean = fdiCode(h, i) == s.litValue
  private def inFdiBringup(h: H, i: Int): Boolean =
    linkInitCode(h, i) == LinkInitState.FDI_BRINGUP.litValue

  private def derailed(h: H): Boolean =
    (0 until 2).exists(i => rankOf(h, i) < 0 || fdiTeardown.contains(fdiCode(h, i)))

  private def name(table: Map[BigInt, String], code: BigInt): String =
    table.getOrElse(code, s"?0x${code.toString(16)}")

  // ============================================================================
  // Logging
  // ============================================================================
  private def setFlags(h: H, i: Int): String = {
    val w = h.io.flags(i).peek().litValue
    val on = DieFlag.names.collect { case (b, n) if ((w >> b) & 1) == 1 => n }
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
    if (debugTrace) println(s"[ladder] $tag :: ${stateSummary(h)}")

  // ============================================================================
  // Stimulus. Copied in SHAPE from LogPhyStagedBringupTest (same asymmetric cold
  // start, same RESET wait, same trigger width) so that a U1 failure is directly
  // comparable with an S0..S7 failure. Copied, not shared: the stimulus is all
  // harness knowledge, and the two ladders must stay free to diverge.
  // ============================================================================

  /** Quiet defaults.
    *
    * `requestActive` is asserted from time zero ON PURPOSE, and this is only
    * safe because the protocol layer gates it internally: it presents
    * FDIStateReq.active on the FDI only while pl_state_sts is reset AND
    * pl_inband_pres is high, i.e. only while the adapter is in FDI_BRINGUP
    * (ProtocolStateController). The adapter EDGE-detects nop->active there
    * (AdapterSM.scala:281-286) and blanket-clears the latch in every other
    * link-init sub-state (AdapterSM.scala:226), so a RAW active level held from
    * reset would consume the edge before the window opens and hang both dies.
    * `runResetWait` asserts that the layer really is holding nop, so if that
    * gate is ever removed this test says so instead of hanging. */
  private def initHarness(h: H, pwrGood: Boolean = true): Unit = {
    for (i <- 0 until 2) {
      h.io.swStartLinkTraining(i).poke(false.B)
      h.io.pwrGood(i).poke(pwrGood.B)
      h.io.protoCtrl(i).poke(ProtoCtrl.word(reqActive = true).U(4.W))
      h.io.txValid.foreach(_(i).poke(false.B))
      h.io.txData.foreach(_(i).poke(0.U))
      h.io.rxReady.foreach(_(i).poke(false.B))
    }
  }

  /** Sit out the hardware RESET minimum wait (3.2M cycles) plus slack, checking
    * RESET-residency invariants before and after. */
  private def runResetWait(h: H): Unit = {
    if (windowWaves) disableWaves()

    h.clock.step(64)
    for (i <- 0 until 2) {
      h.io.ltsmState(i).expect(LTSMState.sRESET, "LTSM must start in RESET")
      h.io.rdiState(i).expect(RDIState.reset, "RDI must start in reset")
      h.io.fdiState(i).expect(FDIState.reset, "FDI must start in reset")
      assert(!flag(h, i, DieFlag.rdiInbandPres), s"die $i: no RDI inband presence in RESET")
      assert(!flag(h, i, DieFlag.fdiInbandPres), s"die $i: no FDI inband presence in RESET")
      assert(!flag(h, i, DieFlag.fdiLpReqActive),
        s"die $i: the protocol layer must NOT present lp_state_req = active before the adapter " +
          s"reaches FDI_BRINGUP. The adapter edge-detects nop->active there " +
          s"(AdapterSM.scala:281-286) and force-clears the latch everywhere else " +
          s"(AdapterSM.scala:226), so an early level consumes the only edge there will ever be " +
          s"and both dies hang in FDI_BRINGUP forever with no timeout and no error bit. That is " +
          s"what the pl_inband_pres gate in ProtocolStateController exists to prevent -- if this " +
          s"fires, that gate was removed or weakened. Observed: ${stateSummary(h)}")
    }
    // Chunked so a hung simulator is diagnosable from the log.
    for (chunk <- 1 to 4) {
      h.clock.step(hwResetWait / 4)
      if (debugTrace) println(s"[ladder] reset wait chunk $chunk/4 done")
    }
    h.clock.step(64)
    for (i <- 0 until 2) {
      h.io.ltsmState(i).expect(LTSMState.sRESET, "no training without a trigger")
    }

    if (windowWaves) enableWaves()
  }

  /** Cold start that triggers ONE die only, leaving the other in sRESET.
    *
    * ASYMMETRIC ON PURPOSE: die 1 must wake on the clock pattern die 0
    * transmits (LinkTrainingSM.scala:1128-1140 -> :216). Two chiplets have no
    * reason to leave RESET on the same cycle, and pulsing both together is the
    * ONE arrival order the hardware never has to handle -- it hides the whole
    * remote-wake path. It is also what CREATES the inter-die skew that U2
    * measures. */
  private def coldStart(h: H, die: Int = 0): Unit = {
    initHarness(h)
    runResetWait(h)
    h.io.swStartLinkTraining(die).poke(true.B)
    h.clock.step(4)
    h.io.swStartLinkTraining(die).poke(false.B)
  }

  private case class WaitResult(ok: Boolean, cycles: Int, derailed: Boolean)

  /** Step in `stride`-cycle chunks until `cond` holds, the guard expires, or a
    * die derails. Always bounded: the guard is a hard ceiling, so no rung can
    * hang the simulator. Derailment ends the wait early -- a die in sTRAINERROR
    * or an FDI in linkError is never going to satisfy a forward milestone. */
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
      id = "U0",
      what = "the PHY acknowledges the adapter's RDI wake request (pl_wake_ack) on both dies, " +
        "straight out of reset and with no training",
      guard = wakeGuard,
      stride = 1,
      blocker =
        "NEW at this level -- the LogPhy ladder could not see this at all, because its stub tied " +
          "lp_wake_req false (LogPhyLoopbackHarness.scala:178) and RDIController.scala:71-72 " +
          "then satisfied the wake term through the `|| !io.rdi.lpWakeReq` escape. The real " +
          "adapter hardwires lp_wake_req TRUE from cycle 0 (D2DAdapter.scala:61), so " +
          "pl_wake_ack is now MANDATORY for RDI ACTIVE entry. It is produced by " +
          "RDIWakeHandshakeResponder, which walks sIDLE -> sUNGATE -> sACK_ASSERT in ~3 cycles " +
          "and can only leave sUNGATE on clocksUngatedAndStable " +
          "(RDIWakeHandshakeResponder.scala:46-52). That bit has NO hardware source in this " +
          "design and reaches the RDIController through a pure " +
          "pass-through from the analog status pin (LogicalPhy.scala:286 -> " +
          "PhyControlSignalTranslator.scala:68 -> LogicalPhy.scala:133), so if a future harness " +
          "or top leaves it DontCare -- which is exactly what UcieDigitalTop.scala:104 does -- " +
          "the responder parks in sUNGATE forever, activeBringupReady stays false, and the LTSM " +
          "sits in sLINKINIT until the 6.4M-cycle residency timeout derails it to sTRAINERROR. " +
          "That failure would surface as U1 red at rdi=reset, 3.2M cycles and one full verilate " +
          "pass later. THIS rung is the cheap version of that question: it costs no RESET wait " +
          "and answers it in about a second. Note the de-assert half of the handshake is never " +
          "exercised anywhere -- lp_wake_req is a constant, so sWAKE_ACTIVE -> sACK_DEASSERT -> " +
          "sIDLE is dead code until a PM rung exists. " +
          "WHAT THIS RUNG DOES NOT PROVE: clock gating. The responder's ungateClocks output has " +
          "no field in PhyControlToPhyIO, so it reaches nothing -- LogicalPhy.scala:292 only " +
          "dontTouches it to stop firtool pruning it. The request side " +
          "is a dangling wire while the answer side is a harness constant, so passing here means " +
          "the wake path is WIRED, not that a gate ever opened. It becomes a real check once " +
          "ungateClocks has a port and clocksUngatedAndStable comes from an analog model",
      reached = (h: H) => bothDies(i => flag(h, i, DieFlag.rdiPlWakeAck)),
    ),
    Stage(
      id = "U1",
      what = "with a real D2DAdapter on the RDI, training still completes: both dies reach " +
        "sACTIVE and RDI pl_state_sts == active from an asymmetric cold start",
      guard = phyFloorGuard,
      stride = 64,
      blocker =
        "FIRST run LogPhyStagedBringupTest. If S0..S8 are green and U1 is red, training itself " +
          "is fine and the defect is on the RDI contact surface that only exists here -- four " +
          "signals changed character when the stub became a real adapter: (a) lp_clk_ack is now " +
          "REGISTERED, so it arrives one cycle after pl_clk_req (D2DAdapter.scala:59-60) where " +
          "the stub answered combinationally (LogPhyLoopbackHarness.scala:172), and " +
          "RDIController.scala:71-73 gates ACTIVE entry on it; (b) lp_stall_ack now comes from " +
          "RDIStallHandler (StallHandler.scala:62-93), whose STALLED state needs " +
          "D2DMainbandModule's mainbandStallDone, itself gated on d2dState === active " +
          "(D2DMainbandModule.scala:63) -- so if the PHY raises pl_stallreq while the adapter " +
          "is still in reset, that handshake can NEVER complete and pl_stallreq wedges high " +
          "(RDIStallRequester's sWAIT_ACK_ASSERT has no release exit); (c) lp_cfg_crd is real " +
          "now (D2DAdapter.scala:90) where the stub returned no credits " +
          "(LogPhyLoopbackHarness.scala:188), so for the first time the RDI cfg credit loop is " +
          "closed at both ends; (d) lp_state_req is driven by AdapterSM, which must hold nop " +
          "through RESET or RDIStateMachine.scala:47-53 never sets resetReqObserved -- " +
          "AdapterSM.scala:45,:95-104 does hold nop until RDI_BRINGUP, so a failure here means " +
          "that changed. Check U0 first: if pl_wake_ack is low, this is the wake gate, not any " +
          "of the four. If S0..S8 are ALSO red, fix them THERE first -- this rung is not the " +
          "gate for training",
      reached = (h: H) => bothDies(i => rankOf(h, i) >= 5 && rdiIs(h, i, RDIState.active)),
    ),
    Stage(
      id = "U2",
      what = "the adapters complete the ADV_CAP exchange over the real serial sideband and both " +
        "raise FDI pl_inband_pres (link init has reached FDI_BRINGUP on both dies)",
      guard = sbExchangeGuard,
      stride = 8,
      blocker =
        "PREDICTED RED when this ladder was written, and the underlying race is STILL " +
          "UNFIXED: the ADV_CAP receive-flag race. AdapterSM latches an incoming ADV_CAP " +
          "ONLY while it already sits in PARAM_EXCH (AdapterSM.scala:239-244), and every other " +
          "reset-state cycle force-clears the flag (AdapterSM.scala:220-226); the sender latches " +
          "paramExchSbMsgSntFlag after one accepted send and NEVER retransmits " +
          "(AdapterSM.scala:107-109,:246-250). So if one die enters PARAM_EXCH more than one " +
          "message flight-time after the other, the early die's ADV_CAP is dropped and the late " +
          "die waits in PARAM_EXCH forever -- no timeout, no error bit. " +
          "A unit-level reproduction with a stubbed PHY pins the same hang and " +
          "measures the boundary: skew <= 6 completes, skew >= 7 hangs, because ADV_CAP takes 7 " +
          "cycles from PARAM_EXCH entry to the peer's sb_rcv pulse. Here the skew is whatever " +
          "REAL training produces between the two dies' RDI-active reports, and this rung " +
          "MEASURES and PRINTS it. Signature: linkInit=PARAM_EXCH with advSnt set and advRcv " +
          "clear on the late die, and FDI_BRINGUP on the early die. FIX (AdapterSM.scala): " +
          "delete the blanket clears at :220-226, replace them with self-holds, and hoist the " +
          "three sb_rcv latches (ADV_CAP, REQ_ACTIVE, RSP_ACTIVE) out of their `is(...)` blocks " +
          "into the enclosing `when(linkStateReg === RDIState.reset)` so a message that arrives " +
          "one sub-state early is still latched; the `.otherwise` branch at :298-307 already " +
          "clears all of them on every exit from reset, so nothing goes stale. VERIFY THE FIX " +
          "CHEAPLY FIRST with a unit-level stubbed-PHY reproduction, which needs no 3.2M " +
          "reset wait -- after the patch the boundary assertion INVERTS (skew=20 must now " +
          "COMPLETE), so update it in the same commit. If " +
          "instead the dies never reach PARAM_EXCH at all (linkInit=INIT_START/RDI_BRINGUP), " +
          "this is NOT the race: INIT_START waits on rdi_pl_inband_pres " +
          "(AdapterSM.scala:229-232) and RDI_BRINGUP on rdi_pl_state_sts == active (:234-237), " +
          "both of which U1 already proved",
      reached = (h: H) => bothDies(i => flag(h, i, DieFlag.fdiInbandPres)),
    ),
    Stage(
      id = "U3",
      what = "both protocol layers latch the negotiated protocol (Streaming, RAW format) while " +
        "the FDI is still in reset",
      guard = flagGuard,
      stride = 8,
      blocker =
        "the latch window is NARROW and closes behind you: ProtocolStateController.scala:48-56 " +
          "captures ONLY while pl_state_sts == reset && pl_inband_pres && pl_protocol_vld, i.e. " +
          "during FDI_BRINGUP, and then holds the value (cleared only by !pl_inband_pres, " +
          ":46-47). That is why this rung sits BELOW the FDI-active rung even though it looks " +
          "like a later milestone -- it is genuinely earlier in time. A failure here means the " +
          "capture never happened rather than that it was lost: check that the adapter asserted " +
          "pl_protocol_vld in FDI_BRINGUP (D2DAdapter.scala:49-52 derives it from inband " +
          "presence, so protoVld clear with fdiPres set is a new defect) and that it advertises " +
          "Streaming/RAW (D2DAdapter.scala:47-48, hardwired)",
      reached = (h: H) => bothDies(i => flag(h, i, DieFlag.negotiatedProto)),
    ),
    Stage(
      id = "U4",
      what = "the protocol layer requests Active inside the adapter's FDI_BRINGUP window, " +
        "REQ_ACTIVE crosses the sideband, and both dies raise FDI pl_rx_active_req",
      guard = sbExchangeGuard,
      stride = 8,
      blocker =
        "RED on any tree without the requestActive patch carried on this branch -- without it " +
          "there is NO WAY for software to ask for Active, and this ladder cannot even " +
          "elaborate. Before the patch, ProtocolStateController drove lp_state_req from " +
          "requestDisable / requestLinkReset / requestRetrain only, defaulting to nop, and " +
          "ProtocolLayerCtrlIO (ProtocolTypes.scala) had no requestActive field; " +
          "ProtocolLayerTest.scala even asserts lp_state_req == nop as the steady state. The " +
          "adapter needs a nop->active EDGE while in FDI_BRINGUP to set transitionToActiveReg " +
          "and send REQ_ACTIVE (AdapterSM.scala:281-286,:118-120); the peer's REQ_ACTIVE is " +
          "the only thing that raises pl_rx_active_req (:114,:263-267), RSP_ACTIVE is only " +
          "sent in reply (:116-117), and INIT_DONE needs BOTH halves (:288-290). Both dies are " +
          "symmetric, so neither ever started and both parked in FDI_BRINGUP with " +
          "pl_inband_pres=1, pl_state_sts=reset, pl_rx_active_req=0, negotiatedProtocolValid=1 " +
          "-- forever, with no timeout and no error bit. The fix, two files: (1) " +
          "ProtocolTypes.scala adds `val requestActive = Input(Bool())` to ProtocolLayerCtrlIO; " +
          "(2) ProtocolStateController.scala appends a LOWEST-priority arm `.elsewhen(" +
          "io.ctrl.requestActive && ((plStateSts === FDIState.reset && plInbandPres) || " +
          "plStateSts is linkError/disabled/linkReset)) { requestedState := FDIStateReq.active }`. " +
          "The pl_inband_pres term is what places the edge INSIDE FDI_BRINGUP (the adapter " +
          "raises pl_inband_pres exactly there, AdapterSM.scala:112,:441); dropping back to nop " +
          "once the FDI leaves reset re-arms the edge for a second bring-up; and the teardown " +
          "term is required because escaping linkError/disabled/linkReset back to reset reads " +
          "the active LEVEL, not an edge (AdapterSM.scala:522-523,:530-531,:540-541). Lowest " +
          "priority so a held requestActive can never mask a teardown request. If the flags show " +
          "fdiReqAct set on both dies but toActive clear, the level is there and the EDGE is " +
          "not -- the window moved. If toActive is set on both dies but reqRcv is clear, the " +
          "edge worked and the message was dropped: that is U2's race one sub-state later, and " +
          "the same AdapterSM patch fixes it",
      reached = (h: H) => bothDies(i => flag(h, i, DieFlag.fdiRxActiveReq)),
    ),
    Stage(
      id = "U5",
      what = "each protocol layer reports its own receiver alive (lp_rx_active_sts) -- generated " +
        "inside the layer, never poked",
      guard = flagGuard,
      stride = 8,
      blocker =
        "none known. ProtocolStateController's RxActiveState FSM (:75-112) goes sIdle -> " +
          "sWaitAssert -> sAsserted on pl_rx_active_req && rxReadyForActive, where " +
          "rxReadyForActive is queue.enq.ready && !rxOverflowReg (ProtocolMainbandRx.scala:48) " +
          "-- both true on an empty depth-2 queue. This is where the harness differs from " +
          "AdapterLoopbackHarness, which POKED lp_rx_active_sts by hand; here it must be " +
          "produced. A stall with the rxOvf flag set means the RX queue overflowed BEFORE any " +
          "traffic was ever sent, which is U8's sticky-pl_valid defect firing early and is " +
          "unrecoverable: rxOverflowReg is cleared only by clearRuntimeState, i.e. only by " +
          "taking the link down (ProtocolStateController.scala:119)",
      reached = (h: H) => bothDies(i => flag(h, i, DieFlag.fdiRxActiveSts)),
    ),
    Stage(
      id = "U6",
      what = "RSP_ACTIVE crosses both ways and both dies reach FDI pl_state_sts == active",
      guard = sbExchangeGuard,
      stride = 8,
      blocker =
        "the FDI_BRINGUP exit needs BOTH halves on each die: it must have SENT its own " +
          "RSP_ACTIVE (activeSbMsgExtRspReg) AND RECEIVED the peer's (activeSbMsgRspRcvFlag) " +
          "(AdapterSM.scala:288-290), and the send is gated on lp_rx_active_sts (:116-117), " +
          "which U5 just proved. Read the rspSnt / rspRcv probe flags: one set and the other " +
          "clear on the same die is U2's receive-flag race again, two sub-states later. " +
          "INIT_DONE is the ONLY door into RDIState.active (activeEntry, :122-127 -> :497-499), " +
          "so there is no alternate route to look for",
      reached = (h: H) => bothDies(i => fdiIs(h, i, FDIState.active)),
    ),
    Stage(
      id = "U7",
      what = "the chip-facing TX interface opens on both dies (mainbandTx.ready) and the link " +
        "holds over a dwell",
      guard = flagGuard,
      stride = 8,
      blocker =
        "mainbandTx.ready is queue.enq.ready && !(stallRequested || !active) " +
          "(ProtocolMainbandTx.scala:34-35). With U6 green the only way this stays low is a held " +
          "pl_stallreq: the adapter's FDIStallHandler asserts it whenever linkmgmt_stallreq is " +
          "up (StallHandler.scala:30-31), and AdapterSM raises that in ACTIVE on linkReset / " +
          "disabled / retrain entry (AdapterSM.scala:415-416). Check the fdiStallReq/stalled " +
          "flags: if stalled is set, something asked for a teardown and `derailed` should " +
          "already have caught it, so a stall with fdi=active is a NEW defect -- the likeliest " +
          "candidate is the sticky valid-framing error, which makes RDIController's " +
          "holdUpperLayerStall permanently true (MainbandLaneController.scala:244-250 never " +
          "clears stickyError, RDIController.scala:75-85), and once the adapter's LSM is active " +
          "the pending pl_stallreq completes and forces fdi.plTrdy low forever. Watch plError in " +
          "the flags",
      reached = (h: H) => bothDies(i => flag(h, i, DieFlag.chipTxReady)),
    ),
    Stage(
      id = "U8",
      what = "the receive path is armed on both dies (rx-active held, chip TX open) -- the beat " +
        "exchange itself is asserted in the rung BODY",
      guard = flagGuard,
      stride = 8,
      blocker =
        "PREDICTED RED when this ladder was written -- in the BODY, not in this predicate -- " +
          "and it now pins the per-beat pl_valid fix carried on this branch. Before that fix " +
          "the adapter's FDI pl_valid was a STICKY LEVEL, not a per-beat pulse: " +
          "D2DMainbandModule set dataBuffRcvFillReg on a received beat and cleared it ONLY " +
          "when rx_active_req dropped, and pl_valid IS that register. rx_active_req stays high " +
          "for the whole of ACTIVE (AdapterSM.scala:422-427), so after the first beat pl_valid " +
          "never fell. ProtocolMainbandRx enqueues on every cycle pl_valid is high " +
          "(ProtocolMainbandRx.scala:38) and the FDI receive direction has NO backpressure by " +
          "spec, so ONE beat sent became N identical beats delivered; with the chip RX drained " +
          "the count just grew, and with it undrained the depth-2 queue fills in two cycles, " +
          "rxOverflowReg latches (:44-46) and rxReadyForActive drops permanently (:48), taking " +
          "U5 down on any later re-arm. It was the exact " +
          "INVERSE of the RDI side, where the PHY correctly delivers a one-cycle pulse " +
          "(MainbandLaneController.scala:240) -- the adapter converted a pulse into a level and " +
          "nothing converted it back. The fix (D2DMainbandModule.scala) drives " +
          "`dataBuffRcvFillReg := rxBeatAcceptedFromRdi` -- a one-cycle pulse " +
          "one cycle behind the RDI beat, which is exactly how long dataBuffRcvReg " +
          "holds the data; dataBuffRcvFillReg has exactly one consumer so nothing else " +
          "changes, and back-to-back RDI beats produce back-to-back FDI pulses with no " +
          "loss. If this rung goes red with ZERO " +
          "beats arriving, look one level down: the RDI round trip is separately gated by " +
          "LogPhyStagedBringupTest S8",
      reached = (h: H) =>
        bothDies(i => flag(h, i, DieFlag.chipTxReady) && flag(h, i, DieFlag.fdiRxActiveSts)),
    ),
    Stage(
      id = "U9",
      what = "same gate as U8 -- the burst and the simultaneous-transmit cases are asserted in " +
        "the rung BODY",
      guard = flagGuard,
      stride = 8,
      blocker =
        "a burst that loses or reorders beats while U8's single beat passes is FLOW CONTROL, not " +
          "framing: the protocol TX queue is 2 deep (ProtocolLayerParams) and drains one beat " +
          "per cycle through a single 1-deep adapter buffer (D2DMainbandModule.scala:56-57," +
          ":99-121) into a single-beat PHY transfer, with NO end-to-end credit anywhere on the " +
          "mainband -- the receiver's 2-deep RX queue silently drops on overflow " +
          "(ProtocolMainbandRx.scala:38-46) and only a status bit records it. A FIRST beat that " +
          "matches followed by a CORRUPTED second beat is a different bug: that is the " +
          "scrambler/descrambler lockstep (LogicalPhy.scala:302,:313), which is " +
          "LogPhyStagedBringupTest S8's territory, one level down. A beat that is byte-exact but " +
          "PERMUTED is lane reversal: txLaneReversalEnabled permutes the TX lanes " +
          "(LogicalPhy.scala:362,:417-421) and there is NO inverse permutation on the RX unpack " +
          "path, so if doLaneReversal ever asserted every FSM would stay green while the data " +
          "came out scrambled. It cannot assert in this harness (lanes are cross-wired straight " +
          "so MBINIT.REVERSALMB resolves to no-reversal, LinkTrainingSM.scala:823-832)",
      reached = (h: H) =>
        bothDies(i => flag(h, i, DieFlag.chipTxReady) && flag(h, i, DieFlag.fdiRxActiveSts)),
    ),
    Stage(
      id = "U10",
      what = "the D2D sideband carried the adapters' own link-init traffic without latching any " +
        "fault bit -- asserted in the rung BODY, per fault bit",
      guard = flagGuard,
      stride = 8,
      blocker =
        "PREDICTED RED when this ladder was written, and deliberately the TOP rung so it " +
          "cancels nothing below it: a latched status bit does not stop the RDI reaching " +
          "active or data crossing, so ordering it low would cancel the whole ladder for a " +
          "cosmetic reason. Back then, with a real adapter attached, at least one of the seven " +
          "sideband fault bits latched on the way to sACTIVE, on BOTH dies; clean since the " +
          "sideband and training fixes on this branch. " +
          "LogPhyStagedBringupTest S7 asserts the SAME thing over the " +
          "same training and PASSES (LogPhyStagedBringupTest.scala:970-972), so training is not " +
          "what latches it -- the difference is the adapter's own messages (ADV_CAP, " +
          "REQ/RSP_ACTIVE) riding the RDI cfg path, which the LogPhy ladder's quiet stub never " +
          "sent, plus the real lp_cfg_crd credit return the stub tied false " +
          "(LogPhyLoopbackHarness.scala:188). This rung reports WHICH bit, which is the whole " +
          "reason the harness carries the seven bits separately instead of OR-ing them as " +
          "LogPhyLoopbackHarness.scala:200-207 does. Triage by bit: " +
          "sbUnhandledCurrentLayerMsgSeen or an sbInvalidRoute* bit means a message was " +
          "addressed to the wrong layer, and the first suspect is SBMsgCreate deriving dstid " +
          "from `src` instead of `dst` (SidebandMessageEncodings.scala:376-379 -- inert today " +
          "only because every call site passes src == dst) or a non-16-bit msgInfo shifting the " +
          "header so the SENDER's own switch drops the packet, which is the D-31 family. " +
          "sbRxPriorityQueuesFullSeen means cfg-credit exhaustion, i.e. the credit loop that is " +
          "closed for the first time in this harness is not actually returning credits. " +
          "sbParityErrSeen with the link otherwise up means RAW-mode parity, D-11's neighbourhood",
      reached = (h: H) => bothDies(i => fdiIs(h, i, FDIState.active)),
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
    val blocker =
      if (st.blocker.isEmpty) "NO known defect -- this is a new failure mode, investigate"
      else st.blocker
    s"[${st.id}] MISSED MILESTONE: ${st.what} -- $why.$prereq " +
      s"Observed at give-up: ${stateSummary(h)}. Known blocker: $blocker"
  }

  /** Walk the ladder from the current point up to and including rung `upTo`,
    * asserting each milestone in turn. Fails at the LOWEST unmet rung, which
    * keeps a blocked run cheap and makes the log name the real culprit rather
    * than the rung under test. */
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
  // Skip-ahead gate. Identical contract to LogPhyStagedBringupTest.scala:649-684:
  // sequential, declaration-order execution (AnyFunSpec default, and build.mill
  // sets testParallelism = false). Do NOT add -P.
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

  // ============================================================================
  // Chip-level data traffic (U8, U9)
  // ============================================================================

  /** Distinct, non-repeating payloads: a per-die tag, a sequence number and a
    * walking pattern, so a swapped beat, a stale beat and a lane permutation all
    * fail differently. Sized from the port so an fdiParams change cannot
    * silently truncate the poke. */
  private def payload(bits: Int, die: Int, seq: Int): BigInt =
    (0 until bits / 16).foldLeft(BigInt(0)) { (acc, i) =>
      val w = ((die + 1) << 12) | ((seq + 1) << 8) | ((i * 7 + die * 3 + seq) & 0xff)
      acc | (BigInt(w & 0xffff) << (i * 16))
    }

  /** Drive `words(i)` out of die i's chip-facing TX while draining BOTH dies'
    * chip-facing RX, then keep draining through `drainCycles` of quiet. Returns
    * what each die received, in arrival order.
    *
    * Both directions are pumped every cycle, so one implementation covers the
    * one-way, burst and simultaneous-transmit cases. Draining concurrently is
    * NOT optional: the FDI receive path has no backpressure and the protocol RX
    * queue is 2 deep, so an undrained receiver would drop beats and blame the
    * link. Draining through the quiet tail is what makes the "exactly once"
    * clause meaningful -- a sticky pl_valid keeps delivering there.
    *
    * Handshake timing: mainbandTx.ready is combinational from queue.enq.ready
    * and the active/stall gates (ProtocolMainbandTx.scala:34-35), so valid+data
    * are poked and ready is peeked in the SAME cycle, before stepping. The RX
    * queue output is exposed straight through (`io.chip <> queue.io.deq`,
    * ProtocolMainbandRx.scala:40), so valid/bits are stable until ready pops
    * them: peek before stepping, and the step with ready high is the pop. */
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
      val accepted = (0 until 2).map(i => pending(i).nonEmpty && flag(h, i, DieFlag.chipTxReady))
      val delivered = (0 until 2).map(i =>
        Option.when(flag(h, i, DieFlag.chipRxValid))(h.io.rxData.get(i).peek().litValue))

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
        s"buffer is full (D2DMainbandModule.scala:99-101). Observed: ${stateSummary(h)}")
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
    // Compared through local Ints on purpose: `assert(received.size == sent.size)`
    // makes ScalaTest's macro pretty-print BOTH collections, which for a stuck
    // pl_valid means sixty 155-digit numbers ahead of the actual message.
    val nGot = received.size
    val nSent = sent.size
    assert(nGot == nSent,
      s"die $from -> die $to delivered $nGot beats, expected $nSent." +
        (if (dup)
           " Every extra beat is a DUPLICATE of one already delivered, which is the sticky FDI " +
             "pl_valid: D2DMainbandModule.scala:139-144 holds dataBuffRcvFillReg (and therefore " +
             "pl_valid, :132) high until rx_active_req drops, so ProtocolMainbandRx re-enqueues " +
             "the same beat every cycle (ProtocolMainbandRx.scala:38). Make it a one-cycle " +
             "pulse: dataBuffRcvFillReg := rxBeatAcceptedFromRdi."
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
          "advancing together -- that is LogPhyStagedBringupTest S8, one level down.")
    }
  }

  private def assertLinkClean(h: H, where: String): Unit =
    for (i <- 0 until 2) {
      h.io.ltsmState(i).expect(LTSMState.sACTIVE, s"$where: the LTSM must stay in sACTIVE")
      h.io.rdiState(i).expect(RDIState.active, s"$where: the RDI must stay active")
      h.io.fdiState(i).expect(FDIState.active, s"$where: the FDI must stay active")
      assert(!flag(h, i, DieFlag.phyTrainError),
        s"$where: die $i raised plTrainError. ${stateSummary(h)}")
      assert(!flag(h, i, DieFlag.rdiPlError),
        s"$where: die $i raised RDI plError (a valid-framing error). stickyError is never " +
          s"cleared (MainbandLaneController.scala:244-250), so this permanently holds " +
          s"RDIController's holdUpperLayerStall and will wedge the TX path. It is invisible at " +
          s"the chip edge -- ProtocolLayer reads none of the FDI error inputs and ties " +
          s"lpLinkError to false (ProtocolStateController.scala:115). ${stateSummary(h)}")
      assert(!flag(h, i, DieFlag.rxOverflow),
        s"$where: die $i overflowed its protocol RX queue. ${stateSummary(h)}")
    }

  // ============================================================================
  // The rungs. One test per milestone; each one that runs (except U0) is a cold
  // start.
  // ============================================================================
  describe("UcieDigital staged bring-up ladder (reset -> trained PHY -> FDI active -> protocol data, one gate per rung)") {

    it("U0: the PHY acknowledges the adapter's RDI wake request out of reset") {
      gateOnLowerRungs(0)
      // The ONE rung that needs no RESET wait: the wake responder runs on the
      // plain module reset, independent of training. Costs about a second.
      simulate(new UcieDigitalLoopbackHarness(), firtoolOpts = noAssertFirtoolOpts) { h =>
        initHarness(h)
        climbTo(h, 0)

        for (i <- 0 until 2) {
          h.io.ltsmState(i).expect(LTSMState.sRESET,
            "the wake handshake must complete without any training")
          h.io.rdiState(i).expect(RDIState.reset, "the RDI must still be in reset")
          assert(!flag(h, i, DieFlag.rdiInbandPres),
            s"die $i reports RDI inband presence before training. ${stateSummary(h)}")
        }
        // It must HOLD: the responder parks in sWAKE_ACTIVE because the adapter
        // never drops lp_wake_req (D2DAdapter.scala:61 is a constant).
        h.clock.step(256)
        for (i <- 0 until 2) {
          assert(flag(h, i, DieFlag.rdiPlWakeAck),
            s"die $i dropped pl_wake_ack while lp_wake_req is still a hardwired true. " +
              s"${stateSummary(h)}")
        }
      }
      stagePassed += 0
    }

    it("U1: trains to sACTIVE with a real D2DAdapter driving the RDI") {
      gateOnLowerRungs(1)
      simulate(new UcieDigitalLoopbackHarness(), firtoolOpts = noAssertFirtoolOpts) { h =>
        coldStart(h)
        climbTo(h, 1)

        for (i <- 0 until 2) {
          h.io.ltsmState(i).expect(LTSMState.sACTIVE, "the debug LTSM state must show ACTIVE")
          h.io.rdiState(i).expect(RDIState.active, "the RDI must be active in sACTIVE")
          assert(!flag(h, i, DieFlag.phyTimedout),
            s"die $i hit a residency timeout. ${stateSummary(h)}")
          assert(!flag(h, i, DieFlag.phyTrainError),
            s"die $i raised plTrainError. ${stateSummary(h)}")
          assert(!flag(h, i, DieFlag.phyRecenter),
            s"die $i still reports recentering in sACTIVE. ${stateSummary(h)}")
          assert(!flag(h, i, DieFlag.rdiStallReq),
            s"die $i has RDI pl_stallreq asserted at sACTIVE entry. The adapter cannot ack it " +
              s"until its own LSM reaches active (D2DMainbandModule.scala:63 gates the drain on " +
              s"d2dState === active) and RDIStallRequester's sWAIT_ACK_ASSERT has no release " +
              s"exit, so this wedges the RDI TX path permanently. ${stateSummary(h)}")
        }
        // Sideband faults are U10's business ON PURPOSE -- see U10's blocker.
        traceStates(h, "U1 at sACTIVE")
      }
      stagePassed += 1
    }

    it("U2: exchanges ADV_CAP and raises FDI inband presence on both dies") {
      gateOnLowerRungs(2)
      simulate(new UcieDigitalLoopbackHarness(), firtoolOpts = noAssertFirtoolOpts) { h =>
        coldStart(h)

        // Coarse climb to sLINKINIT first: everything below it is gated by U1
        // and by the LogPhy ladder, and polling it at cycle resolution would
        // cost millions of peeks.
        val toLinkinit = stepUntil(h, phyFloorGuard, 64)(reachedAtLeast(h, 4))
        assert(toLinkinit.ok,
          s"[U2] the PHYs never reached sLINKINIT within ${toLinkinit.cycles} cycles, so the " +
            s"adapters never had a chance to run. This is U1's territory. " +
            s"Observed: ${stateSummary(h)}")

        // Cycle-resolution watch over the window that CREATES the skew feeding
        // the ADV_CAP race: each die's RDI reaches active on its own schedule
        // (its own sLINKINIT entry plus its own sideband round trip -- the
        // responder goes active when it SENDS, the requester when it RECEIVES),
        // and PARAM_EXCH is entered one cycle later (AdapterSM.scala:234-237).
        val rdiAt = Array(-1, -1)
        val paramAt = Array(-1, -1)
        var n = 0
        while ((rdiAt.contains(-1) || paramAt.contains(-1)) && n < sbExchangeGuard) {
          for (i <- 0 until 2) {
            if (rdiAt(i) < 0 && rdiIs(h, i, RDIState.active)) rdiAt(i) = n
            if (paramAt(i) < 0 && linkInitCode(h, i) >= LinkInitState.PARAM_EXCH.litValue)
              paramAt(i) = n
          }
          if (rdiAt.contains(-1) || paramAt.contains(-1)) { h.clock.step(1); n += 1 }
        }
        assert(!rdiAt.contains(-1),
          s"[U2] an RDI never reached active within $n cycles of sLINKINIT (die0=${rdiAt(0)}, " +
            s"die1=${rdiAt(1)}, -1 = never). That is U1's milestone. " +
            s"Observed: ${stateSummary(h)}")
        assert(!paramAt.contains(-1),
          s"[U2] an adapter never reached PARAM_EXCH within $n cycles (die0=${paramAt(0)}, " +
            s"die1=${paramAt(1)}, -1 = never) even though both RDIs went active at " +
            s"${rdiAt.mkString("/")}. INIT_START waits on rdi_pl_inband_pres and RDI_BRINGUP on " +
            s"rdi_pl_state_sts == active (AdapterSM.scala:229-237), both already proved, so " +
            s"this is a NEW defect. Observed: ${stateSummary(h)}")

        val rdiSkew = math.abs(rdiAt(0) - rdiAt(1))
        val paramSkew = math.abs(paramAt(0) - paramAt(1))
        println(s"[ladder] U2 MEASURED skew: RDI-active $rdiSkew cycles, PARAM_EXCH entry " +
          s"$paramSkew cycles (die0 at ${paramAt(0)}, die1 at ${paramAt(1)}); the documented " +
          s"tolerance is 6")

        val res = stepUntil(h, sbExchangeGuard, 8)(ladder(2).reached(h))
        assert(res.ok,
          s"[U2] the ADV_CAP exchange did not complete within ${res.cycles} cycles. MEASURED " +
            s"skew: RDI-active $rdiSkew cycles, PARAM_EXCH entry $paramSkew cycles (die0 " +
            s"entered at ${paramAt(0)}, die1 at ${paramAt(1)}). " +
            s"A unit-level stubbed-PHY reproduction measured the tolerance at 6 cycles -- ADV_CAP " +
            s"takes 7 cycles from PARAM_EXCH entry to the peer's sb_rcv pulse -- so a PARAM_EXCH " +
            s"skew above that IS the documented race, now reproduced on a real link instead of " +
            s"a poke. ${ladder(2).blocker}. Observed: ${stateSummary(h)}")

        climbTo(h, 2)
        for (i <- 0 until 2) {
          h.io.fdiState(i).expect(FDIState.reset,
            "FDI_BRINGUP is still inside RDIState.reset; pl_state_sts must not have moved yet")
          assert(flag(h, i, DieFlag.fdiProtocolVld),
            s"die $i raised FDI inband presence without pl_protocol_vld, which the adapter " +
              s"derives from the same signal (D2DAdapter.scala:49-52). ${stateSummary(h)}")
        }
        // NOTE deliberately NO assertion on advSnt/advRcv here. Those probes are
        // a LIVE VIEW of PARAM_EXCH, not a record that it happened:
        // AdapterSM.scala:220-221 clears both unconditionally at the top of the
        // reset block and only the PARAM_EXCH arm self-holds them (:242-250), so
        // at FDI_BRINGUP they correctly read false. Asserting them here would be
        // a guaranteed false failure.
      }
      stagePassed += 2
    }

    it("U3: both protocol layers latch the negotiated protocol") {
      gateOnLowerRungs(3)
      simulate(new UcieDigitalLoopbackHarness(), firtoolOpts = noAssertFirtoolOpts) { h =>
        coldStart(h)
        climbTo(h, 3)

        for (i <- 0 until 2) {
          assert(flag(h, i, DieFlag.negotiatedProto),
            s"die $i has no negotiated protocol. ${stateSummary(h)}")
          assert(flag(h, i, DieFlag.fdiInbandPres),
            s"die $i latched a negotiated protocol but has since dropped pl_inband_pres, which " +
              s"clears the latch (ProtocolStateController.scala:46-47). ${stateSummary(h)}")
        }
      }
      stagePassed += 3
    }

    it("U4: the protocol layer requests Active and REQ_ACTIVE crosses") {
      gateOnLowerRungs(4)
      simulate(new UcieDigitalLoopbackHarness(), firtoolOpts = noAssertFirtoolOpts) { h =>
        coldStart(h)
        climbTo(h, 3)

        // Split the wait so the log says WHICH half failed: producing the edge
        // is the protocol layer's job, latching it is the adapter's, and
        // delivering REQ_ACTIVE is the sideband's.
        val level = stepUntil(h, flagGuard, 1)(bothDies(i => flag(h, i, DieFlag.fdiLpReqActive)))
        assert(level.ok,
          s"[U4] the protocol layer never presented lp_state_req = active within " +
            s"${level.cycles} cycles even though both adapters are in FDI_BRINGUP with " +
            s"pl_inband_pres up. This is NOT the sideband -- nothing was sent. " +
            s"${ladder(4).blocker}. Observed: ${stateSummary(h)}")

        val latched =
          stepUntil(h, flagGuard, 1)(bothDies(i => flag(h, i, DieFlag.transitionToActive)))
        assert(latched.ok,
          s"[U4] the adapters saw lp_state_req = active but never latched " +
            s"transitionToActiveReg within ${latched.cycles} cycles. That latch needs a " +
            s"nop->active EDGE while in FDI_BRINGUP (AdapterSM.scala:281-286); a level that was " +
            s"already active when FDI_BRINGUP was entered consumes the edge, and " +
            s"AdapterSM.scala:226 force-clears the register in every other sub-state so it can " +
            s"never be recovered. That is exactly what the pl_inband_pres gate in " +
            s"ProtocolStateController exists to prevent. Observed: ${stateSummary(h)}")

        climbTo(h, 4)
        for (i <- 0 until 2) {
          // Guarded: these probes are cleared on the INIT_DONE transition by the
          // blanket clears at AdapterSM.scala:222-225, so they are only readable
          // while the die is still in FDI_BRINGUP.
          if (inFdiBringup(h, i)) {
            assert(flag(h, i, DieFlag.actReqSent) && flag(h, i, DieFlag.actReqRcvd),
              s"die $i raised pl_rx_active_req without both halves of the REQ_ACTIVE exchange " +
                s"(reqSnt=${flag(h, i, DieFlag.actReqSent)}, " +
                s"reqRcv=${flag(h, i, DieFlag.actReqRcvd)}). ${stateSummary(h)}")
          }
        }
      }
      stagePassed += 4
    }

    it("U5: each protocol layer reports its receiver alive (lp_rx_active_sts)") {
      gateOnLowerRungs(5)
      simulate(new UcieDigitalLoopbackHarness(), firtoolOpts = noAssertFirtoolOpts) { h =>
        coldStart(h)
        climbTo(h, 5)

        for (i <- 0 until 2) {
          assert(!flag(h, i, DieFlag.rxOverflow),
            s"die $i asserted lp_rx_active_sts with rxOverflow already latched -- the protocol " +
              s"RX queue filled before a single beat was ever sent. See U8's blocker. " +
              s"${stateSummary(h)}")
          assert(flag(h, i, DieFlag.fdiRxActiveReq),
            s"die $i asserted lp_rx_active_sts while pl_rx_active_req is low; the handshake must " +
              s"FOLLOW the request, not lead it (ProtocolStateController.scala:84-110). " +
              s"${stateSummary(h)}")
        }
        // NOTE deliberately no "the FDI must still be reset here" check:
        // RSP_ACTIVE can complete within one polling stride of this milestone,
        // so such a check would be a race, not a specification.
      }
      stagePassed += 5
    }

    it("U6: both dies reach FDI active") {
      gateOnLowerRungs(6)
      simulate(new UcieDigitalLoopbackHarness(), firtoolOpts = noAssertFirtoolOpts) { h =>
        coldStart(h)
        climbTo(h, 6)

        for (i <- 0 until 2) {
          h.io.fdiState(i).expect(FDIState.active, "the FDI must reach active")
          assert(flag(h, i, DieFlag.fdiProtocolVld),
            s"die $i is in FDI active without pl_protocol_vld (D2DAdapter.scala:49-52). " +
              s"${stateSummary(h)}")
          assert(flag(h, i, DieFlag.fdiInbandPres),
            s"die $i dropped FDI inband presence on the way to active. ${stateSummary(h)}")
          assert(flag(h, i, DieFlag.rdiLpReqActive),
            s"die $i is in FDI active while asking its PHY for something other than active. " +
              s"${stateSummary(h)}")
          assert(!flag(h, i, DieFlag.fdiLpReqActive),
            s"die $i is in FDI active but the protocol layer is STILL presenting " +
              s"lp_state_req = active. The request must fall back to nop once pl_state_sts " +
              s"leaves reset, or the edge can never be re-armed for a second bring-up and a " +
              s"stale level kicks any later linkReset/disabled straight back to reset " +
              s"(AdapterSM.scala:530-531,:540-541). ${stateSummary(h)}")
        }
      }
      stagePassed += 6
    }

    it("U7: the chip-facing interface opens and the link holds over a dwell") {
      gateOnLowerRungs(7)
      simulate(new UcieDigitalLoopbackHarness(), firtoolOpts = noAssertFirtoolOpts) { h =>
        coldStart(h)
        climbTo(h, 7)

        assertLinkClean(h, "U7 at entry")
        h.clock.step(dwellCycles)
        traceStates(h, s"U7 dwell +$dwellCycles cycles")
        assertLinkClean(h, s"U7 after a $dwellCycles-cycle dwell")
        for (i <- 0 until 2) {
          assert(flag(h, i, DieFlag.chipTxReady),
            s"die $i closed its chip-facing TX interface while idling in active. " +
              s"${stateSummary(h)}")
          assert(!flag(h, i, DieFlag.fdiPlValid),
            s"die $i presents FDI pl_valid while no beat has ever been sent. ${stateSummary(h)}")
          assert(!flag(h, i, DieFlag.fdiStallReq),
            s"die $i has FDI pl_stallreq asserted while idling in active. ${stateSummary(h)}")
        }
      }
      stagePassed += 7
    }

    it("U8: carries one protocol beat in each direction, exactly once") {
      gateOnLowerRungs(8)
      // The first rung that inspects beat data, and therefore the only one below
      // U9 that pays for the 512-bit pack/unpack being live -- see the
      // exposeDataPath note in UcieDigitalLoopbackHarness (measured 7.1x).
      simulate(
        new UcieDigitalLoopbackHarness(exposeDataPath = true),
        firtoolOpts = noAssertFirtoolOpts) { h =>
        coldStart(h)
        climbTo(h, 8)

        val sent = Seq(Seq(payload(h.beatBits, 0, 0)), Seq(payload(h.beatBits, 1, 0)))
        val got = exchange(h, sent)
        checkDelivery(h, from = 0, sent = sent(0), received = got(1))
        checkDelivery(h, from = 1, sent = sent(1), received = got(0))

        for (i <- 0 until 2) {
          assert(!flag(h, i, DieFlag.fdiPlValid),
            s"die $i still holds FDI pl_valid after the beat was delivered. It must be a " +
              s"one-cycle pulse per beat: D2DMainbandModule.scala:139-144 only clears " +
              s"dataBuffRcvFillReg when rx_active_req drops, which never happens in ACTIVE, so " +
              s"it latches high forever and the protocol layer re-enqueues the same beat every " +
              s"cycle. ${stateSummary(h)}")
        }
        assertLinkClean(h, "U8 after one beat each way")
      }
      stagePassed += 8
    }

    it("U9: carries bursts in both directions simultaneously, in order and byte exact") {
      gateOnLowerRungs(9)
      simulate(
        new UcieDigitalLoopbackHarness(exposeDataPath = true),
        firtoolOpts = noAssertFirtoolOpts) { h =>
        coldStart(h)
        climbTo(h, 9)

        // Both dies transmit on the same cycles: each die's scrambler and the
        // peer's descrambler advance together in BOTH directions at once, which
        // a one-way test cannot reach, and the queues see back-to-back beats
        // instead of one isolated beat.
        val sent = Seq(
          (0 until burstLength).map(payload(h.beatBits, 0, _)),
          (0 until burstLength).map(payload(h.beatBits, 1, _)),
        )
        val got = exchange(h, sent)
        checkDelivery(h, from = 0, sent = sent(0), received = got(1))
        checkDelivery(h, from = 1, sent = sent(1), received = got(0))

        assertLinkClean(h, s"U9 after $burstLength beats each way")
      }
      stagePassed += 9
    }

    it("U10: carries the adapters' own sideband traffic without latching a fault") {
      gateOnLowerRungs(10)
      simulate(new UcieDigitalLoopbackHarness(), firtoolOpts = noAssertFirtoolOpts) { h =>
        coldStart(h)
        climbTo(h, 10)

        for (i <- 0 until 2) {
          val latched = DieFlag.sbFaults.collect { case (b, n) if flag(h, i, b) => n }
          assert(latched.isEmpty,
            s"[U10] die $i reached FDI active but latched sideband fault bit(s) " +
              s"${latched.mkString(", ")} while carrying the adapters' link-init traffic. " +
              s"LogPhyStagedBringupTest S7 asserts the same cleanliness over the same training " +
              s"and passes, so this is the D2D traffic, not training. ${ladder(10).blocker}. " +
              s"Observed: ${stateSummary(h)}")
        }
      }
      stagePassed += 10
    }
  }
}
