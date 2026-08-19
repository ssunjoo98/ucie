package edu.berkeley.cs.uciedigital.loopback

import edu.berkeley.cs.uciedigital.logphy._

import chisel3._
import chisel3.simulator.HasSimulator
import edu.berkeley.cs.uciedigital.UcieSimBackend
import chisel3.simulator.scalatest.ChiselSim
import org.scalatest.funspec.AnyFunSpec
import edu.berkeley.cs.uciedigital.interfaces._

/*
  Run:
    rm -rf build/chiselsim/LogPhyStagedBringupTest   # stale svsim workdirs produce broken binaries
    ./mill --no-daemon test.testOnly edu.berkeley.cs.uciedigital.loopback.LogPhyStagedBringupTest
    # one rung only (ScalaTest name filter):
    ./mill --no-daemon test.testOnly edu.berkeley.cs.uciedigital.loopback.LogPhyStagedBringupTest -- -z "S1"

  Waveforms (for DVE / Verdi):
    ./mill --no-daemon test.testOnly edu.berkeley.cs.uciedigital.loopback.LogPhyStagedBringupTest -- -DemitVcd=1
    # Same switch as SidebandChannelRandomTest.scala:26. It is a ChiselSim
    # built-in (chisel3.simulator.scalatest.Cli.EmitVcd is already mixed into the
    # ChiselSim trait), so no code change is needed to enable it.
    #
    # One VCD per rung that actually RUNS (skipped rungs produce none), written
    # into that rung's own svsim workdir:
    #
    #   scala/build/chiselsim/LogPhyStagedBringupTest/
    #     <describe-slug>/<rung-slug>/workdir-verilator/trace.vcd
    #
    # <describe-slug> / <rung-slug> are the describe(...) / it(...) strings with
    # every non-alphanumeric character replaced by '-' (dots survive), e.g.
    #   .../LogicalPhy-staged-bring-up-ladder--reset----active--one-gate-per-rung-
    #       /S0--enters-sSBINIT-on-the-software-training-trigger
    #       /workdir-verilator/trace.vcd
    # Do not hand-derive the slug; list them:
    #   find scala/build/chiselsim/LogPhyStagedBringupTest -name trace.vcd
    #
    # DVE opens VCD directly (File > Open Database ...), or convert first:
    #   vcd2vpd trace.vcd trace.vpd && dve -vpd trace.vpd &
    #
    # WARNING: tracing is armed at time zero, so a rung's VCD covers the whole
    # 3.2M-cycle RESET wait for TWO complete LogicalPhy instances -- expect a
    # multi-GB file. Dump ONE rung at a time (combine -DemitVcd=1 with -z), set
    # `windowWaves` below to cut the idle RESET wait out of the dump, and prefer
    # the terminal trace (`debugTrace`) first: in practice the interesting window
    # is the few thousand cycles right after the trigger.
    # `ltState` / `ltsmState` are harness outputs, so they are visible at the top
    # level and are also printed by the terminal trace.

  Purpose:
    This is a SPECIFICATION ladder, not a characterization suite. Every rung
    asserts "link training must have reached this milestone" and FAILS when it
    has not. It was written while most rungs were red, as a work queue; now that
    the ladder is green it is the regression gate -- a rung going red means a
    fix regressed, and that rung's `blocker` string says where to look.


  The ladder (LTState chain sRESET -> sSBINIT -> sMBINIT -> sMBTRAIN ->
  sLINKINIT -> sACTIVE, split at every externally observable milestone, plus one
  rung above training for the data path):

    S0  ASYMMETRIC start    -> both dies in sSBINIT
    S1  SBINIT completes    -> both dies in sMBINIT
    S2  MBINIT.PARAM        -> negotiatedParamsValid on both dies
    S3  MBINIT completes    -> both dies in sMBTRAIN
    S4  MBTRAIN completes   -> both dies in sLINKINIT
    S5  sLINKINIT           -> plInbandPres on both dies
    S6  RDI bring-up        -> plStateSts == RDIState.active on both dies
    S7  both dies in sACTIVE
    S8  RDI data crosses the link, both directions, byte exact

  Skip-ahead (why you should see CANCELED, not seven identical failures):
    ScalaTest runs each it(...) independently, so without help every rung above
    the current blocker would pay its own 3.2M-cycle cold start only to fail for
    exactly the same reason. Each rung therefore checks, BEFORE calling
    simulate(...), whether a lower rung already ran and did not pass; if so it
    calls ScalaTest's cancel(...) and costs nothing (no elaborate, no verilate,
    no simulation). ScalaTest counts those as CANCELED, which is the honest
    status: "never attempted", as opposed to FAILED = "attempted and the RTL did
    not get there". One failure plus N cancellations is a much better progress
    gauge than N+1 failures.

    Two consequences to know about:
      * This suite assumes SEQUENTIAL, declaration-order execution. AnyFunSpec
        does that by default (no ParallelTestExecution is mixed in here) and
        build.mill sets testParallelism = false. Do NOT add -P (ScalaTest
        parallel execution) or the gate becomes order-dependent noise.
      * The gate only fires for rungs that actually RAN and did not pass, so
        `-z "S4"` in isolation still runs S4 for real (nothing below it ran, so
        there is nothing to be blocked by).

  Ladder status MEASURED 2026-08-13, after seventeen RTL defects were fixed
  (see docs_personal/FIX_LOG.md). S0..S7 all pass; S8 was added the same day and
  has not been measured yet. Timings are cycles spent in that rung's own climb,
  on top of the cold start every rung pays.

    S0  PASS       ASYMMETRIC start: die 0 triggered, die 1 woke in 167 cycles.
    S1  PASS       SBINIT completes -> sMBINIT; 1920 cycles.
    S2  PASS       MBINIT.PARAM interoperability succeeds; +384 cycles. Both
                   dies are in sMBINIT_REPAIRCLK by the time paramsVld rises --
                   sCAL is skipped in one cycle, see S3's blocker note.
    S3  PASS       MBINIT completes -> sMBTRAIN; +3840 cycles.
    S4  PASS       MBTRAIN completes -> sLINKINIT. Seven fixes were needed here.
    S5  PASS       plInbandPres asserted on sLINKINIT entry.
    S6  PASS       RDI reaches active with no lpStateReq poke.
    S7  PASS       both dies in sACTIVE, no sideband fault latched anywhere in
                   training, link held over a 20000-cycle dwell.
    S8  NEW        RDI data round trip; see the rung for what it drives.

  A rung that goes red now is a REGRESSION. Read its blocker string first: each
  one names the defects that used to live there and what to re-check.

  How a failure reads:
    Each rung reports which milestone was missed, how long it waited, the full
    observed state of both dies, whether the miss was the rung under test or a
    lower prerequisite, and the known defect that explains it. A blocked rung
    stops at the LOWEST unmet milestone, so it costs the 3.2M-cycle RESET wait
    plus the guard of the first broken rung -- never the sum of all guards.
    Turn on `debugTrace` to also print a two-die state dump as each rung is
    reached or given up on, so the terminal shows the ladder climbing.

  Observability:
    Mostly the harness's coarse status ports (ltState, ltsmState,
    trainingTimedout, negotiatedParamsValid, plStateSts, plInbandPres,
    plTrainError, plPhyInRecenter, plSpeedmode, sbFaultSeen). Be careful adding
    more: extra observation ports have made the generated Verilator model
    segfault at time zero during VerilatedScope registration
    (LogPhyLoopbackHarness.scala:73-78). The RDI data ports S8 needs are the one
    exception, and they are elaborated only for S8 -- left connected they cost
    every other rung several times its wall clock, because the transmit pack and
    receive unpack stop folding away. Anything finer than these signals --
    substate progress, sideband message contents, credit counts -- has to come
    from a waveform.

  Cost (measured 2026-08-13):
    Every rung that RUNS pays the real 3.2M-cycle RESET minimum wait
    (LinkTrainingSM.scala:108-116) from a cold start; the harness exposes no way
    to shortcut it. All eight training rungs green: 3 min 37 s wall clock, i.e.
    roughly 27 s per rung, each rung a separate elaborate+verilate pass.
    A blocked run is cheaper, not dearer: rungs above the blocker cancel before
    they elaborate. To iterate on one rung use the -z filter rather than merging
    rungs -- merging would destroy the "one gate per test" property.
*/
class LogPhyStagedBringupTest extends AnyFunSpec with ChiselSim {

  // Hardware constants (LinkTrainingSM.scala:87-116): 800 MHz * 8 ms residency
  // timeout; the RESET minimum wait is half of it. Not scalable from outside.
  private val hwTimeoutCycles = 6400000
  private val hwResetWait = hwTimeoutCycles / 2

  // Both settings are mandatory. The Verification layers must be stripped (the
  // SBINIT pattern flood trips SidebandDeserializerDoesNotDropWords,
  // SidebandLinkSerdes.scala:295-299), and the layer-stripped build then needs
  // x-initial=fast or it segfaults at time zero in Verilator's scoped
  // randomization ctor.
  private val noAssertFirtoolOpts = Array(
    "--disable-layers=Verification,Verification.Assert,Verification.Assume,Verification.Cover")

  // Verilator by default, tuned for speed: every rung pays a 3.2M-cycle reset
  // from a cold start, so the -O3 model is worth a lot here. UCIE_SIM_BACKEND=vcs
  // switches to the coverage backend instead -- without this the ladder would
  // stay on Verilator during a coverage run and contribute no vdb at all, which
  // is easy to miss because the run still passes.
  implicit private val ucieSimulator: HasSimulator =
    if (sys.env.get("UCIE_SIM_BACKEND").contains("vcs")) UcieSimBackend.fromEnv
    else
      HasSimulator.simulators.verilator(
        svsim.CommonCompilationSettings(
          optimizationStyle =
            svsim.CommonCompilationSettings.OptimizationStyle.OptimizeForSimulationSpeed))

  /** Print a two-die state dump as each rung is reached or given up on.
    * Terminal output only, always safe to turn on.
    */
  private val debugTrace = false

  /** Trim the VCD down to the part that matters: `runResetWait` calls
    * `disableWaves()` for the idle 3.2M-cycle RESET wait and `enableWaves()`
    * right before the trigger, so the dump starts where the FSMs begin to move.
    *
    * MUST stay false unless the run also passes `-DemitVcd=1`. Without that flag
    * the model is compiled with no trace support and the first disable/enable
    * call aborts every rung at its first clock step ("Cannot enable traces as
    * simulator was not compiled to support them", svsim simulation-driver.cpp),
    * which reads like an RTL failure but is not. ChiselSim exposes no way to
    * query the flag, so the two cannot be tied together automatically. Combine
    * with `-z` so only the rung under investigation dumps.
    */
  private val windowWaves = false

  // ============================================================================
  // Guards. Every wait in this file is bounded; nothing may hang the simulator.
  // A guard is only ever paid in full when its rung FAILS, so the cheap rungs
  // are kept tight and the sideband/mainband rungs get generous headroom
  // (still far below the 6.4M-cycle substate residency timeout, so a rung
  // failing here means "stuck", not "the RTL would have made it eventually").
  // ============================================================================
  // S0 now covers an ASYMMETRIC start (see coldStart), so its guard has to cover
  // the sleeper's wake-up, not just the triggered die's own transition.
  // measured 2026-08-06: triggered die <=8 cycles, sleeper 192 cycles.
  private val sbinitEntryGuard = 4096
  private val sbHandshakeGuard = 400000    // a handful of 64-bit sideband exchanges
  private val mbInitGuard = 800000         // CAL + REPAIRCLK/VAL + REVERSALMB + REPAIRMB
  private val mbTrainGuard = 1500000       // 13 substates, several pattern/eye sweeps
  private val rdiFlagGuard = 4096          // plain registers, one or two cycles

  // ============================================================================
  // Decoding helpers (names for the log; ranks for monotone milestone checks)
  //
  // All four tables are `lazy val` on purpose. Building them touches ChiselEnum
  // literals (LTState.sRESET.litValue etc.), and a plain val would do that while
  // ScalaTest is merely CONSTRUCTING the suite -- outside any Chisel elaboration
  // or simulation context. Laziness defers the first touch to dieSummary/rankOf,
  // which only ever run inside a simulate(...) body.
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

  private lazy val ltsmNames: Map[BigInt, String] = Map(
    LTSMState.sRESET.litValue -> "sRESET",
    LTSMState.sSBINIT.litValue -> "sSBINIT",
    LTSMState.sMBINIT_PARAM.litValue -> "sMBINIT_PARAM",
    LTSMState.sMBINIT_CAL.litValue -> "sMBINIT_CAL",
    LTSMState.sMBINIT_REPAIRCLK.litValue -> "sMBINIT_REPAIRCLK",
    LTSMState.sMBINIT_REPAIRVAL.litValue -> "sMBINIT_REPAIRVAL",
    LTSMState.sMBINIT_REVERSALMB.litValue -> "sMBINIT_REVERSALMB",
    LTSMState.sMBINIT_REPAIRMB.litValue -> "sMBINIT_REPAIRMB",
    LTSMState.sMBTRAIN_VALVREF.litValue -> "sMBTRAIN_VALVREF",
    LTSMState.sMBTRAIN_DATAVREF.litValue -> "sMBTRAIN_DATAVREF",
    LTSMState.sMBTRAIN_SPEEDIDLE.litValue -> "sMBTRAIN_SPEEDIDLE",
    LTSMState.sMBTRAIN_TXSELFCAL.litValue -> "sMBTRAIN_TXSELFCAL",
    LTSMState.sMBTRAIN_RXCLKCAL.litValue -> "sMBTRAIN_RXCLKCAL",
    LTSMState.sMBTRAIN_VALTRAINCENTER.litValue -> "sMBTRAIN_VALTRAINCENTER",
    LTSMState.sMBTRAIN_VALTRAINVREF.litValue -> "sMBTRAIN_VALTRAINVREF",
    LTSMState.sMBTRAIN_DATATRAINCENTER1.litValue -> "sMBTRAIN_DATATRAINCENTER1",
    LTSMState.sMBTRAIN_DATATRAINVREF.litValue -> "sMBTRAIN_DATATRAINVREF",
    LTSMState.sMBTRAIN_RXDESKEW.litValue -> "sMBTRAIN_RXDESKEW",
    LTSMState.sMBTRAIN_DATATRAINCENTER2.litValue -> "sMBTRAIN_DATATRAINCENTER2",
    LTSMState.sMBTRAIN_LINKSPEED.litValue -> "sMBTRAIN_LINKSPEED",
    LTSMState.sMBTRAIN_REPAIR.litValue -> "sMBTRAIN_REPAIR",
    LTSMState.sPHYRETRAIN.litValue -> "sPHYRETRAIN",
    LTSMState.sLINKINIT.litValue -> "sLINKINIT",
    LTSMState.sACTIVE.litValue -> "sACTIVE",
    LTSMState.sTRAINERROR.litValue -> "sTRAINERROR",
    LTSMState.sL1_L2.litValue -> "sL1_L2",
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

  /** Forward progress rank along the bring-up path. Off-path states
    * (sPHYRETRAIN / sTRAINERROR / sL1_L2) deliberately have NO rank: they are
    * not "further along", they are a derailment, and every milestone here is
    * expressed as "rank >= n" so it can never be satisfied by a derailed FSM.
    * Ranks also make the milestones monotone, so a rung cannot be missed by
    * polling with a coarse stride while the FSM races past it.
    */
  private lazy val ltRank: Map[BigInt, Int] = Map(
    LTState.sRESET.litValue -> 0,
    LTState.sSBINIT.litValue -> 1,
    LTState.sMBINIT.litValue -> 2,
    LTState.sMBTRAIN.litValue -> 3,
    LTState.sLINKINIT.litValue -> 4,
    LTState.sACTIVE.litValue -> 5,
  )

  private def ltCode(h: LogPhyLoopbackHarness, i: Int): BigInt =
    h.io.ltState(i).peek().litValue

  private def ltsmCode(h: LogPhyLoopbackHarness, i: Int): BigInt =
    h.io.ltsmState(i).peek().litValue

  private def rdiCode(h: LogPhyLoopbackHarness, i: Int): BigInt =
    h.io.plStateSts(i).peek().litValue

  private def rankOf(h: LogPhyLoopbackHarness, i: Int): Int =
    ltRank.getOrElse(ltCode(h, i), -1)

  private def offPath(h: LogPhyLoopbackHarness): Boolean =
    (0 until 2).exists(i => rankOf(h, i) < 0)

  private def bothDies(cond: Int => Boolean): Boolean = cond(0) && cond(1)

  /** Both dies have reached at least the given point on the bring-up path. */
  private def reachedAtLeast(h: LogPhyLoopbackHarness, r: Int): Boolean =
    bothDies(i => rankOf(h, i) >= r)

  // ============================================================================
  // Logging
  // ============================================================================

  private def dieSummary(h: LogPhyLoopbackHarness, i: Int): String = {
    val lt = ltNames.getOrElse(ltCode(h, i), s"?0x${ltCode(h, i).toString(16)}")
    val ltsm = ltsmNames.getOrElse(ltsmCode(h, i), s"?0x${ltsmCode(h, i).toString(16)}")
    val rdi = rdiNames.getOrElse(rdiCode(h, i), s"?0x${rdiCode(h, i).toString(16)}")
    s"die$i{lt=$lt ltsm=$ltsm rdi=$rdi " +
      s"paramsVld=${h.io.negotiatedParamsValid(i).peekBoolean()} " +
      s"inbandPres=${h.io.plInbandPres(i).peekBoolean()} " +
      s"recenter=${h.io.plPhyInRecenter(i).peekBoolean()} " +
      s"trainErr=${h.io.plTrainError(i).peekBoolean()} " +
      s"timedout=${h.io.trainingTimedout(i).peekBoolean()} " +
      s"sbFault=${h.io.sbFaultSeen(i).peekBoolean()}}"
  }

  private def stateSummary(h: LogPhyLoopbackHarness): String =
    s"${dieSummary(h, 0)} | ${dieSummary(h, 1)}"

  private def traceStates(h: LogPhyLoopbackHarness, tag: String): Unit =
    if (debugTrace) println(s"[staged] $tag :: ${stateSummary(h)}")

  // ============================================================================
  // Stimulus
  // ============================================================================

  /** Drive every harness input to a quiet default before a scenario starts. */
  private def initHarness(h: LogPhyLoopbackHarness, pwrGood: Boolean = true): Unit = {
    for (i <- 0 until 2) {
      h.io.lpStateReq(i).poke(RDIStateReq.nop)
      h.io.swStartLinkTraining(i).poke(false.B)
      h.io.pwrGood(i).poke(pwrGood.B)
      h.io.lpValid.foreach(_(i).poke(false.B))
      h.io.lpIrdy.foreach(_(i).poke(false.B))
      h.io.lpData.foreach(_(i).poke(0.U))
    }
  }

  /** Sit out the hardware RESET minimum wait (3.2M cycles) plus slack,
    * checking RESET-residency invariants before and after.
    */
  private def runResetWait(h: LogPhyLoopbackHarness): Unit = {
    // Waveform window: the 3.2M-cycle RESET wait is the bulk of the run but
    // carries no information (nothing moves until the trigger). With
    // `windowWaves` on, tracing is switched off for it and switched back on
    // just before the trigger, which is where every interesting transition
    // happens. See the `windowWaves` declaration for the usage contract.
    if (windowWaves) disableWaves()

    h.clock.step(64)
    for (i <- 0 until 2) {
      h.io.ltsmState(i).expect(LTSMState.sRESET, "LTSM must start in RESET")
      h.io.plStateSts(i).expect(RDIState.reset, "RDI must start in reset")
      h.io.plInbandPres(i).expect(false.B, "no inband presence in RESET")
    }
    // Chunked so a hung simulator is diagnosable from the log.
    for (chunk <- 1 to 4) {
      h.clock.step(hwResetWait / 4)
      if (debugTrace) println(s"[staged] reset wait chunk $chunk/4 done")
    }
    h.clock.step(64)
    for (i <- 0 until 2) {
      h.io.ltsmState(i).expect(LTSMState.sRESET, "no training without a trigger")
    }

    // Re-arm tracing: from here on every cycle is worth keeping.
    if (windowWaves) enableWaves()
  }

  /** Cold start that triggers ONE die only, leaving the other in sRESET.
    *
    * The sleeper is supposed to wake by itself: while in sRESET it keeps its
    * sideband receiver enabled in RAW mode and counts incoming clock patterns
    * (LinkTrainingSM.scala:1128-1140); two of them raise remoteTriggerTraining
    * (:216), which is an equal partner of swTriggerTraining in the trigger OR
    * (:243).
    */
  private def coldStartOneDie(h: LogPhyLoopbackHarness, die: Int): Unit = {
    initHarness(h)
    runResetWait(h)
    h.io.swStartLinkTraining(die).poke(true.B)
    h.clock.step(4)
    h.io.swStartLinkTraining(die).poke(false.B)
  }

  /** The cold start shared by every rung.
    *
    * ASYMMETRIC ON PURPOSE: only die 0 gets the software trigger. Die 1 has to
    * wake on the clock pattern die 0 transmits.
    *
    * Two chiplets have no reason to leave RESET on the same cycle, and spec
    * 4.5.3 step 1 treats the staggered case as the normal one -- that is the
    * whole reason SBINIT sends four extra pattern iterations after detection
    * ("to allow for any time differences in both UCIe Module and UCIe Module
    * Partner coming out of RESET state"). A testbench that pulses both dies
    * together is the ONE arrival order the hardware never has to handle, and it
    * hides every bug in the remote-wake path -- which is most of the sideband
    * RX datapath.
    *
    * So the ladder starts asymmetric and stays asymmetric. There is no separate
    * "asymmetric start" suite any more; if the remote-wake path breaks, S0 goes
    * red and everything above it cancels, which is the correct signal.
    *
    * Die 0 is the starter by convention. `coldStartOneDie(h, 1)` covers the
    * mirror image; the harness is symmetric (LogPhyLoopbackHarness.scala:92-103
    * builds both dies from the same generator and crosses them straight over),
    * so the mirror adds no coverage and is deliberately not run on every rung.
    */
  private def coldStart(h: LogPhyLoopbackHarness): Unit = coldStartOneDie(h, 0)

  private case class WaitResult(ok: Boolean, cycles: Int, derailed: Boolean)

  /** Step in `stride`-cycle chunks until `cond` holds, the guard expires, or a
    * die derails off the bring-up path (sTRAINERROR / sPHYRETRAIN / sL1_L2).
    * Always bounded: the guard is a hard ceiling, so no rung can hang the
    * simulator. Derailment ends the wait early -- a die in sTRAINERROR is never
    * going to satisfy a forward milestone, and burning the rest of the guard
    * would only slow the run down.
    */
  private def stepUntil(
      h: LogPhyLoopbackHarness,
      guard: Int,
      stride: Int,
  )(cond: => Boolean): WaitResult = {
    var n = 0
    var derailed = false
    while (!cond && !derailed && n < guard) {
      h.clock.step(stride)
      n += stride
      derailed = offPath(h)
    }
    WaitResult(cond, n, derailed && !cond)
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
      reached: LogPhyLoopbackHarness => Boolean,
  )

  private val ladder: Seq[Stage] = Seq(
    Stage(
      id = "S0",
      what = "die 0 takes the software trigger and die 1 wakes on the remote clock " +
        "pattern, so BOTH dies reach sSBINIT from a staggered start",
      guard = sbinitEntryGuard,
      stride = 1,
      blocker =
        "if only die 0 moved, the software trigger works but the REMOTE-WAKE path does " +
          "not: die 1 stays in sRESET because its sideband RX never counted two clock " +
          "patterns (LinkTrainingSM.scala:1128-1140 feeds sbInitPatternCounter, :216 turns " +
          "2 of them into remoteTriggerTraining). That was D-01's second symptom and it is " +
          "the reason this rung is asymmetric -- pulsing both dies together hides it " +
          "completely. If NEITHER die moved, the plain software trigger itself is broken " +
          "(LinkTrainingSM.scala:240, :1142-1143), which is a different and much more " +
          "basic failure",
      reached = (h: LogPhyLoopbackHarness) => reachedAtLeast(h, 1),
    ),
    Stage(
      id = "S1",
      what = "SBINIT completes (clock pattern detected, out-of-reset + done handshakes) " +
        "and both dies enter sMBINIT",
      guard = sbHandshakeGuard,
      stride = 64,
      blocker =
        "was D-01, sideband clock-pattern misrouting: SidebandSwitch.scala:56 routes on " +
          "getDstLayer = msg(57,56), and a raw clock-pattern word 0x5555_5555_5555_5555 " +
          "has 0b01 there, so the logPHY switch read it as D2D and streamed the pattern up " +
          "the RDI cfg path instead of to the LTSM. Fixed by bypassing the switch for RAW " +
          "words (LogPhySidebandChannel.scala). A regression here means that bypass, or " +
          "D-11 (parity must not be checked in RAW mode, SidebandLinkNode.scala:131). " +
          "sbFault=true with the dies stuck in sSBINIT is the D-01 signature: cfg-credit " +
          "exhaustion followed by an RX priority-queue overflow",
      reached = (h: LogPhyLoopbackHarness) => reachedAtLeast(h, 2),
    ),
    Stage(
      id = "S2",
      what = "MBINIT.PARAM exchanges and accepts PHY parameters on both dies " +
        "(negotiatedParamsValid)",
      guard = sbHandshakeGuard,
      stride = 64,
      blocker =
        "none known. Both dies advertise identical settings (maxDataRate=speed4, x16, " +
          "LogPhyLoopbackHarness.scala), so interoperability must succeed " +
          "(MBInitSM.scala:136-145); a failure here is a new defect in the PARAM req/rsp " +
          "encoding or in the interoperability compare. Note the compare uses a bitwise " +
          "AND on maxDataRate, which is not a minimum -- harmless only because both dies " +
          "advertise the same speed here",
      reached = (h: LogPhyLoopbackHarness) =>
        bothDies(i => h.io.negotiatedParamsValid(i).peekBoolean()),
    ),
    Stage(
      id = "S3",
      what = "MBINIT completes (CAL, REPAIRCLK, REPAIRVAL, REVERSALMB, REPAIRMB) and " +
        "both dies enter sMBTRAIN",
      guard = mbInitGuard,
      stride = 64,
      blocker =
        "five defects lived here. A derail to sTRAINERROR with timedout=false points at " +
          "the pattern path: REPAIRCLK's only asserted error is MBInitSM.scala:537 " +
          "(resp.valid && !repairClkSuccess), fed by the remote PatternReader's status " +
          "bits, so re-check the reader's mbRxValid gating (its phase counter must count " +
          "words, not clocks) before anything else. A silent park instead means a " +
          "handshake: REVERSALMB needs its reader started inside the exit gate and its " +
          "DONE substate needs a responderRdy pairing; REPAIRMB needs 16-bit msgInfo " +
          "everywhere (a 15-bit field shifts the header so the SENDER's own switch drops " +
          "the packet) and its width-degrade Mux1H gated on an actual degrade condition. " +
          "sbFault with the ladder otherwise green means a DONE exchange completed without " +
          "waiting for its response (SidebandMessageExchanger clear/set ordering). " +
          "NOTE MBINIT.CAL is SKIPPED, not run: the sticky ready regs (MBInitSM.scala:255-263) " +
          "survive the sPARAM->sCAL transition because the 'set' when() is written after " +
          "the 'clear' when() and last-connect wins, so sCAL lasts one cycle with no " +
          "CAL.DONE exchange. That masks PhyLaneTrainer's selfCalDone := false.B. Fixing " +
          "the sticky ready UNMASKS it and will stall here -- the two must be fixed together",
      reached = (h: LogPhyLoopbackHarness) => reachedAtLeast(h, 3),
    ),
    Stage(
      id = "S4",
      what = "MBTRAIN completes (all 13 substates) and both dies enter sLINKINIT",
      guard = mbTrainGuard,
      stride = 64,
      blocker =
        "the densest rung: seven defects, and the ltsm value in the dump says which. " +
          "sMBTRAIN_VALVREF means PhyLaneTrainer stopped completing -- eight of the " +
          "thirteen states leave s1 only via trainingCtrl.req.start or .complete " +
          "(MBTrainSM.scala:521-526), and this PHY has no calibration hardware, so the " +
          "trainer completes immediately. sMBTRAIN_SPEEDIDLE means the responder's DONE " +
          "exchange: it must arm the SPEEDIDLE names (not LINKSPEED) AND require its own " +
          "exchDone, because that state has no substates and inherits both sticky ready " +
          "bits already set. sMBTRAIN_RXCLKCAL means rxClkCalDone stopped pulsing -- unlike " +
          "TXSELFCAL, which both halves leave together via the rendezvous, RXCLKCAL.s3 is " +
          "entered by a substate change that clears the sticky bit, so it needs a real " +
          "done. sMBTRAIN_LINKSPEED is the only state that runs a real mainband test " +
          "(TxD2CPointTest with LFSR, ungated by the trainer): a wedge there is the " +
          "responder holding rx.ready low across the state, and a spurious width degrade " +
          "is the lane-result polarity (reader bits are PASS flags) or the zero-hot Mux1H. " +
          "Leave the sticky-ready race alone -- MBINIT.CAL and MBTRAIN.TXSELFCAL depend on " +
          "it to skip silently. Speed stays speed4, which dodges the unimplemented " +
          ">32 GT/s error paths (MBTrainSM.scala:736-740, :1119-1124)",
      reached = (h: LogPhyLoopbackHarness) => reachedAtLeast(h, 4),
    ),
    Stage(
      id = "S5",
      what = "both dies assert plInbandPres on sLINKINIT entry",
      guard = rdiFlagGuard,
      stride = 8,
      blocker =
        "none known -- inbandPresent is a plain register set whenever ltState is " +
          "sLINKINIT or sACTIVE (RDIController.scala:150-157), so this rung can only fail " +
          "if S4 was reached and the flag still did not follow within its guard",
      reached = (h: LogPhyLoopbackHarness) => bothDies(i => h.io.plInbandPres(i).peekBoolean()),
    ),
    Stage(
      id = "S6",
      what = "the RDI state machine completes bring-up: plStateSts == RDIState.active on " +
        "both dies",
      guard = sbHandshakeGuard,
      stride = 64,
      blocker =
        "none known -- no lpStateReq poke is needed: RDIController.scala:43-45 forces " +
          "RDIStateReq.active while ltState is sLINKINIT, and the harness already parks " +
          "lpStateReq at nop during RESET so resetReqObserved is set " +
          "(RDIStateMachine.scala:47-53). The ACTIVE req/rsp travels the same sideband " +
          "path as SBINIT, so a stall here after S1 is fixed points at the RDI " +
          "requester/responder pair (RDIStateMachine.scala:255-258, :349-401)",
      reached = (h: LogPhyLoopbackHarness) =>
        bothDies(i => rdiCode(h, i) == RDIState.active.litValue),
    ),
    Stage(
      id = "S7",
      what = "both dies reach sACTIVE -- the link is up",
      guard = rdiFlagGuard,
      stride = 8,
      blocker =
        "none known -- LinkTrainingSM.scala:1264 leaves sLINKINIT for sACTIVE as soon as " +
          "plStateSts is active, so this rung can only fail if S6 was reached and the " +
          "LTSM still did not follow within its guard",
      reached = (h: LogPhyLoopbackHarness) => reachedAtLeast(h, 5),
    ),
    Stage(
      id = "S8",
      what = "the RDI accepts data: plTrdy is high on both dies in sACTIVE",
      guard = rdiFlagGuard,
      stride = 8,
      blocker =
        "plTrdy is gated on rdiStateSts == active (LogicalPhy.scala:474-478) and inside " +
          "the controller on !txBusy && mbLanes.tx.ready " +
          "(MainbandLaneController.scala:186). tx.ready comes from the peer's rx.ready, " +
          "which the controller ties high in sACTIVE (:239), so a stall here means the " +
          "RDI state or the isActive gate, not the data path",
      reached = (h: LogPhyLoopbackHarness) => bothDies(i => h.io.plTrdy(i).peekBoolean()),
    ),
  )

  private def stageFailure(
      h: LogPhyLoopbackHarness,
      stuckIdx: Int,
      targetIdx: Int,
      res: WaitResult,
  ): String = {
    val st = ladder(stuckIdx)
    val target = ladder(targetIdx)
    val why =
      if (res.derailed)
        s"gave up after ${res.cycles} cycles because a die derailed off the bring-up path " +
          "(sTRAINERROR / sPHYRETRAIN / sL1_L2 have no forward rank, so no milestone above " +
          "them can ever be met)"
      else
        s"not reached within the ${st.guard}-cycle guard (waited ${res.cycles}, polled every " +
          s"${st.stride})"
    val prereq =
      if (stuckIdx < targetIdx)
        s" This rung is a PREREQUISITE for ${target.id} (${target.what}), so ${target.id} " +
          s"cannot even be evaluated yet: the ladder is blocked lower down, at ${st.id}."
      else
        " This is the rung under test."
    val blocker =
      if (st.blocker.isEmpty) "NO known defect -- this is a new failure mode, investigate"
      else st.blocker
    s"[${st.id}] MISSED MILESTONE: ${st.what} -- $why.$prereq " +
      s"Observed at give-up: ${stateSummary(h)}. Known blocker: $blocker"
  }

  /** Walk the ladder from the current point up to and including rung `upTo`,
    * asserting each milestone in turn. Fails at the LOWEST unmet rung, which
    * keeps a blocked run cheap and makes the log name the real culprit rather
    * than the rung under test.
    */
  private def climbTo(h: LogPhyLoopbackHarness, upTo: Int): Unit = {
    for (idx <- 0 to upTo) {
      val st = ladder(idx)
      val res = stepUntil(h, st.guard, st.stride)(st.reached(h))
      if (res.ok) traceStates(h, s"${st.id} reached after ${res.cycles} cycles")
      else traceStates(h, s"${st.id} GAVE UP after ${res.cycles} cycles")
      assert(res.ok, stageFailure(h, idx, upTo, res))
    }
  }

  // ============================================================================
  // Skip-ahead gate.
  //
  // ScalaTest gives every it(...) its own independent run, so a rung above the
  // current blocker would otherwise pay a full 3.2M-cycle cold start just to
  // fail with the same message as the rung below it. These two sets let a rung
  // notice that situation and bail out with cancel(...) BEFORE simulate(...) is
  // ever called -- no elaborate, no verilate, no simulated cycle.
  //
  // Assumes sequential, declaration-order execution: AnyFunSpec's default (no
  // ParallelTestExecution here) and build.mill's testParallelism = false.
  // ============================================================================

  private val stageAttempted = scala.collection.mutable.Set.empty[Int]
  private val stagePassed = scala.collection.mutable.Set.empty[Int]

  /** Run this at the very TOP of a rung's body, outside simulate(...).
    *
    * Cancels the rung when some lower rung already ran in this suite and did not
    * pass -- "not attempted" rather than "attempted and failed", which is the
    * accurate status and keeps the failure count equal to the number of distinct
    * blocking defects. A rung whose prerequisites simply were not run (e.g. a -z
    * filter selected this rung alone) is NOT cancelled: nothing blocked it.
    */
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
  // S8 data traffic.
  //
  // An RDI word is rdiParams.nBytes * 8 = 512 bits and, at x16 with a serialiser
  // ratio of 32, fills exactly one mainband beat, so txBusy never engages and a
  // transfer completes in a single cycle (MainbandLaneController.scala:90-96,
  // :124). The loopback crosses the mainband combinationally, so the receiving
  // die presents plValid and plData in that SAME cycle -- the check below reads
  // the receiver before stepping, not after.
  // ============================================================================

  private val burstLength = 4
  private val rdiWordBits = 512

  /** Distinct, non-repeating payloads: a per-die tag, a sequence number and a
    * walking pattern, so a swapped word, a stale word and a lane permutation all
    * fail differently.
    */
  private def payload(die: Int, seq: Int): BigInt =
    (0 until rdiWordBits / 16).foldLeft(BigInt(0)) { (acc, i) =>
      val w = ((die + 1) << 12) | ((seq + 1) << 8) | ((i * 7 + die * 3 + seq) & 0xff)
      acc | (BigInt(w & 0xffff) << (i * 16))
    }

  /** Hold lpValid/lpIrdy until the sender accepts, check the peer received the
    * exact word in that cycle, then release. Returns nothing: it asserts.
    */
  private def sendWord(h: LogPhyLoopbackHarness, from: Int, word: BigInt): Unit = {
    val to = 1 - from
    h.io.lpData.get(from).poke(word.U(rdiWordBits.W))
    h.io.lpValid.get(from).poke(true.B)
    h.io.lpIrdy.get(from).poke(true.B)

    var waited = 0
    while (!h.io.plTrdy(from).peekBoolean() && waited < rdiFlagGuard) {
      h.clock.step(1)
      waited += 1
    }
    assert(h.io.plTrdy(from).peekBoolean(),
      s"die $from never asserted plTrdy within $rdiFlagGuard cycles, so no RDI word could be " +
        s"accepted. plTrdy needs rdiStateSts == active (LogicalPhy.scala:474-478) and " +
        s"!txBusy && mbLanes.tx.ready (MainbandLaneController.scala:186). " +
        s"Observed: ${stateSummary(h)}")

    assert(h.io.plValid(to).peekBoolean(),
      s"die $from accepted an RDI word but die $to did not present plValid in the same cycle. " +
        "The mainband crossover is combinational and one word is one beat, so the receive " +
        "pulse must be simultaneous (MainbandLaneController.scala:240). A missing pulse " +
        "points at the isActive gate on the receive path (LogicalPhy.scala:373,:480-483). " +
        s"Observed: ${stateSummary(h)}")

    val got = h.io.plData.get(to).peek().litValue
    assert(got == word,
      s"die $from -> die $to delivered a corrupted RDI word.\n" +
        s"  sent     0x${word.toString(16)}\n" +
        s"  received 0x${got.toString(16)}\n" +
        "Lane packing and unpacking must be exact inverses " +
        "(MainbandLaneController.scala:174-182 vs :225-235), and the sender's scrambler must " +
        "track the receiver's descrambler word for word (LogicalPhy.scala:302,:313). " +
        "A first word that matches followed by a corrupted second word means the two LFSRs " +
        "are not advancing together.")

    h.clock.step(1)
    h.io.lpValid.get(from).poke(false.B)
    h.io.lpIrdy.get(from).poke(false.B)
  }

  /** Both dies transmit on the same cycles. Each die's scrambler and the peer's
    * descrambler now advance together in both directions at once, which is the
    * case a one-way test cannot reach.
    */
  private def sendBothWays(h: LogPhyLoopbackHarness, word0: BigInt, word1: BigInt): Unit = {
    val words = Seq(word0, word1)
    for (i <- 0 until 2) {
      h.io.lpData.get(i).poke(words(i).U(rdiWordBits.W))
      h.io.lpValid.get(i).poke(true.B)
      h.io.lpIrdy.get(i).poke(true.B)
    }

    var waited = 0
    while (!bothDies(i => h.io.plTrdy(i).peekBoolean()) && waited < rdiFlagGuard) {
      h.clock.step(1)
      waited += 1
    }
    assert(bothDies(i => h.io.plTrdy(i).peekBoolean()),
      s"both dies must assert plTrdy to transmit simultaneously. Observed: ${stateSummary(h)}")

    for (i <- 0 until 2) {
      val to = 1 - i
      assert(h.io.plValid(to).peekBoolean(),
        s"simultaneous transmit: die $to lost the word from die $i. Observed: ${stateSummary(h)}")
      val got = h.io.plData.get(to).peek().litValue
      assert(got == words(i),
        s"simultaneous transmit corrupted die $i -> die $to.\n" +
          s"  sent     0x${words(i).toString(16)}\n" +
          s"  received 0x${got.toString(16)}\n" +
          "One-way transfers passing while this fails means the two directions share LFSR " +
          "state they should not (LogicalPhy.scala:295-322).")
    }

    h.clock.step(1)
    for (i <- 0 until 2) {
      h.io.lpValid.foreach(_(i).poke(false.B))
      h.io.lpIrdy.foreach(_(i).poke(false.B))
    }
  }

  // ============================================================================
  // The rungs. One test per milestone; each one that runs is a cold start.
  // ============================================================================

  describe("LogicalPhy staged bring-up ladder (reset -> sbinit -> mbinit -> mbtrain -> linkinit -> active, one gate per rung)") {

    it("S0: die 0 is triggered, die 1 wakes on the remote clock pattern, both reach sSBINIT") {
      gateOnLowerRungs(0)
      simulate(new LogPhyLoopbackHarness(), firtoolOpts = noAssertFirtoolOpts) { h =>
        coldStart(h)  // asymmetric: die 0 only

        // Split the wait so the log names WHICH half failed. The triggered die
        // owns swTriggerTraining and must move almost immediately; the sleeper
        // has to hear two clock patterns first, so it is strictly later.
        val started = stepUntil(h, guard = sbinitEntryGuard, stride = 1)(rankOf(h, 0) >= 1)
        traceStates(h, s"S0: die 0 (triggered) after ${started.cycles} cycles")
        assert(started.ok,
          s"[S0] die 0 did not reach sSBINIT within ${started.cycles} cycles even though it " +
            s"received the software trigger. This is NOT the remote-wake path -- the plain " +
            s"trigger itself is broken (LinkTrainingSM.scala:240, :1142-1143). " +
            s"Observed: ${stateSummary(h)}")

        val woke = stepUntil(h, guard = sbinitEntryGuard, stride = 1)(rankOf(h, 1) >= 1)
        traceStates(h, s"S0: die 1 (remote-woken) after ${woke.cycles} cycles")
        assert(woke.ok,
          s"[S0] die 1 never left sRESET within ${woke.cycles} cycles. It was never triggered " +
            s"by software ON PURPOSE -- it is supposed to wake on the clock pattern die 0 is " +
            s"transmitting (LinkTrainingSM.scala:1128-1140 -> :216 remoteTriggerTraining). " +
            s"Die 0 moved, so the trigger and the TX path are fine; this is the sideband RX " +
            s"path. Observed: ${stateSummary(h)}")

        climbTo(h, 0)

        // Checked at the instant of entry (no stepping in between), so these
        // cannot be spoiled by further progress once S1 starts working.
        for (i <- 0 until 2) {
          h.io.ltsmState(i).expect(LTSMState.sSBINIT, "SBINIT entry must show the SBINIT substate")
          h.io.plPhyInRecenter(i).expect(true.B, "plPhyInRecenter must be high in SBINIT")
          h.io.plStateSts(i).expect(RDIState.reset, "RDI stays in reset during SBINIT")
          assert(!h.io.trainingTimedout(i).peekBoolean(),
            s"die $i already reports trainingTimedout at SBINIT entry")
        }
      }
      stagePassed += 0
    }

    it("S1: completes SBINIT and enters sMBINIT") {
      gateOnLowerRungs(1)
      simulate(new LogPhyLoopbackHarness(), firtoolOpts = noAssertFirtoolOpts) { h =>
        coldStart(h)
        climbTo(h, 1)

        // Reaching MBINIT is not enough: SBINIT must have gotten there cleanly.
        // sbFaultSeen is the signature of the D-01 flood (cfg-credit exhaustion
        // -> RX priority-queue overflow); if it is latched while MBINIT was
        // still reached, the sideband is limping, not fixed.
        for (i <- 0 until 2) {
          assert(!h.io.sbFaultSeen(i).peekBoolean(),
            s"die $i entered sMBINIT but latched a sideband fault on the way " +
              s"(parity / RX queue overflow / invalid route / unhandled msg -- " +
              s"see LogPhyLoopbackHarness.scala:174-181 for the OR terms). " +
              s"Observed: ${stateSummary(h)}")
          assert(!h.io.trainingTimedout(i).peekBoolean(),
            s"die $i hit a residency timeout during SBINIT. Observed: ${stateSummary(h)}")
          assert(!h.io.plTrainError(i).peekBoolean(),
            s"die $i raised plTrainError during SBINIT. Observed: ${stateSummary(h)}")
        }
      }
      stagePassed += 1
    }

    it("S2: negotiates PHY parameters in MBINIT.PARAM (negotiatedParamsValid)") {
      gateOnLowerRungs(2)
      simulate(new LogPhyLoopbackHarness(), firtoolOpts = noAssertFirtoolOpts) { h =>
        coldStart(h)
        climbTo(h, 2)

        for (i <- 0 until 2) {
          h.io.negotiatedParamsValid(i).expect(true.B,
            "MBINIT.PARAM must publish negotiated settings")
          // Both dies advertise maxDataRate = speed4, so nothing may have
          // changed the PHY frequency request yet (LinkTrainingSM.scala:195).
          h.io.plSpeedmode(i).expect(SpeedMode.speed4,
            "no speed change is expected before MBTRAIN.SPEEDIDLE")
          assert(!h.io.plTrainError(i).peekBoolean(),
            s"die $i raised plTrainError during MBINIT.PARAM. Observed: ${stateSummary(h)}")
        }
      }
      stagePassed += 2
    }

    it("S3: completes MBINIT and enters sMBTRAIN") {
      gateOnLowerRungs(3)
      simulate(new LogPhyLoopbackHarness(), firtoolOpts = noAssertFirtoolOpts) { h =>
        coldStart(h)
        climbTo(h, 3)

        for (i <- 0 until 2) {
          assert(!h.io.trainingTimedout(i).peekBoolean(),
            s"die $i hit a residency timeout during MBINIT. Observed: ${stateSummary(h)}")
          assert(!h.io.plTrainError(i).peekBoolean(),
            s"die $i raised plTrainError during MBINIT. Observed: ${stateSummary(h)}")
          assert(!h.io.sbFaultSeen(i).peekBoolean(),
            s"die $i latched a sideband fault during MBINIT. Observed: ${stateSummary(h)}")
        }
      }
      stagePassed += 3
    }

    it("S4: completes MBTRAIN and enters sLINKINIT") {
      gateOnLowerRungs(4)
      simulate(new LogPhyLoopbackHarness(), firtoolOpts = noAssertFirtoolOpts) { h =>
        coldStart(h)
        climbTo(h, 4)

        for (i <- 0 until 2) {
          assert(!h.io.trainingTimedout(i).peekBoolean(),
            s"die $i hit a residency timeout during MBTRAIN. Observed: ${stateSummary(h)}")
          assert(!h.io.plTrainError(i).peekBoolean(),
            s"die $i raised plTrainError during MBTRAIN. Observed: ${stateSummary(h)}")
          // Both dies advertise speed4 and MBTRAIN.SPEEDIDLE programs the
          // negotiated rate, so the loopback must still be at speed4.
          h.io.plSpeedmode(i).expect(SpeedMode.speed4,
            "MBTRAIN must settle at the negotiated speed4 in this loopback")
        }
      }
      stagePassed += 4
    }

    it("S5: asserts plInbandPres in sLINKINIT") {
      gateOnLowerRungs(5)
      simulate(new LogPhyLoopbackHarness(), firtoolOpts = noAssertFirtoolOpts) { h =>
        coldStart(h)
        climbTo(h, 5)

        for (i <- 0 until 2) {
          h.io.plInbandPres(i).expect(true.B, "LINKINIT must present inband presence to the RDI")
          assert(!h.io.plTrainError(i).peekBoolean(),
            s"die $i raised plTrainError at LINKINIT. Observed: ${stateSummary(h)}")
        }
      }
      stagePassed += 5
    }

    it("S6: brings the RDI up to plStateSts=active") {
      gateOnLowerRungs(6)
      simulate(new LogPhyLoopbackHarness(), firtoolOpts = noAssertFirtoolOpts) { h =>
        coldStart(h)
        climbTo(h, 6)

        for (i <- 0 until 2) {
          h.io.plStateSts(i).expect(RDIState.active, "the RDI must reach active")
          h.io.plInbandPres(i).expect(true.B, "inband presence must still be held at RDI active")
          assert(!h.io.plTrainError(i).peekBoolean(),
            s"die $i raised plTrainError during RDI bring-up. Observed: ${stateSummary(h)}")
        }
      }
      stagePassed += 6
    }

    it("S7: reaches sACTIVE on both dies with a clean link") {
      gateOnLowerRungs(7)
      simulate(new LogPhyLoopbackHarness(), firtoolOpts = noAssertFirtoolOpts) { h =>
        coldStart(h)
        climbTo(h, 7)

        for (i <- 0 until 2) {
          h.io.ltsmState(i).expect(LTSMState.sACTIVE, "the debug LTSM state must show ACTIVE")
          h.io.plStateSts(i).expect(RDIState.active, "the RDI must be active in sACTIVE")
          h.io.plInbandPres(i).expect(true.B, "inband presence must be held in sACTIVE")
          h.io.plSpeedmode(i).expect(SpeedMode.speed4, "the link must run at the negotiated speed4")
          h.io.plPhyInRecenter(i).expect(false.B, "recentering must be finished in sACTIVE")
          assert(!h.io.trainingTimedout(i).peekBoolean(),
            s"die $i reached sACTIVE but a residency timeout had fired. " +
              s"Observed: ${stateSummary(h)}")
          assert(!h.io.plTrainError(i).peekBoolean(),
            s"die $i reached sACTIVE with plTrainError set. Observed: ${stateSummary(h)}")
          assert(!h.io.sbFaultSeen(i).peekBoolean(),
            s"die $i reached sACTIVE but a sideband fault was latched during training. " +
              s"Observed: ${stateSummary(h)}")
        }

        // The link must stay up, not just touch sACTIVE for one cycle.
        h.clock.step(20000)
        traceStates(h, "S7 dwell +20000 cycles")
        for (i <- 0 until 2) {
          h.io.ltsmState(i).expect(LTSMState.sACTIVE, "the link must stay in sACTIVE")
          h.io.plStateSts(i).expect(RDIState.active, "the RDI must stay active")
          assert(!h.io.plTrainError(i).peekBoolean(),
            s"die $i dropped into plTrainError while idling in sACTIVE. " +
              s"Observed: ${stateSummary(h)}")
        }
      }
      stagePassed += 7
    }

    it("S8: carries RDI data across the link in both directions") {
      gateOnLowerRungs(8)
      // The only rung that inspects received data, and therefore the only one
      // that pays for the receive unpack being live -- see the plData note in
      // LogPhyLoopbackHarness.
      simulate(
        new LogPhyLoopbackHarness(exposeDataPath = true),
        firtoolOpts = noAssertFirtoolOpts) { h =>
        coldStart(h)
        climbTo(h, 8)

        // One word each way, then a back-to-back burst in each direction, then
        // both dies transmitting on the same cycles. The burst is the point: the
        // scrambler advances once per accepted word on the sender and the
        // descrambler once per accepted word on the receiver
        // (LogicalPhy.scala:302,:313), so a lockstep error corrupts the SECOND
        // word onwards while the first still matches.
        sendWord(h, 0, payload(0, 0))
        sendWord(h, 1, payload(1, 0))

        for (n <- 1 until burstLength) sendWord(h, 0, payload(0, n))
        for (n <- 1 until burstLength) sendWord(h, 1, payload(1, n))

        for (n <- 0 until burstLength) sendBothWays(h, payload(0, n), payload(1, n))

        // Data traffic must not disturb the link itself.
        traceStates(h, "S8 after data traffic")
        for (i <- 0 until 2) {
          h.io.ltsmState(i).expect(LTSMState.sACTIVE, "the link must stay in sACTIVE while carrying data")
          h.io.plStateSts(i).expect(RDIState.active, "the RDI must stay active while carrying data")
          assert(!h.io.plTrainError(i).peekBoolean(),
            s"die $i raised plTrainError while carrying data. Observed: ${stateSummary(h)}")
          assert(!h.io.sbFaultSeen(i).peekBoolean(),
            s"die $i latched a sideband fault while carrying data. Observed: ${stateSummary(h)}")
        }
      }
      stagePassed += 8
    }
  }

}
