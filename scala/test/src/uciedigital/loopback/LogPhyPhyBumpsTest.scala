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
    rm -rf build/chiselsim/LogPhyPhyBumpsTest
    ./mill --no-daemon test.testOnly edu.berkeley.cs.uciedigital.loopback.LogPhyPhyBumpsTest
    # one rung only:
    ./mill --no-daemon test.testOnly edu.berkeley.cs.uciedigital.loopback.LogPhyPhyBumpsTest -- -z "S0"
    # clocks/reset smoke test only (seconds, not minutes):
    ./mill --no-daemon test.testOnly edu.berkeley.cs.uciedigital.loopback.LogPhyPhyBumpsTest -- -z "P0"

  What this is:
    The S0..S8 ladder of LogPhyStagedBringupTest, rerun against
    LogPhyPhyBumpsHarness -- i.e. with the real `Phy` inserted on both sides and
    the crossover moved down to PhyBumpsIO. The rung logic is a VERBATIM COPY of
    the green ladder, on purpose: the green ladder is 9/9, so any rung that goes
    red here is attributable to the PHY and to nothing else. Do not "improve" the
    rungs while copying -- that destroys the only property that makes the result
    readable.

  Deliberate differences from the copied file, and only these:
    1. Harness type.
    2. Every wait is expressed in DIGITAL (ucieClk) cycles and goes through
       stepD(), which multiplies by h.cyclesPerDigital = 16. The harness clock is
       now the mainband bit clock and the digital clock is bypassClkP/16 (see
       LogPhyPhyBumpsHarness's header): 3.2M digital cycles of RESET wait is
       51.2M harness steps. Getting this wrong makes S0 fail for a reason that
       has nothing to do with the PHY.
    3. Simulator settings, see `ucieSimulator` below: --timing is now mandatory
       and the default timescale is moved.
    4. A P0 smoke test ahead of the ladder that proves the clock tree and the
       reset release before 20 minutes of RESET wait get blamed on training.
    5. The S1 message cites LogPhyLoopbackHarness.scala:200-207 for the sbFault
       OR terms; the copied file still cites the pre-move :174-181.

  Ladder status MEASURED 2026-08-15 (Verilator 5.050, whole suite 6 min 46 s for
  all 8 rungs incl. the three diagnostics; the direct ladder is 3 min 37 s):

    P0  PASS  clock tree alive, ucieRst released, ucieClk = bypassClkP/16 exactly
    S0  PASS  die 1 woke in 167 digital cycles
    S1  PASS  SBINIT completes; 1920 digital cycles
    S2  PASS  MBINIT.PARAM negotiates; +384 digital cycles
    S3  FAIL  die 1 derails to sTRAINERROR after 640 digital cycles while die 0
              sits in sMBINIT_REPAIRCLK. timedout=false, sbFault=false.
    S4..S8    CANCELED (blocked by S3)

  S0/S1/S2 reproduce the direct ladder's cycle counts EXACTLY (167 / 1920 / 384,
  LogPhyStagedBringupTest.scala:98-102). The sideband path through two TxDrivers
  and two receivers is bit-identical, as predicted.

  S3's blocker is NOT D-15, and it is NOT the missing per-word valid either.
  Three measurements, in the order they were taken:
    * D15 rung: routing clkP/clkN around the PHY changes nothing -- still S2, so
      RxIO's missing clkP/clkN is not what stops this.
    * DMB rung, bumps at bit-clock granularity over the REPAIRCLK window, per die
      over 640 digital cycles: 192 words SENT (tx.valid high) against 564-575
      RECEIVED (rxDivClk rises) and 568-576 delivered to PatternReader. That gap
      is real -- PhyIO has no per-word valid -- but it is not the cause.
    * DCLK rung, the same lane on both sides of the serdes, received words
      filtered on mbRxValid so idle cycles cannot dilute the histogram:
          trk SENT     to the serializer = phase0 x64, phase1 x64, phase2 x64
          trk RECEIVED at PatternReader  = phase0 x568/576, and nothing else
          trk on the lane while tx.valid is LOW = phase0 x448
  The forwarded clock IS the pattern under test. clkP carries the same 48-bit
  clkRepairWord as trk (PatternWriter.scala:231-233), rx_clock_lane.v:38 forwards
  it verbatim, and ucie_clk_dist_network.sv:32,35 makes it the clock of every
  rx_data_lane. So the trk lane is sampled by a clock carrying its OWN waveform:
  every clock edge lands on the same bit polarity, and the recovered word is
  1,0,1,0... = phase0 no matter which phase was transmitted. The 16 STATIC ZEROS
  that close each 48-bit period reinforce it -- rx_data_lane.v:81,93 shifts only
  on clk edges, so that third of the period is not sampled at all. The reference
  free-runs 0,1,2 (PatternReader.scala:283-285), so ~2/3 of comparisons are dirty
  and MBInitSM.scala:537 derails to sTRAINERROR.

  The edge budget closes to 2%: 448 idle slots clock fully + 192 burst slots lose
  a third = 576 expected rxDivClk rises against 564/575 measured.

  A valid bit on PhyIO would fix the DMB gap and NOT this rung. See the S3 blocker
  string for the full chain and for what is ruled out.
*/
class LogPhyPhyBumpsTest extends AnyFunSpec with ChiselSim {

  // Hardware constants (LinkTrainingSM.scala:87-116), in DIGITAL cycles.
  private val hwTimeoutCycles = 6400000
  private val hwResetWait = hwTimeoutCycles / 2

  private val noAssertFirtoolOpts = Array(
    "--disable-layers=Verification,Verification.Assert,Verification.Assume,Verification.Cover")

  /* Two changes from the direct ladder's settings, both forced by the PHY:
     --timing:  ucie_pll.v:137 is `always #(CLK_PERIOD_PS/2) clk = ~clk;` and
                Verilator refuses to elaborate it otherwise (%Error-NEEDTIMINGOPT,
                which -Wno-fatal does NOT suppress). The Pll BlackBox is
                instantiated unconditionally at Phy.scala:181.
     -Wno-fatal (disableFatalExitOnWarnings): 15 MULTIDRIVEN/WIDTHEXPAND warnings
                from tx_lane.v, rx_data_lane.v, ucie_clk_div4.v, ucie_rst_sync.v,
                plus the undriven clkMuxP_in0. All fatal by default.

     defaultTimescale is moved to 1ps/100fs. ucie_pll.v is the ONLY file in the
     tree with its own `timescale (1ps/100fs), so with the svsim default of
     1ns/100ps its free-running oscillator fires ~16 times per harness clock
     period and every one of those costs a full eval (simulation-driver.cpp's
     eventsPending loop). Moving the default makes the harness clock period 1ps,
     so the oscillator fires once per ~62 clock periods instead. It cannot change
     any result: pll.io.vp_out/vn_out are read nowhere in src/ and
     ucie_clk_dist_network.sv never assigns clkMuxP_in0, so the PLL drives
     nothing. This is a wall-clock knob, not a timing model.

     Utils.verilatorSettings is deliberately NOT reused: it also forces a VCD
     TraceStyle unconditionally (Utils.scala:27-38), which would compile tracing
     into every rung and collide with -DemitVcd.
  */
  implicit private val ucieSimulator: HasSimulator =
    if (sys.env.get("UCIE_SIM_BACKEND").contains("vcs")) UcieSimBackend.fromEnv
    else
      HasSimulator.simulators.verilator(
        svsim.CommonCompilationSettings(
          optimizationStyle =
            svsim.CommonCompilationSettings.OptimizationStyle.OptimizeForSimulationSpeed,
          defaultTimescale = Some(
            svsim.CommonCompilationSettings.Timescale(
              svsim.CommonCompilationSettings.Timescale.Time(
                svsim.CommonCompilationSettings.Timescale.Magnitude.One,
                svsim.CommonCompilationSettings.Timescale.Unit.ps),
              svsim.CommonCompilationSettings.Timescale.Time(
                svsim.CommonCompilationSettings.Timescale.Magnitude.Hundred,
                svsim.CommonCompilationSettings.Timescale.Unit.fs)))),
        svsim.verilator.Backend.CompilationSettings.default
          .withTiming(
            Some(svsim.verilator.Backend.CompilationSettings.Timing.TimingEnabled))
          .withDisableFatalExitOnWarnings(true))

  private val debugTrace = true

  /** MUST stay false unless the run also passes -DemitVcd=1: without it the
    * model has no trace support and the first disable/enable aborts the rung at
    * its first clock step.
    */
  private val windowWaves = false

  // ============================================================================
  // Guards, in DIGITAL cycles -- same numbers as the direct ladder.
  // ============================================================================
  private val sbinitEntryGuard = 4096
  private val sbHandshakeGuard = 400000
  private val mbInitGuard = 800000
  private val mbTrainGuard = 1500000
  private val rdiFlagGuard = 4096

  /** One digital (ucieClk) cycle. Every wait in this file goes through here. */
  private def stepD(h: LogPhyPhyBumpsHarness, n: Int): Unit =
    h.clock.step(n * h.cyclesPerDigital)

  // ============================================================================
  // Decoding helpers (lazy for the same reason as the copied file: they touch
  // ChiselEnum literals, which must not happen at suite construction time).
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

  private lazy val ltRank: Map[BigInt, Int] = Map(
    LTState.sRESET.litValue -> 0,
    LTState.sSBINIT.litValue -> 1,
    LTState.sMBINIT.litValue -> 2,
    LTState.sMBTRAIN.litValue -> 3,
    LTState.sLINKINIT.litValue -> 4,
    LTState.sACTIVE.litValue -> 5,
  )

  private def ltCode(h: LogPhyPhyBumpsHarness, i: Int): BigInt =
    h.io.ltState(i).peek().litValue

  private def ltsmCode(h: LogPhyPhyBumpsHarness, i: Int): BigInt =
    h.io.ltsmState(i).peek().litValue

  private def rdiCode(h: LogPhyPhyBumpsHarness, i: Int): BigInt =
    h.io.plStateSts(i).peek().litValue

  private def rankOf(h: LogPhyPhyBumpsHarness, i: Int): Int =
    ltRank.getOrElse(ltCode(h, i), -1)

  private def offPath(h: LogPhyPhyBumpsHarness): Boolean =
    (0 until 2).exists(i => rankOf(h, i) < 0)

  private def bothDies(cond: Int => Boolean): Boolean = cond(0) && cond(1)

  private def reachedAtLeast(h: LogPhyPhyBumpsHarness, r: Int): Boolean =
    bothDies(i => rankOf(h, i) >= r)

  // ============================================================================
  // Logging
  // ============================================================================

  private def dieSummary(h: LogPhyPhyBumpsHarness, i: Int): String = {
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

  private def stateSummary(h: LogPhyPhyBumpsHarness): String =
    s"${dieSummary(h, 0)} | ${dieSummary(h, 1)}"

  private def traceStates(h: LogPhyPhyBumpsHarness, tag: String): Unit =
    if (debugTrace) println(s"[phybumps] $tag :: ${stateSummary(h)}")

  // ============================================================================
  // Stimulus
  // ============================================================================

  private def initHarness(h: LogPhyPhyBumpsHarness, pwrGood: Boolean = true): Unit = {
    for (i <- 0 until 2) {
      h.io.lpStateReq(i).poke(RDIStateReq.nop)
      h.io.swStartLinkTraining(i).poke(false.B)
      h.io.pwrGood(i).poke(pwrGood.B)
      h.io.lpValid.foreach(_(i).poke(false.B))
      h.io.lpIrdy.foreach(_(i).poke(false.B))
      h.io.lpData.foreach(_(i).poke(0.U))
    }
  }

  private def runResetWait(h: LogPhyPhyBumpsHarness): Unit = {
    if (windowWaves) disableWaves()

    stepD(h, 64)
    for (i <- 0 until 2) {
      h.io.ltsmState(i).expect(LTSMState.sRESET, "LTSM must start in RESET")
      h.io.plStateSts(i).expect(RDIState.reset, "RDI must start in reset")
      h.io.plInbandPres(i).expect(false.B, "no inband presence in RESET")
    }
    // Chunked so a hung simulator is diagnosable from the log. Each chunk is
    // 800k digital cycles = 12.8M harness steps.
    for (chunk <- 1 to 4) {
      val t0 = System.nanoTime()
      stepD(h, hwResetWait / 4)
      if (debugTrace)
        println(f"[phybumps] reset wait chunk $chunk/4 done " +
          f"(${(System.nanoTime() - t0) / 1e9}%.1f s)")
    }
    stepD(h, 64)
    for (i <- 0 until 2) {
      h.io.ltsmState(i).expect(LTSMState.sRESET, "no training without a trigger")
    }

    if (windowWaves) enableWaves()
  }

  private def coldStartOneDie(h: LogPhyPhyBumpsHarness, die: Int): Unit = {
    initHarness(h)
    runResetWait(h)
    h.io.swStartLinkTraining(die).poke(true.B)
    stepD(h, 4)
    h.io.swStartLinkTraining(die).poke(false.B)
  }

  /** ASYMMETRIC ON PURPOSE: only die 0 gets the software trigger; die 1 has to
    * wake on the clock pattern die 0 transmits over the sideband.
    */
  private def coldStart(h: LogPhyPhyBumpsHarness): Unit = coldStartOneDie(h, 0)

  private case class WaitResult(ok: Boolean, cycles: Int, derailed: Boolean)

  private def stepUntil(
      h: LogPhyPhyBumpsHarness,
      guard: Int,
      stride: Int,
  )(cond: => Boolean): WaitResult = {
    var n = 0
    var derailed = false
    while (!cond && !derailed && n < guard) {
      stepD(h, stride)
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
      reached: LogPhyPhyBumpsHarness => Boolean,
  )

  private val ladder: Seq[Stage] = Seq(
    Stage(
      id = "S0",
      what = "die 0 takes the software trigger and die 1 wakes on the remote clock " +
        "pattern, so BOTH dies reach sSBINIT from a staggered start",
      guard = sbinitEntryGuard,
      stride = 1,
      blocker =
        "with the PHY in the loop, first suspect the clock tree, not the sideband. " +
          "Run the P0 smoke test: if txDivClk is not toggling then pllBypassEn or the " +
          "clkMux leg is wrong (ucie_clk_dist_network.sv:21 assigns clkMuxP_in1 only) and " +
          "ucieClk never runs, so the LTSM cannot leave RESET for any reason at all. " +
          "If P0 is clean, this is the same rung as the direct ladder: only die 0 moving " +
          "means the REMOTE-WAKE path (LinkTrainingSM.scala:1128-1140 -> :216) is broken, " +
          "which through the PHY means the sideband bumps -- but tx_driver.v:87 is " +
          "`assign dout = din` and Phy.scala:161-179 adds no register, so a sideband " +
          "difference from the direct ladder would be a wiring error here, not RTL. " +
          "If NEITHER die moved, the software trigger itself " +
          "(LinkTrainingSM.scala:240, :1142-1143)",
      reached = (h: LogPhyPhyBumpsHarness) => reachedAtLeast(h, 1),
    ),
    Stage(
      id = "S1",
      what = "SBINIT completes (clock pattern detected, out-of-reset + done handshakes) " +
        "and both dies enter sMBINIT",
      guard = sbHandshakeGuard,
      stride = 64,
      blocker =
        "SBINIT is entirely sideband and the sideband path through the PHY is a " +
          "combinational identity, so this rung should read exactly as it does in the " +
          "direct ladder. Its known blocker there was D-01 (SidebandSwitch.scala:56 " +
          "misrouting raw clock-pattern words up the D2D cfg path) and D-11 (parity must " +
          "not be checked in RAW mode, SidebandLinkNode.scala:131). sbFault=true with the " +
          "dies stuck in sSBINIT is that signature",
      reached = (h: LogPhyPhyBumpsHarness) => reachedAtLeast(h, 2),
    ),
    Stage(
      id = "S2",
      what = "MBINIT.PARAM exchanges and accepts PHY parameters on both dies " +
        "(negotiatedParamsValid)",
      guard = sbHandshakeGuard,
      stride = 64,
      blocker =
        "none known. PARAM is sideband-only and both dies advertise identical settings " +
          "(maxDataRate=speed4, x16), so interoperability must succeed " +
          "(MBInitSM.scala:136-145)",
      reached = (h: LogPhyPhyBumpsHarness) =>
        bothDies(i => h.io.negotiatedParamsValid(i).peekBoolean()),
    ),
    Stage(
      id = "S3",
      what = "MBINIT completes (CAL, REPAIRCLK, REPAIRVAL, REVERSALMB, REPAIRMB) and " +
        "both dies enter sMBTRAIN",
      guard = mbInitGuard,
      stride = 64,
      blocker =
        "MEASURED BLOCKER (2026-08-15), and it is NOT D-15. MBINIT.REPAIRCLK's pattern " +
          "cannot survive a real PHY, because the forwarded clock IS the pattern under " +
          "test. PatternWriter.scala:231-233 puts the same clkRepairWord on clkP, clkN " +
          "and trk; Phy.scala:245-246 serializes clkP onto the txClkP bump; on the peer " +
          "rx_clock_lane.v:38 is `assign clkout = clkin` and ucie_clk_dist_network.sv:32,35 " +
          "makes that bump the clock of rxClkDiv AND of all 18 rx_data_lane. The 48-bit " +
          "pattern 0x000055555555 is 32 alternating bits followed by 16 STATIC ZEROS, so " +
          "for 16 of every 48 UI the forwarded clock has no transitions at all, and " +
          "rx_data_lane.v:81,93 shifts only on clk edges. Worse, because trk carries the " +
          "SAME waveform as the clock sampling it, every surviving edge lands on the same " +
          "bit polarity, so the recovered word is 1,0,1,0... = phase0 whatever was sent. " +
          "MEASURED by the DCLK rung, per die, one 640-cycle window, received words " +
          "filtered on mbRxValid: trk SENT to the serializer = phase0 x64 + phase1 x64 + " +
          "phase2 x64, a perfect 3-phase sequence; trk RECEIVED at PatternReader = " +
          "phase0 x568/576 and nothing else. The edge budget closes: 448 idle slots (the " +
          "lane parks at phase0 between bursts, measured) clock fully and the 192 burst " +
          "slots lose a third, predicting 576 rxDivClk rises against 564/575 measured. " +
          "PatternReader.scala:310 compares trk against clkRepairRefPattern, which " +
          "free-runs through all three phases (:283-285), so ~2/3 of comparisons are dirty " +
          "and MBInitSM.scala:537 (resp.valid && !repairClkSuccess) derails to " +
          "sTRAINERROR. SECOND, INDEPENDENT hole -- measured, real, but NOT what stops " +
          "this rung: PhyIO carries no per-word valid (Phy.scala:129-130, io.tx a bare " +
          "Input and io.rx a bare Output), so DMB sees 192 words sent arrive as ~576 " +
          "delivered (448 idle slots at full clock rate plus 128 real ones) and " +
          "PatternWriter's phase (advanced on fire, PatternWriter.scala:193) cannot stay " +
          "locked to PatternReader's (advanced on mbRxValid, PatternReader.scala:246). " +
          "Adding a valid bit to PhyIO or a framing/idle encoding fixes THAT hole only: " +
          "the reader would advance on real words and still see a constant phase0. " +
          "Ruled out by measurement, do not re-investigate: (a) D-15 -- the D15 rung " +
          "routes clkP/clkN around the PHY and the result is unchanged; (b) a dead RX " +
          "clock domain -- rxDivRst is low for the whole window and 564+ words are " +
          "deserialized; (c) serdes bit corruption -- the received word is a clean, valid " +
          "phase0 reference word, so the serdes is lossy for this pattern, not noisy. " +
          "Fixing this needs RTL: either CLKREPAIR must keep the clock lane transitioning " +
          "through the whole pattern, or the receiver needs a sampling clock that does not " +
          "come from the lane under test. Note the integrated top inherits both holes -- " +
          "TileLink.scala:733-736 fakes clkP/clkN as constants and :723,737 sources valid " +
          "from AsyncQueue occupancy, which is local to one die and never crosses the link",
      reached = (h: LogPhyPhyBumpsHarness) => reachedAtLeast(h, 3),
    ),
    Stage(
      id = "S4",
      what = "MBTRAIN completes (all 13 substates) and both dies enter sLINKINIT",
      guard = mbTrainGuard,
      stride = 64,
      blocker =
        "the densest rung; the ltsm value in the dump says which substate. The seven " +
          "defects that lived here are listed in LogPhyStagedBringupTest.scala's S4 " +
          "blocker and are all fixed, so through the PHY suspect the mainband first: " +
          "sMBTRAIN_RXCLKCAL drives the forwarded-clock pattern on clkP/clkN " +
          "(LogicalPhy.scala:406-411), which is the same D-15 gap as S3, and " +
          "sMBTRAIN_LINKSPEED is the one state that runs a real mainband data test " +
          "(TxD2CPointTest with LFSR) and therefore the first rung that actually depends " +
          "on the 32:1 serdes preserving a word",
      reached = (h: LogPhyPhyBumpsHarness) => reachedAtLeast(h, 4),
    ),
    Stage(
      id = "S5",
      what = "both dies assert plInbandPres on sLINKINIT entry",
      guard = rdiFlagGuard,
      stride = 8,
      blocker =
        "none known -- inbandPresent is a plain register set whenever ltState is " +
          "sLINKINIT or sACTIVE (RDIController.scala:150-157)",
      reached = (h: LogPhyPhyBumpsHarness) =>
        bothDies(i => h.io.plInbandPres(i).peekBoolean()),
    ),
    Stage(
      id = "S6",
      what = "the RDI state machine completes bring-up: plStateSts == RDIState.active on " +
        "both dies",
      guard = sbHandshakeGuard,
      stride = 64,
      blocker =
        "none known -- no lpStateReq poke is needed (RDIController.scala:43-45 forces " +
          "RDIStateReq.active while ltState is sLINKINIT). The ACTIVE req/rsp travels the " +
          "sideband, so a stall here after S1 is fixed points at the RDI " +
          "requester/responder pair (RDIStateMachine.scala:255-258, :349-401)",
      reached = (h: LogPhyPhyBumpsHarness) =>
        bothDies(i => rdiCode(h, i) == RDIState.active.litValue),
    ),
    Stage(
      id = "S7",
      what = "both dies reach sACTIVE -- the link is up",
      guard = rdiFlagGuard,
      stride = 8,
      blocker =
        "none known -- LinkTrainingSM.scala:1264 leaves sLINKINIT for sACTIVE as soon as " +
          "plStateSts is active",
      reached = (h: LogPhyPhyBumpsHarness) => reachedAtLeast(h, 5),
    ),
    Stage(
      id = "S8",
      what = "the RDI accepts data: plTrdy is high on both dies in sACTIVE",
      guard = rdiFlagGuard,
      stride = 8,
      blocker =
        "plTrdy is gated on rdiStateSts == active (LogicalPhy.scala:474-478) and on " +
          "!txBusy && mbLanes.tx.ready (MainbandLaneController.scala:186). Through the PHY " +
          "tx.ready is tied true by the harness (PhyIO has no handshake), so a stall here " +
          "is the RDI state, not the data path. A CORRUPTED word, by contrast, is the " +
          "serdes: the loopback is no longer combinational, one word takes 16 harness " +
          "steps each way, and there is no word aligner",
      reached = (h: LogPhyPhyBumpsHarness) => bothDies(i => h.io.plTrdy(i).peekBoolean()),
    ),
  )

  private def stageFailure(
      h: LogPhyPhyBumpsHarness,
      stuckIdx: Int,
      targetIdx: Int,
      res: WaitResult,
  ): String = {
    val st = ladder(stuckIdx)
    val target = ladder(targetIdx)
    val why =
      if (res.derailed)
        s"gave up after ${res.cycles} digital cycles because a die derailed off the " +
          "bring-up path (sTRAINERROR / sPHYRETRAIN / sL1_L2 have no forward rank)"
      else
        s"not reached within the ${st.guard}-cycle guard (waited ${res.cycles} digital " +
          s"cycles, polled every ${st.stride})"
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

  private def climbTo(h: LogPhyPhyBumpsHarness, upTo: Int): Unit = {
    for (idx <- 0 to upTo) {
      val st = ladder(idx)
      val res = stepUntil(h, st.guard, st.stride)(st.reached(h))
      if (res.ok) traceStates(h, s"${st.id} reached after ${res.cycles} digital cycles")
      else traceStates(h, s"${st.id} GAVE UP after ${res.cycles} digital cycles")
      assert(res.ok, stageFailure(h, idx, upTo, res))
    }
  }

  // ============================================================================
  // Skip-ahead gate. Assumes sequential, declaration-order execution
  // (AnyFunSpec default; build.mill sets testParallelism = false).
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
          s"pointless ${hwResetWait * 16}-step RESET wait plus a full verilate pass.")
    }
    stageAttempted += stage
  }

  // ============================================================================
  // S8 data traffic. Unlike the direct ladder the loopback is NOT combinational
  // any more: a word crosses through a 32:1 serializer and a 1:32 deserializer,
  // so the receive pulse cannot be simultaneous with the transmit. sendWord
  // therefore polls for plValid instead of asserting it in the same cycle.
  // ============================================================================

  private val burstLength = 4
  private val rdiWordBits = 512

  private def payload(die: Int, seq: Int): BigInt =
    (0 until rdiWordBits / 16).foldLeft(BigInt(0)) { (acc, i) =>
      val w = ((die + 1) << 12) | ((seq + 1) << 8) | ((i * 7 + die * 3 + seq) & 0xff)
      acc | (BigInt(w & 0xffff) << (i * 16))
    }

  private def sendWord(h: LogPhyPhyBumpsHarness, from: Int, word: BigInt): Unit = {
    val to = 1 - from
    h.io.lpData.get(from).poke(word.U(rdiWordBits.W))
    h.io.lpValid.get(from).poke(true.B)
    h.io.lpIrdy.get(from).poke(true.B)

    var waited = 0
    while (!h.io.plTrdy(from).peekBoolean() && waited < rdiFlagGuard) {
      stepD(h, 1)
      waited += 1
    }
    assert(h.io.plTrdy(from).peekBoolean(),
      s"die $from never asserted plTrdy within $rdiFlagGuard digital cycles. plTrdy needs " +
        s"rdiStateSts == active (LogicalPhy.scala:474-478) and !txBusy && mbLanes.tx.ready " +
        s"(MainbandLaneController.scala:186). Observed: ${stateSummary(h)}")

    // The serdes adds latency, so wait for the receive pulse rather than
    // demanding it in the accept cycle.
    var rxWait = 0
    while (!h.io.plValid(to).peekBoolean() && rxWait < rdiFlagGuard) {
      stepD(h, 1)
      rxWait += 1
    }
    assert(h.io.plValid(to).peekBoolean(),
      s"die $from accepted an RDI word but die $to never presented plValid within " +
        s"$rdiFlagGuard digital cycles. Through the PHY the word travels " +
        s"LogicalPhy -> tx_lane (32:1) -> bump -> rx_data_lane (1:32) -> LogicalPhy with " +
        s"no aligner; a lost word means the RX word clock (the peer's forwarded clock, " +
        s"ucie_clk_dist_network.sv:32,:35) is not running or the receive isActive gate is " +
        s"closed (LogicalPhy.scala:373,:480-483). Observed: ${stateSummary(h)}")

    val got = h.io.plData.get(to).peek().litValue
    assert(got == word,
      s"die $from -> die $to delivered a corrupted RDI word.\n" +
        s"  sent     0x${word.toString(16)}\n" +
        s"  received 0x${got.toString(16)}\n" +
        "A whole-word ROTATION points at the serdes framing (tx_lane.v:218 resets its " +
        "counter to 3'b1, rx_data_lane.v:78 resets its to 3'b0). Bit-level corruption on " +
        "the SECOND word onward points at the scrambler/descrambler pair " +
        "(LogicalPhy.scala:302,:313) instead.")

    stepD(h, 1)
    h.io.lpValid.get(from).poke(false.B)
    h.io.lpIrdy.get(from).poke(false.B)
  }

  // ============================================================================
  // P0: clocks and resets. Cheap, and it must pass before any rung failure is
  // believed -- with a RawModule PHY the entire digital clock tree is a harness
  // responsibility, so "the LTSM never left RESET" has two very different
  // causes and this test separates them.
  // ============================================================================

  describe("Phy-in-the-loop smoke test") {
    it("P0: the PHY clock tree runs, reset releases and ucieClk is bypassClkP/16") {
      simulate(
        new LogPhyPhyBumpsHarness(exposeProbe = true),
        firtoolOpts = noAssertFirtoolOpts) { h =>
        initHarness(h)

        /** Probe both dies once per harness step, `n` steps. */
        def sample(n: Int): Seq[(BigInt, BigInt)] =
          (0 until n).map { _ =>
            val v = (h.io.phyProbe.get(0).peek().litValue, h.io.phyProbe.get(1).peek().litValue)
            h.clock.step(1)
            v
          }

        // Past the harness reset stretcher and the three-flop ucie_rst_sync.
        stepD(h, 16)
        val trace = sample(8 * h.cyclesPerDigital)

        for ((probe, i) <- Seq(trace.map(_._1), trace.map(_._2)).zipWithIndex) {
          assert(probe.map(v => (v >> 1) & 1).distinct.size > 1,
            s"die $i: txDivClk never toggled. That is the whole mainband clock tree: " +
              "bypassClkP -> ucie_clk_dist_network (clkMuxP_in1) -> ucie_clkmux -> " +
              "ucie_clk_div4.clkout_3 (Phy.scala:201-204). If it is dead, either " +
              "regs.pllBypassEn is not 1 (ucie_clkmux.v:7 then selects clkMuxP_in0, which " +
              "ucie_clk_dist_network.sv never assigns) or divResetb is still held " +
              "(ucie_clk_div4.v:12). Nothing in this suite can pass until this does.")

          assert(probe.forall(v => (v & 1) == 0),
            s"die $i: ucieRst is still asserted. ucie_rst_sync.v needs three edges of " +
              "ucieClk (= digitalBypassClk = txDivClk here) after rstbAsync rises " +
              "(Phy.scala:146-149). If txDivClk toggles and this still fails, the reset " +
              "stretcher in the harness is not releasing.")

          // The load-bearing claim of this harness: one ucieClk cycle per
          // cyclesPerDigital harness steps. If this ratio is wrong every cycle
          // constant in the ladder is wrong and S0 fails for a reason that has
          // nothing to do with the PHY.
          val cnt = probe.map(v => (v >> 16) & 0xff)
          val ticks = cnt.sliding(2).count(w => w(0) != w(1))
          val expected = trace.length / h.cyclesPerDigital
          assert(ticks == expected,
            s"die $i: the ucieClk-domain counter advanced $ticks times in " +
              s"${trace.length} harness steps; ${h.cyclesPerDigital}:1 requires $expected. " +
              "ucieClk is digitalBypassClk is txDivClk = clkout_3 of ucie_clk_div4 " +
              "(clk/16, ucie_clk_div4.v:11-24), matching tx_lane.v's 32 bits over 16 clkp " +
              "periods at DDR. A different ratio means digitalBypassClk is wired to the " +
              "wrong divider tap.")
          assert(cnt.sliding(2).forall(w => w(0) == w(1) || w(1) == (w(0) + 1) % 256),
            s"die $i: the ucieClk-domain counter did not advance monotonically by one.")
        }

        for (i <- 0 until 2) {
          h.io.ltsmState(i).expect(LTSMState.sRESET, "LTSM must be in RESET after reset")
          h.io.plStateSts(i).expect(RDIState.reset, "RDI must be in reset after reset")
        }

        // Nothing is transmitted during RESET (the LTSM cannot leave it for
        // 3.2M digital cycles), so the bumps are all static here by design and
        // are NOT checked. S0 is the first rung that exercises them.

        // Throughput, so the cost of a rung is a measurement and not a guess.
        val n = 200000
        val t0 = System.nanoTime()
        h.clock.step(n)
        val dt = (System.nanoTime() - t0) / 1e9
        println(f"[phybumps] P0 throughput: $n steps in $dt%.1f s = ${n / dt}%.0f steps/s; " +
          f"one rung's RESET wait is ${hwResetWait.toLong * h.cyclesPerDigital} steps " +
          f"= ${hwResetWait.toLong * h.cyclesPerDigital / (n / dt) / 60}%.1f min")
        traceStates(h, "P0 end")
      }
    }
  }

  // ============================================================================
  // The rungs. One test per milestone; each one that runs is a cold start.
  // ============================================================================

  describe("LogicalPhy -> Phy -> bumps staged bring-up ladder") {

    it("S0: die 0 is triggered, die 1 wakes on the remote clock pattern, both reach sSBINIT") {
      gateOnLowerRungs(0)
      simulate(new LogPhyPhyBumpsHarness(), firtoolOpts = noAssertFirtoolOpts) { h =>
        coldStart(h)

        val started = stepUntil(h, guard = sbinitEntryGuard, stride = 1)(rankOf(h, 0) >= 1)
        traceStates(h, s"S0: die 0 (triggered) after ${started.cycles} digital cycles")
        assert(started.ok,
          s"[S0] die 0 did not reach sSBINIT within ${started.cycles} digital cycles even " +
            s"though it received the software trigger. Run P0 first: with the PHY in the " +
            s"loop a dead ucieClk produces exactly this symptom. If P0 is clean, the plain " +
            s"trigger is broken (LinkTrainingSM.scala:240, :1142-1143). " +
            s"Observed: ${stateSummary(h)}")

        val woke = stepUntil(h, guard = sbinitEntryGuard, stride = 1)(rankOf(h, 1) >= 1)
        traceStates(h, s"S0: die 1 (remote-woken) after ${woke.cycles} digital cycles")
        assert(woke.ok,
          s"[S0] die 1 never left sRESET within ${woke.cycles} digital cycles. It was never " +
            s"triggered by software ON PURPOSE -- it wakes on the clock pattern die 0 " +
            s"transmits (LinkTrainingSM.scala:1128-1140 -> :216). Die 0 moved, so the " +
            s"trigger and the TX path are fine; this is the sideband RX path, which now " +
            s"runs through die 0's sbTxData TxDriver and die 1's sbRxData bump. " +
            s"Observed: ${stateSummary(h)}")

        climbTo(h, 0)

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
      simulate(new LogPhyPhyBumpsHarness(), firtoolOpts = noAssertFirtoolOpts) { h =>
        coldStart(h)
        climbTo(h, 1)

        for (i <- 0 until 2) {
          assert(!h.io.sbFaultSeen(i).peekBoolean(),
            s"die $i entered sMBINIT but latched a sideband fault on the way " +
              s"(parity / RX queue overflow / invalid route / unhandled msg -- " +
              s"see LogPhyLoopbackHarness.scala:200-207 for the OR terms). " +
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
      simulate(new LogPhyPhyBumpsHarness(), firtoolOpts = noAssertFirtoolOpts) { h =>
        coldStart(h)
        climbTo(h, 2)

        for (i <- 0 until 2) {
          h.io.negotiatedParamsValid(i).expect(true.B,
            "MBINIT.PARAM must publish negotiated settings")
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
      simulate(new LogPhyPhyBumpsHarness(), firtoolOpts = noAssertFirtoolOpts) { h =>
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
      simulate(new LogPhyPhyBumpsHarness(), firtoolOpts = noAssertFirtoolOpts) { h =>
        coldStart(h)
        climbTo(h, 4)

        for (i <- 0 until 2) {
          assert(!h.io.trainingTimedout(i).peekBoolean(),
            s"die $i hit a residency timeout during MBTRAIN. Observed: ${stateSummary(h)}")
          assert(!h.io.plTrainError(i).peekBoolean(),
            s"die $i raised plTrainError during MBTRAIN. Observed: ${stateSummary(h)}")
          h.io.plSpeedmode(i).expect(SpeedMode.speed4,
            "MBTRAIN must settle at the negotiated speed4 in this loopback")
        }
      }
      stagePassed += 4
    }

    it("S5: asserts plInbandPres in sLINKINIT") {
      gateOnLowerRungs(5)
      simulate(new LogPhyPhyBumpsHarness(), firtoolOpts = noAssertFirtoolOpts) { h =>
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
      simulate(new LogPhyPhyBumpsHarness(), firtoolOpts = noAssertFirtoolOpts) { h =>
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
      simulate(new LogPhyPhyBumpsHarness(), firtoolOpts = noAssertFirtoolOpts) { h =>
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

        stepD(h, 20000)
        traceStates(h, "S7 dwell +20000 digital cycles")
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
      simulate(
        new LogPhyPhyBumpsHarness(exposeDataPath = true),
        firtoolOpts = noAssertFirtoolOpts) { h =>
        coldStart(h)
        climbTo(h, 8)

        sendWord(h, 0, payload(0, 0))
        sendWord(h, 1, payload(1, 0))

        for (n <- 1 until burstLength) sendWord(h, 0, payload(0, n))
        for (n <- 1 until burstLength) sendWord(h, 1, payload(1, n))

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

  // ============================================================================
  // D-15 isolation. Same ladder, same cold start, one difference: the two
  // mainband clock lanes -- and ONLY those two -- are crossed directly between
  // the LogicalPhy instances instead of through the PHY, because RxIO has no
  // clkP/clkN to carry them (Phy.scala:21-25). Everything else, including all
  // 16 data lanes, valid and track, still travels the full
  // LogicalPhy -> Phy -> bumps -> Phy -> LogicalPhy path.
  //
  // This is a MEASUREMENT, not a rung: it decides whether S3's stop is the
  // missing clock-lane deserializer alone or whether the 32:1 serdes is also
  // implicated. It is deliberately outside the ladder's gate so it always runs.
  // ============================================================================

  describe("D-15 isolation") {
    it("D15: with clkP/clkN routed around the PHY, how far does MBINIT get?") {
      simulate(
        new LogPhyPhyBumpsHarness(rxClkLanesFromPeer = true),
        firtoolOpts = noAssertFirtoolOpts) { h =>
        coldStart(h)
        // Reports, does not assert: the ladder's S3 already carries the
        // failure, and a second red test for the same blocker would make the
        // failure count stop equalling the number of distinct defects.
        var top = "none"
        var stopped = false
        for (idx <- 0 to 3 if !stopped) {
          val st = ladder(idx)
          val res = stepUntil(h, st.guard, st.stride)(st.reached(h))
          if (res.ok) top = st.id else stopped = true
        }
        println(s"[phybumps] D15 highest rung reached with clkP/clkN bypassing the PHY: $top " +
          s":: ${stateSummary(h)}")
        println("[phybumps] D15 MEASURED 2026-08-15: S2, identical to the default harness. " +
          "Supplying real clkP/clkN does NOT clear S3, so D-15 is not the blocker.")
      }
    }
  }

  // ============================================================================
  // D-MB: what the mainband actually does during MBINIT.REPAIRCLK.
  //
  // Not a rung -- a measurement. Climbs to S2 and then samples the PHY probe
  // once per digital cycle across the window in which S3 derails, so the report
  // is "the forwarded clock never arrived" or "words arrived and compared bad"
  // rather than a guess between them.
  // ============================================================================

  describe("mainband instrumentation") {
    it("DMB: sample the PHY probe across the MBINIT.REPAIRCLK window") {
      simulate(
        new LogPhyPhyBumpsHarness(exposeProbe = true),
        firtoolOpts = noAssertFirtoolOpts) { h =>
        coldStart(h)
        for (idx <- 0 to 2) {
          val st = ladder(idx)
          val res = stepUntil(h, st.guard, st.stride)(st.reached(h))
          assert(res.ok, s"DMB could not reach ${st.id}: ${stateSummary(h)}")
        }
        traceStates(h, "DMB at S2")

        // Sample at BIT-CLOCK granularity: the forwarded mainband clock is a
        // serialized data pattern, so anything sampled per digital cycle
        // aliases it into noise.
        val steps = 640 * h.cyclesPerDigital
        val p0 = new Array[BigInt](steps)
        val p1 = new Array[BigInt](steps)
        for (k <- 0 until steps) {
          p0(k) = h.io.phyProbe.get(0).peek().litValue
          p1(k) = h.io.phyProbe.get(1).peek().litValue
          h.clock.step(1)
        }

        def bit(v: BigInt, b: Int): Int = ((v >> b) & 1).toInt
        def rises(vs: Seq[BigInt], b: Int): Int =
          vs.sliding(2).count(w => bit(w(0), b) == 0 && bit(w(1), b) == 1)
        def longestFlat(vs: Seq[BigInt], b: Int): Int = {
          var best = 0
          var run = 0
          vs.sliding(2).foreach { w =>
            if (bit(w(0), b) == bit(w(1), b)) { run += 1; if (run > best) best = run } else run = 0
          }
          best
        }

        val refs = Seq(BigInt("55555555", 16), BigInt("55550000", 16), BigInt("00005555", 16))
        for ((vs, die) <- Seq(p0.toSeq, p1.toSeq).zipWithIndex) {
          // One sample per digital cycle: take the sample at each tick of the
          // ucieClk-domain counter in the probe.
          val digital = vs.sliding(2).collect {
            case Seq(a, b) if ((a >> 16) & 0xff) != ((b >> 16) & 0xff) => b
          }.toSeq
          val trk = digital.map(v => (v >> 32) & BigInt("ffffffff", 16))

          println(s"[phybumps] DMB die$die over $steps bit-clock steps (${digital.length} digital cycles):")
          println(s"[phybumps]   fwd clock OUT (txClkP bump): rises=${rises(vs, 2)} " +
            s"longest gap with no edge=${longestFlat(vs, 2)} bit-clocks")
          println(s"[phybumps]   fwd clock IN  (rxClkP bump): rises=${rises(vs, 8)} " +
            s"longest gap with no edge=${longestFlat(vs, 8)} bit-clocks")
          println(s"[phybumps]   WORDS SENT     (tx.valid high digital cycles)     = ${digital.count(v => bit(v, 11) == 1)}")
          println(s"[phybumps]   WORDS RECEIVED (rxDivClk rising edges)            = ${rises(vs, 13)}")
          println(s"[phybumps]   WORDS DELIVERED to PatternReader (mbRxValid high) = ${digital.count(v => bit(v, 10) == 1)}")
          println(s"[phybumps]   rxDivRst high digital cycles = ${digital.count(v => bit(v, 9) == 1)}")
          println(f"[phybumps]   trk distinct=${trk.distinct.size} matching a CLKREPAIR reference=${trk.count(refs.contains)}/${trk.length}")
          println("[phybumps]   trk values seen: " + trk.distinct.take(8).map(v => f"0x$v%08x").mkString(","))
        }
        println(s"[phybumps] DMB derailed=${offPath(h)}")
        traceStates(h, "DMB end")
      }
    }
  }

  // ============================================================================
  // The CLKREPAIR round trip, measured on ONE lane.
  //
  // DMB shows 192 words sent against ~576 delivered and concludes the missing
  // per-word valid is the blocker. That is real but it is not the whole story,
  // and this rung separates the two: it puts the word HANDED to the serializer
  // and the word PatternReader COMPARES side by side, same lane (trk), same
  // cycle, one serdes apart. If the sets differ the link is lossy for this
  // pattern and no amount of framing repairs it.
  // ============================================================================

  describe("CLKREPAIR round trip") {
    it("DCLK: compare the trk word sent with the trk word PatternReader receives") {
      simulate(
        new LogPhyPhyBumpsHarness(exposeProbe = true),
        firtoolOpts = noAssertFirtoolOpts) { h =>
        coldStart(h)
        for (idx <- 0 to 2) {
          val st = ladder(idx)
          val res = stepUntil(h, st.guard, st.stride)(st.reached(h))
          assert(res.ok, s"DCLK could not reach ${st.id}: ${stateSummary(h)}")
        }
        traceStates(h, "DCLK at S2")

        val steps = 640 * h.cyclesPerDigital
        val p0 = new Array[BigInt](steps)
        val p1 = new Array[BigInt](steps)
        for (k <- 0 until steps) {
          p0(k) = h.io.phyProbe.get(0).peek().litValue
          p1(k) = h.io.phyProbe.get(1).peek().litValue
          h.clock.step(1)
        }

        val m32 = BigInt("ffffffff", 16)
        def bit(v: BigInt, b: Int): Int = ((v >> b) & 1).toInt
        // The three words a 48-bit 0x000055555555 pattern becomes at a 32-bit
        // serializer ratio (PatternReader.scala:147-155, :179-181).
        val refNames = Map(
          BigInt("55555555", 16) -> "phase0", BigInt("55550000", 16) -> "phase1",
          BigInt("00005555", 16) -> "phase2")
        def hist(vs: Seq[BigInt]): String =
          vs.groupBy(identity).toSeq.sortBy(-_._2.length).take(6).map { case (v, g) =>
            f"0x$v%08x${refNames.get(v).map("(" + _ + ")").getOrElse("")}=${g.length}"
          }.mkString(" ")

        for ((vs, die) <- Seq(p0.toSeq, p1.toSeq).zipWithIndex) {
          val digital = vs.sliding(2).collect {
            case Seq(a, b) if ((a >> 16) & 0xff) != ((b >> 16) & 0xff) => b
          }.toSeq
          val sending = digital.filter(v => bit(v, 11) == 1)
          val txTrk = sending.map(v => (v >> 64) & m32)
          // Only cycles PatternReader actually counts: counterEn is
          // sDetect && mbRxValid (PatternReader.scala:246), so idle cycles must
          // not dilute the histogram or "phase0 only" would be an artifact of
          // the AsyncQueue holding its last word.
          val delivering = digital.filter(v => bit(v, 10) == 1)
          val rxTrk = delivering.map(v => (v >> 96) & m32)
          val rxRises = vs.sliding(2).count(w => bit(w(0), 13) == 0 && bit(w(1), 13) == 1)

          // What the lane carries when the writer is NOT bursting. clkP and trk
          // carry the same word in CLKREPAIR (PatternWriter.scala:231-233), so
          // this says whether the forwarded clock is alive between bursts and
          // therefore whether an rxDivClk edge count can be attributed at all.
          val idle = digital.filter(v => bit(v, 11) == 0)
          val idleTrk = idle.map(v => (v >> 64) & m32)

          println(s"[phybumps] DCLK die$die over ${digital.length} digital cycles, " +
            s"${sending.length} with mainband tx.valid high, " +
            s"${delivering.length} with mbRxValid high:")
          println(s"[phybumps]   trk SENT     to the serializer : ${hist(txTrk)}")
          println(s"[phybumps]   trk RECEIVED at PatternReader  : ${hist(rxTrk)}")
          println(s"[phybumps]   trk on the lane while tx.valid is LOW : ${hist(idleTrk)}")
          println(s"[phybumps]   rxDivClk rises = $rxRises  (a clock that never " +
            s"stalls would give ${digital.length})")
        }
        traceStates(h, "DCLK end")
      }
    }
  }

}
