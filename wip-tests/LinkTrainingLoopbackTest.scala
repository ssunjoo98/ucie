package edu.berkeley.cs.uciedigital.logphy

import chisel3._
import chisel3.simulator.scalatest.ChiselSim
import org.scalatest.funspec.AnyFunSpec
import edu.berkeley.cs.uciedigital.interfaces._

import scala.collection.mutable

/*
  Run:
    ./mill --no-daemon test.testOnly edu.berkeley.cs.uciedigital.logphy.LinkTrainingLoopbackTest

  Coverage:
    Full training:  two complete LogicalPhy instances cross-wired at the
                    analog boundary (LogPhyLoopbackHarness) walk the entire
                    LTSM graph on real signals: RESET minimum wait ->
                    SBINIT clock-pattern detection / OUT_OF_RESET / DONE
                    req-rsp over the 1-bit serial sideband (SBInit.scala),
                    MBINIT PARAM negotiation (128b sideband exchange,
                    MBInitSM.scala:461-487) and CAL/REPAIRCLK/REPAIRVAL/
                    REVERSALMB/REPAIRMB with the real PatternWriter/Reader
                    over the crossed mainband, all twelve MBTRAIN substates
                    including the mandatory LINKSPEED TX D2C point test
                    (MBTrainSM.scala), LINKINIT's RDI REQ/RSP_ACTIVE sideband
                    handshake (RDIStateMachine.scala:139-145, forced from
                    LTSM LINKINIT via RDIController.scala:43-45), and ACTIVE.
    Trigger:        swStartLinkTraining is edge-detected
                    (LinkTrainingSM.scala freshTrainingTrigger), so the pulse
                    is applied only after the RESET minimum wait
                    (ltsmTimeoutCycles/2); an edge before it would be lost.
                    lpStateReq stays nop: poking it active early would start
                    the RDI ACTIVE sideband exchange during SBINIT, where
                    SBInit's consume-all RX ready collides with the RDI
                    responder's ready (LogicalPhy.scala:199-207 one-consumer
                    assertion).
    Ordering:       the 26-value LTSMState debug encoding is numerically
                    ordered along the bring-up path (Types.scala:78-105), so
                    the sampled state must be monotonically non-decreasing
                    per die up to ACTIVE, and the repair/retrain/error states
                    (sMBTRAIN_REPAIR, sPHYRETRAIN, sTRAINERROR) must never be
                    observed -- a LINKSPEED point-test failure would surface
                    as exactly those states.
    Enablement:     PhyLaneTrainer autoRespond=true closes the four
                    calibration handshakes; the companion test below pins the
                    default-stub behavior instead.
    Stub blocker:   with the default PhyLaneTrainer (autoRespond=false),
                    mbInit.selfCalDone is constant false
                    (PhyLaneTrainer.scala), so MBINIT.CAL never sends its
                    DONE request (MBInitSM.scala:489-501 sCAL) and training
                    stalls until the residency timeout: trainingTimedout ->
                    TrainError REQ/RSP over the real sideband
                    (LinkTrainingSM.scala TrainError logic) -> TRAINERROR ->
                    RESET re-entry, RDI escalates to linkError
                    (RDIStateMachine.scala:134-138) and plTrainError latches
                    after the single episode exhausts retryTrainingAmt=0.
                    This is the documented blocker for reaching ACTIVE with
                    unmodified RTL. Timeouts here use the parameterized
                    ltsmTimeoutCycles (LinkTrainingSM.scala); hardware keeps
                    the 6.4M-cycle default.

  NOTE: guard values are sized from measured runs (full bring-up completes in
  about 21k cycles past the trigger with ltsmTimeoutCycles=16384; the stub run
  needs the CAL residency timeout plus the TrainError handshake).
*/
class LinkTrainingLoopbackTest extends AnyFunSpec with ChiselSim {

  // Timeout scale for the happy path: RESET minimum wait is half of this.
  private val trainTimeoutCycles = 16384
  private val trainResetWait = trainTimeoutCycles / 2

  // Timeout scale for the stub-blocked run (stalls for a full timeout, so
  // keep it small).
  private val stubTimeoutCycles = 4096
  private val stubResetWait = stubTimeoutCycles / 2

  private val debugTrace = true

  private def bothDies(cond: Int => Boolean): Boolean = cond(0) && cond(1)

  // Drive every harness input to a quiet default before a scenario starts.
  private def initHarness(h: LogPhyLoopbackHarness): Unit = {
    for (i <- 0 until 2) {
      h.io.lpStateReq(i).poke(RDIStateReq.nop)
      h.io.swStartLinkTraining(i).poke(false.B)
    }
  }

  private def ltsmCode(h: LogPhyLoopbackHarness, i: Int): BigInt =
    h.io.ltsmState(i).peek().litValue

  /** Per-die record of every sampled LTSM state: first-seen set plus a
    * monotonicity check over the numerically ordered bring-up encoding.
    */
  private class StateTrace {
    val seen = mutable.LinkedHashSet[BigInt]()
    private var last: BigInt = -1
    var monotonic = true
    def record(v: BigInt): Unit = {
      if (v < last) monotonic = false
      last = v
      seen += v
    }
    def sawState(s: LTSMState.Type): Boolean = seen.contains(s.litValue)
  }

  /** Step in `stride`-cycle chunks until cond holds or the guard expires,
    * recording both dies' LTSM states at every sample. Returns whether cond
    * held. Sampling is safe because every asserted milestone is either
    * sticky or a multi-hundred-cycle state.
    */
  private def waitForTracking(
      h: LogPhyLoopbackHarness,
      traces: Seq[StateTrace],
      guard: Int,
      stride: Int = 8,
  )(cond: => Boolean): Boolean = {
    var n = 0
    def sample(): Unit = {
      for (i <- 0 until 2) {
        val v = ltsmCode(h, i)
        if (debugTrace && !traces(i).seen.contains(v)) {
          println(s"[trace] cycle~$n die $i entered LTSM state 0x${v.toString(16)}")
        }
        traces(i).record(v)
      }
    }
    sample()
    while (!cond && n < guard) {
      h.clock.step(stride)
      n += stride
      sample()
    }
    cond
  }

  describe("LogicalPhy two-instance loopback link training") {

    it("walks RESET -> SBINIT -> MBINIT -> MBTRAIN -> LINKINIT -> ACTIVE on both dies") {
      simulate(new LogPhyLoopbackHarness(
        ltsmTimeoutCycles = trainTimeoutCycles,
        phyTrainerAutoRespond = true,
      )) { h =>
        initHarness(h)

        // Phase 0: RESET residency. Both dies must sit in RESET with the RDI
        // reporting reset and no inband presence.
        h.clock.step(64)
        for (i <- 0 until 2) {
          h.io.ltsmState(i).expect(LTSMState.sRESET, "LTSM must start in RESET")
          h.io.plStateSts(i).expect(RDIState.reset, "RDI must start in reset")
          h.io.plInbandPres(i).expect(false.B, "no inband presence in RESET")
        }

        // Step past the RESET minimum wait (ltsmTimeoutCycles/2). Without a
        // trigger the LTSM must still be parked in RESET afterwards.
        h.clock.step(trainResetWait + 64)
        for (i <- 0 until 2) {
          h.io.ltsmState(i).expect(LTSMState.sRESET, "no training without a trigger")
        }

        // Trigger: software start pulse on both dies (edge-detected; only
        // valid now that the minimum wait has elapsed).
        h.io.swStartLinkTraining(0).poke(true.B)
        h.io.swStartLinkTraining(1).poke(true.B)
        h.clock.step(4)
        h.io.swStartLinkTraining(0).poke(false.B)
        h.io.swStartLinkTraining(1).poke(false.B)

        val traces = Seq.fill(2)(new StateTrace)

        // Phase 1: SBINIT entry on both dies (a couple of cycles after the
        // trigger edge).
        assert(
          waitForTracking(h, traces, guard = 64, stride = 1) {
            bothDies(i => ltsmCode(h, i) == LTSMState.sSBINIT.litValue)
          },
          "both dies must enter SBINIT right after the trigger",
        )
        for (i <- 0 until 2) {
          h.io.plPhyInRecenter(i).expect(true.B, "plPhyInRecenter must be high in SBINIT")
        }

        // Phase 2: SBINIT completion is observable through the sticky
        // negotiated-parameter flag set by MBINIT.PARAM, which can only be
        // reached after the SBINIT DONE req/rsp handshake.
        assert(
          waitForTracking(h, traces, guard = 20000) {
            bothDies(i => h.io.negotiatedParamsValid(i).peekBoolean())
          },
          "both dies must finish SBINIT and negotiate PHY parameters in MBINIT.PARAM",
        )
        for (i <- 0 until 2) {
          assert(!h.io.trainingTimedout(i).peekBoolean(), s"die $i timed out during SBINIT/PARAM")
          assert(!h.io.sbFaultSeen(i).peekBoolean(), s"die $i saw a sideband fault during SBINIT/PARAM")
        }

        // Phase 3: full MBINIT + MBTRAIN + LINKINIT to ACTIVE, with the RDI
        // state machines completing their REQ/RSP_ACTIVE exchange.
        assert(
          waitForTracking(h, traces, guard = 60000) {
            bothDies { i =>
              ltsmCode(h, i) == LTSMState.sACTIVE.litValue &&
              h.io.plStateSts(i).peek().litValue == RDIState.active.litValue
            }
          },
          "both dies must reach LTSM ACTIVE with RDI active",
        )

        // Health: no timeout, no fatal error, no sideband fault, no repair
        // or speed degrade along the way.
        for (i <- 0 until 2) {
          assert(!h.io.trainingTimedout(i).peekBoolean(), s"die $i hit the residency timeout")
          assert(!h.io.plTrainError(i).peekBoolean(), s"die $i reported plTrainError")
          assert(!h.io.sbFaultSeen(i).peekBoolean(), s"die $i saw a sideband fault")
          h.io.plInbandPres(i).expect(true.B, "inband presence must be set from LINKINIT onwards")
          h.io.plPhyInRecenter(i).expect(false.B, "plPhyInRecenter must drop in ACTIVE")
          h.io.plSpeedmode(i).expect(SpeedMode.speed4, "speed4 negotiation must not change the clock")
        }

        // Ordering: the sampled debug state walked the bring-up encoding
        // monotonically and hit the key milestones on both dies.
        for (i <- 0 until 2) {
          val t = traces(i)
          assert(t.monotonic, s"die $i LTSM state regressed during bring-up: ${t.seen}")
          for (milestone <- Seq(
              LTSMState.sSBINIT,
              LTSMState.sMBINIT_PARAM,
              LTSMState.sMBINIT_REPAIRCLK,
              LTSMState.sMBINIT_REVERSALMB,
              LTSMState.sMBTRAIN_LINKSPEED,
              LTSMState.sLINKINIT,
              LTSMState.sACTIVE,
            )) {
            assert(t.sawState(milestone), s"die $i never showed $milestone; saw ${t.seen}")
          }
          for (forbidden <- Seq(
              LTSMState.sMBTRAIN_REPAIR,
              LTSMState.sPHYRETRAIN,
              LTSMState.sTRAINERROR,
            )) {
            assert(!t.sawState(forbidden), s"die $i entered $forbidden during a clean bring-up")
          }
        }

        // Stability: ACTIVE must hold with the inputs left as-is.
        h.clock.step(500)
        for (i <- 0 until 2) {
          h.io.ltsmState(i).expect(LTSMState.sACTIVE, "ACTIVE must be stable")
          h.io.plStateSts(i).expect(RDIState.active, "RDI active must be stable")
          h.io.plInbandPres(i).expect(true.B, "inband presence must be stable")
        }
      }
    }

    it("documents the PhyLaneTrainer stub blocker: MBINIT.CAL stalls into timeout, TrainError, and RDI linkError") {
      simulate(new LogPhyLoopbackHarness(
        ltsmTimeoutCycles = stubTimeoutCycles,
        phyTrainerAutoRespond = false,
      )) { h =>
        initHarness(h)

        h.clock.step(stubResetWait + 64)
        h.io.swStartLinkTraining(0).poke(true.B)
        h.io.swStartLinkTraining(1).poke(true.B)
        h.clock.step(4)
        h.io.swStartLinkTraining(0).poke(false.B)
        h.io.swStartLinkTraining(1).poke(false.B)

        val traces = Seq.fill(2)(new StateTrace)

        // SBINIT and MBINIT.PARAM are purely digital handshakes and still
        // complete against the stub trainer.
        assert(
          waitForTracking(h, traces, guard = 20000) {
            bothDies(i => h.io.negotiatedParamsValid(i).peekBoolean())
          },
          "SBINIT and MBINIT.PARAM must complete even with the stub trainer",
        )

        // BUG(stub): PhyLaneTrainer drives mbInit.selfCalDone constant false,
        // so MBINIT.CAL never issues its CAL_DONE request and both dies sit
        // in sMBINIT_CAL until the residency timeout fires. This pins the
        // current behavior; once a real lane trainer (or its integration) is
        // implemented, bring-up must instead proceed to REPAIRCLK and this
        // test should be updated.
        assert(
          waitForTracking(h, traces, guard = stubTimeoutCycles + 4000) {
            bothDies(i => h.io.trainingTimedout(i).peekBoolean())
          },
          "expected the documented MBINIT.CAL stall to end in the residency timeout; " +
            "if training progressed, the PhyLaneTrainer stub was implemented -- update this test",
        )

        // The dies never got past CAL.
        for (i <- 0 until 2) {
          val t = traces(i)
          assert(t.sawState(LTSMState.sMBINIT_CAL), s"die $i never reached MBINIT.CAL; saw ${t.seen}")
          val beyondCal = t.seen.filter(v =>
            v > LTSMState.sMBINIT_CAL.litValue && v < LTSMState.sTRAINERROR.litValue)
          assert(beyondCal.isEmpty, s"die $i progressed past MBINIT.CAL with the stub trainer: $beyondCal")
        }

        // Timeout escalation: TrainError handshake over the real sideband,
        // TRAINERROR entry, RESET re-entry; with retryTrainingAmt=0 the
        // single episode is fatal (plTrainError) and the RDI SM escalates to
        // linkError via the LINKMGMT_RDI_REQ/RSP_LINKERROR exchange.
        assert(
          waitForTracking(h, traces, guard = 2 * stubTimeoutCycles + 8000) {
            bothDies { i =>
              ltsmCode(h, i) == LTSMState.sRESET.litValue &&
              h.io.plStateSts(i).peek().litValue == RDIState.linkError.litValue &&
              h.io.plTrainError(i).peekBoolean()
            }
          },
          "both dies must fall back to RESET with RDI linkError and a fatal training error",
        )
        for (i <- 0 until 2) {
          assert(traces(i).sawState(LTSMState.sTRAINERROR),
            s"die $i never showed TRAINERROR on the way down; saw ${traces(i).seen}")
        }
      }
    }
  }
}
