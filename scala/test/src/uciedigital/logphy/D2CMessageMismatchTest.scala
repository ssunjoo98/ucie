package edu.berkeley.cs.uciedigital.logphy

import edu.berkeley.cs.uciedigital.UcieSimPrefs
import chisel3._
import chisel3.simulator.scalatest.ChiselSim
import edu.berkeley.cs.uciedigital.sideband._
import org.scalatest.funspec.AnyFunSpec

/*
  Run:
    ./mill --no-daemon test.testOnly edu.berkeley.cs.uciedigital.logphy.D2CMessageMismatchTest

  Coverage:
    Encoding pins:  the exact requester-TX-message vs responder-reference
                    pairs used by the MBTRAIN handshakes, evaluated with the
                    RTL's own SBMsgCreate/SBMsgCompare (the same functions the
                    FSMs elaborate), so a mismatch here is exactly a deadlock
                    in silicon:
                      - MBTRAIN.SPEEDIDLE DONE req and resp directions
                        (MBTrainSM.scala:677/680 requester vs :1793/:1796
                        responder)
                      - TX-init D2C eye sweep END handshake
                        (TxD2CEyeWidthSweep.scala:179/181 requester vs
                        :336/:346 responder)
                      - RX-init D2C eye sweep START handshake
                        (RxD2CEyeWidthSweep.scala:139/144 requester vs
                        :317/:322 responder)
                    plus matched-pair positive controls proving the probe
                    itself accepts a correct pairing.
    Start deadlock: RxD2CPointTestRequester driven exactly as the LTSM would
                    (start held, sideband TX ready, pattern reader idle) must
                    emit its START request -- pinned to the current broken
                    behavior (never emits anything).

  These pin the four "B. suspected bugs" from
  ~/project/ucie/study/scenarios/00_fsm_states/ALL_FSMS.md section 2 that live
  in the MBTRAIN chain. They were unreachable in the loopback
  because training parks earlier, in MBINIT.CAL (PhyLaneTrainer stub), so they
  are pinned at unit level here.
*/

// Elaborates each requester-produced message against the responder's
// rxRefBitPattern, using the same SBMsgCreate/SBMsgCompare the RTL uses.
// A `false` accepted-output is a proven req/resp deadlock.
class SbMsgCompatProbe extends Module {
  val io = IO(new Bundle {
    // BUG probes (all expected false on current RTL).
    val speedIdleDoneReqAccepted = Output(Bool())
    val speedIdleDoneRespAccepted = Output(Bool())
    val txEyeSweepEndReqAccepted = Output(Bool())
    val rxEyeSweepStartReqAccepted = Output(Bool())
    // Positive controls (expected true; validate the probe method).
    val speedIdleSelfPairAccepted = Output(Bool())
    val linkSpeedSelfPairAccepted = Output(Bool())
  })

  // MBTRAIN.SPEEDIDLE DONE, requester -> responder direction: the requester sends
  // SPEEDIDLE_DONE_REQ and the responder now arms the same name.
  io.speedIdleDoneReqAccepted := SBMsgCompare(
    SBMsgCreate(SBM.MBTRAIN_SPEEDIDLE_DONE_REQ, "PHY", "PHY", true),
    SBM.MBTRAIN_SPEEDIDLE_DONE_REQ)

  // MBTRAIN.SPEEDIDLE DONE, responder -> requester direction.
  io.speedIdleDoneRespAccepted := SBMsgCompare(
    SBMsgCreate(SBM.MBTRAIN_SPEEDIDLE_DONE_RESP, "PHY", "PHY", true),
    SBM.MBTRAIN_SPEEDIDLE_DONE_RESP)

  // TX-init D2C eye sweep END: requester sends END_TX_INIT_D2C_EYE_SWEEP_REQ
  // (subcode 0x06, TxD2CEyeWidthSweep.scala:179); responder matches only
  // END_TX_INIT_D2C_POINT_TEST_REQ (subcode 0x04, :336).
  io.txEyeSweepEndReqAccepted := SBMsgCompare(
    SBMsgCreate(SBM.END_TX_INIT_D2C_EYE_SWEEP_REQ, "PHY", "PHY", true),
    SBM.END_TX_INIT_D2C_POINT_TEST_REQ)

  // RX-init D2C eye sweep START: requester sends
  // START_RX_INIT_D2C_EYE_SWEEP_REQ (subcode 0x0A,
  // RxD2CEyeWidthSweep.scala:139); responder waits for the TX-init name
  // START_TX_INIT_D2C_EYE_SWEEP_REQ (subcode 0x05, :317).
  io.rxEyeSweepStartReqAccepted := SBMsgCompare(
    SBMsgCreate(SBM.START_RX_INIT_D2C_EYE_SWEEP_REQ, "PHY", "PHY", true),
    SBM.START_TX_INIT_D2C_EYE_SWEEP_REQ)

  // Controls: a message compared against its own base must be accepted.
  io.speedIdleSelfPairAccepted := SBMsgCompare(
    SBMsgCreate(SBM.MBTRAIN_SPEEDIDLE_DONE_REQ, "PHY", "PHY", true),
    SBM.MBTRAIN_SPEEDIDLE_DONE_REQ)
  io.linkSpeedSelfPairAccepted := SBMsgCompare(
    SBMsgCreate(SBM.MBTRAIN_LINKSPEED_DONE_REQ, "PHY", "PHY", true),
    SBM.MBTRAIN_LINKSPEED_DONE_REQ)
}

class D2CMessageMismatchTest extends AnyFunSpec with ChiselSim with UcieSimPrefs {

  describe("MBTRAIN sideband message compatibility (requester TX vs responder reference)") {

    it("pins the SPEEDIDLE/LINKSPEED and eye-sweep START/END name mismatches") {
      simulate(new SbMsgCompatProbe) { probe =>
        probe.clock.step(2)

        // Positive controls first: the probe method accepts matched pairs.
        probe.io.speedIdleSelfPairAccepted.expect(true.B,
          "control: SPEEDIDLE_DONE_REQ must match its own base encoding")
        probe.io.linkSpeedSelfPairAccepted.expect(true.B,
          "control: LINKSPEED_DONE_REQ must match its own base encoding")

        // REGRESSION: MBTRAIN.SPEEDIDLE used to deadlock because the responder
        // armed the LINKSPEED names (subcode 0x19) against the requester's
        // SPEEDIDLE names (subcode 0x04). Both directions must now pair.
        probe.io.speedIdleDoneReqAccepted.expect(true.B,
          "SPEEDIDLE DONE req must be accepted by the responder reference")
        probe.io.speedIdleDoneRespAccepted.expect(true.B,
          "SPEEDIDLE DONE resp must be accepted by the requester reference")

        // BUG: TX-initiated D2C eye sweep END deadlock. Requester emits the
        // EYE_SWEEP END name (subcode 0x06, TxD2CEyeWidthSweep.scala:179);
        // responder state 3/4 matches the POINT_TEST END name (subcode 0x04,
        // :336/:346). The sweep can start but never terminate cleanly.
        probe.io.txEyeSweepEndReqAccepted.expect(false.B,
          "TX eye-sweep END req unexpectedly accepted -- mismatch fixed, update this pin")

        // BUG: RX-initiated D2C eye sweep START deadlock. Requester emits
        // START_RX_INIT_D2C_EYE_SWEEP_REQ (subcode 0x0A,
        // RxD2CEyeWidthSweep.scala:139); responder state 0 waits for the
        // TX-init name (subcode 0x05, :317), so an RX-initiated sweep never
        // starts.
        probe.io.rxEyeSweepStartReqAccepted.expect(false.B,
          "RX eye-sweep START req unexpectedly accepted -- mismatch fixed, update this pin")
      }
    }
  }

  describe("RxD2CPointTestRequester start behavior") {

    it("pins the start deadlock: the START request is never emitted on the sideband") {
      simulate(new RxD2CPointTestRequester(new AfeParams(), new SidebandParams())) { dut =>
        // Drive exactly what the LTSM would drive in MBTRAIN when kicking off
        // an RX-initiated point test: start held, a legal pattern type, an
        // always-ready sideband TX, quiet sideband RX, pattern reader idle.
        dut.io.start.poke(true.B)
        dut.io.patternType.poke(PatternSelect.VALTRAIN)
        dut.io.linkTrainingParameters.clockPhase.poke(0.U)
        dut.io.linkTrainingParameters.dataPattern.poke(0.U)
        dut.io.linkTrainingParameters.validPattern.poke(0.U)
        dut.io.linkTrainingParameters.patternMode.poke(0.U)
        dut.io.linkTrainingParameters.iterationCount.poke(0.U)
        dut.io.linkTrainingParameters.idleCount.poke(0.U)
        dut.io.linkTrainingParameters.burstCount.poke(0.U)
        dut.io.linkTrainingParameters.maxErrorThreshold.poke(0.U)
        dut.io.linkTrainingParameters.comparisonMode.poke(0.U)
        dut.io.sbLaneIo.tx.ready.poke(true.B)
        dut.io.sbLaneIo.rx.valid.poke(false.B)
        dut.io.sbLaneIo.rx.bits.data.poke(0.U)
        dut.io.patternReaderIo.req.ready.poke(true.B)
        dut.io.patternReaderIo.resp.valid.poke(false.B)

        // BUG: RxD2CPointTest.scala:117-123 -- state 0 assigns
        // rxRefBitPattern.valid TWICE (117 and 122) but never asserts
        // sbMsgExchanger.io.req.valid, so START_RX_INIT_D2C_POINT_TEST_REQ
        // is never handed to the exchanger, exchDone can never fire, and the
        // requester is dead on arrival. Sampled every cycle: the sideband TX
        // must stay silent and done must never rise.
        for (n <- 0 until 2048) {
          assert(!dut.io.sbLaneIo.tx.valid.peekBoolean(),
            s"cycle $n: START request emitted -- the req.valid bug was fixed; " +
              "rewrite this pin as a progression test")
          assert(!dut.io.done.peekBoolean(), s"cycle $n: done rose without any exchange")
          dut.clock.step(1)
        }
      }
    }
  }
}
