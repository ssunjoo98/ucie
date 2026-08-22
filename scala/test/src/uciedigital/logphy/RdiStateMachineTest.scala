package edu.berkeley.cs.uciedigital.logphy

import chisel3._
import chisel3.simulator.scalatest.ChiselSim
import org.scalatest.funspec.AnyFunSpec
import edu.berkeley.cs.uciedigital.interfaces._
import edu.berkeley.cs.uciedigital.sideband._

/*
  Run:
    ./mill --no-daemon test.testOnly edu.berkeley.cs.uciedigital.logphy.RdiStateMachineTest

  Coverage:
    LT-RDI-01: RDI Reset -> Active bring-up on a single RDIController DUT.
               Wake handshake and clock ungate ordering (RDIController.scala:35-39,
               RDIWakeHandshakeResponder.scala), clock handshake before any
               sideband traffic (RDIController.scala:53-66), the
               {LINKMGMT_RDI_REQ_ACTIVE -> LINKMGMT_RDI_RSP_ACTIVE} exchange
               (RDIStateMachine.scala:133-146, 254-258) with the response
               injected on sbLaneIo.rx, plInbandPres set on LINKINIT entry
               (RDIController.scala:150-154), doingRdiBringup window
               (RDIController.scala:158-159).
    LT-RDI-02: Active -> Retrain/LinkReset/Disabled requester transitions
               (RDIStateMachine.scala:147-172) each completing with the matching
               RSP injection, returns to Active from Retrain/LinkReset/Disabled,
               and remote-initiated (responder-path) Retrain -> LinkReset
               transitions (RDIStateMachine.scala:349-448).
    LT-RDI-08: Stall trigger conditions at the controller level
               (RDIController.scala:68-87): upper-layer state requests out of
               ACTIVE raise plStallReq; a valid framing error raises plStallReq
               only while the LTSM is in sACTIVE; documented-current-behavior
               test showing the state transition does not wait for lpStallAck.
    LT-RDI-03: PM L1/L2 requests rejected by PMNAK on both paths: a local
               lpStateReq=l1/l2 sends REQ_L1/REQ_L2, takes the injected
               RSP_PMNAK into activePmNak and returns to active once the
               request drops (RDIStateMachine.scala:71-73, 164-172, 259-266);
               a remote REQ_L1 is answered with RSP_PMNAK through the
               responder (RDIStateMachine.scala:373-378, 402-407).
    LT-RDI-04: trainingTimeout preempts ACTIVE into the {REQ,RSP}_LINKERROR
               exchange and linkError, and an lpStateReq=active recovers to
               active (RDIStateMachine.scala:148-151, 243-247, 271-274).
    LT-RDI-10: plInbandPres LTSM tracking (RDIController.scala:150-154):
               set on sLINKINIT/sACTIVE, held across intermediate states,
               cleared on sRESET/sTRAINERROR.

  Notes:
    The tests model the D2D adapter side of the clock/stall handshakes with a
    simple follow-the-request responder (stepAdapter). cfgSidebandActive is held
    high before and after bring-up so pl_clk_req stays asserted across
    transitions: the Verification assertions check combinational clock-keep
    conditions one cycle before the clock requester FSM can raise pl_clk_req,
    so any covered activity that starts from an idle clock handshake aborts the
    simulation. All three windows are documented as BUG tests at the bottom.
 */
class RdiStateMachineTest extends AnyFunSpec with ChiselSim {

  private val sbParams = new SidebandParams()

  private def mkDut = new RDIController(sbParams)

  // Sideband message field encodings, mirroring SBMsgCreate's bit layout
  // (SidebandMessageEncodings.scala:363-394). SBMsgCompare only checks
  // opcode [4:0], msgCode [21:14] and msgSubcode [39:32].
  private val OpMsgWithoutData = BigInt(0x12) // SBMsgOpcode.MessageWithoutData
  private val ReqCode = 0x01 // LINKMGMT_RDI_REQ_* msgCode
  private val RspCode = 0x02 // LINKMGMT_RDI_RSP_* msgCode
  private val SubActive = 0x01
  private val SubPmNak = 0x02
  private val SubL1 = 0x04
  private val SubL2 = 0x08
  private val SubLinkReset = 0x09
  private val SubLinkError = 0x0a
  private val SubRetrain = 0x0b
  private val SubDisable = 0x0c

  // Build a 128-bit sideband message the responder/requester comparators accept.
  // srcid = PHY (b010) at [31:29], dstid = remote|PHY (b110) at [58:56].
  private def sbMsg(msgCode: Int, msgSubcode: Int): BigInt =
    OpMsgWithoutData |
      (BigInt(msgCode) << 14) |
      (BigInt(2) << 29) |
      (BigInt(msgSubcode) << 32) |
      (BigInt(6) << 56)

  // Drive every DUT input to a quiet default before a scenario starts.
  private def initDut(dut: RDIController): Unit = {
    dut.io.rdi.lpStateReq.poke(RDIStateReq.nop)
    dut.io.rdi.lpWakeReq.poke(false.B)
    dut.io.rdi.lpClkAck.poke(false.B)
    dut.io.rdi.lpStallAck.poke(false.B)
    dut.io.sbLaneIo.tx.ready.poke(true.B)
    dut.io.sbLaneIo.rx.valid.poke(false.B)
    dut.io.sbLaneIo.rx.bits.data.poke(0.U)
    dut.io.ltsmState.poke(LTState.sRESET)
    dut.io.doRdiBringup.poke(false.B)
    dut.io.trainingTimeout.poke(false.B)
    dut.io.validFramingError.poke(false.B)
    dut.io.cfgSidebandActive.poke(false.B)
    dut.io.plPhyInRecenter.poke(false.B)
    dut.io.clocksUngatedAndStable.poke(true.B)
  }

  // Step one cycle while modeling the adapter side of the clock and stall
  // handshakes: lp*Ack follows pl*Req (like the D2D adapter's staged acks).
  private def stepAdapter(dut: RDIController): Unit = {
    dut.io.rdi.lpClkAck.poke(dut.io.rdi.plClkReq.peekBoolean().B)
    dut.io.rdi.lpStallAck.poke(dut.io.rdi.plStallReq.peekBoolean().B)
    dut.clock.step()
  }

  // Wait for a TX sideband message, check its {opcode, msgCode, msgSubcode}
  // encoding, and let the (always-ready) lane consume it.
  private def expectTxMsg(
      dut: RDIController,
      msgCode: Int,
      msgSubcode: Int,
      maxCycles: Int = 50
  ): Unit = {
    var cycles = 0
    while (!dut.io.sbLaneIo.tx.valid.peekBoolean() && cycles < maxCycles) {
      stepAdapter(dut)
      cycles += 1
    }
    assert(
      dut.io.sbLaneIo.tx.valid.peekBoolean(),
      s"DUT never presented the expected sideband message (msgCode=$msgCode, msgSubcode=$msgSubcode)"
    )
    val data = dut.io.sbLaneIo.tx.bits.data.peek().litValue
    assert(
      (data & 0x1f) == OpMsgWithoutData,
      s"TX opcode mismatch: got 0x${(data & 0x1f).toString(16)}, expected MessageWithoutData"
    )
    assert(
      ((data >> 14) & 0xff) == msgCode,
      s"TX msgCode mismatch: got 0x${((data >> 14) & 0xff)
          .toString(16)}, expected 0x${msgCode.toHexString}"
    )
    assert(
      ((data >> 32) & 0xff) == msgSubcode,
      s"TX msgSubcode mismatch: got 0x${((data >> 32) & 0xff)
          .toString(16)}, expected 0x${msgSubcode.toHexString}"
    )
    stepAdapter(dut) // tx.ready is held high, so the beat is consumed here
  }

  // Inject one RX sideband message and hold it until the DUT accepts it.
  private def injectRxMsg(
      dut: RDIController,
      msgCode: Int,
      msgSubcode: Int,
      maxCycles: Int = 50
  ): Unit = {
    dut.io.sbLaneIo.rx.valid.poke(true.B)
    dut.io.sbLaneIo.rx.bits.data.poke(sbMsg(msgCode, msgSubcode).U)
    var cycles = 0
    while (!dut.io.sbLaneIo.rx.ready.peekBoolean() && cycles < maxCycles) {
      stepAdapter(dut)
      cycles += 1
    }
    assert(
      dut.io.sbLaneIo.rx.ready.peekBoolean(),
      s"DUT never accepted the injected sideband message (msgCode=$msgCode, msgSubcode=$msgSubcode)"
    )
    stepAdapter(dut)
    dut.io.sbLaneIo.rx.valid.poke(false.B)
  }

  private def waitForState(
      dut: RDIController,
      state: RDIState.Type,
      maxCycles: Int = 50
  ): Unit = {
    var cycles = 0
    while (
      dut.io.rdi.plStateSts
        .peek()
        .litValue != state.litValue && cycles < maxCycles
    ) {
      stepAdapter(dut)
      cycles += 1
    }
    dut.io.rdi.plStateSts
      .expect(state, s"RDI state machine never reached $state")
  }

  // Complete the Reset -> Active bring-up and settle into a steady ACTIVE
  // state (ltsmState = sACTIVE, cfgSidebandActive holding the clock request).
  private def bringupToActive(dut: RDIController): Unit = {
    initDut(dut)
    dut.clock.step(2) // let resetReqObserved latch while lpStateReq == nop
    dut.io.rdi.plStateSts.expect(RDIState.reset)

    // Warm up the clock handshake before any covered activity starts:
    // plPhyInRecenter is the only clock-keep condition without a same-cycle
    // coverage assertion (RDIController.scala:53-60 vs 122-133), so it is the
    // only trigger that can legally raise the handshake from idle. Triggering
    // doRdiBringup or cfgSidebandActive with an idle clock handshake trips a
    // fatal assertion; see the documented assertion-window tests below.
    dut.io.plPhyInRecenter.poke(true.B)
    var cycles = 0
    while (!dut.io.rdi.plClkReq.peekBoolean() && cycles < 20) {
      dut.clock.step()
      cycles += 1
    }
    assert(
      dut.io.rdi.plClkReq.peekBoolean(),
      "plClkReq must rise for the clock warm-up"
    )
    dut.io.rdi.lpClkAck.poke(true.B)
    dut.clock.step()
    dut.io.cfgSidebandActive.poke(
      true.B
    ) // now covered: plClkReq is already high
    dut.io.plPhyInRecenter.poke(false.B)

    dut.io.doRdiBringup.poke(true.B)
    dut.io.doingRdiBringup
      .expect(true.B, "bring-up trigger must report doingRdiBringup")

    expectTxMsg(dut, ReqCode, SubActive)
    injectRxMsg(dut, RspCode, SubActive)
    waitForState(dut, RDIState.active)

    // Steady state: LT reached ACTIVE; keep the sideband cfg path active so
    // pl_clk_req stays asserted across later transitions (see the assertion
    // window documented in the last test of this file).
    dut.io.doRdiBringup.poke(false.B)
    dut.io.cfgSidebandActive.poke(true.B)
    dut.io.ltsmState.poke(LTState.sACTIVE)
    stepAdapter(dut)
  }

  // Locally requested transition out of ACTIVE: stall the upper layer, run the
  // {REQ, RSP} exchange, land in `target`, then release the stall.
  private def expectLocalTransition(
      dut: RDIController,
      req: RDIStateReq.Type,
      msgSubcode: Int,
      target: RDIState.Type
  ): Unit = {
    dut.io.rdi.plStallReq.expect(false.B)
    dut.io.rdi.lpStateReq.poke(req)
    stepAdapter(dut)
    dut.io.rdi.plStallReq.expect(
      true.B,
      "a state request out of ACTIVE must stall the upper layer (LT-RDI-08 trigger)"
    )
    expectTxMsg(dut, ReqCode, msgSubcode)
    injectRxMsg(dut, RspCode, msgSubcode)
    waitForState(dut, target)
    dut.io.rdi.lpStateReq.poke(RDIStateReq.nop)
    var cycles = 0
    while (dut.io.rdi.plStallReq.peekBoolean() && cycles < 20) {
      stepAdapter(dut)
      cycles += 1
    }
    dut.io.rdi.plStallReq.expect(
      false.B,
      "the stall must be released after the transition completes"
    )
  }

  // Return to ACTIVE from retrain/linkReset/disabled via the requester path.
  private def expectReturnToActive(dut: RDIController): Unit = {
    dut.io.rdi.lpStateReq.poke(RDIStateReq.active)
    expectTxMsg(dut, ReqCode, SubActive)
    injectRxMsg(dut, RspCode, SubActive)
    waitForState(dut, RDIState.active)
    dut.io.rdi.lpStateReq.poke(RDIStateReq.nop)
    stepAdapter(dut)
  }

  describe("RDIController Reset -> Active bring-up (LT-RDI-01)") {

    it("stays in RESET with quiet inputs and no sideband traffic") {
      simulate(mkDut) { dut =>
        initDut(dut)
        dut.clock.step(2)
        for (_ <- 0 until 8) {
          dut.io.rdi.plStateSts.expect(RDIState.reset)
          dut.io.rdi.plInbandPres.expect(false.B)
          dut.io.sbLaneIo.tx.valid
            .expect(false.B, "no sideband message may be sent in idle RESET")
          dut.clock.step()
        }
      }
    }

    it("wakes, ungates clocks, exchanges {REQ,RSP}_ACTIVE and reaches ACTIVE") {
      simulate(mkDut) { dut =>
        initDut(dut)
        dut.io.clocksUngatedAndStable.poke(false.B)
        dut.clock.step(2)
        dut.io.rdi.plStateSts.expect(RDIState.reset)
        dut.io.rdi.plClkReq.expect(false.B)

        // Wake handshake: the responder must ungate clocks before acking.
        dut.io.rdi.lpWakeReq.poke(true.B)
        dut.clock.step()
        dut.io.ungateClocks
          .expect(true.B, "a wake request must ungate clocks before the ack")
        dut.io.rdi.plWakeAck.expect(false.B)
        dut.clock.step(3)
        dut.io.rdi.plWakeAck
          .expect(false.B, "no wake ack while clocks are not stable")
        dut.io.clocksUngatedAndStable.poke(true.B)
        dut.clock.step(2)
        dut.io.rdi.plWakeAck
          .expect(true.B, "wake ack must follow once clocks are stable")

        // Warm up the clock handshake before LINKINIT entry (starting any
        // covered activity from an idle clock handshake trips a fatal
        // assertion; see the documented assertion-window tests below).
        dut.io.plPhyInRecenter.poke(true.B)
        var cycles = 0
        while (!dut.io.rdi.plClkReq.peekBoolean() && cycles < 20) {
          dut.clock.step()
          cycles += 1
        }
        assert(
          dut.io.rdi.plClkReq.peekBoolean(),
          "plClkReq must rise before sideband traffic"
        )
        dut.io.rdi.lpClkAck.poke(true.B)
        dut.clock.step()
        dut.io.cfgSidebandActive.poke(true.B)
        dut.io.plPhyInRecenter.poke(false.B)

        // LTSM entering LINKINIT forces the bring-up request and sets
        // pl_inband_pres one cycle later.
        dut.io.ltsmState.poke(LTState.sLINKINIT)
        dut.io.doingRdiBringup.expect(true.B)
        dut.clock.step()
        dut.io.rdi.plInbandPres
          .expect(true.B, "LINKINIT entry must set plInbandPres")

        expectTxMsg(dut, ReqCode, SubActive)
        dut.io.rdi.plStateSts.expect(
          RDIState.reset,
          "the state must not change before the RSP_ACTIVE response arrives"
        )

        injectRxMsg(dut, RspCode, SubActive)
        waitForState(dut, RDIState.active)
        dut.io.doingRdiBringup.expect(false.B)
        dut.io.rdi.plInbandPres.expect(true.B)
      }
    }

    it("completes bring-up via doRdiBringup with the adapter-model helper") {
      simulate(mkDut) { dut =>
        bringupToActive(dut)
        dut.io.rdi.plStateSts.expect(RDIState.active)
        dut.io.rdi.plInbandPres
          .expect(true.B, "sACTIVE must keep plInbandPres set")
      }
    }
  }

  describe("RDIController Active -> Retrain/LinkReset/Disabled (LT-RDI-02)") {

    it("transitions ACTIVE -> RETRAIN on a local request and back to ACTIVE") {
      simulate(mkDut) { dut =>
        bringupToActive(dut)
        expectLocalTransition(
          dut,
          RDIStateReq.retrain,
          SubRetrain,
          RDIState.retrain
        )
        expectReturnToActive(dut)
      }
    }

    it(
      "transitions ACTIVE -> LINKRESET on a local request and back to ACTIVE"
    ) {
      simulate(mkDut) { dut =>
        bringupToActive(dut)
        expectLocalTransition(
          dut,
          RDIStateReq.linkReset,
          SubLinkReset,
          RDIState.linkReset
        )
        expectReturnToActive(dut)
      }
    }

    it("transitions ACTIVE -> DISABLED on a local request and back to ACTIVE") {
      simulate(mkDut) { dut =>
        bringupToActive(dut)
        expectLocalTransition(
          dut,
          RDIStateReq.disabled,
          SubDisable,
          RDIState.disabled
        )
        expectReturnToActive(dut)
      }
    }

    it(
      "responds to remote REQ_RETRAIN then remote REQ_LINKRESET (responder path)"
    ) {
      simulate(mkDut) { dut =>
        bringupToActive(dut)

        // Remote die requests retrain: the responder must send RSP_RETRAIN and
        // move the state to RETRAIN without any local lpStateReq.
        injectRxMsg(dut, ReqCode, SubRetrain)
        // NOTE: current RTL only stalls the upper layer for *local* state
        // requests out of ACTIVE (RDIController.scala:75-82); a remote-initiated
        // transition does not raise plStallReq.
        dut.io.rdi.plStallReq.expect(
          false.B,
          "current behavior: remote-initiated transitions do not stall the upper layer"
        )
        expectTxMsg(dut, RspCode, SubRetrain)
        waitForState(dut, RDIState.retrain)

        // Remote die then requests link reset from RETRAIN.
        injectRxMsg(dut, ReqCode, SubLinkReset)
        expectTxMsg(dut, RspCode, SubLinkReset)
        waitForState(dut, RDIState.linkReset)
      }
    }
  }

  describe("RDIController PM L1/L2 PMNAK rejection (LT-RDI-03)") {

    it(
      "first local PM request completes a PMNAK round trip; documents that a second one wedges"
    ) {
      simulate(mkDut) { dut =>
        bringupToActive(dut)

        var firstExchangeDone = false
        for (
          (req, sub) <- Seq((RDIStateReq.l1, SubL1), (RDIStateReq.l2, SubL2))
        ) {
          // A PM request out of ACTIVE stalls the upper layer like any other
          // local state request (RDIController.scala:75-84).
          dut.io.rdi.plStallReq.expect(false.B)
          dut.io.rdi.lpStateReq.poke(req)
          // The stall requester registers startStall, so plStallReq rises a
          // couple of cycles after the request is seen (RDIController.scala:84).
          var riseCycles = 0
          while (!dut.io.rdi.plStallReq.peekBoolean() && riseCycles < 8) {
            stepAdapter(dut)
            riseCycles += 1
          }
          dut.io.rdi.plStallReq.expect(
            true.B,
            s"a PM request (subcode 0x${sub.toHexString}) out of ACTIVE must stall the upper layer"
          )

          if (!firstExchangeDone) {
            // First PM request: full round trip works. REQ_L1 goes out, the
            // injected PMNAK lands the SM in activePmNak (never l1/l2 — PM
            // entry is unimplemented, RDIStateMachine.scala:259-266), and
            // dropping the request returns to active.
            expectTxMsg(dut, ReqCode, sub)
            injectRxMsg(dut, RspCode, SubPmNak)
            waitForState(dut, RDIState.activePmNak)
            dut.io.rdi.lpStateReq.poke(RDIStateReq.nop)
            stepAdapter(dut)
            waitForState(dut, RDIState.active)
            var rel = 0
            while (dut.io.rdi.plStallReq.peekBoolean() && rel < 20) {
              stepAdapter(dut)
              rel += 1
            }
            dut.io.rdi.plStallReq.expect(
              false.B,
              "the stall must be released after the PMNAK round trip"
            )
            firstExchangeDone = true
          } else {
            // BUG: a SECOND PM request after a completed PMNAK round trip
            // never emits its REQ_* beat — the requester wedges with the
            // stall held even though the SM re-arms pmL1/pmL2 from active.
            // Same one-shot pattern as the adapter's ADV_CAP race. Flip this
            // branch when the RTL is fixed.
            var txSeen = false
            var waitC = 0
            while (!txSeen && waitC < 50) {
              txSeen = dut.io.sbLaneIo.tx.valid.peekBoolean()
              if (!txSeen) { stepAdapter(dut); waitC += 1 }
            }
            assert(
              !txSeen,
              s"second REQ (subcode 0x${sub.toHexString}) unexpectedly appeared — PM path was fixed, update this test"
            )

            // BUG(continued): dropping the wedged request does NOT release
            // the stall. releaseStall requires leaving active
            // (RDIController.scala:85, currentState =/= active), but the
            // wedged exchange never left active — so plStallReq sticks high
            // forever. The upper layer is permanently stalled.
            dut.io.rdi.lpStateReq.poke(RDIStateReq.nop)
            var stuck = 0
            while (dut.io.rdi.plStallReq.peekBoolean() && stuck < 20) {
              stepAdapter(dut)
              stuck += 1
            }
            dut.io.rdi.plStallReq.expect(
              true.B,
              "current behavior: the stall never releases after a wedged PM request (sticky stall bug)"
            )
            waitForState(dut, RDIState.active)
          }
        }
      }
    }

    it("answers a remote REQ_L1 with RSP_PMNAK and settles back in ACTIVE") {
      simulate(mkDut) { dut =>
        bringupToActive(dut)

        // Remote die requests L1: the responder must reply RSP_PMNAK
        // (RDIStateMachine.scala:373-378, 402-407) with target activePmNak.
        injectRxMsg(dut, ReqCode, SubL1)
        dut.io.rdi.plStallReq.expect(
          false.B,
          "current behavior: remote-initiated transitions do not stall the upper layer"
        )
        expectTxMsg(dut, RspCode, SubPmNak)

        // With lpStateReq at nop the activePmNak visit is a single-cycle
        // bounce back to active (RDIStateMachine.scala:71-73): the state may
        // only ever be active or activePmNak, and it must settle in active.
        val legal = Set(RDIState.active.litValue, RDIState.activePmNak.litValue)
        for (_ <- 0 until 6) {
          val s = dut.io.rdi.plStateSts.peek().litValue
          assert(
            legal.contains(s),
            s"remote REQ_L1 must never leave {active, activePmNak}, saw $s"
          )
          dut.io.rdi.plStallReq.expect(
            false.B,
            "current behavior: no stall across the remote PMNAK exchange"
          )
          stepAdapter(dut)
        }
        waitForState(dut, RDIState.active)
      }
    }
  }

  describe("RDIController trainingTimeout -> linkError (LT-RDI-04)") {

    it(
      "preempts ACTIVE into linkError on trainingTimeout and recovers to ACTIVE"
    ) {
      simulate(mkDut) { dut =>
        bringupToActive(dut)

        // A training timeout starts the {REQ,RSP}_LINKERROR exchange from
        // ACTIVE (RDIStateMachine.scala:148-151, 271-274).
        dut.io.trainingTimeout.poke(true.B)
        expectTxMsg(dut, ReqCode, SubLinkError)
        dut.io.trainingTimeout.poke(false.B)

        // NOTE: current RTL does not stall the upper layer for the
        // timeout-driven exit -- holdUpperLayerStall only covers local
        // lpStateReq transitions and framing errors (RDIController.scala:75-82).
        dut.io.rdi.plStallReq.expect(
          false.B,
          "current behavior: a timeout-driven linkError exit does not stall the upper layer"
        )

        injectRxMsg(dut, RspCode, SubLinkError)
        waitForState(dut, RDIState.linkError)

        // Recovery: linkError accepts an active request and runs the
        // {REQ,RSP}_ACTIVE exchange back to ACTIVE
        // (RDIStateMachine.scala:243-247).
        expectReturnToActive(dut)
        dut.io.rdi.plStateSts.expect(RDIState.active)
      }
    }
  }

  describe("RDIController plInbandPres LTSM tracking (LT-RDI-10)") {

    it(
      "sets plInbandPres on LT ACTIVE, holds it across training states, and clears it on RESET/TRAINERROR"
    ) {
      simulate(mkDut) { dut =>
        initDut(dut)
        dut.clock.step(2)

        // Warm up the clock handshake first (same reason as bringupToActive):
        // plInbandPres high in RDI RESET must be covered by plClkReq
        // (RDIController.scala:118-121).
        dut.io.plPhyInRecenter.poke(true.B)
        var cycles = 0
        while (!dut.io.rdi.plClkReq.peekBoolean() && cycles < 20) {
          dut.clock.step()
          cycles += 1
        }
        assert(
          dut.io.rdi.plClkReq.peekBoolean(),
          "plClkReq must rise for the clock warm-up"
        )
        dut.io.rdi.lpClkAck.poke(true.B)
        dut.clock.step()
        dut.io.cfgSidebandActive.poke(true.B)
        dut.io.plPhyInRecenter.poke(false.B)
        dut.io.rdi.plInbandPres
          .expect(false.B, "plInbandPres must be low before any LT progress")

        // Set rule: sACTIVE (like sLINKINIT, but without forcing a bring-up
        // exchange) raises inband presence one cycle later.
        dut.io.ltsmState.poke(LTState.sACTIVE)
        dut.clock.step()
        dut.io.rdi.plInbandPres
          .expect(true.B, "LT ACTIVE must set plInbandPres")

        // Hold rule: intermediate training states neither set nor clear.
        dut.io.ltsmState.poke(LTState.sMBTRAIN)
        for (_ <- 0 until 4) {
          dut.clock.step()
          dut.io.rdi.plInbandPres
            .expect(true.B, "intermediate LT states must hold plInbandPres")
        }

        // Clear rule: sTRAINERROR drops it.
        dut.io.ltsmState.poke(LTState.sTRAINERROR)
        dut.clock.step()
        dut.io.rdi.plInbandPres
          .expect(false.B, "LT TRAINERROR must clear plInbandPres")

        // Set again, then clear via sRESET.
        dut.io.ltsmState.poke(LTState.sACTIVE)
        dut.clock.step()
        dut.io.rdi.plInbandPres
          .expect(true.B, "LT ACTIVE must set plInbandPres again")
        dut.io.ltsmState.poke(LTState.sRESET)
        dut.clock.step()
        dut.io.rdi.plInbandPres
          .expect(false.B, "LT RESET must clear plInbandPres")
      }
    }
  }

  describe("RDIController stall trigger conditions (LT-RDI-08)") {

    it(
      "stalls on a valid framing error while LT is ACTIVE and holds the stall"
    ) {
      simulate(mkDut) { dut =>
        bringupToActive(dut)
        dut.io.rdi.plStallReq.expect(false.B)

        dut.io.validFramingError.poke(true.B)
        stepAdapter(dut)
        dut.io.rdi.plStallReq.expect(
          true.B,
          "a valid framing error in LT ACTIVE must stall the upper layer"
        )

        // Complete the stall handshake with the adapter model.
        var cycles = 0
        while (!dut.io.rdi.lpStallAck.peekBoolean() && cycles < 10) {
          stepAdapter(dut)
          cycles += 1
        }

        // NOTE: current RTL keeps the stall pending after the error clears
        // because releaseStall requires the RDI state to leave ACTIVE
        // (RDIController.scala:85). Document that plStallReq stays asserted.
        dut.io.validFramingError.poke(false.B)
        for (_ <- 0 until 4) {
          stepAdapter(dut)
          dut.io.rdi.plStallReq.expect(
            true.B,
            "current behavior: the stall persists in ACTIVE after the framing error clears"
          )
        }
      }
    }

    it("does not stall on a framing error while LT is not ACTIVE") {
      simulate(mkDut) { dut =>
        initDut(dut)
        dut.clock.step(2)
        dut.io.validFramingError.poke(true.B)
        for (_ <- 0 until 6) {
          dut.io.rdi.plStallReq.expect(
            false.B,
            "a framing error outside LT ACTIVE must not trigger a stall"
          )
          dut.clock.step()
        }
      }
    }

    it("documents that the state transition does not wait for lpStallAck") {
      simulate(mkDut) { dut =>
        bringupToActive(dut)

        // Keep the clock handshake alive but never answer the stall request.
        dut.io.rdi.lpClkAck.poke(true.B)
        dut.io.rdi.lpStallAck.poke(false.B)
        dut.io.rdi.lpStateReq.poke(RDIStateReq.retrain)
        dut.clock.step()
        dut.io.rdi.plStallReq.expect(true.B)

        // Walk the {REQ, RSP}_RETRAIN exchange with plain steps (no stall ack).
        var cycles = 0
        while (!dut.io.sbLaneIo.tx.valid.peekBoolean() && cycles < 20) {
          dut.clock.step()
          cycles += 1
        }
        assert(
          dut.io.sbLaneIo.tx.valid.peekBoolean(),
          "REQ_RETRAIN was never sent"
        )
        val data = dut.io.sbLaneIo.tx.bits.data.peek().litValue
        assert(
          ((data >> 32) & 0xff) == SubRetrain,
          "expected REQ_RETRAIN encoding"
        )
        dut.clock.step()

        dut.io.sbLaneIo.rx.valid.poke(true.B)
        dut.io.sbLaneIo.rx.bits.data.poke(sbMsg(RspCode, SubRetrain).U)
        cycles = 0
        while (!dut.io.sbLaneIo.rx.ready.peekBoolean() && cycles < 20) {
          dut.clock.step()
          cycles += 1
        }
        assert(
          dut.io.sbLaneIo.rx.ready.peekBoolean(),
          "RSP_RETRAIN was never accepted"
        )
        dut.clock.step()
        dut.io.sbLaneIo.rx.valid.poke(false.B)

        cycles = 0
        while (
          dut.io.rdi.plStateSts
            .peek()
            .litValue != RDIState.retrain.litValue && cycles < 20
        ) {
          dut.clock.step()
          cycles += 1
        }
        // BUG: the RDI controller starts and completes the sideband transition
        // without waiting for the stall handshake to finish
        // (RDIController.scala:84-87 vs RDIStateMachine.scala:147-159): the
        // state below reaches RETRAIN even though lpStallAck was never
        // asserted, and plStallReq is left pending in sWAIT_ACK_ASSERT.
        dut.io.rdi.plStateSts.expect(
          RDIState.retrain,
          "current behavior: the transition completes with the stall handshake still pending"
        )
        dut.io.rdi.plStallReq.expect(
          true.B,
          "current behavior: plStallReq is left pending after the transition"
        )
      }
    }
  }

  describe("RDIController clock coverage across transitions") {

    // The clock handshake requester needs one cycle (sIDLE -> sWAIT_ACK_ASSERT)
    // to raise plClkReq after keepClockRequested rises, but the Verification
    // assertions in RDIController.scala:122-133 check the *combinational*
    // request conditions. Any trigger that raises a clock-keep condition and a
    // covered activity in the same cycle therefore aborts the simulation.

    it(
      "documents the fatal assertion when bring-up starts with an idle clock handshake"
    ) {
      // BUG: asserting doRdiBringup while the clock handshake is idle makes
      // mustHoldClocksUntilStateChanges true combinationally (currentState is
      // RESET and effectiveLpStateReq is forced to active), but plClkReq can
      // only rise one cycle later, so the assertion "pl_clk_req must remain
      // asserted while leaving RESET/PM states" (RDIController.scala:122-125)
      // fires. Real bring-up only survives because cfg sideband activity has
      // already raised the clock handshake beforehand.
      val thrown = intercept[Throwable] {
        simulate(mkDut) { dut =>
          initDut(dut)
          dut.clock.step(2)
          dut.io.rdi.plClkReq.expect(false.B)
          dut.io.doRdiBringup.poke(true.B)
          for (_ <- 0 until 4) stepAdapter(dut)
        }
      }
      assert(
        thrown != null,
        "expected the documented bring-up clock-hold assertion to fire; if this " +
          "now passes the assertion window was fixed -- update this test"
      )
    }

    it(
      "documents the fatal assertion when cfg sideband activity starts with an idle clock handshake"
    ) {
      // BUG: the same one-cycle window applies to cfgSidebandActive
      // (RDIController.scala:130-133): the assertion checks the input level
      // while the clock requester FSM still needs a cycle to leave sIDLE.
      val thrown = intercept[Throwable] {
        simulate(mkDut) { dut =>
          initDut(dut)
          dut.clock.step(2)
          dut.io.rdi.plClkReq.expect(false.B)
          dut.io.cfgSidebandActive.poke(true.B)
          for (_ <- 0 until 4) stepAdapter(dut)
        }
      }
      assert(
        thrown != null,
        "expected the documented cfg-sideband clock-coverage assertion to fire; if " +
          "this now passes the assertion window was fixed -- update this test"
      )
    }

    it(
      "documents the fatal assertion when a transition starts after clocks were released in ACTIVE"
    ) {
      // BUG: with all clock-keep conditions released in ACTIVE, a new local
      // state request makes sidebandBusy rise one cycle before the clock
      // requester can re-assert plClkReq. This violates both "Sideband traffic
      // to the Adapter must be covered by the clock handshake"
      // (RDIController.scala:126-129) and the ACTIVE-state prerequisite check
      // (RDIController.scala:114-117), aborting the simulation.
      val thrown = intercept[Throwable] {
        simulate(mkDut) { dut =>
          bringupToActive(dut)

          // Release every clock-keep condition so the 4-phase clock handshake
          // fully unwinds (plClkReq and lpClkAck both drop).
          dut.io.cfgSidebandActive.poke(false.B)
          var cycles = 0
          while (dut.io.rdi.plClkReq.peekBoolean() && cycles < 10) {
            stepAdapter(dut)
            cycles += 1
          }
          dut.io.rdi.plClkReq.expect(
            false.B,
            "plClkReq must be released once no keep condition holds"
          )
          stepAdapter(dut) // let lpClkAck drop and the requester reach sIDLE
          dut.io.rdi.lpClkAck.poke(false.B)
          dut.clock.step()

          dut.io.rdi.lpStateReq.poke(RDIStateReq.retrain)
          for (_ <- 0 until 4) stepAdapter(dut)
        }
      }
      assert(
        thrown != null,
        "expected the documented clock-coverage assertion to fire; if this " +
          "now passes the assertion window was fixed -- update this test"
      )
    }
  }
}
