package edu.berkeley.cs.uciedigital.d2dadapter

import chisel3._
import chisel3.simulator.scalatest.ChiselSim
import org.scalatest.funspec.AnyFunSpec
import edu.berkeley.cs.uciedigital.interfaces._
import edu.berkeley.cs.uciedigital.sideband._

/*
  Run:
    ./mill --no-daemon test.testOnly edu.berkeley.cs.uciedigital.d2dadapter.D2DAdapterBringupTest

  Coverage:
    Full bring-up:  two D2DAdapter instances with cross-wired RDI sideband
                    lanes (AdapterLoopbackHarness) walk the whole reset-state
                    choreography: inband presence -> RDI active request
                    (AdapterSM.scala:229-237), PHY active -> ADV_CAP exchange
                    over the real serializer/switch (AdapterSM.scala:239-254),
                    nop->active FDI edge -> REQ_ACTIVE (AdapterSM.scala:281-286),
                    RX-active handshake -> RSP_ACTIVE -> INIT_DONE -> LSM
                    active on both dies (AdapterSM.scala:256-295, 497-499).
    Skew:           one PHY reports active N cycles later than the other.
                    N inside the sideband flight time still completes; N
                    beyond it trips the PARAM_EXCH receive-flag race and
                    hangs bring-up forever (documented-hang test below).
                    Measured boundary in this configuration: N<=6 completes,
                    N>=7 hangs (ADV_CAP takes 7 cycles from PARAM_EXCH entry
                    to the peer's sb_rcv pulse).
    Link reset:     from active, one die's linkReset request crosses as
                    REQ/RSP_LINKRESET and both dies complete the stall
                    handshake into linkReset (AdapterSM.scala:139-151,
                    310-359, 415-419, 501-507).
    Retrain trap:   documented bug -- a PHY retrain report plus the stall/RX
                    handshakes moves active->retrain, but the retrain case
                    (AdapterSM.scala:512-520) has no retrain->active branch,
                    so the LSM stays trapped even after the PHY reports
                    active again (and an FDI active request does not help).
                    The only exits are disabled/linkReset/linkError; the
                    linkReset escape is exercised (retrain->linkReset needs
                    no stall, AdapterSM.scala:517-518).
    Disabled:       from active, one die's disabled request crosses as
                    REQ/RSP_DISABLED and both dies complete the stall
                    handshake into disabled (AdapterSM.scala:153-169,
                    361-412, 504-505); FDI active requests recover both to
                    reset (AdapterSM.scala:530-532) and a second bring-up
                    completes after a fresh nop->active edge.
*/
class D2DAdapterBringupTest extends AnyFunSpec with ChiselSim {

  private val fdiParams = new FdiParams(64, 32)
  private val rdiParams = RdiParams(64, 32)
  private val sbParams = new SidebandParams()

  private val guardCycles = 200

  private def mkHarness = new AdapterLoopbackHarness(fdiParams, rdiParams, sbParams)

  private def bothDies(cond: Int => Boolean): Boolean = cond(0) && cond(1)

  // Drive every harness input to a quiet default before a scenario starts.
  private def initHarness(h: AdapterLoopbackHarness): Unit = {
    for (i <- 0 until 2) {
      h.io.plInbandPres(i).poke(false.B)
      h.io.plStateSts(i).poke(RDIState.reset)
      h.io.lpStateReq(i).poke(FDIStateReq.nop)
      h.io.lpRxActiveSts(i).poke(false.B)
      h.io.lpStallAck(i).poke(false.B)
    }
  }

  // Step until cond holds or the guard expires; returns whether cond held.
  private def waitFor(h: AdapterLoopbackHarness, guard: Int = guardCycles)(
      cond: => Boolean): Boolean = {
    var n = 0
    while (!cond && n < guard) {
      h.clock.step()
      n += 1
    }
    cond
  }

  private def fdiStateIs(h: AdapterLoopbackHarness, i: Int, s: FDIState.Type): Boolean =
    h.io.fdiPlStateSts(i).peek().litValue == s.litValue

  // Run the full bring-up choreography with die 1's PHY-active report delayed
  // by `skew` cycles. Returns true iff both dies reach FDI active in time.
  private def runBringup(h: AdapterLoopbackHarness, skew: Int): Boolean = {
    // Phase 1: both PHYs report sideband-alive; the adapters must request
    // RDI active while still holding FDI in reset (AdapterSM.scala:229-237).
    h.io.plInbandPres(0).poke(true.B)
    h.io.plInbandPres(1).poke(true.B)
    val phase1 = waitFor(h) {
      bothDies(i => h.io.rdiLpStateReq(i).peek().litValue == RDIStateReq.active.litValue)
    }
    if (!phase1) return false

    // Phase 2: PHY training completes (die 1 delayed by skew cycles); both
    // dies enter PARAM_EXCH and exchange ADV_CAP over the crossed sideband,
    // then report inband presence from FDI_BRINGUP (AdapterSM.scala:239-254).
    h.io.plStateSts(0).poke(RDIState.active)
    if (skew > 0) h.clock.step(skew)
    h.io.plStateSts(1).poke(RDIState.active)
    val phase2 = waitFor(h)(bothDies(i => h.io.fdiPlInbandPres(i).peekBoolean()))
    if (!phase2) return false

    // Phase 3: protocol requests active. The request is edge-detected against
    // a nop history (AdapterSM.scala:281-286), so this poke is the required
    // nop->active transition; REQ_ACTIVE crosses and each die raises
    // plRxActiveReq when the remote request lands (AdapterSM.scala:114, 263-267).
    h.io.lpStateReq(0).poke(FDIStateReq.active)
    h.io.lpStateReq(1).poke(FDIStateReq.active)
    val phase3 = waitFor(h)(bothDies(i => h.io.fdiPlRxActiveReq(i).peekBoolean()))
    if (!phase3) return false

    // Phase 4: protocol reports its receiver alive; RSP_ACTIVE crosses; both
    // dies hit INIT_DONE and the LSM enters active (AdapterSM.scala:288-295,
    // 497-499).
    h.io.lpRxActiveSts(0).poke(true.B)
    h.io.lpRxActiveSts(1).poke(true.B)
    waitFor(h)(bothDies(i => fdiStateIs(h, i, FDIState.active)))
  }

  describe("D2DAdapter two-instance loopback bring-up") {

    it("completes the full bring-up choreography to active on both dies") {
      simulate(mkHarness) { h =>
        initHarness(h)
        h.clock.step(2)

        // Phase 1: sideband-alive from both PHY stubs.
        h.io.plInbandPres(0).poke(true.B)
        h.io.plInbandPres(1).poke(true.B)
        assert(
          waitFor(h) {
            bothDies(i => h.io.rdiLpStateReq(i).peek().litValue == RDIStateReq.active.litValue)
          },
          "both adapters must request RDI active after inband presence"
        )
        h.io.fdiPlStateSts(0).expect(FDIState.reset, "FDI must still report reset during RDI bringup")
        h.io.fdiPlStateSts(1).expect(FDIState.reset, "FDI must still report reset during RDI bringup")

        // Phase 2: PHY active on both dies; ADV_CAP crosses the real wires.
        h.io.plStateSts(0).poke(RDIState.active)
        h.io.plStateSts(1).poke(RDIState.active)
        assert(
          waitFor(h)(bothDies(i => h.io.fdiPlInbandPres(i).peekBoolean())),
          "both dies must report FDI inband presence after the ADV_CAP exchange"
        )

        // Phase 3: nop->active edge on both FDIs; REQ_ACTIVE crosses.
        h.io.lpStateReq(0).poke(FDIStateReq.active)
        h.io.lpStateReq(1).poke(FDIStateReq.active)
        assert(
          waitFor(h)(bothDies(i => h.io.fdiPlRxActiveReq(i).peekBoolean())),
          "both dies must raise plRxActiveReq after the REQ_ACTIVE exchange"
        )

        // Phase 4: RX alive on both; RSP_ACTIVE crosses; LSMs reach active.
        h.io.lpRxActiveSts(0).poke(true.B)
        h.io.lpRxActiveSts(1).poke(true.B)
        assert(
          waitFor(h)(bothDies(i => fdiStateIs(h, i, FDIState.active))),
          "both dies must reach FDI active after the RSP_ACTIVE exchange"
        )
        h.io.fdiPlProtocolVld(0).expect(true.B, "plProtocolVld must be high in active")
        h.io.fdiPlProtocolVld(1).expect(true.B, "plProtocolVld must be high in active")

        // Stability: the link must hold active with the inputs left as-is.
        h.clock.step(20)
        for (i <- 0 until 2) {
          h.io.fdiPlStateSts(i).expect(FDIState.active, "active must be stable")
          h.io.fdiPlInbandPres(i).expect(true.B, "inband presence must be stable in active")
          h.io.fdiPlRxActiveReq(i).expect(true.B, "plRxActiveReq must be stable in active")
          h.io.fdiPlProtocolVld(i).expect(true.B, "plProtocolVld must be stable in active")
        }
      }
    }

    for (skew <- Seq(1, 5)) {
      it(s"completes bring-up when one PHY reports active $skew cycle(s) late") {
        simulate(mkHarness) { h =>
          initHarness(h)
          h.clock.step(2)
          // Within the sideband flight time the late die reaches PARAM_EXCH
          // before the early die's ADV_CAP pulse lands, so nothing is lost.
          assert(runBringup(h, skew), s"bring-up must tolerate a $skew-cycle plStateSts skew")
          h.io.fdiPlStateSts(0).expect(FDIState.active, "early die must reach active")
          h.io.fdiPlStateSts(1).expect(FDIState.active, "late die must reach active")
        }
      }
    }

    it("documents a bring-up hang when one PHY reports active 20 cycles late") {
      simulate(mkHarness) { h =>
        initHarness(h)
        h.clock.step(2)

        // BUG: link init only latches an incoming ADV_CAP while it already
        // sits in PARAM_EXCH (AdapterSM.scala:239-244); every other
        // reset-state cycle forces the receive flag false
        // (AdapterSM.scala:220-226). ADV_CAP delivery through the real
        // serializer/switch takes 7 cycles from PARAM_EXCH entry, so when the
        // late die's PHY reports active more than 6 cycles behind its peer,
        // the early die's ADV_CAP sb_rcv pulse fires while the late die is
        // still in RDI_BRINGUP and is dropped. The early die never
        // retransmits (paramExchSbMsgSntFlag, AdapterSM.scala:107-109,
        // 246-250), so the late die waits in PARAM_EXCH forever and bring-up
        // hangs. Measured boundary: skew<=6 completes, skew>=7 hangs.
        // This test pins the current (buggy) behavior; update it once the
        // race is fixed (e.g. latch sb_rcv across all link-init substates or
        // retransmit ADV_CAP until acknowledged).
        assert(
          !runBringup(h, skew = 20),
          "expected the documented ADV_CAP-race hang; if bring-up now completes the race was fixed -- update this test"
        )

        // Current behavior: the early die consumed the late die's ADV_CAP and
        // moved on to FDI_BRINGUP (inband presence high), while the late die
        // is stuck in PARAM_EXCH with its FDI fully down.
        h.io.fdiPlInbandPres(0).expect(true.B, "early die reaches FDI_BRINGUP on the late die's ADV_CAP")
        h.io.fdiPlInbandPres(1).expect(false.B, "late die never leaves PARAM_EXCH after its RX pulse was dropped")
        h.io.fdiPlStateSts(0).expect(FDIState.reset, "early die cannot finish bring-up without a partner")
        h.io.fdiPlStateSts(1).expect(FDIState.reset, "late die holds FDI reset forever")

        // A protocol active request cannot rescue the link either: the early
        // die's REQ_ACTIVE pulse is likewise dropped by the late die stuck in
        // PARAM_EXCH (only ADV_CAP is latched there, AdapterSM.scala:239-244).
        h.io.lpStateReq(0).poke(FDIStateReq.active)
        h.io.lpStateReq(1).poke(FDIStateReq.active)
        assert(
          !waitFor(h)(bothDies(i => fdiStateIs(h, i, FDIState.active))),
          "expected the hang to persist through a protocol active request"
        )
        h.io.fdiPlStateSts(0).expect(FDIState.reset, "early die must still be down")
        h.io.fdiPlStateSts(1).expect(FDIState.reset, "late die must still be down")
      }
    }

    it("tears down to linkReset from active via REQ/RSP_LINKRESET and the stall handshake") {
      simulate(mkHarness) { h =>
        initHarness(h)
        h.clock.step(2)
        assert(runBringup(h, skew = 0), "precondition: full bring-up must succeed")

        // Park the state requests at nop first: an lp_state_req==active level
        // would immediately kick the LSM back out of linkReset
        // (AdapterSM.scala:540-543).
        h.io.lpStateReq(0).poke(FDIStateReq.nop)
        h.io.lpStateReq(1).poke(FDIStateReq.nop)
        h.clock.step(2)

        // Die 0's protocol requests linkReset (level-detected outside the
        // reset state, AdapterSM.scala:321-328). REQ_LINKRESET crosses; die 1
        // answers RSP_LINKRESET (AdapterSM.scala:146-150).
        h.io.lpStateReq(0).poke(FDIStateReq.linkReset)

        // Active exit needs the stall handshake plus RX deactivation
        // (AdapterSM.scala:506-507): whenever a die raises plStallReq, ack the
        // stall and drop lpRxActiveSts so rxDeactive can assert.
        var n = 0
        while (!bothDies(i => fdiStateIs(h, i, FDIState.linkReset)) && n < guardCycles) {
          for (i <- 0 until 2) {
            if (h.io.fdiPlStallReq(i).peekBoolean()) {
              h.io.lpStallAck(i).poke(true.B)
              h.io.lpRxActiveSts(i).poke(false.B)
            }
          }
          h.clock.step()
          n += 1
        }
        h.io.fdiPlStateSts(0).expect(FDIState.linkReset, "requester die must reach linkReset")
        h.io.fdiPlStateSts(1).expect(FDIState.linkReset, "responder die must reach linkReset")
        // plInbandPres is registered off the link state and drops one cycle
        // after linkReset entry (AdapterSM.scala:443-448).
        h.clock.step(2)
        h.io.fdiPlStateSts(0).expect(FDIState.linkReset, "linkReset must be stable")
        h.io.fdiPlStateSts(1).expect(FDIState.linkReset, "linkReset must be stable")
        h.io.fdiPlInbandPres(0).expect(false.B, "inband presence must drop in linkReset")
        h.io.fdiPlInbandPres(1).expect(false.B, "inband presence must drop in linkReset")
      }
    }

    it("documents the retrain trap: no retrain->active exit even after the PHY reports active again") {
      simulate(mkHarness) { h =>
        initHarness(h)
        h.clock.step(2)
        assert(runBringup(h, skew = 0), "precondition: full bring-up must succeed")

        // Park the state requests at nop: a stale active level would kick
        // later states straight back to reset (AdapterSM.scala:540-543).
        h.io.lpStateReq(0).poke(FDIStateReq.nop)
        h.io.lpStateReq(1).poke(FDIStateReq.nop)
        h.clock.step(2)

        // Die 0's PHY reports retrain. The active exit needs the stall
        // handshake plus RX deactivation (AdapterSM.scala:508-509), so ack
        // the stall and drop lpRxActiveSts whenever plStallReq rises
        // (D2D-LSM-04 entry leg).
        h.io.plStateSts(0).poke(RDIState.retrain)
        var n = 0
        while (!fdiStateIs(h, 0, FDIState.retrain) && n < guardCycles) {
          if (h.io.fdiPlStallReq(0).peekBoolean()) {
            h.io.lpStallAck(0).poke(true.B)
            h.io.lpRxActiveSts(0).poke(false.B)
          }
          h.clock.step()
          n += 1
        }
        h.io.fdiPlStateSts(0).expect(FDIState.retrain, "die 0 must reach retrain via stall + RX deactivation")
        h.io.fdiPlStateSts(1).expect(FDIState.active, "die 1 sees no retrain choreography and stays active")
        h.io.lpStallAck(0).poke(false.B)

        // The PHY finishes retraining and reports active again.
        h.io.plStateSts(0).poke(RDIState.active)

        // BUG: the LSM retrain case (AdapterSM.scala:512-520) only has
        // linkError/disabled/linkReset exits -- there is no retrain->active
        // branch, so the adapter is trapped in retrain even though the PHY
        // is back to active. UCIe 3.0 10.3.3.4 (Retrain State Rules)
        // expects a return to Active once retraining completes. This test
        // pins the current (buggy) behavior; update it once the
        // retrain->active exit is implemented.
        assert(
          !waitFor(h, 100)(fdiStateIs(h, 0, FDIState.active)),
          "expected the documented retrain trap; if die 0 reaches active the missing edge was added -- update this test"
        )
        h.io.fdiPlStateSts(0).expect(FDIState.retrain, "die 0 must still be trapped in retrain")

        // A protocol active request cannot rescue it either: nothing in the
        // retrain case consumes fdi_lp_state_req.
        h.io.lpStateReq(0).poke(FDIStateReq.active)
        assert(
          !waitFor(h, 50)(fdiStateIs(h, 0, FDIState.active)),
          "expected the trap to persist through a protocol active request"
        )
        h.io.fdiPlStateSts(0).expect(FDIState.retrain, "die 0 must still be trapped in retrain")

        // The only exits are disabled/linkReset/linkError. A linkReset
        // request escapes: REQ_LINKRESET crosses from retrain, die 1 answers
        // RSP_LINKRESET from active (with its own stall handshake), and die 0
        // takes retrain->linkReset without a stall (AdapterSM.scala:517-518)
        // -- covering D2D-LSM-07 on the way out.
        h.io.lpStateReq(0).poke(FDIStateReq.linkReset)
        n = 0
        while (!bothDies(i => fdiStateIs(h, i, FDIState.linkReset)) && n < guardCycles) {
          if (h.io.fdiPlStallReq(1).peekBoolean()) {
            h.io.lpStallAck(1).poke(true.B)
            h.io.lpRxActiveSts(1).poke(false.B)
          }
          h.clock.step()
          n += 1
        }
        h.io.fdiPlStateSts(0).expect(FDIState.linkReset, "die 0 must escape retrain into linkReset")
        h.io.fdiPlStateSts(1).expect(FDIState.linkReset, "die 1 must follow into linkReset")
      }
    }

    it("tears down to disabled via REQ/RSP_DISABLED, recovers to reset, and completes a second bring-up") {
      simulate(mkHarness) { h =>
        initHarness(h)
        h.clock.step(2)
        assert(runBringup(h, skew = 0), "precondition: full bring-up must succeed")

        // Park the state requests at nop first: an active level would exit
        // disabled immediately (AdapterSM.scala:530-532).
        h.io.lpStateReq(0).poke(FDIStateReq.nop)
        h.io.lpStateReq(1).poke(FDIStateReq.nop)
        h.clock.step(2)

        // Die 0's protocol requests disabled (level-detected outside reset,
        // AdapterSM.scala:374-378). REQ_DISABLED crosses; die 1 answers
        // RSP_DISABLED (AdapterSM.scala:166-168). Active exit needs the
        // stall handshake plus RX deactivation on both dies
        // (AdapterSM.scala:504-505) -- D2D-SB-02 / D2D-LSM-05.
        h.io.lpStateReq(0).poke(FDIStateReq.disabled)
        var n = 0
        while (!bothDies(i => fdiStateIs(h, i, FDIState.disabled)) && n < guardCycles) {
          for (i <- 0 until 2) {
            if (h.io.fdiPlStallReq(i).peekBoolean()) {
              h.io.lpStallAck(i).poke(true.B)
              h.io.lpRxActiveSts(i).poke(false.B)
            }
          }
          h.clock.step()
          n += 1
        }
        h.io.fdiPlStateSts(0).expect(FDIState.disabled, "requester die must reach disabled")
        h.io.fdiPlStateSts(1).expect(FDIState.disabled, "responder die must reach disabled")
        h.clock.step(2)
        h.io.fdiPlStateSts(0).expect(FDIState.disabled, "disabled must be stable")
        h.io.fdiPlStateSts(1).expect(FDIState.disabled, "disabled must be stable")
        h.io.fdiPlInbandPres(0).expect(false.B, "inband presence must drop in disabled")
        h.io.fdiPlInbandPres(1).expect(false.B, "inband presence must drop in disabled")
        h.io.lpStallAck(0).poke(false.B)
        h.io.lpStallAck(1).poke(false.B)

        // Recovery (D2D-LSM-09): FDI active requests exit disabled to reset
        // on both dies (AdapterSM.scala:530-532).
        h.io.lpStateReq(0).poke(FDIStateReq.active)
        h.io.lpStateReq(1).poke(FDIStateReq.active)
        assert(
          waitFor(h)(bothDies(i => fdiStateIs(h, i, FDIState.reset))),
          "both dies must recover from disabled to reset on FDI active requests"
        )

        // Re-bring-up (D2D-BRINGUP-10): with plInbandPres/plStateSts still up,
        // link init restarts from INIT_START and the ADV_CAP exchange repeats
        // (flags were cleared while outside reset, AdapterSM.scala:298-307).
        assert(
          waitFor(h)(bothDies(i => h.io.fdiPlInbandPres(i).peekBoolean())),
          "second bring-up must re-run the ADV_CAP exchange up to FDI_BRINGUP"
        )

        // The stale active level cannot produce the nop->active edge required
        // in FDI_BRINGUP (AdapterSM.scala:281-286), so pulse through nop.
        h.io.lpStateReq(0).poke(FDIStateReq.nop)
        h.io.lpStateReq(1).poke(FDIStateReq.nop)
        h.clock.step(2)
        h.io.lpStateReq(0).poke(FDIStateReq.active)
        h.io.lpStateReq(1).poke(FDIStateReq.active)
        assert(
          waitFor(h)(bothDies(i => h.io.fdiPlRxActiveReq(i).peekBoolean())),
          "REQ_ACTIVE must cross again during the second bring-up"
        )
        h.io.lpRxActiveSts(0).poke(true.B)
        h.io.lpRxActiveSts(1).poke(true.B)
        assert(
          waitFor(h)(bothDies(i => fdiStateIs(h, i, FDIState.active))),
          "both dies must reach active again after the disabled round trip"
        )
      }
    }
  }
}
