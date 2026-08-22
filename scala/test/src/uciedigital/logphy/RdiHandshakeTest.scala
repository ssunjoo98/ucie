package edu.berkeley.cs.uciedigital.logphy

import chisel3._
import chisel3.simulator.scalatest.ChiselSim
import org.scalatest.funspec.AnyFunSpec

/*
  Run:
    ./mill --no-daemon test.testOnly edu.berkeley.cs.uciedigital.logphy.RdiHandshakeTest

  Coverage:
    LT-RDI-06: RDIClockHandshakeRequester 4-phase rules
               (RDIClockHandshakeRequester.scala:53-90): request rises after
               start; request held while the ack is delayed; doneHandshake
               tracks the ack; on release the request de-asserts strictly
               before the ack (Rule 3); idle is reached only after the ack
               drops; immediate-ack and early-release (abort) sweeps.
    LT-RDI-07: RDIWakeHandshakeResponder ordering
               (RDIWakeHandshakeResponder.scala:40-71): a wake request ungates
               clocks before any ack; no ack until clocksUngatedAndStable;
               the ack holds while the request is high (Rule 3) and drops only
               after the request drops; back-to-back wake handshakes.
    LT-RDI-08: RDIStallRequester 4-phase rules
               (RDIStallRequester.scala:46-82): a rising pl_stallreq only when
               lp_stallack is de-asserted (Rule 2); isStalled only after the
               ack; on release the request falls while the ack is still high
               (Rule 3); idle only after the ack drops.
 */
class RdiHandshakeTest extends AnyFunSpec with ChiselSim {

  private def initClk(dut: RDIClockHandshakeRequester): Unit = {
    dut.io.ctrl.startHandshake.poke(false.B)
    dut.io.ctrl.releaseReq.poke(false.B)
    dut.io.rdi.lpClkAck.poke(false.B)
  }

  private def initStall(dut: RDIStallRequester): Unit = {
    dut.io.ctrl.startStall.poke(false.B)
    dut.io.ctrl.releaseStall.poke(false.B)
    dut.io.rdi.lpStallAck.poke(false.B)
  }

  describe("RDIClockHandshakeRequester 4-phase handshake (LT-RDI-06)") {

    it("completes all four phases with a delayed ack") {
      simulate(new RDIClockHandshakeRequester()) { dut =>
        initClk(dut)
        dut.clock.step()
        dut.io.rdi.plClkReq.expect(false.B)
        dut.io.ctrl.inIdle.expect(true.B)
        dut.io.ctrl.doneHandshake.expect(false.B)

        // Phase 1: request rises one cycle after start.
        dut.io.ctrl.startHandshake.poke(true.B)
        dut.clock.step()
        dut.io.rdi.plClkReq
          .expect(true.B, "plClkReq must rise after startHandshake")
        dut.io.ctrl.doneHandshake.expect(false.B)
        dut.io.ctrl.inIdle.expect(false.B)

        // The request is held while the responder delays the ack.
        for (_ <- 0 until 3) {
          dut.clock.step()
          dut.io.rdi.plClkReq
            .expect(true.B, "plClkReq must hold until lpClkAck")
          dut.io.ctrl.doneHandshake.expect(false.B)
        }

        // Phase 2: ack rises -> the handshake is done, request still held.
        dut.io.rdi.lpClkAck.poke(true.B)
        dut.clock.step()
        dut.io.rdi.plClkReq.expect(true.B)
        dut.io.ctrl.doneHandshake
          .expect(true.B, "doneHandshake must track the ack")

        // Level-sensitive hold while the request level stays high.
        for (_ <- 0 until 3) {
          dut.clock.step()
          dut.io.rdi.plClkReq.expect(true.B)
          dut.io.ctrl.doneHandshake.expect(true.B)
        }

        // Phase 3: release -> the request must fall BEFORE the ack (Rule 3).
        dut.io.ctrl.startHandshake.poke(false.B)
        dut.clock.step()
        dut.io.rdi.plClkReq
          .expect(false.B, "plClkReq must de-assert before lpClkAck (Rule 3)")
        dut.io.ctrl.doneHandshake.expect(false.B)
        dut.io.ctrl.inIdle.expect(false.B)

        // The requester waits (not idle) while the ack lingers.
        for (_ <- 0 until 3) {
          dut.clock.step()
          dut.io.rdi.plClkReq.expect(false.B)
          dut.io.ctrl.inIdle.expect(false.B, "no idle until the ack drops")
        }

        // Phase 4: ack drops -> back to idle.
        dut.io.rdi.lpClkAck.poke(false.B)
        dut.clock.step()
        dut.io.ctrl.inIdle
          .expect(true.B, "idle must be reached after the ack drops")
        dut.io.rdi.plClkReq.expect(false.B)
      }
    }

    it("completes with an immediate (same-observation-cycle) ack") {
      simulate(new RDIClockHandshakeRequester()) { dut =>
        initClk(dut)
        dut.clock.step()
        dut.io.ctrl.startHandshake.poke(true.B)
        dut.clock.step()
        dut.io.rdi.plClkReq.expect(true.B)
        // Responder acks combinationally in the same cycle the request is seen.
        dut.io.rdi.lpClkAck.poke(true.B)
        dut.clock.step()
        dut.io.ctrl.doneHandshake.expect(true.B)
        dut.io.rdi.plClkReq.expect(true.B)

        // Release via the explicit releaseReq control.
        dut.io.ctrl.releaseReq.poke(true.B)
        dut.clock.step()
        dut.io.rdi.plClkReq
          .expect(false.B, "releaseReq must drop the request before the ack")
        dut.io.rdi.lpClkAck.poke(false.B)
        dut.clock.step()
        dut.io.ctrl.inIdle.expect(true.B)
      }
    }

    it("aborts a pending request cleanly when released before any ack") {
      simulate(new RDIClockHandshakeRequester()) { dut =>
        initClk(dut)
        dut.clock.step()
        dut.io.ctrl.startHandshake.poke(true.B)
        dut.clock.step()
        dut.io.rdi.plClkReq.expect(true.B)
        dut.io.ctrl.doneHandshake.expect(false.B)

        // Early release: the request level drops while no ack ever came.
        dut.io.ctrl.startHandshake.poke(false.B)
        dut.clock.step()
        dut.io.ctrl.inIdle
          .expect(true.B, "an unacked request must abort back to idle")
        dut.io.rdi.plClkReq.expect(false.B)
        dut.io.ctrl.doneHandshake.expect(false.B)
      }
    }
  }

  describe(
    "RDIWakeHandshakeResponder wake handshake and clock ungate (LT-RDI-07)"
  ) {

    it("ungates clocks before acking and orders req/ack edges per the rules") {
      simulate(new RDIWakeHandshakeResponder()) { dut =>
        dut.io.rdi.lpWakeReq.poke(false.B)
        dut.io.ctrl.clocksUngatedAndStable.poke(false.B)
        dut.clock.step()
        dut.io.rdi.plWakeAck.expect(false.B)
        dut.io.ctrl.ungateClocks.expect(false.B)

        // A wake request first requests clock ungating, with no ack yet.
        dut.io.rdi.lpWakeReq.poke(true.B)
        dut.clock.step()
        dut.io.ctrl.ungateClocks
          .expect(true.B, "a wake request must ungate clocks first")
        dut.io.rdi.plWakeAck.expect(false.B)

        // No ack while the clocks are not yet stable (Rule 2).
        for (_ <- 0 until 3) {
          dut.clock.step()
          dut.io.ctrl.ungateClocks.expect(true.B)
          dut.io.rdi.plWakeAck
            .expect(false.B, "no wake ack until clocks are stable")
        }

        // Clocks become stable -> ack asserts, ungate stays high.
        dut.io.ctrl.clocksUngatedAndStable.poke(true.B)
        dut.clock.step()
        dut.io.rdi.plWakeAck.expect(true.B, "the ack must follow stable clocks")
        dut.io.ctrl.ungateClocks.expect(true.B)

        // Rule 3: the ack holds while the request stays high.
        for (_ <- 0 until 3) {
          dut.clock.step()
          dut.io.rdi.plWakeAck
            .expect(true.B, "the ack must hold while lpWakeReq is high")
          dut.io.ctrl.ungateClocks.expect(true.B)
        }

        // The requester drops the request first; the ack drops afterwards.
        dut.io.rdi.lpWakeReq.poke(false.B)
        dut.clock.step()
        dut.io.rdi.plWakeAck
          .expect(false.B, "the ack must drop only after the request drops")
        dut.clock.step()
        dut.io.rdi.plWakeAck.expect(false.B)
        dut.io.ctrl.ungateClocks
          .expect(false.B, "clock ungating ends after the handshake")
      }
    }

    it("supports back-to-back wake handshakes with already-stable clocks") {
      simulate(new RDIWakeHandshakeResponder()) { dut =>
        dut.io.rdi.lpWakeReq.poke(false.B)
        dut.io.ctrl.clocksUngatedAndStable.poke(true.B)
        dut.clock.step()

        for (round <- 0 until 2) {
          dut.io.rdi.lpWakeReq.poke(true.B)
          dut.clock.step()
          dut.io.rdi.plWakeAck
            .expect(false.B, s"round $round: ack needs the ungate bubble cycle")
          dut.clock.step()
          dut.io.rdi.plWakeAck
            .expect(true.B, s"round $round: ack must assert after the bubble")

          dut.io.rdi.lpWakeReq.poke(false.B)
          dut.clock.step(2)
          dut.io.rdi.plWakeAck.expect(
            false.B,
            s"round $round: ack must clear after the request drops"
          )
          dut.clock.step()
        }
      }
    }
  }

  describe("RDIStallRequester 4-phase stall handshake (LT-RDI-08)") {

    it("runs the full stall sequence with delayed ack and ordered edges") {
      simulate(new RDIStallRequester()) { dut =>
        initStall(dut)
        dut.clock.step()
        dut.io.rdi.plStallReq.expect(false.B)
        dut.io.ctrl.inIdle.expect(true.B)
        dut.io.ctrl.isStalled.expect(false.B)

        // Trigger: the request rises one cycle after startStall.
        dut.io.ctrl.startStall.poke(true.B)
        dut.clock.step()
        dut.io.rdi.plStallReq
          .expect(true.B, "plStallReq must rise after startStall")
        dut.io.ctrl.isStalled
          .expect(false.B, "not stalled before the adapter acks")
        dut.io.ctrl.startStall
          .poke(false.B) // the FSM must have latched the request

        // The request holds while the adapter looks for a clean flit boundary.
        for (_ <- 0 until 3) {
          dut.clock.step()
          dut.io.rdi.plStallReq.expect(true.B)
          dut.io.ctrl.isStalled.expect(false.B)
        }

        // Adapter acks -> cleanly stalled.
        dut.io.rdi.lpStallAck.poke(true.B)
        dut.clock.step()
        dut.io.ctrl.isStalled
          .expect(true.B, "isStalled must assert once the ack is seen")
        dut.io.rdi.plStallReq.expect(true.B)

        // Hold the stall until the FSM releases it.
        for (_ <- 0 until 2) {
          dut.clock.step()
          dut.io.ctrl.isStalled.expect(true.B)
        }

        // Rule 3: on release the request falls while the ack is still high.
        dut.io.ctrl.releaseStall.poke(true.B)
        dut.clock.step()
        dut.io.rdi.plStallReq.expect(
          false.B,
          "plStallReq must fall while lpStallAck is asserted (Rule 3)"
        )
        dut.io.ctrl.isStalled.expect(false.B)
        dut.io.ctrl.inIdle.expect(false.B, "no idle until the ack drops")
        dut.io.ctrl.releaseStall.poke(false.B)

        // The handshake completes only after the ack drops.
        dut.clock.step()
        dut.io.ctrl.inIdle.expect(false.B)
        dut.io.rdi.lpStallAck.poke(false.B)
        dut.clock.step()
        dut.io.ctrl.inIdle
          .expect(true.B, "idle must be reached after the ack drops")
      }
    }

    it("never raises plStallReq while lpStallAck is still asserted (Rule 2)") {
      simulate(new RDIStallRequester()) { dut =>
        initStall(dut)
        dut.clock.step()

        // A stale ack blocks the rising edge of a new request.
        dut.io.rdi.lpStallAck.poke(true.B)
        dut.io.ctrl.startStall.poke(true.B)
        for (_ <- 0 until 4) {
          dut.clock.step()
          dut.io.rdi.plStallReq.expect(
            false.B,
            "plStallReq must not rise while lpStallAck is asserted (Rule 2)"
          )
          dut.io.ctrl.inIdle.expect(true.B)
        }

        // Once the ack clears, the request may start.
        dut.io.rdi.lpStallAck.poke(false.B)
        dut.clock.step()
        dut.io.rdi.plStallReq.expect(
          true.B,
          "plStallReq must start once the stale ack has cleared"
        )
      }
    }
  }
}
