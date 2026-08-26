package edu.berkeley.cs.uciedigital.loopback

import chisel3._
import chisel3.simulator.scalatest.ChiselSim
import edu.berkeley.cs.uciedigital.interfaces._
import edu.berkeley.cs.uciedigital.logphy._
import org.scalatest.funspec.AnyFunSpec

// Walks two cross-wired LogicalPhy instances through link training, one LTSM
// state per test. Only die 0 takes the software trigger, so die 1 has to wake
// on the sideband clock pattern die 0 transmits, which is the arrival order two
// chiplets actually see. Each test starts from reset and climbs to the state it
// names, so a failure names the first state the link cannot reach.
class LogPhyStagedBringupTest extends AnyFunSpec with ChiselSim {
  // The LTSM asserts on substates this test drives through on its way up.
  private val firtoolOpts = Array(
    "--disable-layers=Verification,Verification.Assert,Verification.Assume,Verification.Cover"
  )

  // LTSM reset minimum wait, half of the 6.4M-cycle substate timeout.
  private val resetWait = 3200000

  // Forward bring-up path. A die that derails lands outside it and never
  // satisfies a milestone.
  private val forwardPath = Seq(
    LTState.sRESET,
    LTState.sSBINIT,
    LTState.sMBINIT,
    LTState.sMBTRAIN,
    LTState.sLINKINIT,
    LTState.sACTIVE
  )

  private def ltState(h: LogPhyLoopbackHarness, die: Int): BigInt =
    h.io.ltState(die).peek().litValue

  private def reached(
      h: LogPhyLoopbackHarness,
      target: LTState.Type
  ): Boolean = {
    val goal = forwardPath.indexOf(target)
    (0 until 2).forall { die =>
      val here = forwardPath.indexWhere(_.litValue == ltState(h, die))
      here >= goal
    }
  }

  private def states(h: LogPhyLoopbackHarness): String =
    (0 until 2)
      .map(die => s"die$die=${h.io.ltState(die).peek()}")
      .mkString(", ")

  private def coldStart(h: LogPhyLoopbackHarness): Unit = {
    for (die <- 0 until 2) {
      h.io.lpStateReq(die).poke(RDIStateReq.nop)
      h.io.swStartLinkTraining(die).poke(false.B)
      h.io.pwrGood(die).poke(true.B)
    }
    h.clock.step(resetWait + 128)
    for (die <- 0 until 2) {
      h.io.ltState(die).expect(LTState.sRESET, "no training without a trigger")
    }
    h.io.swStartLinkTraining(0).poke(true.B)
    h.clock.step(4)
    h.io.swStartLinkTraining(0).poke(false.B)
  }

  private def climbTo(
      h: LogPhyLoopbackHarness,
      target: LTState.Type,
      limit: Int
  ): Unit = {
    var left = limit
    while (left > 0 && !reached(h, target)) {
      h.clock.step(1)
      left -= 1
    }
    assert(
      reached(h, target),
      s"did not reach $target within $limit cycles: ${states(h)}"
    )
  }

  describe("LogicalPhy link training") {
    it("leaves RESET and enters SBINIT on both dies") {
      simulate(new LogPhyLoopbackHarness(), firtoolOpts = firtoolOpts) { h =>
        coldStart(h)
        climbTo(h, LTState.sSBINIT, 4096)
        for (die <- 0 until 2) {
          h.io.plPhyInRecenter(die).expect(true.B)
          h.io.plStateSts(die).expect(RDIState.reset)
        }
      }
    }

    it("completes SBINIT and enters MBINIT") {
      simulate(new LogPhyLoopbackHarness(), firtoolOpts = firtoolOpts) { h =>
        coldStart(h)
        climbTo(h, LTState.sMBINIT, 400000)
        for (die <- 0 until 2) {
          h.io.sbFaultSeen(die).expect(false.B, "sideband fault during SBINIT")
          h.io.trainingTimedout(die).expect(false.B)
        }
      }
    }

    it("negotiates PHY parameters in MBINIT.PARAM") {
      simulate(new LogPhyLoopbackHarness(), firtoolOpts = firtoolOpts) { h =>
        coldStart(h)
        climbTo(h, LTState.sMBINIT, 400000)
        var left = 400000
        while (
          left > 0 && !(0 until 2).forall(
            h.io.negotiatedParamsValid(_).peekBoolean()
          )
        ) {
          h.clock.step(1)
          left -= 1
        }
        for (die <- 0 until 2) {
          h.io
            .negotiatedParamsValid(die)
            .expect(
              true.B,
              s"parameter exchange did not complete: ${states(h)}"
            )
          h.io.plTrainError(die).expect(false.B)
        }
      }
    }
  }
}
