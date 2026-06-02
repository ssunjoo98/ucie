package edu.berkeley.cs.uciedigital.logphy

import chisel3._
import chisel3.simulator.scalatest.ChiselSim
import org.scalatest.funspec.AnyFunSpec

import scala.util.Random

class UcieLFSRTest extends AnyFunSpec with ChiselSim {
  val serializerRatio = 32
  val lanes = 16
  val deterministicSteps = 8
  val randomSteps = 32
  val randomSeed = 0x5eedL

  val lfsrWidth = 23
  val polynomial = BigInt(0x210125)
  val laneSeeds = Seq(
    BigInt(0x1DBFBC),
    BigInt(0x0607BB),
    BigInt(0x1EC760),
    BigInt(0x18C0DB),
    BigInt(0x010F12),
    BigInt(0x19CFC9),
    BigInt(0x0277CE),
    BigInt(0x1BB807)
  )

  private class ReferenceLFSR(initSeed: BigInt, poly: BigInt, width: Int) {
    private val mask = (BigInt(1) << width) - 1
    private var state = initSeed

    def reset(): Unit = {
      state = initSeed
    }

    def outputWord(numBits: Int): BigInt = {
      var nextState = state
      var word = BigInt(0)

      for (bit <- 0 until numBits) {
        val msb = (nextState >> (width - 1)) & 1
        word |= msb << bit
        nextState = advanceOneBit(nextState, msb)
      }

      word
    }

    def advance(numBits: Int): Unit = {
      for (_ <- 0 until numBits) {
        val msb = (state >> (width - 1)) & 1
        state = advanceOneBit(state, msb)
      }
    }

    private def advanceOneBit(currentState: BigInt, msb: BigInt): BigInt = {
      val shifted = (currentState << 1) & mask
      if (msb == 1) {
        shifted ^ poly
      } else {
        shifted
      }
    }
  }

  private def params = AfeParams(mbSerializerRatio = serializerRatio, mbLanes = lanes)

  private def laneReferenceModels(): Seq[ReferenceLFSR] =
    Seq.tabulate(lanes) { lane =>
      new ReferenceLFSR(laneSeeds(lane % laneSeeds.length), polynomial, lfsrWidth)
    }

  private def clearControls(dut: UcieLFSR): Unit = {
    for (lane <- 0 until lanes) {
      dut.io.increment(lane).poke(false.B)
      dut.io.resetLfsr(lane).poke(false.B)
    }
  }

  private def expectOutputs(dut: UcieLFSR, refs: Seq[ReferenceLFSR], context: String): Unit = {
    for (lane <- 0 until lanes) {
      val expected = refs(lane).outputWord(serializerRatio)
      val actual = dut.io.lfsrOutput(lane).peek().litValue
      assert(
        actual == expected,
        s"$context lane $lane output mismatch: expected 0x${expected.toString(16)}, got 0x${actual.toString(16)}"
      )
    }
  }

  describe("UcieLFSR") {
    it("verifies initial output uses the UCIe lane seed order modulo eight") {
      simulate(new UcieLFSR(params)) { dut =>
        clearControls(dut)

        expectOutputs(dut, laneReferenceModels(), "initial")
      }
    }

    it("matches the reference model when all lanes increment") {
      simulate(new UcieLFSR(params)) { dut =>
        val refs = laneReferenceModels()
        clearControls(dut)

        for (step <- 0 until deterministicSteps) {
          expectOutputs(dut, refs, s"deterministic step $step before increment")

          for (lane <- 0 until lanes) {
            dut.io.increment(lane).poke(true.B)
            dut.io.resetLfsr(lane).poke(false.B)
          }
          dut.clock.step()

          refs.foreach(_.advance(serializerRatio))
          clearControls(dut)
          expectOutputs(dut, refs, s"deterministic step $step after increment")
        }
      }
    }

    it("matches the reference model with randomized per-lane reset and increment") {
      simulate(new UcieLFSR(params)) { dut =>
        val refs = laneReferenceModels()
        val random = new Random(randomSeed)
        clearControls(dut)

        for (step <- 0 until randomSteps) {
          expectOutputs(dut, refs, s"random step $step before control update")

          val controls = Seq.tabulate(lanes) { lane =>
            val doReset = random.nextBoolean()
            val doIncrement = random.nextBoolean()

            dut.io.resetLfsr(lane).poke(doReset.B)
            dut.io.increment(lane).poke(doIncrement.B)

            (doReset, doIncrement)
          }

          dut.clock.step()

          for (((doReset, doIncrement), ref) <- controls.zip(refs)) {
            if (doReset) {
              ref.reset()
            } else if (doIncrement) {
              ref.advance(serializerRatio)
            }
          }

          clearControls(dut)
          expectOutputs(dut, refs, s"random step $step after control update")
        }
      }
    }
  }
}
