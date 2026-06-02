package edu.berkeley.cs.uciedigital.logphy

import chisel3._
import chisel3.simulator.scalatest.ChiselSim
import chisel3.util._
import edu.berkeley.cs.uciedigital.utils.ReferenceLFSR
import org.scalatest.funspec.AnyFunSpec
import scala.util.Random

// Harness
class PatternWriterWithLfsrHarness(afeParams: AfeParams) extends Module {
  val io = IO(new Bundle {
    val interfaceIo = new PatternWriterIO
    val mbTxLaneIo = Decoupled(new MainbandLanes(afeParams.mbLanes, afeParams.mbSerializerRatio))
    val txLfsrCtrl = new Bundle {
      val valid = Output(Bool())
      val resetLfsr = Output(Bool())
      val increment = Output(Bool())
    }
  })

  val patternWriter = Module(new PatternWriter(afeParams))
  val txLfsr = Module(new UcieLFSR(afeParams))

  patternWriter.io.interfaceIo.req.valid := io.interfaceIo.req.valid
  patternWriter.io.interfaceIo.req.bits := io.interfaceIo.req.bits
  io.interfaceIo.req.ready := patternWriter.io.interfaceIo.req.ready
  io.interfaceIo.resp.complete := patternWriter.io.interfaceIo.resp.complete

  io.mbTxLaneIo.valid := patternWriter.io.mbTxLaneIo.valid
  io.mbTxLaneIo.bits := patternWriter.io.mbTxLaneIo.bits
  patternWriter.io.mbTxLaneIo.ready := io.mbTxLaneIo.ready

  patternWriter.io.txLfsrCtrl.pattern := txLfsr.io.lfsrOutput
  txLfsr.io.increment := VecInit(Seq.fill(afeParams.mbLanes)(patternWriter.io.txLfsrCtrl.increment))
  txLfsr.io.resetLfsr := VecInit(Seq.fill(afeParams.mbLanes)(patternWriter.io.txLfsrCtrl.resetLfsr))

  io.txLfsrCtrl.valid := patternWriter.io.txLfsrCtrl.valid
  io.txLfsrCtrl.resetLfsr := patternWriter.io.txLfsrCtrl.resetLfsr
  io.txLfsrCtrl.increment := patternWriter.io.txLfsrCtrl.increment
}

class PatternWriterTest extends AnyFunSpec with ChiselSim {
  val serializerRatio = 32
  val lanes = 16
  val maxReadyLowCycles = 3
  val randomSeed = 0x70617474L
  val printDebug = false          // Set true to see output to screen

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

  case class ExpectedMainband(
    data: Seq[BigInt],
    valid: BigInt,
    clkP: BigInt,
    clkN: BigInt,
    trk: BigInt
  )

  case class ScoreboardRow(field: String, actual: Bits, expected: BigInt)

  def params = AfeParams(mbSerializerRatio = serializerRatio, mbLanes = lanes)

  def exactNumCycles(patternName: String, numBits: Int): Int = {
    require(
      numBits % serializerRatio == 0,
      s"$patternName has $numBits bits, which is not divisible by serializer ratio $serializerRatio"
    )
    numBits / serializerRatio
  }

  def repeatedPatternWords(pattern: BigInt, patternWidth: Int): Seq[BigInt] = {
    val commonDivisor = BigInt(patternWidth).gcd(BigInt(serializerRatio)).toInt
    val numPhases = patternWidth / commonDivisor

    Seq.tabulate(numPhases) { phase =>
      Seq.tabulate(serializerRatio) { bit =>
        ((pattern >> ((phase * serializerRatio + bit) % patternWidth)) & 1) << bit
      }.foldLeft(BigInt(0))(_ | _)
    }
  }

  val clkRepairPatternWords = repeatedPatternWords(BigInt("000055555555", 16), 48)
  val valTrainPatternWords = repeatedPatternWords(BigInt("00001111", 2), 8)
  val fwClkPPatternWords = repeatedPatternWords(BigInt("01010101", 2), 8)
  val fwClkNPatternWords = repeatedPatternWords(BigInt("10101010", 2), 8)
  val perLaneIdPatternWords = Seq.tabulate(lanes) { lane =>
    val perLaneIdPattern =
      (BigInt("1010", 2) << 12) | (BigInt(lane & 0xff) << 4) | BigInt("1010", 2)
    repeatedPatternWords(perLaneIdPattern, 16)
  }

  val clkRepairCycles = exactNumCycles("CLKREPAIR", 128 * 48)
  val valTrainCycles = exactNumCycles("VALTRAIN", 128 * 8)
  val perLaneIdCycles = exactNumCycles("PERLANEID", 128 * 16)
  val lfsrCycles = exactNumCycles("LFSR", 4096)

  def laneReferenceModels(): Seq[ReferenceLFSR] =
    Seq.tabulate(lanes) { lane =>
      new ReferenceLFSR(laneSeeds(lane % laneSeeds.length), polynomial, lfsrWidth)
    }

  def clearRequest(dut: PatternWriterWithLfsrHarness): Unit = {
    dut.io.interfaceIo.req.valid.poke(false.B)
    dut.io.interfaceIo.req.bits.patternType.poke(PatternSelect.CLKREPAIR)
  }

  def hexWord(value: BigInt): String = {
    val hexDigits = (serializerRatio + 3) / 4
    "0x" + value.toString(16).reverse.padTo(hexDigits, '0').reverse
  }

  def padRight(value: String, width: Int): String =
    value + (" " * math.max(0, width - value.length))

  def scoreboardRows(dut: PatternWriterWithLfsrHarness, expected: ExpectedMainband): Seq[ScoreboardRow] =
    Seq(
      ScoreboardRow("valid", dut.io.mbTxLaneIo.bits.valid, expected.valid),
      ScoreboardRow("clkP", dut.io.mbTxLaneIo.bits.clkP, expected.clkP),
      ScoreboardRow("clkN", dut.io.mbTxLaneIo.bits.clkN, expected.clkN),
      ScoreboardRow("trk", dut.io.mbTxLaneIo.bits.trk, expected.trk)
    ) ++ expected.data.zipWithIndex.map {
      case (word, lane) => ScoreboardRow(s"data[$lane]", dut.io.mbTxLaneIo.bits.data(lane), word)
    }

  def checkScoreboard(
    dut: PatternWriterWithLfsrHarness,
    patternName: String,
    fireCount: Int,
    totalCycles: Int,
    expected: ExpectedMainband,
    context: String,
    printThisCycle: Boolean
  ): Unit = {
    val rows = scoreboardRows(dut, expected)

    if (printDebug && printThisCycle) {
      val fieldColumnWidth = math.max(10, rows.map(_.field.length).max)
      val valueColumnWidth = math.max(10, hexWord(BigInt(0)).length)
      val divider = "=" * (fieldColumnWidth + (2 * valueColumnWidth) + 24)
      val separator = "-" * (fieldColumnWidth + (2 * valueColumnWidth) + 24)

      println(s"[PatternWriterTest] $divider")
      println(s"[PatternWriterTest] $patternName fire ${fireCount + 1}/$totalCycles")
      println(
        s"[PatternWriterTest] complete=${dut.io.interfaceIo.resp.complete.peek().litToBoolean} " +
          s"lfsrValid=${dut.io.txLfsrCtrl.valid.peek().litToBoolean} " +
          s"lfsrReset=${dut.io.txLfsrCtrl.resetLfsr.peek().litToBoolean} " +
          s"lfsrIncrement=${dut.io.txLfsrCtrl.increment.peek().litToBoolean}"
      )
      println(s"[PatternWriterTest] $separator")
      println(
        s"[PatternWriterTest] ${padRight("field", fieldColumnWidth)} ${padRight("actual", valueColumnWidth)} " +
          s"${padRight("expected", valueColumnWidth)}"
      )
      println(s"[PatternWriterTest] $separator")
      rows.foreach { row =>
        val actualValue = row.actual.peek().litValue
        println(
          s"[PatternWriterTest] ${padRight(row.field, fieldColumnWidth)} " +
            s"${padRight(hexWord(actualValue), valueColumnWidth)} ${padRight(hexWord(row.expected), valueColumnWidth)}"
        )
      }
    }

    rows.foreach { row =>
      row.actual.expect(row.expected.U, s"$context ${row.field} expected ${hexWord(row.expected)}")
    }
  }

  def expectedOutput(patternType: PatternSelect.Type, fireCount: Int, refs: Seq[ReferenceLFSR]): ExpectedMainband = {
    patternType match {
      case PatternSelect.CLKREPAIR =>
        val clkRepairWord = clkRepairPatternWords(fireCount % clkRepairPatternWords.length)
        ExpectedMainband(
          data = Seq.fill(lanes)(BigInt(0)),
          valid = BigInt(0),
          clkP = clkRepairWord,
          clkN = clkRepairWord,
          trk = clkRepairWord
        )

      case PatternSelect.VALTRAIN =>
        ExpectedMainband(
          data = Seq.fill(lanes)(BigInt(0)),
          valid = valTrainPatternWords(fireCount % valTrainPatternWords.length),
          clkP = fwClkPPatternWords(fireCount % fwClkPPatternWords.length),
          clkN = fwClkNPatternWords(fireCount % fwClkNPatternWords.length),
          trk = BigInt(0)
        )

      case PatternSelect.PERLANEID =>
        ExpectedMainband(
          data = Seq.tabulate(lanes) { lane =>
            perLaneIdPatternWords(lane)(fireCount % perLaneIdPatternWords(lane).length)
          },
          valid = valTrainPatternWords(fireCount % valTrainPatternWords.length),
          clkP = fwClkPPatternWords(fireCount % fwClkPPatternWords.length),
          clkN = fwClkNPatternWords(fireCount % fwClkNPatternWords.length),
          trk = BigInt(0)
        )

      case PatternSelect.LFSR =>
        ExpectedMainband(
          data = refs.map(_.peekOutputWord(serializerRatio)),
          valid = valTrainPatternWords.head,
          clkP = fwClkPPatternWords.head,
          clkN = fwClkNPatternWords.head,
          trk = BigInt(0)
        )
    }
  }

  // Ensures signals controlling the LFSR are correct when it is/isn't in
  // LFSR mode
  def expectLfsrCtrl(
    dut: PatternWriterWithLfsrHarness,
    isLfsrPattern: Boolean,
    requestCycle: Boolean,
    txFire: Boolean,
    finalFire: Boolean,
    context: String
  ): Unit = {
    dut.io.txLfsrCtrl.valid.expect(
      (isLfsrPattern && !requestCycle).B,
      s"$context txLfsrCtrl.valid"
    )
    dut.io.txLfsrCtrl.resetLfsr.expect(
      (isLfsrPattern && requestCycle).B,
      s"$context txLfsrCtrl.resetLfsr"
    )
    dut.io.txLfsrCtrl.increment.expect(
      (isLfsrPattern && !requestCycle && txFire && !finalFire).B,
      s"$context txLfsrCtrl.increment"
    )
  }

  def randomlyPulseRequestWhileBusy(dut: PatternWriterWithLfsrHarness, random: Random, context: String): Unit = {
    val pulseBusyRequest = random.nextBoolean()
    dut.io.interfaceIo.req.valid.poke(pulseBusyRequest.B)
    dut.io.interfaceIo.req.bits.patternType.poke(PatternSelect.LFSR)
    dut.io.interfaceIo.req.ready.expect(false.B, s"$context req.ready while busy")
  }

  def runPattern(
    dut: PatternWriterWithLfsrHarness,
    patternName: String,
    patternType: PatternSelect.Type,
    totalCycles: Int,
    random: Random
  ): Unit = {
    val isLfsrPattern = patternName == "LFSR"
    val refs = laneReferenceModels()

    // Setup
    dut.io.interfaceIo.req.bits.patternType.poke(patternType)
    dut.io.interfaceIo.req.valid.poke(true.B)
    dut.io.interfaceIo.req.ready.expect(true.B, s"$patternName request req.ready")
    dut.io.mbTxLaneIo.valid.expect(false.B, s"$patternName request mbTxLaneIo.valid")
    dut.io.interfaceIo.resp.complete.expect(false.B, s"$patternName request resp.complete")
    expectLfsrCtrl(dut, isLfsrPattern, requestCycle = true, txFire = false, finalFire = false, s"$patternName request")
    dut.clock.step()
    clearRequest(dut)

    // Run the pattern for the number of cycles
    for (fireCount <- 0 until totalCycles) {
      val readyLowCycles = random.nextInt(maxReadyLowCycles + 1)
      val expected = expectedOutput(patternType, fireCount, refs)

      // Keep mbTxLaneIo.ready low for some random number of cycles before HIGH
      for (attempt <- 0 to readyLowCycles) {
        val txFire = attempt == readyLowCycles
        val finalFire = txFire && (fireCount == totalCycles - 1)
        val context =
          if (txFire) s"$patternName fire $fireCount"
          else s"$patternName fire $fireCount stall $attempt"

        dut.io.mbTxLaneIo.ready.poke(txFire.B)
        randomlyPulseRequestWhileBusy(dut, random, context)
        dut.io.mbTxLaneIo.valid.expect(true.B, s"$context valid")
        dut.io.interfaceIo.resp.complete.expect(finalFire.B, s"$context complete")
        checkScoreboard(
          dut,
          patternName,
          fireCount,
          totalCycles,
          expected,
          context,
          printThisCycle = txFire   // prints when there's no backpressure
        )
        expectLfsrCtrl(
          dut,
          isLfsrPattern,
          requestCycle = false,
          txFire = txFire,
          finalFire = finalFire,
          context
        )
        dut.clock.step()
        clearRequest(dut)
      }

      if (isLfsrPattern && fireCount != totalCycles - 1) {
        refs.foreach(_.advanceState(serializerRatio))
      }
    }

    dut.io.mbTxLaneIo.ready.poke(false.B)
    clearRequest(dut)
    dut.io.interfaceIo.req.ready.expect(true.B, s"$patternName done req.ready")
    dut.io.mbTxLaneIo.valid.expect(false.B, s"$patternName done mbTxLaneIo.valid")
    dut.io.interfaceIo.resp.complete.expect(false.B, s"$patternName done resp.complete")
    expectLfsrCtrl(dut, isLfsrPattern = false, requestCycle = false, txFire = false, finalFire = false, s"$patternName done")
  }

  describe("PatternWriter") {
    it("writes CLKREPAIR with randomized request delay and TX backpressure") {
      simulate(new PatternWriterWithLfsrHarness(params)) { dut =>
        val random = new Random(randomSeed)

        clearRequest(dut)
        dut.io.mbTxLaneIo.ready.poke(false.B)

        runPattern(dut, "CLKREPAIR", PatternSelect.CLKREPAIR, clkRepairCycles, random)
      }
    }

    it("writes VALTRAIN with randomized request delay and TX backpressure") {
      simulate(new PatternWriterWithLfsrHarness(params)) { dut =>
        val random = new Random(randomSeed)

        clearRequest(dut)
        dut.io.mbTxLaneIo.ready.poke(false.B)

        runPattern(dut, "VALTRAIN", PatternSelect.VALTRAIN, valTrainCycles, random)
      }
    }

    it("writes PERLANEID with randomized request delay and TX backpressure") {
      simulate(new PatternWriterWithLfsrHarness(params)) { dut =>
        val random = new Random(randomSeed)

        clearRequest(dut)
        dut.io.mbTxLaneIo.ready.poke(false.B)

        runPattern(dut, "PERLANEID", PatternSelect.PERLANEID, perLaneIdCycles, random)
      }
    }

    it("writes LFSR with randomized request delay and TX backpressure") {
      simulate(new PatternWriterWithLfsrHarness(params)) { dut =>
        val random = new Random(randomSeed)

        clearRequest(dut)
        dut.io.mbTxLaneIo.ready.poke(false.B)

        runPattern(dut, "LFSR", PatternSelect.LFSR, lfsrCycles, random)
      }
    }
  }
}
