/*
  Description:
    PatternWriter takes care of sending the various patterns done during training of the mainband.

  NOTE:
  * Supports serializer ratios that exactly divide the programmed finite pattern lengths.
  * Doesn't support burst mode at the moment since burst mode isn't used for regular link training
  operations.
    - Burst mode used in compilance and debug modes
*/

package edu.berkeley.cs.uciedigital.logphy

import chisel3._
import chisel3.layer.block
import chisel3.layers.Verification
import chisel3.ltl._
import chisel3.util._

class PatternWriterIO extends Bundle { 
  val req = Flipped(Decoupled(new Bundle {
    val patternType = PatternSelect()
  }))
  val resp = new Bundle {
    val complete = Output(Bool())
  }
}

class PatternWriter(afeParams: AfeParams) extends Module {
  val io = IO(new Bundle {
    val interfaceIo = new PatternWriterIO
    val mbTxLaneIo = Decoupled(new MainbandLanes(afeParams.mbLanes, afeParams.mbSerializerRatio))
    val txLfsrCtrl = new Bundle {
      val pattern = Input(Vec(afeParams.mbLanes, UInt(afeParams.mbSerializerRatio.W)))
      val increment = Output(Bool())
      val resetLfsr = Output(Bool())
      val valid = Output(Bool())
    }
  })

  val serRatio = afeParams.mbSerializerRatio
  require(serRatio > 0, "PatternWriter requires a positive mainband serializer ratio")
  require(
    Seq(8, 16, 32, 64).contains(afeParams.mbLanes),
    "PatternWriter supports spec-defined 8, 16, 32, or 64 mainband lanes"
  )

  def exactNumCycles(patternName: String, numBits: Int): Int = {
    // No padding of bits when sending the training patterns on the mb lanes
    require(
      numBits % serRatio == 0,
      s"PatternWriter $patternName pattern length $numBits must be divisible by serializer ratio $serRatio"
    )
    numBits / serRatio
  }

  def repeatedPatternWords(pattern: BigInt, patternWidth: Int): Seq[UInt] = {
    // Distint number of serializer-width words before the pattern alignment
    // repeats
    val numPhases = patternWidth / BigInt(patternWidth).gcd(BigInt(serRatio)).toInt

    // Create a table with the different phases of the word of the pattern, then
    // populate each phase with appropriate bits depending on the serializer
    // ratio    
    Seq.tabulate(numPhases) { phase =>
      VecInit(Seq.tabulate(serRatio) { bit =>
        ((pattern >> (phase * serRatio + bit) % patternWidth) & 1).U(1.W)
      }).asUInt
    }
  }

  def selectPatternWord(words: Seq[UInt], phase: UInt, phaseWidth: Int): UInt = {
    require(words.nonEmpty, "PatternWriter pattern look up must contain at least one word")

    if (words.length == 1) {
      words.head
    } else {
      MuxLookup(phase, words.head)(
        words.zipWithIndex.map { 
          case (word, idx) => (idx.U(phaseWidth.W) -> word) 
        }
      )
    }
  }

  // Status reg
  val inProgress = RegInit(false.B)  
  val patternTypeReg = RegInit(PatternSelect.CLKREPAIR)
  
  // Pattern & Counter setup   
  // --- Clock Repair ---
  // 16 clock cycles followed by 8 cycles of low
  // PATTERN:  1010_1010_1010_1010_1010_1010_1010_1010_0000_0000_0000_0000 (48 bits) 
  val clkPatternWidth = 48
  val clkRepairPattern = BigInt("000055555555", 16)
  val clkRepairPatternWords = repeatedPatternWords(clkRepairPattern, clkPatternWidth)

  val clkRepairNumIter = 128
  val clkRepairPatternNumBits = clkRepairNumIter * clkPatternWidth
  val clkRepairPatternNumCycles = exactNumCycles("CLKREPAIR", clkRepairPatternNumBits)

  // --- Valtrain --- 
  // four 1's followed by four 0's
  val valTrainPattern = BigInt("00001111", 2)
  val valTrainWidth = 8
  val valTrainPatternWords = repeatedPatternWords(valTrainPattern, valTrainWidth)

  val valTrainNumIter = 128
  val valTrainNumBits = valTrainNumIter * valTrainWidth
  val valTrainNumCycles = exactNumCycles("VALTRAIN", valTrainNumBits)
    
  // --- PerLane ID --- 
  val perLanePatternWidth = 16
  val perLaneIdPatternWords = Seq.tabulate(afeParams.mbLanes) { lane =>
    val perLaneIdPattern =
      (BigInt("1010", 2) << 12) | (BigInt(lane & 0xff) << 4) | BigInt("1010", 2)
    repeatedPatternWords(perLaneIdPattern, perLanePatternWidth)
  }

  val perLaneNumIter = 128
  val perLaneNumBits = perLaneNumIter * perLanePatternWidth
  val perLaneNumCycles = exactNumCycles("PERLANEID", perLaneNumBits)

  // --- LFSR ---
  val lfsrNumBits = 4096
  val lfsrNumCycles = exactNumCycles("LFSR", lfsrNumBits)

  // --- Forwarded Clock ---
  val fwClkPPattern = BigInt("01010101", 2)
  val fwClkNPattern = BigInt("10101010", 2)
  val fwClkPatternWidth = 8
  val fwClkPPatternWords = repeatedPatternWords(fwClkPPattern, fwClkPatternWidth)
  val fwClkNPatternWords = repeatedPatternWords(fwClkNPattern, fwClkPatternWidth)


  // Cycles and word-phase counter logic
  val largestCycleCount = Seq(clkRepairPatternNumCycles, valTrainNumCycles, 
                              perLaneNumCycles, lfsrNumCycles).max
  val cycleCountWidth = log2Ceil(math.max(2, largestCycleCount))
  val cycleCount = RegInit(0.U(cycleCountWidth.W))

  val maxPatternPhases = Seq(
    clkRepairPatternWords.length,
    valTrainPatternWords.length,
    perLaneIdPatternWords.head.length,
    fwClkPPatternWords.length,
    fwClkNPatternWords.length
  ).max
  val patternPhaseWidth = log2Ceil(math.max(2, maxPatternPhases))
  val patternPhase = RegInit(0.U(patternPhaseWidth.W))

  val maxCycleCount = Wire(UInt(cycleCountWidth.W))
  maxCycleCount := (clkRepairPatternNumCycles - 1).U
  switch(patternTypeReg) {
    is(PatternSelect.CLKREPAIR) { maxCycleCount := (clkRepairPatternNumCycles - 1).U }
    is(PatternSelect.VALTRAIN)  { maxCycleCount := (valTrainNumCycles - 1).U }
    is(PatternSelect.PERLANEID) { maxCycleCount := (perLaneNumCycles - 1).U }
    is(PatternSelect.LFSR)      { maxCycleCount := (lfsrNumCycles - 1).U }
  }

  // Indicating when to wrap back around during of phase of the word
  val patternPhaseLimit = Wire(UInt(patternPhaseWidth.W))
  patternPhaseLimit := (clkRepairPatternWords.length - 1).U
  switch(patternTypeReg) {
    is(PatternSelect.CLKREPAIR) { patternPhaseLimit := (clkRepairPatternWords.length - 1).U }
    is(PatternSelect.VALTRAIN)  { patternPhaseLimit := (valTrainPatternWords.length - 1).U }
    is(PatternSelect.PERLANEID) { patternPhaseLimit := (perLaneIdPatternWords.head.length - 1).U }
    is(PatternSelect.LFSR)      { patternPhaseLimit := 0.U }
  }

  val clkRepairWord = selectPatternWord(clkRepairPatternWords, patternPhase, patternPhaseWidth)
  val valTrainWord = selectPatternWord(valTrainPatternWords, patternPhase, patternPhaseWidth)
  val fwClkPWord = selectPatternWord(fwClkPPatternWords, patternPhase, patternPhaseWidth)
  val fwClkNWord = selectPatternWord(fwClkNPatternWords, patternPhase, patternPhaseWidth)

  io.mbTxLaneIo.bits := 0.U.asTypeOf(chiselTypeOf(io.mbTxLaneIo.bits))
  io.mbTxLaneIo.valid := inProgress

  io.interfaceIo.req.ready := !inProgress
  io.interfaceIo.resp.complete := io.mbTxLaneIo.fire && (cycleCount === maxCycleCount)

  io.txLfsrCtrl.valid := inProgress && (patternTypeReg === PatternSelect.LFSR)
  io.txLfsrCtrl.resetLfsr :=
    io.interfaceIo.req.fire && (io.interfaceIo.req.bits.patternType === PatternSelect.LFSR)
  io.txLfsrCtrl.increment :=
    inProgress && (patternTypeReg === PatternSelect.LFSR) && io.mbTxLaneIo.fire &&
    (cycleCount =/= maxCycleCount)

  switch(patternTypeReg) {
    is(PatternSelect.CLKREPAIR) {
      io.mbTxLaneIo.bits.clkP := clkRepairWord
      io.mbTxLaneIo.bits.clkN := clkRepairWord
      io.mbTxLaneIo.bits.trk := clkRepairWord
    }
    is(PatternSelect.VALTRAIN) {
      io.mbTxLaneIo.bits.valid := valTrainWord
      io.mbTxLaneIo.bits.clkP := fwClkPWord
      io.mbTxLaneIo.bits.clkN := fwClkNWord
    }
    is(PatternSelect.PERLANEID) {
      for (lane <- 0 until afeParams.mbLanes) {
        io.mbTxLaneIo.bits.data(lane) :=
          selectPatternWord(perLaneIdPatternWords(lane), patternPhase, patternPhaseWidth)
      }
      io.mbTxLaneIo.bits.valid := valTrainWord
      io.mbTxLaneIo.bits.clkP := fwClkPWord
      io.mbTxLaneIo.bits.clkN := fwClkNWord
    }
    is(PatternSelect.LFSR) {
      for (lane <- 0 until afeParams.mbLanes) {
        io.mbTxLaneIo.bits.data(lane) := io.txLfsrCtrl.pattern(lane)
      }
      io.mbTxLaneIo.bits.valid := valTrainWord
      io.mbTxLaneIo.bits.clkP := fwClkPWord
      io.mbTxLaneIo.bits.clkN := fwClkNWord
    }
  }

  // Accepting a request
  when(io.interfaceIo.req.fire) {
    inProgress := true.B
    patternTypeReg := io.interfaceIo.req.bits.patternType
    cycleCount := 0.U
    patternPhase := 0.U
  }.elsewhen(io.mbTxLaneIo.fire) { // Sending pattern to PHY
    when(cycleCount === maxCycleCount) {
      inProgress := false.B
      cycleCount := 0.U
      patternPhase := 0.U
    }.otherwise {
      cycleCount := cycleCount + 1.U

      when(patternTypeReg =/= PatternSelect.LFSR) {
        when(patternPhase === patternPhaseLimit) {
          patternPhase := 0.U
        }.otherwise {
          patternPhase := patternPhase + 1.U
        }
      }
    }
  }

  // Assertions
  block(Verification) {
    block(Verification.Assert) {
      AssertProperty(
        Sequence.BoolSequence(io.interfaceIo.req.fire) |=> 
          Sequence.BoolSequence(inProgress),
        label = Some("PatternWriterReqFireStartsPattern")
      )
      AssertProperty(
        Sequence.BoolSequence(io.mbTxLaneIo.valid && !io.mbTxLaneIo.ready) |=> 
          Sequence.BoolSequence(io.mbTxLaneIo.valid),
        label = Some("PatternWriterStaysValidUnderBackpressure")
      )

      AssertProperty(
        Sequence.BoolSequence(io.mbTxLaneIo.fire && cycleCount === maxCycleCount) |=> 
          Sequence.BoolSequence(!inProgress),
        label = Some("PatternWriterFinalFireClearsInProgress")
      )
      AssertProperty(
          Sequence.BoolSequence(cycleCount <= maxCycleCount),
        label = Some("PatternWriterCycleCountWithinPattern")
      )

      AssertProperty(
        Sequence.BoolSequence(!(io.txLfsrCtrl.resetLfsr && io.txLfsrCtrl.increment)),
        label = Some("PatternWriterLfsrResetAndIncrementMutuallyExclusive")
      )

      AssertProperty(
        Sequence.BoolSequence(io.interfaceIo.resp.complete) |=>
          Sequence.BoolSequence(!io.interfaceIo.resp.complete),
        label = Some("PatternWriterCompleteIsOneCyclePulse")
      )

      // Used because Chisel LTL Assertion doesn't have $stable for the output
      val heldLastCycle = RegNext(io.mbTxLaneIo.valid && !io.mbTxLaneIo.ready, false.B)
      val previousOutputBits = RegNext(io.mbTxLaneIo.bits.asUInt)
      val previousCycleCount = RegNext(cycleCount)
      val previousPatternPhase = RegNext(patternPhase)
      val previousPatternType = RegNext(patternTypeReg)

      AssertProperty(
        Sequence.BoolSequence(heldLastCycle) |->
          Sequence.BoolSequence(
            io.mbTxLaneIo.bits.asUInt === previousOutputBits &&
            cycleCount === previousCycleCount &&
            patternPhase === previousPatternPhase &&
            patternTypeReg === previousPatternType
          ),
        label = Some("PatternWriterHoldsStateAndBitsUnderBackpressure")
      )
    }
  }
}
