package edu.berkeley.cs.uciedigital.logphy

import chisel3._
import chisel3.simulator.scalatest.ChiselSim
import chisel3.util._
import edu.berkeley.cs.uciedigital.interfaces._
import edu.berkeley.cs.uciedigital.sideband._
import org.scalatest.funspec.AnyFunSpec

class MBTrainLoopbackHarness(
  afeParams: AfeParams = new AfeParams(),
  sbParams: SidebandParams = new SidebandParams()
) extends Module {
  val io = IO(new Bundle {
    val start = Input(Bool())
    val pllLock = Input(Bool())
    val autoRunLinkOps = Input(Bool())
    val autoCompleteSelfCal = Input(Bool())

    val currentState = Output(MBTrainState())
    val done = Output(Bool())
    val error = Output(Bool())
  })

  val dut = Module(new MBTrainSM(afeParams, sbParams))
  val txPtReq = Module(new TxD2CPointTestRequester(afeParams, sbParams))
  val txPtResp = Module(new TxD2CPointTestResponder(afeParams, sbParams))
  val rxPtReq = Module(new RxD2CPointTestRequester(afeParams, sbParams))
  val rxPtResp = Module(new RxD2CPointTestResponder(afeParams, sbParams))
  val patternWriter = Module(new PatternWriter(afeParams))
  val patternReader = Module(new PatternReader(afeParams))
  val scrambler = Module(new UcieLFSR(afeParams))
  val descrambler = Module(new UcieLFSR(afeParams))
  val rxQueue = Module(new Queue(UInt(sbParams.sbNodeMsgWidth.W), sbParams.sbLinkAsyncQueueDepth))

  dut.io.fsmCtrl.start := io.start
  dut.io.goToState.valid := false.B
  dut.io.goToState.bits := MBTrainGoToState.goToSPEEDIDLE
  dut.io.negotiatedMaxDataRate := SpeedMode.speed16
  dut.io.pllLock := io.pllLock
  dut.io.phyInRetrain := false.B
  dut.io.interpretBy8Lane := false.B
  dut.io.maxErrorThresholdPerLane := 0.U
  dut.io.changeInRuntimeLinkCtrlRegs := false.B
  dut.io.currLocalTxFunctionalLanes := "b011".U
  dut.io.currRemoteTxFunctionalLanes := "b011".U

  dut.io.txEyeSweepReqIntfIo.done := false.B
  dut.io.txEyeSweepReqIntfIo.eyeSweepTestResults.valid := false.B
  dut.io.txEyeSweepReqIntfIo.eyeSweepTestResults.bits := 0.U.asTypeOf(dut.io.txEyeSweepReqIntfIo.eyeSweepTestResults.bits)
  dut.io.txEyeSweepRespIntfIo.done := false.B

  dut.io.rxEyeSweepReqIntfIo.done := false.B
  dut.io.rxEyeSweepReqIntfIo.eyeSweepTestResults.valid := false.B
  dut.io.rxEyeSweepReqIntfIo.eyeSweepTestResults.bits := 0.U.asTypeOf(dut.io.rxEyeSweepReqIntfIo.eyeSweepTestResults.bits)
  dut.io.rxEyeSweepRespIntfIo.done := false.B
  dut.io.rxEyeSweepRespIntfIo.remoteEyeSweepTestResults.valid := false.B
  dut.io.rxEyeSweepRespIntfIo.remoteEyeSweepTestResults.bits := 0.U.asTypeOf(dut.io.rxEyeSweepRespIntfIo.remoteEyeSweepTestResults.bits)

  txPtReq.io.start := dut.io.txPtTestReqIntfIo.start
  txPtReq.io.patternType := dut.io.txPtTestReqIntfIo.patternType
  txPtReq.io.linkTrainingParameters := dut.io.txPtTestReqIntfIo.linkTrainingParameters
  dut.io.txPtTestReqIntfIo.done := txPtReq.io.done
  dut.io.txPtTestReqIntfIo.ptTestResults := txPtReq.io.txInitPtTestResults

  txPtResp.io.start := dut.io.txPtTestRespIntfIo.start
  txPtResp.io.patternType := dut.io.txPtTestRespIntfIo.patternType
  dut.io.txPtTestRespIntfIo.done := txPtResp.io.done

  rxPtReq.io.start := dut.io.rxPtTestReqIntfIo.start
  rxPtReq.io.patternType := dut.io.rxPtTestReqIntfIo.patternType
  rxPtReq.io.linkTrainingParameters := dut.io.rxPtTestReqIntfIo.linkTrainingParameters
  dut.io.rxPtTestReqIntfIo.done := rxPtReq.io.done
  dut.io.rxPtTestReqIntfIo.ptTestResults := rxPtReq.io.rxInitPtTestLocalResults

  rxPtResp.io.start := dut.io.rxPtTestRespIntfIo.start
  rxPtResp.io.patternType := dut.io.rxPtTestRespIntfIo.patternType
  dut.io.rxPtTestRespIntfIo.done := rxPtResp.io.done

  val txClients = Seq(
    dut.io.requesterSbLaneIo.tx,
    dut.io.responderSbLaneIo.tx,
    txPtReq.io.sbLaneIo.tx,
    txPtResp.io.sbLaneIo.tx,
    rxPtReq.io.sbLaneIo.tx,
    rxPtResp.io.sbLaneIo.tx
  )
  val chosen = PriorityEncoderOH(VecInit(txClients.map(_.valid)))
  val anyTxValid = txClients.map(_.valid).reduce(_ || _)

  rxQueue.io.enq.valid := anyTxValid
  rxQueue.io.enq.bits := Mux1H(chosen, txClients.map(_.bits.data))
  txClients.zipWithIndex.foreach { case (client, idx) =>
    client.ready := rxQueue.io.enq.ready && chosen(idx)
  }

  val rxClients = Seq(
    dut.io.requesterSbLaneIo.rx,
    dut.io.responderSbLaneIo.rx,
    txPtReq.io.sbLaneIo.rx,
    txPtResp.io.sbLaneIo.rx,
    rxPtReq.io.sbLaneIo.rx,
    rxPtResp.io.sbLaneIo.rx
  )
  rxClients.foreach { client =>
    client.valid := rxQueue.io.deq.valid
    client.bits.data := rxQueue.io.deq.bits
  }
  rxQueue.io.deq.ready := rxClients.map(_.ready).reduce(_ || _)

  val writerClients = Seq(
    (txPtReq.io.usingPatternWriter, txPtReq.io.patternWriterIo),
    (rxPtResp.io.usingPatternWriter, rxPtResp.io.patternWriterIo)
  )
  val readerClients = Seq(
    (txPtResp.io.usingPatternReader, txPtResp.io.patternReaderIo),
    (rxPtReq.io.usingPatternReader, rxPtReq.io.patternReaderIo)
  )

  patternWriter.io.interfaceIo.req.valid := false.B
  patternWriter.io.interfaceIo.req.bits := DontCare
  patternWriter.io.interfaceIo.functionalLanes := "b011".U
  writerClients.foreach { case (active, clientIo) =>
    clientIo.resp := patternWriter.io.interfaceIo.resp
    clientIo.req.ready := false.B
    when(active) {
      patternWriter.io.interfaceIo.req.valid := clientIo.req.valid
      patternWriter.io.interfaceIo.req.bits := clientIo.req.bits
      patternWriter.io.interfaceIo.functionalLanes := clientIo.functionalLanes
      clientIo.req.ready := patternWriter.io.interfaceIo.req.ready
    }
  }

  patternReader.io.interfaceIo.req.valid := false.B
  patternReader.io.interfaceIo.req.bits := DontCare
  patternReader.io.interfaceIo.functionalLanes := "b011".U
  readerClients.foreach { case (active, clientIo) =>
    clientIo.resp := patternReader.io.interfaceIo.resp
    clientIo.req.ready := false.B
    when(active) {
      patternReader.io.interfaceIo.req.valid := clientIo.req.valid
      patternReader.io.interfaceIo.req.bits := clientIo.req.bits
      patternReader.io.interfaceIo.functionalLanes := clientIo.functionalLanes
      clientIo.req.ready := patternReader.io.interfaceIo.req.ready
    }
  }

  patternWriter.io.txLfsrCtrl.pattern := scrambler.io.lfsrOutput
  patternReader.io.rxLfsrCtrl.pattern := descrambler.io.lfsrOutput

  scrambler.io.increment := VecInit(Seq.fill(afeParams.mbLanes)(patternWriter.io.txLfsrCtrl.increment))
  scrambler.io.resetLfsr := VecInit(Seq.fill(afeParams.mbLanes)(patternWriter.io.txLfsrCtrl.resetLfsr))

  descrambler.io.increment := VecInit(Seq.fill(afeParams.mbLanes)(patternReader.io.rxLfsrCtrl.increment))
  descrambler.io.resetLfsr := VecInit(Seq.fill(afeParams.mbLanes)(patternReader.io.rxLfsrCtrl.resetLfsr))

  val zeroMainband = 0.U.asTypeOf(chiselTypeOf(patternReader.io.mbRxLaneIo))
  patternReader.io.mbRxLaneIo := Mux(patternWriter.io.mbTxLaneIo.valid, patternWriter.io.mbTxLaneIo.bits, zeroMainband)

  val autoTxSelfCalDone = RegNext(io.autoCompleteSelfCal && dut.io.trainingCtrl.txSelfCalStart, false.B)
  val autoRxClkCalDone = RegNext(io.autoCompleteSelfCal && dut.io.trainingCtrl.rxClkCalStart, false.B)

  dut.io.trainingCtrl.txSelfCalDone := autoTxSelfCalDone
  dut.io.trainingCtrl.rxClkCalDone := autoRxClkCalDone

  val prevReadyForReq = RegNext(dut.io.trainingCtrl.req.readyForReq, false.B)
  val readyForReqRise = dut.io.trainingCtrl.req.readyForReq && !prevReadyForReq
  val trainerIssueComplete = RegInit(false.B)
  val trainerStartPulse = WireDefault(false.B)
  val trainerCompletePulse = WireDefault(false.B)

  when(!io.autoRunLinkOps) {
    trainerIssueComplete := false.B
  }.elsewhen(readyForReqRise) {
    when(!trainerIssueComplete) {
      trainerStartPulse := true.B
      trainerIssueComplete := true.B
    }.otherwise {
      trainerCompletePulse := true.B
      trainerIssueComplete := false.B
    }
  }

  dut.io.trainingCtrl.req.start := trainerStartPulse
  dut.io.trainingCtrl.req.complete := trainerCompletePulse
  dut.io.trainingCtrl.req.testKind := TrainingTestType.PointTest

  io.currentState := dut.io.currentState
  io.done := dut.io.fsmCtrl.done
  io.error := dut.io.fsmCtrl.error
}

class MBTrainSMTest extends AnyFunSpec with ChiselSim {
  describe("MBTrainSM") {
    it("traverses the supported nominal MBTRAIN states with real point-test loopback") {
      simulate(new MBTrainLoopbackHarness()) { dut =>
        dut.io.start.poke(false.B)
        dut.io.pllLock.poke(true.B)
        dut.io.autoRunLinkOps.poke(true.B)
        dut.io.autoCompleteSelfCal.poke(true.B)
        dut.clock.step(2)

        dut.io.start.poke(true.B)

        var seenStates = Set.empty[BigInt]
        var cycles = 0
        while (!dut.io.done.peekBoolean() && !dut.io.error.peekBoolean() && cycles < 60000) {
          val state = dut.io.currentState.peek().litValue
          seenStates += state
          dut.clock.step()
          cycles += 1
        }

        assert(
          dut.io.done.peekBoolean() || dut.io.error.peekBoolean(),
          s"MBTRAIN made no forward progress within $cycles cycles"
        )

        Seq(
          MBTrainState.sVALVREF,
          MBTrainState.sDATAVREF,
          MBTrainState.sSPEEDIDLE,
          MBTrainState.sTXSELFCAL,
          MBTrainState.sRXCLKCAL,
          MBTrainState.sVALTRAINCENTER,
          MBTrainState.sVALTRAINVREF,
          MBTrainState.sDATATRAINCENTER1,
          MBTrainState.sDATATRAINVREF,
          MBTrainState.sRXDESKEW,
          MBTrainState.sDATATRAINCENTER2,
          MBTrainState.sLINKSPEED
        ).foreach { state =>
          assert(seenStates.contains(state.litValue), s"Expected to visit $state, saw $seenStates")
        }
      }
    }
  }
}
