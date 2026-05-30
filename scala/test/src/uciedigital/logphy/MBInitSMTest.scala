package edu.berkeley.cs.uciedigital.logphy

import chisel3._
import chisel3.simulator.scalatest.ChiselSim
import chisel3.util._
import edu.berkeley.cs.uciedigital.interfaces._
import edu.berkeley.cs.uciedigital.sideband._
import org.scalatest.funspec.AnyFunSpec

class MBInitLoopbackHarness(
  afeParams: AfeParams = new AfeParams(),
  sbParams: SidebandParams = new SidebandParams()
) extends Module {
  val io = IO(new Bundle {
    val start = Input(Bool())
    val selfCalDone = Input(Bool())

    val currentState = Output(MBInitState())
    val done = Output(Bool())
    val error = Output(Bool())
    val interoperableParamsNotFound = Output(Bool())
  })

  val dut = Module(new MBInitSM(afeParams, sbParams))
  val txPtReq = Module(new TxD2CPointTestRequester(afeParams, sbParams))
  val txPtResp = Module(new TxD2CPointTestResponder(afeParams, sbParams))
  val patternWriter = Module(new PatternWriter(afeParams))
  val patternReader = Module(new PatternReader(afeParams))
  val scrambler = Module(new UcieLFSR(afeParams))
  val descrambler = Module(new UcieLFSR(afeParams))
  val rxQueue = Module(new Queue(UInt(sbParams.sbNodeMsgWidth.W), sbParams.sbLinkAsyncQueueDepth))

  dut.io.fsmCtrl.start := io.start
  dut.io.mbInitCalDone := io.selfCalDone

  dut.io.localPhySettings.valid := true.B
  dut.io.localPhySettings.bits.moduleId := 0.U
  dut.io.localPhySettings.bits.voltageSwing := 0.U
  dut.io.localPhySettings.bits.maxDataRate := SpeedMode.speed16.asUInt
  dut.io.localPhySettings.bits.clockMode := 0.U
  dut.io.localPhySettings.bits.clockPhase := 0.U
  dut.io.localPhySettings.bits.ucieSx8 := 0.U
  dut.io.localPhySettings.bits.sbFeatExt := 0.U
  dut.io.localPhySettings.bits.txAdjRuntime := 0.U

  txPtReq.io.start := dut.io.txPtTestReqInterfaceIo.start
  txPtReq.io.patternType := dut.io.txPtTestReqInterfaceIo.patternType
  txPtReq.io.linkTrainingParameters := dut.io.txPtTestReqInterfaceIo.linkTrainingParameters
  dut.io.txPtTestReqInterfaceIo.done := txPtReq.io.done
  dut.io.txPtTestReqInterfaceIo.ptTestResults := txPtReq.io.txInitPtTestResults

  txPtResp.io.start := dut.io.txPtTestRespInterfaceIo.start
  txPtResp.io.patternType := dut.io.txPtTestRespInterfaceIo.patternType
  dut.io.txPtTestRespInterfaceIo.done := txPtResp.io.done

  val txClients = Seq(
    dut.io.requesterSbLaneIo.tx,
    dut.io.responderSbLaneIo.tx,
    txPtReq.io.sbLaneIo.tx,
    txPtResp.io.sbLaneIo.tx
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
    txPtResp.io.sbLaneIo.rx
  )
  rxClients.foreach { client =>
    client.valid := rxQueue.io.deq.valid
    client.bits.data := rxQueue.io.deq.bits
  }
  rxQueue.io.deq.ready := rxClients.map(_.ready).reduce(_ || _)

  val writerClients = Seq(
    (dut.io.usingPatternWriter, dut.io.patternWriterIo),
    (txPtReq.io.usingPatternWriter, txPtReq.io.patternWriterIo)
  )
  val readerClients = Seq(
    (dut.io.usingPatternReader, dut.io.patternReaderIo),
    (txPtResp.io.usingPatternReader, txPtResp.io.patternReaderIo)
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

  io.currentState := dut.io.currentState
  io.done := dut.io.fsmCtrl.done
  io.error := dut.io.fsmCtrl.error
  io.interoperableParamsNotFound := dut.io.interoperableParamsNotFound
}

class MBInitSMTest extends AnyFunSpec with ChiselSim {
  describe("MBInitSM") {
    it("runs the supported loopback flow with the real pattern engines attached") {
      simulate(new MBInitLoopbackHarness()) { dut =>
        dut.io.start.poke(false.B)
        dut.io.selfCalDone.poke(false.B)
        dut.clock.step(2)

        dut.io.start.poke(true.B)

        var seenStates = Set.empty[BigInt]
        var selfCalIssued = false
        var cycles = 0
        while (!dut.io.done.peekBoolean() && !dut.io.error.peekBoolean() && cycles < 30000) {
          seenStates += dut.io.currentState.peek().litValue

          if (!selfCalIssued && dut.io.currentState.peek().litValue == MBInitState.sCAL.litValue) {
            dut.io.selfCalDone.poke(true.B)
            selfCalIssued = true
          } else {
            dut.io.selfCalDone.poke(false.B)
          }

          dut.clock.step()
          cycles += 1
        }

        assert(
          dut.io.done.peekBoolean() || dut.io.error.peekBoolean(),
          s"MBINIT made no forward progress within $cycles cycles"
        )

        Seq(
          MBInitState.sPARAM,
          MBInitState.sCAL,
          MBInitState.sREPAIRCLK,
          MBInitState.sREPAIRVAL,
          MBInitState.sREVERSALMB,
          MBInitState.sREPAIRMB
        ).foreach { state =>
          assert(seenStates.contains(state.litValue), s"Expected to visit $state, saw $seenStates")
        }
      }
    }
  }
}
