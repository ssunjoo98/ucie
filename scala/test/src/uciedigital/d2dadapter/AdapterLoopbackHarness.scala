package edu.berkeley.cs.uciedigital.d2dadapter

import chisel3._
import edu.berkeley.cs.uciedigital.interfaces._
import edu.berkeley.cs.uciedigital.sideband._

/**
  * Two-adapter loopback harness for full bring-up choreography tests.
  *
  * Two D2DAdapter instances have their RDI sideband config lanes cross-wired,
  * so each adapter's serialized sideband TX (rdi.lpCfg/lpCfgVld/lpCfgCrd)
  * feeds the peer's RX (rdi.plCfg/plCfgVld/plCfgCrd). The real
  * D2DSidebandModule pair therefore exercises serialization, parity, routing,
  * and credits against each other instead of a hand-rolled testbench model.
  *
  * The logical PHY under each adapter is stubbed by testbench-poked
  * plInbandPres / plStateSts inputs. The protocol layer above each adapter is
  * driven and observed through individual FDI signals rather than a full
  * flipped Fdi bundle: only the state-management subset matters here, and
  * flat Vec(2, _) ports keep the poke/peek surface unambiguous.
  *
  * Index convention: die 0 and die 1; die i's sideband RX comes from die 1-i.
  */
class AdapterLoopbackHarness(
  val fdiParams: FdiParams,
  val rdiParams: RdiParams,
  val sbParams: SidebandParams,
) extends Module {
  val io = IO(new Bundle {
    // Logical-PHY stub inputs, one per die (poked by the testbench).
    val plInbandPres = Input(Vec(2, Bool()))
    val plStateSts = Input(Vec(2, RDIState()))

    // Protocol-layer inputs, one per die (poked by the testbench).
    val lpStateReq = Input(Vec(2, FDIStateReq()))
    val lpRxActiveSts = Input(Vec(2, Bool()))
    val lpStallAck = Input(Vec(2, Bool()))

    // FDI observations, one per die (peeked by the testbench).
    val fdiPlStateSts = Output(Vec(2, FDIState()))
    val fdiPlInbandPres = Output(Vec(2, Bool()))
    val fdiPlRxActiveReq = Output(Vec(2, Bool()))
    val fdiPlProtocolVld = Output(Vec(2, Bool()))
    val fdiPlStallReq = Output(Vec(2, Bool()))

    // RDI observation: the state request each adapter drives to its PHY stub.
    val rdiLpStateReq = Output(Vec(2, RDIStateReq()))
  })

  val duts = Seq.fill(2)(Module(new D2DAdapter(fdiParams, rdiParams, sbParams)))

  for (i <- 0 until 2) {
    val dut = duts(i).io
    val peer = duts(1 - i).io

    // Sideband crossover: this die's RX config lanes are the peer's TX lanes,
    // and the TX credit return it consumes is the peer's RX credit return
    // (D2DAdapter.scala:85-90).
    dut.rdi.plCfg := peer.rdi.lpCfg
    dut.rdi.plCfgVld := peer.rdi.lpCfgVld
    dut.rdi.plCfgCrd := peer.rdi.lpCfgCrd

    // Logical-PHY state stub, driven by the testbench.
    dut.rdi.plStateSts := io.plStateSts(i)
    dut.rdi.plInbandPres := io.plInbandPres(i)

    // Remaining RDI inputs tied off to a quiet PHY.
    dut.rdi.plTrdy := false.B
    dut.rdi.plValid := false.B
    dut.rdi.plData := 0.U
    dut.rdi.plError := false.B
    dut.rdi.plCError := false.B
    dut.rdi.plNfError := false.B
    dut.rdi.plTrainError := false.B
    dut.rdi.plPhyInRecenter := false.B
    dut.rdi.plStallReq := false.B
    dut.rdi.plSpeedmode := SpeedMode.speed4
    dut.rdi.plMaxSpeedmode := false.B
    dut.rdi.plLnkCfg := LinkWidth.x16
    dut.rdi.plClkReq := false.B
    dut.rdi.plWakeAck := false.B

    // Protocol-layer drive (testbench-controlled state-management subset).
    dut.fdi.lpStateReq := io.lpStateReq(i)
    dut.fdi.lpRxActiveSts := io.lpRxActiveSts(i)
    dut.fdi.lpStallAck := io.lpStallAck(i)

    // Remaining FDI inputs tied off to a quiet protocol layer.
    dut.fdi.lpIrdy := false.B
    dut.fdi.lpValid := false.B
    dut.fdi.lpData := 0.U
    dut.fdi.lpLinkError := false.B
    dut.fdi.lpClkAck := false.B
    dut.fdi.lpWakeReq := false.B
    dut.fdi.lpCfg := 0.U
    dut.fdi.lpCfgVld := false.B
    dut.fdi.plCfgCrd := false.B

    // Observations.
    io.fdiPlStateSts(i) := dut.fdi.plStateSts
    io.fdiPlInbandPres(i) := dut.fdi.plInbandPres
    io.fdiPlRxActiveReq(i) := dut.fdi.plRxActiveReq
    io.fdiPlProtocolVld(i) := dut.fdi.plProtocolVld
    io.fdiPlStallReq(i) := dut.fdi.plStallReq
    io.rdiLpStateReq(i) := dut.rdi.lpStateReq
  }
}
