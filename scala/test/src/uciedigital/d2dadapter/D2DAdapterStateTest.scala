package edu.berkeley.cs.uciedigital.d2dadapter

import chisel3._
import chisel3.simulator.scalatest.ChiselSim
import org.scalatest.funspec.AnyFunSpec
import edu.berkeley.cs.uciedigital.interfaces._
import edu.berkeley.cs.uciedigital.sideband._

/*
  Run:
    ./mill --no-daemon test.testOnly edu.berkeley.cs.uciedigital.d2dadapter.D2DAdapterStateTest

  Coverage:
    Disabled entry:   remote-originated {REQ,RSP}_DISABLED exchange on a single
                      DUT. A hand-built 128-bit REQ_DISABLED (wire format per
                      SBMsgCreate, SidebandMessageEncodings.scala:363-394) is
                      injected as ncWidth-wide beats on rdi.plCfg, the
                      RSP_DISABLED reply is captured off rdi.lpCfg and
                      field/parity-checked, and the LSM moves reset->disabled
                      (AdapterSM.scala:162-168, 383-405, 493-494).
    Disabled exit:    both recovery triggers back to reset: FDI active request
                      and PHY reset report (AdapterSM.scala:527-533).
    LinkReset entry/exit: the same choreography with {REQ,RSP}_LINKRESET
                      (AdapterSM.scala:144-150, 330-352, 495-496, 535-543).
    LinkError oscillation: documented bug -- with the PHY holding linkError
                      and RX deactivated, the LSM bounces reset<->linkError
                      every cycle because the linkError exit condition fires
                      while the PHY still reports linkError
                      (AdapterSM.scala:521-526).
    PM negative:      l1/l2 FDI state requests and an injected REQ_L1 sideband
                      message are ignored: no state change, no sideband TX, no
                      RDI state request. PM states/messages exist only as
                      encodings (Types.scala:89-90, D2DAdapterConstants.scala)
                      with no consumer in AdapterSM.
*/
class D2DAdapterStateTest extends AnyFunSpec with ChiselSim {

  private val fdiParams = new FdiParams(64, 32)
  private val rdiParams = RdiParams(64, 32)
  private val sbParams = new SidebandParams()

  // One 128-bit sideband message crosses the ncWidth-wide RDI config lines as
  // this many serial beats (SidebandInterfaceSerdes.scala:39).
  private val sbBeatsPerMsg = sbParams.sbNodeMsgWidth / rdiParams.ncWidth
  private val ncMask = (BigInt(1) << rdiParams.ncWidth) - 1

  private val guardCycles = 100

  private def mkDut = new D2DAdapter(fdiParams, rdiParams, sbParams)

  // Drive every DUT input to a quiet default before a scenario starts.
  private def initDut(dut: D2DAdapter): Unit = {
    // FDI inputs (protocol layer -> adapter).
    dut.io.fdi.lpIrdy.poke(false.B)
    dut.io.fdi.lpValid.poke(false.B)
    dut.io.fdi.lpData.poke(0.U)
    dut.io.fdi.lpStateReq.poke(FDIStateReq.nop)
    dut.io.fdi.lpLinkError.poke(false.B)
    dut.io.fdi.lpRxActiveSts.poke(false.B)
    dut.io.fdi.lpStallAck.poke(false.B)
    dut.io.fdi.lpClkAck.poke(false.B)
    dut.io.fdi.lpWakeReq.poke(false.B)
    dut.io.fdi.lpCfg.poke(0.U)
    dut.io.fdi.lpCfgVld.poke(false.B)
    dut.io.fdi.plCfgCrd.poke(false.B)

    // RDI inputs (logical PHY -> adapter).
    dut.io.rdi.plTrdy.poke(false.B)
    dut.io.rdi.plValid.poke(false.B)
    dut.io.rdi.plData.poke(0.U)
    dut.io.rdi.plStateSts.poke(RDIState.reset)
    dut.io.rdi.plInbandPres.poke(false.B)
    dut.io.rdi.plError.poke(false.B)
    dut.io.rdi.plCError.poke(false.B)
    dut.io.rdi.plNfError.poke(false.B)
    dut.io.rdi.plTrainError.poke(false.B)
    dut.io.rdi.plPhyInRecenter.poke(false.B)
    dut.io.rdi.plStallReq.poke(false.B)
    dut.io.rdi.plSpeedmode.poke(SpeedMode.speed4)
    dut.io.rdi.plMaxSpeedmode.poke(false.B)
    dut.io.rdi.plLnkCfg.poke(LinkWidth.x16)
    dut.io.rdi.plClkReq.poke(false.B)
    dut.io.rdi.plCfg.poke(0.U)
    dut.io.rdi.plCfgVld.poke(false.B)
    dut.io.rdi.plCfgCrd.poke(false.B)
  }

  // Step until cond holds or the guard expires; returns whether cond held.
  private def waitUntil(dut: D2DAdapter, guard: Int = guardCycles)(
      cond: => Boolean): Boolean = {
    var n = 0
    while (!cond && n < guard) {
      dut.clock.step()
      n += 1
    }
    cond
  }

  private def fdiStateIs(dut: D2DAdapter, s: FDIState.Type): Boolean =
    dut.io.fdi.plStateSts.peek().litValue == s.litValue

  // ---------------------------------------------------------------------
  // Remote sideband message injection (wire-level, as the peer adapter
  // would send it).
  //
  // 128-bit layout mirrors SBMsgCreate (SidebandMessageEncodings.scala:389-392):
  //   [4:0]     opcode          (MessageWithoutData = 0b10010 = 0x12)
  //   [21:14]   msgCode
  //   [31:29]   srcid           (D2D = 001)
  //   [39:32]   msgSubcode
  //   [58:56]   dstid           (remote bit | D2D = 101)
  //   [62]      CP = XOR(bits[61:0]) (even parity, SidebandInterfaceNode.scala:100-110)
  //   [63]      DP = 0          (MessageWithoutData carries no data)
  //   [127:64]  data = 0
  // The switch routes it to the D2D layer via dstid[57:56] == 01
  // (SidebandSwitch.scala:56, 94) and D2DSidebandModule decodes it into a
  // 1-cycle sb_rcv pulse (D2DSidebandModule.scala:88-114).
  // ---------------------------------------------------------------------
  private def linkMgmtMsg(msgCode: Int, msgSubcode: Int): BigInt = {
    var m = BigInt(0x12)                     // opcode: MessageWithoutData
    m |= BigInt(msgCode & 0xff) << 14
    m |= BigInt(1) << 29                     // srcid: D2D
    m |= BigInt(msgSubcode & 0xff) << 32
    m |= BigInt(5) << 56                     // dstid: remote | D2D
    val cp = (0 to 61).foldLeft(0)((acc, i) => acc ^ ((m >> i) & 1).toInt)
    m | (BigInt(cp) << 62)
  }

  // {LinkMgmt.Adapter0.*} wire encodings (SidebandMessageEncodings.scala:88-99).
  private val REQ_DISABLED_MSG = linkMgmtMsg(0x03, 0x0c)
  private val REQ_LINKRESET_MSG = linkMgmtMsg(0x03, 0x09)
  private val REQ_L1_MSG = linkMgmtMsg(0x03, 0x04)

  private def opcodeOf(msg: BigInt): Int = (msg & 0x1f).toInt
  private def msgCodeOf(msg: BigInt): Int = ((msg >> 14) & 0xff).toInt
  private def msgSubcodeOf(msg: BigInt): Int = ((msg >> 32) & 0xff).toInt
  private def cpOk(msg: BigInt): Boolean = {
    val cp = (0 to 61).foldLeft(0)((acc, i) => acc ^ ((msg >> i) & 1).toInt)
    ((msg >> 62) & 1).toInt == cp
  }

  // Drive a 128-bit message LSB-first onto rdi.plCfg as consecutive beats;
  // the deserializer aborts on any gap (SidebandInterfaceSerdes.scala:116-128).
  private def injectSbMsg(dut: D2DAdapter, msg: BigInt): Unit = {
    for (b <- 0 until sbBeatsPerMsg) {
      dut.io.rdi.plCfg.poke(((msg >> (b * rdiParams.ncWidth)) & ncMask).U)
      dut.io.rdi.plCfgVld.poke(true.B)
      dut.clock.step()
    }
    dut.io.rdi.plCfgVld.poke(false.B)
    dut.io.rdi.plCfg.poke(0.U)
  }

  // Wait for a serialized TX message on rdi.lpCfg and reassemble it LSB-first.
  private def captureSbMsg(dut: D2DAdapter, guard: Int = guardCycles): BigInt = {
    assert(
      waitUntil(dut, guard)(dut.io.rdi.lpCfgVld.peekBoolean()),
      "expected a sideband TX message on rdi.lpCfg"
    )
    var msg = BigInt(0)
    for (b <- 0 until sbBeatsPerMsg) {
      assert(
        dut.io.rdi.lpCfgVld.peekBoolean(),
        s"sideband TX beats must be back-to-back (missing beat $b)"
      )
      msg |= dut.io.rdi.lpCfg.peek().litValue << (b * rdiParams.ncWidth)
      dut.clock.step()
    }
    msg
  }

  private def expectLinkMgmtRsp(msg: BigInt, msgCode: Int, msgSubcode: Int, name: String): Unit = {
    assert(opcodeOf(msg) == 0x12, s"$name must use MessageWithoutData, got opcode 0x${opcodeOf(msg).toHexString}")
    assert(msgCodeOf(msg) == msgCode, s"$name msgCode mismatch, got 0x${msgCodeOf(msg).toHexString}")
    assert(msgSubcodeOf(msg) == msgSubcode, s"$name msgSubcode mismatch, got 0x${msgSubcodeOf(msg).toHexString}")
    assert(cpOk(msg), s"$name control parity (CP) must be correct")
  }

  describe("D2DAdapter remote-originated disabled/linkReset choreography") {

    it("answers an injected REQ_DISABLED with RSP_DISABLED, enters disabled, and recovers to reset via both triggers") {
      simulate(mkDut) { dut =>
        initDut(dut)
        // Keep the PHY status away from reset: the disabled state exits as
        // soon as rdi.plStateSts == reset (AdapterSM.scala:530-532), so a
        // quiet plStateSts=reset would make disabled unobservable.
        dut.io.rdi.plStateSts.poke(RDIState.disabled)
        dut.clock.step(2)
        dut.io.fdi.plStateSts.expect(FDIState.reset)

        // Remote requests disabled while the LSM sits in reset (D2D-LSM-02).
        injectSbMsg(dut, REQ_DISABLED_MSG)

        // The adapter must answer with RSP_DISABLED (AdapterSM.scala:166-168)
        // serialized onto the RDI config lines (D2D-SB-02).
        val rsp = captureSbMsg(dut)
        expectLinkMgmtRsp(rsp, msgCode = 0x04, msgSubcode = 0x0c, name = "RSP_DISABLED")

        // Sent RSP + received REQ complete the 2-way exchange: disabledEntry
        // with RX deactivated moves reset -> disabled (AdapterSM.scala:493-494).
        assert(
          waitUntil(dut)(fdiStateIs(dut, FDIState.disabled)),
          "LSM must enter disabled after the {REQ,RSP}_DISABLED exchange"
        )
        dut.clock.step(2)
        for (_ <- 0 until 4) {
          dut.io.fdi.plStateSts.expect(FDIState.disabled, "disabled must be stable")
          dut.io.fdi.plInbandPres.expect(false.B, "inband presence must drop in disabled")
          dut.io.rdi.lpStateReq.expect(RDIStateReq.disabled, "adapter must request RDI disabled while in disabled")
          dut.clock.step()
        }

        // Recovery trigger 1 (D2D-LSM-09): FDI active request exits disabled.
        dut.io.fdi.lpStateReq.poke(FDIStateReq.active)
        assert(
          waitUntil(dut)(fdiStateIs(dut, FDIState.reset)),
          "FDI active request must return the LSM from disabled to reset"
        )
        dut.clock.step(2)
        dut.io.fdi.plStateSts.expect(FDIState.reset, "reset must be stable after recovery")

        // Second round: request flags were cleared while in disabled
        // (AdapterSM.scala:406-412), so a fresh remote REQ_DISABLED restarts
        // the exchange from scratch.
        dut.io.fdi.lpStateReq.poke(FDIStateReq.nop)
        dut.clock.step(2)
        injectSbMsg(dut, REQ_DISABLED_MSG)
        val rsp2 = captureSbMsg(dut)
        expectLinkMgmtRsp(rsp2, msgCode = 0x04, msgSubcode = 0x0c, name = "second RSP_DISABLED")
        assert(
          waitUntil(dut)(fdiStateIs(dut, FDIState.disabled)),
          "LSM must re-enter disabled on a second exchange"
        )

        // Recovery trigger 2 (D2D-LSM-09): PHY reporting reset exits disabled.
        dut.io.rdi.plStateSts.poke(RDIState.reset)
        assert(
          waitUntil(dut)(fdiStateIs(dut, FDIState.reset)),
          "PHY reset report must return the LSM from disabled to reset"
        )
        dut.clock.step(2)
        dut.io.fdi.plStateSts.expect(FDIState.reset, "reset must be stable after PHY-triggered recovery")
      }
    }

    it("answers an injected REQ_LINKRESET with RSP_LINKRESET, enters linkReset, and recovers to reset via both triggers") {
      simulate(mkDut) { dut =>
        initDut(dut)
        // Same stability requirement as disabled: linkReset exits on
        // rdi.plStateSts == reset (AdapterSM.scala:540-543).
        dut.io.rdi.plStateSts.poke(RDIState.linkReset)
        dut.clock.step(2)
        dut.io.fdi.plStateSts.expect(FDIState.reset)

        // Remote requests linkReset while the LSM sits in reset (D2D-LSM-03).
        injectSbMsg(dut, REQ_LINKRESET_MSG)

        // The adapter must answer with RSP_LINKRESET (AdapterSM.scala:148-150).
        val rsp = captureSbMsg(dut)
        expectLinkMgmtRsp(rsp, msgCode = 0x04, msgSubcode = 0x09, name = "RSP_LINKRESET")

        // 2-way exchange complete: reset -> linkReset (AdapterSM.scala:495-496).
        assert(
          waitUntil(dut)(fdiStateIs(dut, FDIState.linkReset)),
          "LSM must enter linkReset after the {REQ,RSP}_LINKRESET exchange"
        )
        dut.clock.step(2)
        for (_ <- 0 until 4) {
          dut.io.fdi.plStateSts.expect(FDIState.linkReset, "linkReset must be stable")
          dut.io.fdi.plInbandPres.expect(false.B, "inband presence must drop in linkReset")
          dut.io.rdi.lpStateReq.expect(RDIStateReq.linkReset, "adapter must request RDI linkReset while in linkReset")
          dut.clock.step()
        }

        // Recovery trigger 1 (D2D-LSM-10): FDI active request exits linkReset.
        dut.io.fdi.lpStateReq.poke(FDIStateReq.active)
        assert(
          waitUntil(dut)(fdiStateIs(dut, FDIState.reset)),
          "FDI active request must return the LSM from linkReset to reset"
        )
        dut.clock.step(2)
        dut.io.fdi.plStateSts.expect(FDIState.reset, "reset must be stable after recovery")

        // Second round with the other trigger: flags were cleared while in
        // linkReset (AdapterSM.scala:353-359), so the exchange restarts fresh.
        dut.io.fdi.lpStateReq.poke(FDIStateReq.nop)
        dut.clock.step(2)
        injectSbMsg(dut, REQ_LINKRESET_MSG)
        val rsp2 = captureSbMsg(dut)
        expectLinkMgmtRsp(rsp2, msgCode = 0x04, msgSubcode = 0x09, name = "second RSP_LINKRESET")
        assert(
          waitUntil(dut)(fdiStateIs(dut, FDIState.linkReset)),
          "LSM must re-enter linkReset on a second exchange"
        )

        // Recovery trigger 2 (D2D-LSM-10): PHY reporting reset exits linkReset.
        dut.io.rdi.plStateSts.poke(RDIState.reset)
        assert(
          waitUntil(dut)(fdiStateIs(dut, FDIState.reset)),
          "PHY reset report must return the LSM from linkReset to reset"
        )
        dut.clock.step(2)
        dut.io.fdi.plStateSts.expect(FDIState.reset, "reset must be stable after PHY-triggered recovery")
      }
    }
  }

  describe("D2DAdapter linkError oscillation") {

    it("oscillates between reset and linkError every cycle while the PHY holds linkError with RX deactivated") {
      simulate(mkDut) { dut =>
        initDut(dut)
        dut.clock.step(2)

        // BUG: the linkError exit condition (AdapterSM.scala:522-523) sends
        // the LSM back to reset when (fdi_lp_state_req == active OR
        // rdi_pl_state_sts == linkError) && rxDeactive -- i.e. it exits
        // *because* the PHY still reports linkError. With lpRxActiveSts low
        // (rxDeactive true) and plStateSts held at linkError, the LSM
        // therefore bounces reset -> linkError -> reset every cycle instead
        // of holding linkError until recovery conditions are met (UCIe 3.0
        // 10.3.3.7 expects LinkError to hold until the Physical Layer has
        // left LinkError and the protocol requests Active). The condition
        // was most likely meant to be `=/= RDIState.linkError`. This test
        // pins the current (buggy) oscillation; update it once the exit
        // condition is fixed.
        dut.io.fdi.lpRxActiveSts.poke(false.B)
        dut.io.rdi.plStateSts.poke(RDIState.linkError)

        assert(
          waitUntil(dut, 4)(fdiStateIs(dut, FDIState.linkError)),
          "LSM must enter linkError on the PHY linkError report"
        )

        val resetLit = FDIState.reset.litValue
        val linkErrorLit = FDIState.linkError.litValue
        val samples = (0 until 8).map { _ =>
          val v = dut.io.fdi.plStateSts.peek().litValue
          dut.clock.step()
          v
        }
        assert(
          samples.forall(v => v == resetLit || v == linkErrorLit),
          s"oscillation must only visit reset/linkError, saw $samples"
        )
        samples.sliding(2).foreach { pair =>
          assert(
            pair.head != pair.last,
            s"fdi.plStateSts must toggle every cycle (documented oscillation), saw $samples"
          )
        }
      }
    }
  }

  describe("D2DAdapter power-management (unimplemented) negatives") {

    it("ignores FDI l1/l2 state requests: no state change, no sideband TX, no RDI request") {
      simulate(mkDut) { dut =>
        initDut(dut)
        dut.clock.step(2)

        // PM is unimplemented: FDIStateReq.l1/l2 encodings exist
        // (Types.scala:89-90) but AdapterSM only ever compares
        // fdi_lp_state_req against active/linkReset/disabled, so an l1 or l2
        // request must be completely inert (D2D-PM-01).
        for (req <- Seq(FDIStateReq.l1, FDIStateReq.l2)) {
          dut.io.fdi.lpStateReq.poke(FDIStateReq.nop)
          dut.clock.step(2)
          dut.io.fdi.lpStateReq.poke(req)
          for (_ <- 0 until 16) {
            dut.io.fdi.plStateSts.expect(FDIState.reset, "PM request must not change the link state")
            dut.io.rdi.lpCfgVld.expect(false.B, "PM request must not trigger any sideband TX")
            dut.io.rdi.lpStateReq.expect(RDIStateReq.nop, "PM request must not generate an RDI state request")
            dut.clock.step()
          }
        }
      }
    }

    it("drops an injected REQ_L1 sideband message without a response or state change") {
      simulate(mkDut) { dut =>
        initDut(dut)
        dut.clock.step(2)

        // The decoder maps {LinkMgmt.Adapter0.Req.L1} to SideBandMessage.REQ_L1
        // (D2DSidebandModule.scala:91-92), but no AdapterSM state consumes it:
        // no RSP_PMNAK, no RSP_L1, no state transition (D2D-PM-01).
        injectSbMsg(dut, REQ_L1_MSG)
        for (_ <- 0 until 40) {
          dut.io.fdi.plStateSts.expect(FDIState.reset, "REQ_L1 must not change the link state")
          dut.io.rdi.lpCfgVld.expect(false.B, "REQ_L1 must not be answered on the sideband")
          dut.io.rdi.lpStateReq.expect(RDIStateReq.nop, "REQ_L1 must not generate an RDI state request")
          dut.clock.step()
        }
      }
    }
  }
}
