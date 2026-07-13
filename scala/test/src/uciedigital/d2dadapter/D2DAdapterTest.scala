package edu.berkeley.cs.uciedigital.d2dadapter

import chisel3._
import chisel3.simulator.scalatest.ChiselSim
import org.scalatest.funspec.AnyFunSpec
import edu.berkeley.cs.uciedigital.interfaces._
import edu.berkeley.cs.uciedigital.sideband._

/*
  Run:
    ./mill --no-daemon test.testOnly edu.berkeley.cs.uciedigital.d2dadapter.D2DAdapterTest

  Coverage:
    Reset semantics:    FDI reports reset out of reset; the RDI reset-state
                        gating (9bec310) blocks lpIrdy while the PHY reports
                        reset (D2DAdapter.scala:119-120).
    Handshakes:         lpWakeReq -> plWakeAck and plClkReq -> lpClkAck
                        one-cycle ack registers (D2DAdapter.scala:56-60).
    Constants:          raw-format streaming is hardwired
                        (D2DAdapter.scala:47-48, 54, 61).
    Link error:         unconditional FDI -> RDI lpLinkError forwarding
                        (AdapterSM.scala:216); PHY-reported linkError entry
                        and recovery (AdapterSM.scala:491-492, 521-526).
    Bringup front half: inband presence and PHY active status drive the RDI
                        active request and the first ADV_CAP serialization
                        onto the RDI config lines (AdapterSM.scala:229-237,
                        102-109; D2DAdapter.scala:88-89).

*/
class D2DAdapterTest extends AnyFunSpec with ChiselSim {

  private val fdiParams = new FdiParams(64, 32)
  private val rdiParams = RdiParams(64, 32)
  private val sbParams = new SidebandParams()

  // One 128-bit sideband message leaves the adapter as this many serial beats
  // on the ncWidth-wide RDI config lines (SidebandInterfaceSerdes.scala:39).
  private val sbBeatsPerMsg = sbParams.sbNodeMsgWidth / rdiParams.ncWidth

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

  describe("D2DAdapter reset semantics") {

    it("reports FDI reset state out of reset") {
      simulate(mkDut) { dut =>
        initDut(dut)
        dut.clock.step(2)
        dut.io.fdi.plStateSts.expect(FDIState.reset)
      }
    }

    it("never presents lpIrdy to RDI while the PHY reports reset (9bec310 gating)") {
      simulate(mkDut) { dut =>
        initDut(dut)
        // Even if protocol pushes data (lpIrdy/lpValid asserted), the adapter
        // must gate lpIrdy while RDI reports reset (D2DAdapter.scala:119-120).
        dut.io.fdi.lpIrdy.poke(true.B)
        dut.io.fdi.lpValid.poke(true.B)
        for (_ <- 0 until 8) {
          dut.io.rdi.lpIrdy.expect(false.B, "lpIrdy must stay low while RDI plStateSts == reset")
          dut.clock.step()
        }
      }
    }
  }

  describe("D2DAdapter adjacent-layer handshakes") {

    it("acks FDI wake request one cycle later") {
      simulate(mkDut) { dut =>
        initDut(dut)
        dut.clock.step(2)
        dut.io.fdi.plWakeAck.expect(false.B)
        dut.io.fdi.lpWakeReq.poke(true.B)
        dut.clock.step()
        dut.io.fdi.plWakeAck.expect(true.B, "plWakeAck must follow lpWakeReq after 1 cycle")
        dut.io.fdi.lpWakeReq.poke(false.B)
        dut.clock.step()
        dut.io.fdi.plWakeAck.expect(false.B)
      }
    }

    it("acks RDI clock request one cycle later") {
      simulate(mkDut) { dut =>
        initDut(dut)
        dut.clock.step(2)
        dut.io.rdi.lpClkAck.expect(false.B)
        dut.io.rdi.plClkReq.poke(true.B)
        dut.clock.step()
        dut.io.rdi.lpClkAck.expect(true.B, "lpClkAck must follow plClkReq after 1 cycle")
      }
    }
  }

  describe("D2DAdapter protocol constants") {

    it("exposes raw-format streaming protocol and constant req lines") {
      simulate(mkDut) { dut =>
        initDut(dut)
        dut.clock.step()
        dut.io.fdi.plProtocol.expect(FDIProtocol.streamingNoManagementTransport)
        dut.io.fdi.plProtocolFlitFmt.expect(FDIFlitFormat.rawFormat)
        dut.io.fdi.plClkReq.expect(true.B)
        dut.io.rdi.lpWakeReq.expect(true.B)
      }
    }
  }

  describe("D2DAdapter link-error behavior") {

    it("forwards protocol lpLinkError to RDI with a one-cycle delay") {
      simulate(mkDut) { dut =>
        // The forwarding register is unconditional, independent of link state
        // (AdapterSM.scala:216; wired at D2DAdapter.scala:65, 68).
        initDut(dut)
        dut.clock.step(2)
        dut.io.rdi.lpLinkError.expect(false.B)
        dut.io.fdi.lpLinkError.poke(true.B)
        dut.clock.step()
        dut.io.rdi.lpLinkError.expect(true.B, "lpLinkError must reach RDI one cycle after FDI asserts it")
        dut.io.fdi.lpLinkError.poke(false.B)
        dut.clock.step()
        dut.io.rdi.lpLinkError.expect(false.B, "lpLinkError must clear one cycle after FDI deasserts it")
      }
    }

    it("enters linkError on a PHY linkError report and recovers to reset on an active request") {
      simulate(mkDut) { dut =>
        initDut(dut)
        dut.clock.step(2)

        // Hold lpRxActiveSts high so rxDeactive stays low and the LSM holds in
        // linkError instead of bouncing back to reset (AdapterSM.scala:85, 521-526).
        dut.io.fdi.lpRxActiveSts.poke(true.B)
        dut.io.rdi.plStateSts.poke(RDIState.linkError)
        dut.clock.step()
        for (_ <- 0 until 4) {
          dut.io.fdi.plStateSts.expect(FDIState.linkError, "PHY linkError must preempt reset-state bringup")
          dut.io.fdi.plInbandPres.expect(false.B, "plInbandPres must stay low during linkError")
          dut.clock.step()
        }

        // Recovery: PHY leaves linkError, protocol deactivates RX and requests
        // active, so the LSM returns to reset (AdapterSM.scala:521-526).
        dut.io.rdi.plStateSts.poke(RDIState.reset)
        dut.io.fdi.lpRxActiveSts.poke(false.B)
        dut.io.fdi.lpStateReq.poke(FDIStateReq.active)
        dut.clock.step(2)
        dut.io.fdi.plStateSts.expect(FDIState.reset, "active request with RX deactivated must return the LSM to reset")
      }
    }
  }

  describe("D2DAdapter bringup behavior") {

    it("requests RDI active and serializes ADV_CAP once the PHY reports inband presence and active state") {
      simulate(mkDut) { dut =>
        initDut(dut)
        dut.clock.step(2)

        // Sideband-alive report moves link init INIT_START -> RDI_BRINGUP,
        // which pins rdi_lp_state_req to active (AdapterSM.scala:229-237,
        // 102-104, 459-460; wired at D2DAdapter.scala:69-71).
        dut.io.rdi.plInbandPres.poke(true.B)
        dut.clock.step(2)
        dut.io.rdi.lpStateReq.expect(RDIStateReq.active, "adapter must request RDI active during RDI bringup")

        // PHY training completion moves RDI_BRINGUP -> PARAM_EXCH, where the
        // adapter sends ADV_CAP (AdapterSM.scala:105-109) through the D2D
        // sideband module (D2DSidebandModule.scala:150-158) out onto the RDI
        // config lines (D2DAdapter.scala:88-89).
        dut.io.rdi.plStateSts.poke(RDIState.active)
        var guard = 0
        while (!dut.io.rdi.lpCfgVld.peekBoolean() && guard < 50) {
          dut.clock.step()
          guard += 1
        }
        dut.io.rdi.lpCfgVld.expect(true.B, "ADV_CAP must be serialized onto rdi.lpCfg after PARAM_EXCH entry")

        // A single 128-bit message serializes as exactly sbBeatsPerMsg
        // back-to-back beats (SidebandInterfaceSerdes.scala:39, 53-66).
        var beats = 0
        while (dut.io.rdi.lpCfgVld.peekBoolean() && beats < sbBeatsPerMsg + 4) {
          beats += 1
          dut.clock.step()
        }
        assert(
          beats == sbBeatsPerMsg,
          s"expected $sbBeatsPerMsg consecutive sideband beats for ADV_CAP, saw $beats"
        )

        // Without a remote ADV_CAP the adapter must hold in PARAM_EXCH: FDI
        // still reports reset and the RDI active request is held
        // (AdapterSM.scala:252-254, 106).
        for (_ <- 0 until 16) {
          dut.io.fdi.plStateSts.expect(FDIState.reset, "LSM must stay in reset without a remote ADV_CAP")
          dut.io.rdi.lpStateReq.expect(RDIStateReq.active, "RDI active request must be held in PARAM_EXCH")
          dut.clock.step()
        }
      }
    }
  }
}
