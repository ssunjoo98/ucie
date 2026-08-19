package edu.berkeley.cs.uciedigital.sideband

import edu.berkeley.cs.uciedigital.UcieSimPrefs
import chisel3._
import chisel3.simulator.scalatest.ChiselSim
import org.scalatest.funspec.AnyFunSpec

/*
  D-11 regression: the RAW-mode parity check must be skipped.

  WHY THIS IS A SEPARATE FILE
    SidebandLinkNodeTest already has "delivers a 64-bit message in RAW mode", but it
    runs on a freshly reset node. The deserializer's dataReg is zero there, so the DP
    check passes by coincidence and the case is green with OR without the fix. It
    cannot observe this defect. This file reproduces the one history that does.

  THE DEFECT
    SidebandLinkDeserializer assembles words with dataReg.bitSet(counter, bit) and
    never clears dataReg between words. In RAW mode maxBits is 63, so bits[127:64]
    are simply left alone:

      first RAW word after reset   -> bits[127:64] == 0        -> DP passes (luck)
      RAW word after a PACKET word -> bits[127:64] == stale payload -> DP may fail

    The clock pattern's opcode is bits[4:0] of 0x5555_5555_5555_5555 = 0b10101 = 0x15,
    which is NOT in OpsThatDontUseDPField, so doDpCalculation is true and the stale
    payload really is folded into calculatedDP. expectedDP is header bit 63 = 0.
    A stale payload with odd parity therefore drops the word.

  WHY IT MATTERS
    That is exactly the retry path. sTRAINERROR -> sRESET puts the sideband back into
    RAW, but ltsmInReset only resets LTSM submodules -- the sideband serdes keeps its
    dataReg. So the first training attempt works and every retry wedges in SBINIT.
    The loopback harness sets retryTrainingAmt = 0, which is why the ladder never
    showed it.

  HOW TO USE THIS TEST
    It must FAIL before the fix and PASS after. Confirm both directions:
      1. revert SidebandLinkNode.scala:131 to `val parityError = cpError || dpError`
      2. run -> this test fails, the rest of the suite still passes
      3. re-apply the fix
      4. run -> everything passes
    A test that has only ever been seen green proves nothing.

  Run:
    ./mill test.testOnly edu.berkeley.cs.uciedigital.sideband.SidebandRawParityRetryTest
*/
class SidebandRawParityRetryTest extends AnyFunSpec with ChiselSim with UcieSimPrefs {
  val msgW = 128
  val linkW = 1

  // SBINIT clock pattern. Header bit 63 is 0, so expectedDP is 0 and any stale
  // payload with odd parity produces a DP mismatch.
  val clkPattern = BigInt("5555555555555555", 16)

  def mask(w: Int): BigInt = (BigInt(1) << w) - 1

  def parity(x: BigInt): Int = {
    var v = x
    var p = 0
    while (v != 0) { p ^= (v & 1).toInt; v >>= 1 }
    p
  }

  /** CP over header[61:0] (even parity, reserved bits included); DP over the payload. */
  def withParity(header: BigInt, payload: BigInt): BigInt = {
    val cp = parity(header & mask(62))
    val dp = parity(payload)
    (payload << 64) | (header & mask(62)) | (BigInt(cp) << 62) | (BigInt(dp) << 63)
  }

  def mkNode() = new SidebandLinkNode(msgW, linkW, 32, 512, SidebandPriorityQueueDepths())

  /** Payload for the priming PACKET word.
    *
    * It has to have ODD parity. The clock pattern's expectedDP is its header bit 63,
    * which is 0 for 0x5555_5555_5555_5555, so a stale payload only produces a DP
    * mismatch when its own parity is 1. 0xA5 is 0b1010_0101 -- four ones per byte,
    * thirty-two over eight bytes, i.e. EVEN -- so the base value is nudged by one bit.
    */
  val payload: BigInt = {
    val base = BigInt("A5A5A5A5A5A5A5A5", 16)
    if (parity(base) == 1) base else base ^ 1
  }

  /** One bit onto the link, toggling the forwarded clock. */
  def driveBit(c: SidebandLinkNode, bit: BigInt): Unit = {
    c.io.rxIn.bits.poke(bit.U)
    c.io.rxIn.fwClock.poke(true.B)
    c.clock.step()
    c.io.rxIn.fwClock.poke(false.B)
    c.clock.step()
  }

  /** Serialize a message as 64-bit chunks separated by 32-bit idle gaps. */
  def feedRxSerial(c: SidebandLinkNode, msg: BigInt, bitWidth: Int): Unit = {
    val total = bitWidth + (bitWidth / 64) * 32
    for (i <- 0 until total) {
      if ((i < 64) || (i >= 96 && i < 160)) {
        val bitIdx = if (i >= 96) i - 32 else i
        driveBit(c, (msg >> bitIdx) & 1)
      } else {
        c.io.rxIn.bits.poke(0.U)
        c.io.rxIn.fwClock.poke(false.B)
        c.clock.step()
      }
    }
    c.io.rxIn.bits.poke(0.U)
    c.io.rxIn.fwClock.poke(false.B)
  }

  /** Wait for a word on rxOut, consume it, and drop ready again.
    *
    * ready is raised HERE and nowhere else. Holding it high across feedRxSerial would
    * let the word fire and dequeue during the hundreds of clock steps that feeding
    * takes, so the check afterwards would see an empty queue and read it as a drop.
    * Returns None if nothing arrives inside the guard.
    */
  def awaitRxOut(c: SidebandLinkNode, guardCycles: Int = 200): Option[BigInt] = {
    c.io.rxOut.ready.poke(true.B)
    var n = 0
    while (!c.io.rxOut.valid.peek().litToBoolean && n < guardCycles) {
      c.clock.step()
      n += 1
    }
    val out =
      if (c.io.rxOut.valid.peek().litToBoolean) {
        val v = c.io.rxOut.bits.peek().litValue
        c.clock.step() // consume
        Some(v)
      } else None
    c.io.rxOut.ready.poke(false.B)
    out
  }

  describe("SidebandLinkNode RAW-mode parity (D-11)") {

    it("delivers a RAW word that follows a PACKET word (retry path)") {
      simulate(mkNode()) { c =>
        // rxOut.ready stays LOW here; awaitRxOut raises it. See its comment.
        c.io.ctrl.txMode.poke(SBRxTxMode.PACKET)

        // 1. A 128-bit PACKET message whose payload has ODD parity, so it leaves a
        //    dataReg upper half that will not fold to zero. MessageWith64bData (0x1B)
        //    is a with-data opcode, so its own DP is computed and correct here.
        c.io.ctrl.rxMode.poke(SBRxTxMode.PACKET)
        assert(parity(payload) == 1, "payload must have odd parity to dirty the DP fold")
        val pkt = withParity(SBMsgOpcode.MessageWith64bData.litValue, payload)

        feedRxSerial(c, pkt, 128)
        val got1 = awaitRxOut(c)
        assert(got1.isDefined,
          "the PACKET message itself was dropped -- this test's premise is broken, " +
            "not D-11. Check the parity helper against SidebandLinkNode:112-129.")
        assert(got1.get == pkt, s"PACKET payload corrupted: got 0x${got1.get.toString(16)}")

        // 2. Back to RAW, exactly as sRESET does on a retry. dataReg[127:64] still
        //    holds the payload above; only bits[63:0] get overwritten.
        c.io.ctrl.rxMode.poke(SBRxTxMode.RAW)
        c.clock.step(4)

        feedRxSerial(c, clkPattern, 64)
        val got2 = awaitRxOut(c)

        assert(got2.isDefined,
          "D-11: the RAW clock-pattern word was DROPPED. The DP check ran over the " +
            "stale payload left in the deserializer's dataReg (RAW only writes " +
            "bits[63:0]), and the pattern's opcode 0x15 is not in " +
            "OpsThatDontUseDPField so doDpCalculation is true. Fix is " +
            "SidebandLinkNode.scala:131 -- gate parityError on rxMode =/= RAW. " +
            "On silicon this means the first training attempt works and every " +
            "retry wedges in SBINIT.")
        // Only the low 64 bits are meaningful in RAW mode. rxOut is 128 bits wide and
        // the deserializer never clears dataReg, so bits[127:64] still hold the
        // previous PACKET payload -- that leftover is the whole mechanism behind this
        // defect, and seeing it here is expected, not a corruption. Both consumers
        // mask it off: SBInit.scala:87 and LinkTrainingSM.scala:1130 compare
        // data(63,0).
        assert((got2.get & mask(64)) == clkPattern,
          s"RAW word arrived corrupted in its low 64 bits: got " +
            s"0x${(got2.get & mask(64)).toString(16)}, expected 0x${clkPattern.toString(16)} " +
            s"(full word 0x${got2.get.toString(16)})")

        // The word must not be counted as a parity error either: sbParityErr is
        // sticky and feeds LogicalPhy's sideband fault status, which the bring-up
        // ladder checks at rung S1.
        c.io.err.sbParityErr.expect(false.B,
          "a RAW word raised the sticky sideband parity error. Even when the word " +
            "still gets through, this flag propagates to sbFaultSeen and makes a " +
            "healthy SBINIT look like a limping one.")
      }
    }

    it("still drops a PACKET word with a corrupt DP bit") {
      // Guard against over-correction: the fix must not disable parity in PACKET mode.
      simulate(mkNode()) { c =>
        c.io.ctrl.rxMode.poke(SBRxTxMode.PACKET)

        val good = withParity(SBMsgOpcode.MessageWith64bData.litValue, payload)
        val bad = good ^ (BigInt(1) << 63) // flip DP

        feedRxSerial(c, bad, 128)
        val got = awaitRxOut(c)
        assert(got.isEmpty,
          "a PACKET message with a bad DP bit was delivered. The RAW guard must not " +
            "weaken PACKET-mode parity -- check that the added term is " +
            "(rxMode =/= RAW) and not something that is always true.")
        c.io.err.sbParityErr.expect(true.B, "a dropped PACKET word must set sbParityErr")
      }
    }
  }
}
