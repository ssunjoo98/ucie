// Emit SystemVerilog for the modules exercised by the coverage testbenches
// under verilog/coverage_testbenches/. Regenerate with:
//   ./mill test.runMain edu.berkeley.cs.uciedigital.EmitCoverageRtl
//
// Split emission is required: the RTL's temporal SVA (AssertProperty) must be
// emitted as separate verification-layer files; inlining them into a single
// file makes firtool fail with "verification operation used in a
// non-verification context" (same reason the upstream Main* emit apps fail).
package edu.berkeley.cs.uciedigital

import circt.stage.ChiselStage
import edu.berkeley.cs.uciedigital.interfaces._
import edu.berkeley.cs.uciedigital.sideband.SidebandParams
import edu.berkeley.cs.uciedigital.d2dadapter.D2DAdapter
import edu.berkeley.cs.uciedigital.logphy._

object EmitCoverageRtl extends App {
  private val baseDir = "../verilog/coverage_testbenches/rtl"
  private val opts = Array(
    "--split-verilog",
    "--disable-all-randomization",
    "--strip-debug-info",
    "--lowering-options=disallowLocalVariables"
  )
  private def emit(sub: String)(gen: => chisel3.RawModule): Unit =
    ChiselStage.emitSystemVerilogFile(gen, args = Array("-td", s"$baseDir/$sub"), firtoolOpts = opts)

  emit("d2dadapter")(new D2DAdapter(new FdiParams(64, 32), RdiParams(64, 32), new SidebandParams()))
  emit("rdi_controller")(new RDIController(new SidebandParams()))
  emit("rdi_handshakes")(new RDIClockHandshakeRequester())
  emit("rdi_handshakes")(new RDIWakeHandshakeResponder())
  emit("rdi_handshakes")(new RDIStallRequester())
  // Same parameterization as the loopback harnesses (LogPhyLoopbackHarness /
  // FullStackLoopbackHarness). NOTE for SV testbenches: the sideband link
  // serializer instantiates the `ser21` BlackBox; its implementation lives at
  // scala/resources/vsrc/ser21.v and is NOT listed in the emitted filelist.f,
  // so it must be added to the compile list by hand.
  emit("logphy")(new LogicalPhy(new AfeParams(), new SidebandParams(), RdiParams(64, 32)))
}
