/*
  Description:
    UcieDigitalTop wires together the protocol layer,
    die-to-die adapter, logical PHY, and the UCIe register block.

    The main interface parameters live in UcieDigitalTopParams. In general:
    - FDI/RDI/sideband width parameters define the top-level interface shape.
    - LogicalPhyTopParams contains the primary tuning knobs for bring-up and
      training experiments, such as retry width and sideband timeout depth.
*/
package edu.berkeley.cs.uciedigital.top

import chisel3._
import chisel3.util.Decoupled
import org.chipsalliance.cde.config.Parameters
import org.chipsalliance.diplomacy.lazymodule._
import freechips.rocketchip.prci.{ClockSourceNode, ClockSourceParameters}
import edu.berkeley.cs.uciedigital.d2dadapter.D2DAdapter
import edu.berkeley.cs.uciedigital.logphy._
import edu.berkeley.cs.uciedigital.protocol._
import edu.berkeley.cs.uciedigital.regs._
import edu.berkeley.cs.uciedigital.sideband.SidebandParams

class UcieDigitalTopChipIO(protocolParams: ProtocolTopParams) extends Bundle {
  val mainbandTx = Flipped(Decoupled(new ProtocolRawBeat(protocolParams.fdi.nBytes)))
  val mainbandRx = Decoupled(new ProtocolRawBeat(protocolParams.fdi.nBytes))
}

class UcieDigitalTopPhyIO(afeParams: AfeParams, sbParams: SidebandParams) extends Bundle {
  val mainbandLink = new MainbandLaneIO(afeParams)
  val sidebandLink = new SidebandPhyLinkIO(sbParams.sbLinkWidth)
}

class UcieDigitalTopIO(params: UcieDigitalTopParams) extends Bundle {
  val chipFacingIo = new UcieDigitalTopChipIO(params.protocol)
  val phyFacingIo  = new UcieDigitalTopPhyIO(params.logPhy.afe, params.logPhy.sideband)
}

class UcieDigitalTop(params: UcieDigitalTopParams = UcieDigitalTopParams.default())(implicit p: Parameters)
    extends LazyModule {
  private val validatedParams = params.validate()
  override lazy val desiredName = "UcieDigitalTop"

  val regs = LazyModule(new UcieRegTop(validatedParams.regs))
  val regNode = regs.node
  val intNode = regs.intNode
  private val regClockSource = ClockSourceNode(Seq(ClockSourceParameters()))
  regs.clockNode := regClockSource

  override lazy val module = new UcieDigitalTopImpl
  class UcieDigitalTopImpl extends LazyModuleImp(this) {
    val io = IO(new UcieDigitalTopIO(validatedParams))

    val (regClk, _) = regClockSource.out(0)
    regClk.clock := clock
    regClk.reset := reset

    val protocolLayer = Module(new ProtocolLayer(
      params = validatedParams.protocol.layer,
      fdiParams = validatedParams.protocol.fdi,
      sbParams = validatedParams.adapter.sideband
    ))
    val d2dAdapter = Module(new D2DAdapter(
      fdiParams = validatedParams.adapter.fdi,
      rdiParams = validatedParams.adapter.rdi,
      sbParams = validatedParams.adapter.sideband
    ))
    val logicalPhy = Module(new LogicalPhy(
      afeParams = validatedParams.logPhy.afe,
      sbParams = validatedParams.logPhy.sideband,
      rdiParams = validatedParams.logPhy.rdi,
      retryW = validatedParams.logPhy.retryW,
      desTimeoutCycles = validatedParams.logPhy.desTimeoutCycles,
      queueDepths = validatedParams.logPhy.queueDepths
    ))

    // Internal connection
    protocolLayer.io.fdi <> d2dAdapter.io.fdi
    d2dAdapter.io.rdi <> logicalPhy.io.rdi

    // Chip-facing connection
    protocolLayer.io.mainbandTx <> io.chipFacingIo.mainbandTx
    io.chipFacingIo.mainbandRx <> protocolLayer.io.mainbandRx

    // PHY-facing connection
    io.phyFacingIo.mainbandLink <> logicalPhy.io.analog.mainband
    io.phyFacingIo.sidebandLink <> logicalPhy.io.analog.sidebandLink

    // TODO: pending connection -- status to regs. Layer ctrl/status + PHY macro ctrl/status + the
    // register-block bundles are not yet cross-wired, working on bug fixes; tie off inputs, keep outputs (dontTouch) so nothing is pruned.
    protocolLayer.io.ctrl := DontCare
    logicalPhy.io.ctrl := DontCare
    logicalPhy.io.analog.status := DontCare // PHY->logphy status
    dontTouch(protocolLayer.io.status)
    dontTouch(logicalPhy.io.status)
    dontTouch(logicalPhy.io.analog.ctrl) // logphy->PHY control

    regs.module.io.linkReset := false.B
    regs.module.io.adapterToRegs := 0.U.asTypeOf(new AdapterToRegs)
    regs.module.io.phyToRegs := 0.U.asTypeOf(new PhyToRegs(validatedParams.regs.numModules))
    regs.module.io.linkToRegs := 0.U.asTypeOf(new LinkToRegs)
    regs.module.io.mailboxSideband.req.ready := true.B
    regs.module.io.mailboxSideband.resp.valid := false.B
    regs.module.io.mailboxSideband.resp.bits := 0.U.asTypeOf(new MailboxSbResp)
    regs.module.io.phyToVendor.foreach(_ := 0.U.asTypeOf(new PhyToVendor))
    regs.module.io.d2dToVendor.foreach(_ := DontCare)
    dontTouch(regs.module.io.regsToAdapter)
    dontTouch(regs.module.io.regsToPhy)
    dontTouch(regs.module.io.regsToLink)
    dontTouch(regs.module.io.mailboxSideband.req)
    regs.module.io.vendorToPhy.foreach(dontTouch(_))
    regs.module.io.linkEventIrq.foreach(dontTouch(_))
    regs.module.io.linkErrorIrq.foreach(dontTouch(_))
  }
}
