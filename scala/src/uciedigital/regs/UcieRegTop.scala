// Diplomatic top of the UCIe register region with its TLRegisterNode and register map.
package edu.berkeley.cs.uciedigital.regs

import chisel3._
import org.chipsalliance.cde.config.Parameters
import org.chipsalliance.diplomacy.lazymodule._
import freechips.rocketchip.prci.{ClockSinkDomain, ClockSinkParameters}
import freechips.rocketchip.diplomacy.{AddressSet, SimpleDevice}
import freechips.rocketchip.tilelink.TLRegisterNode
import freechips.rocketchip.interrupts.{IntSourceNode, IntSourcePortSimple}

class UcieRegTop(val params: UcieRegParams, val beatBytes: Int = 4)(implicit p: Parameters)
    extends ClockSinkDomain(ClockSinkParameters()) {

  private val alloc = params.allocation

  val device = new SimpleDevice("ucie-regs", Seq("ucbbar,ucie-regs"))

  val node = TLRegisterNode(
    address = Seq(AddressSet(params.baseAddress, alloc.regionSize - 1)),
    device = device,
    deviceKey = "reg/control",
    beatBytes = beatBytes,
    undefZero = true
  )

  val intNode: Option[IntSourceNode] =
    if (params.diplomaticIntRouting) {
      Some(IntSourceNode(IntSourcePortSimple(num = 1, sources = 2)))
    } else None

  override lazy val module = new UcieRegTopImpl
  class UcieRegTopImpl extends Impl {
    val io = IO(new Bundle {
      val linkReset = Input(Bool())
      val adapterToRegs = Input(new AdapterToRegs)
      val regsToAdapter = Output(new RegsToAdapter)
      val phyToRegs = Input(new PhyToRegs(params.numModules))
      val regsToPhy = Output(new RegsToPhy(params.numModules))
      val linkToRegs = Input(new LinkToRegs)
      val regsToLink = Output(new RegsToLink)
      val mailboxSideband = new MailboxToSideband
      val vendorToPhy = if (params.hasVendorPhyBlock) Some(Output(new VendorToPhy)) else None
      val phyToVendor = if (params.hasVendorPhyBlock) Some(Input(new PhyToVendor)) else None
      val vendorToD2d = if (params.hasVendorD2dBlock) Some(Output(new VendorToD2d)) else None
      val d2dToVendor = if (params.hasVendorD2dBlock) Some(Input(new D2dToVendor)) else None
      val linkEventIrq = if (params.diplomaticIntRouting) None else Some(Output(Bool()))
      val linkErrorIrq = if (params.diplomaticIntRouting) None else Some(Output(Bool()))
    })

    val regMapEntries: Seq[freechips.rocketchip.regmapper.RegField.Map] =
      withClockAndReset(clock, reset) {
      val (stickyReset, nonStickyReset) = UcieResets(reset, io.linkReset)
      val f = new RegFieldTypes(stickyReset, nonStickyReset)

      val dvsec = new UcieLinkDvsecRegs(f, io.linkToRegs, io.regsToLink, io.mailboxSideband, params)
      val adapterMerged = Wire(new AdapterToRegs)
      adapterMerged := io.adapterToRegs
      when(dvsec.mailboxHeaderLog1.valid) {
        adapterMerged.headerLog1.valid := true.B
        adapterMerged.headerLog1.bits := dvsec.mailboxHeaderLog1.bits
      }
      val block =
        new D2DPhyRegisterBlock(f, adapterMerged, io.regsToAdapter, io.phyToRegs, io.regsToPhy, params)

      val vendorEntries: Seq[freechips.rocketchip.regmapper.RegField.Map] =
        alloc.phyVendorBase.toSeq.flatMap { base =>
          new PhyVendorRegBlock(f, params, io.phyToVendor.get, io.vendorToPhy.get).entries(base)
        } ++ alloc.d2dVendorBase.toSeq.flatMap { base =>
          new D2dVendorRegBlock(f, params, io.d2dToVendor.get, io.vendorToD2d.get).entries(base)
        }

      val entries: Seq[freechips.rocketchip.regmapper.RegField.Map] =
        dvsec.entries(alloc.dvsecBase) ++ block.entries(alloc.d2dPhyBase) ++ vendorEntries
      node.regmap(entries: _*)

      intNode match {
        case Some(n) =>
          val (ints, _) = n.out(0)
          ints(0) := dvsec.linkEventIrq
          ints(1) := dvsec.linkErrorIrq
        case None =>
          io.linkEventIrq.get := dvsec.linkEventIrq
          io.linkErrorIrq.get := dvsec.linkErrorIrq
      }

      entries
    }
  }
}
