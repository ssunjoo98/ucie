// Register<->layer wiring for UcieDigitalTop. A plain object, not a Module: apply is called
// from inside UcieDigitalTopImpl and elaborates the same wires with no new ports.
package edu.berkeley.cs.uciedigital.regs

import chisel3._
import edu.berkeley.cs.uciedigital.interfaces.{FDIFlitFormat, FDIState}
import edu.berkeley.cs.uciedigital.logphy.{LTState, LogicalPhy}
import edu.berkeley.cs.uciedigital.protocol.ProtocolLayer

object UcieRegBridge {

  /** Wire registers <-> layers. `liveRegs` is the ONE interface driving the datapath
    * (selected in UcieDigitalTop); the internal and external blocks are both tied off here.
    */
  def apply(
      logicalPhy: LogicalPhy,
      protocolLayer: ProtocolLayer,
      liveRegs: UcieRegBlockIO,
      internalRegs: Option[UcieRegTopIO],
      externalRegs: Option[UcieRegBlockIO],
      regParams: UcieRegParams
  ): Unit = {
    val regsToLink = liveRegs.regsToLink
    val regsToPhy = liveRegs.regsToPhy

    // ==========================================================================================
    // Control down: registers -> layers
    // ==========================================================================================
    logicalPhy.io.ctrl.pwrGood := true.B // no register field and no pin source anywhere (D-19)
    logicalPhy.io.ctrl.retryTrainingAmt := 0.U // no register field
    logicalPhy.io.ctrl.swStartLinkTraining := regsToLink.startTraining // Link Control b10 fire
    logicalPhy.io.ctrl.maxErrorThresholdPerLane := regsToPhy.ts4.head(15, 4)
    logicalPhy.io.ctrl.changeInRuntimeLinkCtrlRegsDetected := false.B // alters MBTRAIN; no field
    logicalPhy.io.ctrl.runtimeLinkCtrlBusyBit := false.B // no field (linkTestBusy runs the other way)
    logicalPhy.io.ctrl.runtimeRequestForRepair := false.B // changes the training path; keep off

    logicalPhy.io.ctrl.localPhyParamSettings.valid := true.B
    logicalPhy.io.ctrl.localPhyParamSettings.bits.voltageSwing := 0.U // tx_vswing is RO capability
    // NOT target_link_speed: that field resets to 0x4 and MBInitSM ANDs the two dies' masks
    // (MBInitSM.scala:109), so a die nobody wrote would negotiate a different speed than one
    // that was written. Speed selection stays tied until both ends are written by construction.
    logicalPhy.io.ctrl.localPhyParamSettings.bits.maxDataRate := 0.U
    logicalPhy.io.ctrl.localPhyParamSettings.bits.clockMode := regsToPhy.phyControl.rxClockModeSelect
    logicalPhy.io.ctrl.localPhyParamSettings.bits.clockPhase := regsToPhy.phyControl.rxClockPhaseSelect
    logicalPhy.io.ctrl.localPhyParamSettings.bits.ucieSx8 := regsToPhy.phyControl.forceX8Width
    logicalPhy.io.ctrl.localPhyParamSettings.bits.sbFeatExt := 0.U // no field
    logicalPhy.io.ctrl.localPhyParamSettings.bits.txAdjRuntime := 0.U // no field
    logicalPhy.io.ctrl.localPhyParamSettings.bits.moduleId := 0.U // no field

    // Training Setup 1/2/4 unpacked; the packing is LogPhyRegsPerModule.scala:36,:44,:54.
    val ts1 = regsToPhy.ts1.head
    val ts2 = regsToPhy.ts2.head
    val ts4 = regsToPhy.ts4.head
    logicalPhy.io.ctrl.linkTrainingParameters.dataPattern := ts1(2, 0)
    logicalPhy.io.ctrl.linkTrainingParameters.validPattern := ts1(5, 3)
    logicalPhy.io.ctrl.linkTrainingParameters.clockPhase := ts1(9, 6)
    logicalPhy.io.ctrl.linkTrainingParameters.patternMode := ts1(10)
    logicalPhy.io.ctrl.linkTrainingParameters.burstCount := ts1(26, 11)
    logicalPhy.io.ctrl.linkTrainingParameters.idleCount := ts2(15, 0)
    logicalPhy.io.ctrl.linkTrainingParameters.iterationCount := ts2(31, 16)
    logicalPhy.io.ctrl.linkTrainingParameters.maxErrorThreshold := ts4(31, 16)
    logicalPhy.io.ctrl.linkTrainingParameters.comparisonMode := 0.U // ts4 is full; no bit exists

    // requestActive must be a LEVEL held across ADV_CAP -> REQ_ACTIVE -> RSP_ACTIVE, so it comes
    // from start_link_training's pending bit, not its one-cycle fire. Safe to hold from the write
    // because ProtocolStateController gates it into the FDI_BRINGUP window itself.
    protocolLayer.io.ctrl.requestActive := regsToLink.startTrainingPending
    protocolLayer.io.ctrl.requestRetrain := false.B // retrain_link is a pulse, this is a level (D-40)
    protocolLayer.io.ctrl.requestLinkReset := false.B // no register field
    protocolLayer.io.ctrl.requestDisable := false.B // no register field

    // ==========================================================================================
    // Status up: layers -> registers
    // ==========================================================================================
    val ltState = logicalPhy.io.status.ltState
    val phyTraining =
      (ltState === LTState.sSBINIT) || (ltState === LTState.sMBINIT) ||
        (ltState === LTState.sMBTRAIN) || (ltState === LTState.sLINKINIT) ||
        (ltState === LTState.sPHYRETRAIN)
    val fdiActive = protocolLayer.io.status.linkState === FDIState.active

    val linkStatus = Wire(new LinkToRegs)
    linkStatus := 0.U.asTypeOf(new LinkToRegs) // fields with no source stay zero
    linkStatus.linkUp := fdiActive
    linkStatus.linkTraining := phyTraining
    // trainingDone clears start_link_training's pending bit, which is also what HOLDS
    // requestActive -- so it must not fire before the FDI is up, or the request drops mid-bring-up.
    linkStatus.trainingDone := fdiActive
    linkStatus.rawFormatEnabled := protocolLayer.io.status.negotiatedProtocolValid &&
      (protocolLayer.io.status.negotiatedFlitFormat === FDIFlitFormat.rawFormat)
    linkStatus.flitFormat := protocolLayer.io.status.negotiatedFlitFormat.asUInt
    linkStatus.linkWidthEnabled := logicalPhy.io.rdi.plLnkCfg.asUInt
    linkStatus.linkSpeedEnabled := logicalPhy.io.status.negotiatedPhyParamSettings.bits.maxDataRate

    /** Drive a register block's inputs. Only the LIVE one gets real link status; the other exists
      * but reaches nothing, so it is tied off exactly as before. */
    def wireRegBlock(rb: UcieRegBlockIO, live: Boolean): Unit = {
      // Non-sticky reset for every RW field, including the pending bit holding the link up:
      // never wire this to an FDI teardown state.
      rb.linkReset := false.B
      rb.adapterToRegs := 0.U.asTypeOf(new AdapterToRegs) // D2DAdapterIO has no status port
      rb.phyToRegs := 0.U.asTypeOf(new PhyToRegs(regParams.numModules)) // no counterpart
      rb.linkToRegs := (if (live) linkStatus else 0.U.asTypeOf(new LinkToRegs))
      rb.mailboxSideband.req.ready := true.B
      rb.mailboxSideband.resp.valid := false.B
      rb.mailboxSideband.resp.bits := 0.U.asTypeOf(new MailboxSbResp)
      rb.phyToVendor.foreach(_ := 0.U.asTypeOf(new PhyToVendor))
      rb.d2dToVendor.foreach(_ := DontCare)
      dontTouch(rb.regsToAdapter) // no adapter port to receive masks/severity/threshold
      dontTouch(rb.mailboxSideband.req) // no sideband register-access handler exists
      rb.vendorToPhy.foreach(dontTouch(_)) // no LTSM consumer for the training overrides
      if (!live) { dontTouch(rb.regsToPhy); dontTouch(rb.regsToLink) }
    }

    internalRegs.foreach { rbio =>
      wireRegBlock(rbio, live = rbio eq liveRegs)
      rbio.linkEventIrq.foreach(dontTouch(_))
      rbio.linkErrorIrq.foreach(dontTouch(_))
    }
    // External reg block: drive our side so the integrator only needs a single <>.
    externalRegs.foreach { rb => wireRegBlock(rb, live = rb eq liveRegs) }
  }
}
