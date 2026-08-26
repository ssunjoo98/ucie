package edu.berkeley.cs.uciedigital.loopback

import chisel3._
import edu.berkeley.cs.uciedigital.interfaces._
import edu.berkeley.cs.uciedigital.logphy._
import edu.berkeley.cs.uciedigital.sideband._

// Two LogicalPhy instances cross-wired at the analog boundary, so every
// training exchange travels the real digital path. Sideband bits and their
// forwarded clock are wired together with no added skew, and mainband lanes go
// straight across, so REVERSALMB resolves to no reversal.
//
// The analog macro is not modelled. LogicalPhy reads two status bits from it,
// pllLock and clocksUngatedAndStable, and both are tied high here.
//
// Above the RDI the harness is a quiet adapter: it acks the clock and stall
// handshakes and drives nothing else, so the PHY never waits on a partner that
// is not part of this test.
class LogPhyLoopbackHarness(
    val afeParams: AfeParams = new AfeParams(),
    val sbParams: SidebandParams = new SidebandParams(),
    val rdiParams: RdiParams = RdiParams(64, 32)
) extends Module {
  val io = IO(new Bundle {
    val lpStateReq = Input(Vec(2, RDIStateReq()))
    val swStartLinkTraining = Input(Vec(2, Bool()))
    val pwrGood = Input(Vec(2, Bool()))

    val ltState = Output(Vec(2, LTState()))
    val trainingTimedout = Output(Vec(2, Bool()))
    val negotiatedParamsValid = Output(Vec(2, Bool()))
    val plStateSts = Output(Vec(2, RDIState()))
    val plTrainError = Output(Vec(2, Bool()))
    val plPhyInRecenter = Output(Vec(2, Bool()))
    val sbFaultSeen = Output(Vec(2, Bool()))
  })

  val duts = Seq.fill(2)(
    Module(
      new LogicalPhy(
        afeParams = afeParams,
        sbParams = sbParams,
        rdiParams = rdiParams
      )
    )
  )

  for (i <- 0 until 2) {
    val dut = duts(i).io
    val peer = duts(1 - i).io

    dut.analog.sidebandLink.in.bits := peer.analog.sidebandLink.out.bits
    dut.analog.sidebandLink.in.fwClock := peer.analog.sidebandLink.out.fwClock

    dut.analog.mainband.rx.bits := peer.analog.mainband.tx.bits
    dut.analog.mainband.rx.valid := peer.analog.mainband.tx.valid
    dut.analog.mainband.tx.ready := peer.analog.mainband.rx.ready

    dut.analog.status.pllLock := true.B
    dut.analog.status.clocksUngatedAndStable := true.B

    dut.ctrl.pwrGood := io.pwrGood(i)
    dut.ctrl.swStartLinkTraining := io.swStartLinkTraining(i)
    dut.ctrl.retryTrainingAmt := 0.U
    dut.ctrl.maxErrorThresholdPerLane := 0.U
    dut.ctrl.changeInRuntimeLinkCtrlRegsDetected := false.B
    dut.ctrl.runtimeLinkCtrlBusyBit := false.B
    dut.ctrl.runtimeRequestForRepair := false.B
    dut.ctrl.swRetrainRequest := false.B
    dut.ctrl.linkOpParamOverride := false.B
    dut.ctrl.clockPhaseSelect := 0.U

    // Both dies advertise the same parameters, so PARAM interoperates and
    // SPEEDIDLE performs no real frequency change.
    dut.ctrl.localPhyParamSettings.valid := true.B
    dut.ctrl.localPhyParamSettings.bits.voltageSwing := 0.U
    dut.ctrl.localPhyParamSettings.bits.maxDataRate := 0.U
    dut.ctrl.localPhyParamSettings.bits.clockMode := 0.U
    dut.ctrl.localPhyParamSettings.bits.clockPhase := 0.U
    dut.ctrl.localPhyParamSettings.bits.ucieSx8 := 0.U
    dut.ctrl.localPhyParamSettings.bits.sbFeatExt := 0.U
    dut.ctrl.localPhyParamSettings.bits.txAdjRuntime := 0.U
    dut.ctrl.localPhyParamSettings.bits.moduleId := 0.U

    dut.ctrl.linkTrainingParameters.clockPhase := 0.U
    dut.ctrl.linkTrainingParameters.dataPattern := 0.U
    dut.ctrl.linkTrainingParameters.validPattern := 0.U
    dut.ctrl.linkTrainingParameters.patternMode := 0.U
    dut.ctrl.linkTrainingParameters.iterationCount := 0.U
    dut.ctrl.linkTrainingParameters.idleCount := 0.U
    dut.ctrl.linkTrainingParameters.burstCount := 0.U
    dut.ctrl.linkTrainingParameters.maxErrorThreshold := 0.U
    dut.ctrl.linkTrainingParameters.comparisonMode := 0.U

    dut.rdi.lclk := false.B
    dut.rdi.lpStateReq := io.lpStateReq(i)
    dut.rdi.lpClkAck := dut.rdi.plClkReq
    dut.rdi.lpStallAck := dut.rdi.plStallReq
    dut.rdi.lpIrdy := false.B
    dut.rdi.lpValid := false.B
    dut.rdi.lpData := 0.U
    dut.rdi.lpLinkError := false.B
    dut.rdi.lpWakeReq := false.B
    dut.rdi.lpCfg := 0.U
    dut.rdi.lpCfgVld := false.B
    dut.rdi.lpCfgCrd := false.B

    io.ltState(i) := dut.status.ltState
    io.trainingTimedout(i) := dut.status.trainingTimedout
    io.negotiatedParamsValid(i) := dut.status.negotiatedPhyParamSettings.valid
    io.plStateSts(i) := dut.rdi.plStateSts
    io.plTrainError(i) := dut.rdi.plTrainError
    io.plPhyInRecenter(i) := dut.rdi.plPhyInRecenter
    io.sbFaultSeen(i) :=
      dut.status.sideband.sbParityErrSeen ||
        dut.status.sideband.sbRxPriorityQueuesFullSeen ||
        dut.status.sideband.sbDeserializerTimedoutSeen ||
        dut.status.sideband.sbInvalidRouteUpperSeen ||
        dut.status.sideband.sbInvalidRouteCurrSeen ||
        dut.status.sideband.sbInvalidRouteLowerSeen ||
        dut.status.sideband.sbUnhandledCurrentLayerMsgSeen
  }
}
