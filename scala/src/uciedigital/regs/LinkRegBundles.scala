// Link datapath status/control bundles.
package edu.berkeley.cs.uciedigital.regs

import chisel3._

class LinkToRegs extends Bundle {
  val linkUp = Bool()
  val linkTraining = Bool()
  val rawFormatEnabled = Bool()
  val x32AdvPkgEnabled = Bool()
  val linkWidthEnabled = UInt(4.W)
  val linkSpeedEnabled = UInt(4.W)
  val flitFormat = UInt(4.W)
  val statusChanged = Bool()
  val bwChanged = Bool()
  val corrErr = Bool()
  val uncorrNonFatal = Bool()
  val uncorrFatal = Bool()
  val trainingDone = Bool()
  val retrainDone = Bool()
}

class RegsToLink extends Bundle {
  val rawFormatEnable = Bool()
  val targetWidth = UInt(4.W)
  val targetSpeed = UInt(4.W)
  val startTraining = Bool()
  // Held pending bit of start_link_training. The FDI Active request is a level that must survive
  // the whole ADV_CAP -> REQ/RSP_ACTIVE choreography, which the one-cycle `startTraining` cannot.
  val startTrainingPending = Bool()
  val retrain = Bool()
  val corrProtoReport = Bool()
  val nonFatalProtoReport = Bool()
  val fatalProtoReport = Bool()
}
