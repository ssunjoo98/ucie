package edu.berkeley.cs.uciedigital.loopback

import edu.berkeley.cs.uciedigital.logphy._
import edu.berkeley.cs.uciedigital.phy._
import edu.berkeley.cs.uciedigital.phy.macros._

import chisel3._
import chisel3.util._
import freechips.rocketchip.util.{AsyncQueue, AsyncQueueParams}
import edu.berkeley.cs.uciedigital.interfaces._
import edu.berkeley.cs.uciedigital.sideband._

/**
  * Two-die loopback with the REAL analog PHY in the loop:
  *
  *   LogicalPhy -> Phy -> PhyBumpsIO  ><  PhyBumpsIO <- Phy <- LogicalPhy
  *
  * Same experiment as LogPhyLoopbackHarness, one step further along the
  * horizontal axis: that harness cross-wires the two LogicalPhy at
  * `io.analog` directly, this one routes every bit through a `Phy` and crosses
  * only at the bumps. Nothing above the RDI changes; the port list, the RDI
  * quiet adapter and the sbFault OR are copied from it so a rung going red is
  * attributable to the PHY and to nothing else.
  *
  * ---------------------------------------------------------------------------
  * CLOCKING. Phy is a RawModule whose whole clock tree enters at the bumps
  * (Phy.scala:140-153), so the harness owns three of them per die:
  *
  *   bypassClkP       = the harness clock            (mainband bit clock)
  *   bypassClkN       = its exact inverse            (tx_lane.v:221,:266 is DDR;
  *                                                    a non-inverse degenerates it)
  *   digitalBypassClk = phy.io.clkRst.txDivClk       = bypassClkP / 16
  *
  * The /16 is not a choice: tx_lane.v shifts on both edges of a 3-bit counter,
  * so one 32-bit word takes exactly 16 bypassClkP periods, and ucie_clk_div4.v
  * clkout_3 divides by the same 16 (Phy.scala:201-204). Driving the digital
  * domain from txDivClk makes ucieClk and txDivClk literally the same net, so
  * the TX side needs no CDC at all -- TileLink.scala:690-698 needs its TX
  * AsyncQueue only because it runs ucieClk at 800 MHz against a 500 MHz word
  * clock. The RX side still needs one; see below.
  *
  * CONSEQUENCE FOR THE TESTBENCH: 1 digital cycle = 16 harness clock steps.
  * Every cycle constant in the test must be scaled; LogPhyPhyBumpsTest routes
  * every step through one accessor for that reason.
  *
  * ---------------------------------------------------------------------------
  * THINGS THAT MUST BE TIED OR NOTHING MOVES (all verified against the models):
  *
  *   regs.pllBypassEn := true   ucie_clk_dist_network.sv:21 drives clkMuxP_in1
  *     only; clkMuxP_in0 is declared (:5) and never assigned, and the PLL's
  *     vp_out/vn_out are read nowhere in src/. With pllBypassEn=0 ucie_clkmux.v:7
  *     selects the undriven leg and there is no TX clock at all. (D-19's root.)
  *   txctl(*).dll_reset := false   tx_lane.v:270 pins every TX bump to 0 while
  *     it is high; the register-block reset default is true (TileLink.scala:228).
  *   status.pllLock / clocksUngatedAndStable := true   still tied at the
  *     LogicalPhy, exactly as before and as TileLink.scala:753-754 does: PhyIO
  *     has no lock output anywhere, so inserting the PHY does not fix D-19.
  *
  * divResetb is tied RELEASED and never pulsed. tx_lane.v:218 resets its counter
  * to 3'b1 while rx_data_lane.v:78 resets its to 3'b0, so a real negedge on
  * divResetb desynchronises the serializer from the deserializer and rotates
  * every word by 18 bits. With no edge, --x-initial fast starts both at 0 and
  * they agree. This works under Verilator only; VCS +initreg randomises them.
  *
  * ---------------------------------------------------------------------------
  * WHAT PhyIO CANNOT CARRY (report these, do not paper over them silently):
  *
  *   1. RxIO has no clkP/clkN (Phy.scala:21-25) while MainbandLanes needs them
  *      (Bundles.scala:54-55) and PatternReader.scala:306-308 compares them in
  *      MBINIT.REPAIRCLK. `rxClkLanesFromPeer` selects the two available
  *      answers: TileLink's constants (default, TileLink.scala:735-736) or a
  *      direct peer crossover of those two lanes only. This is D-15.
  *   2. mainband tx.ready / rx.valid have no counterpart in PhyIO (tx is a bare
  *      Input, rx a bare Output). TX needs none: ucieClk IS txDivClk here, and
  *      the serializer takes exactly one word per txDivClk period, so
  *      tx.ready := true is rate-exact. RX does need one, and it comes from an
  *      AsyncQueue on rxDivClk -> ucieClk, copied from TileLink.scala:700-707.
  *      See the comment at the instantiation for why nothing cheaper works.
  *   3. PhyControlToPhyIO (lane enables, freqSel, clockPhaseSelect,
  *      doElectricalIdle*) has no counterpart either and is dropped. D-41.
  *
  * MEASURED 2026-08-15: neither (1) nor (2) is what stops the ladder at S3.
  * The blocker is that during MBINIT.REPAIRCLK the forwarded clock IS the
  * pattern under test: clkP carries the same 48-bit clkRepairWord as trk,
  * rx_clock_lane.v:38 forwards it verbatim and ucie_clk_dist_network.sv:32,35
  * makes it the clock of every rx_data_lane. So trk is sampled by a clock
  * carrying its own waveform -- every edge lands on the same bit polarity and
  * the recovered word is phase0 whatever was sent -- and the 16 static zeros
  * closing each 48 UI are not sampled at all (rx_data_lane.v:81,93 shifts only
  * on clk edges). Measured per die: three pattern phases sent (64 each), one
  * phase received (568/576 of 568/576, filtered on mbRxValid).
  * (2) is real and costs 192 sent against ~576 delivered, but fixing it alone
  * would not clear S3. Full chain in the S3 blocker string of LogPhyPhyBumpsTest.
  */
class LogPhyPhyBumpsHarness(
  val afeParams: AfeParams = new AfeParams(),
  val sbParams: SidebandParams = new SidebandParams(),
  val rdiParams: RdiParams = RdiParams(64, 32),
  val exposeDataPath: Boolean = false,
  val exposeProbe: Boolean = false,
  val rxClkLanesFromPeer: Boolean = false,
) extends Module {
  require(afeParams.mbLanes == 16, "ucie_clk_dist_network.sv hardcodes 20/18 lane clocks")
  require(afeParams.mbSerializerRatio == Phy.SerdesRatio, "tx_lane.v is 32:1")

  val numLanes = afeParams.mbLanes

  /** Harness clock steps per digital (ucieClk) cycle. */
  val cyclesPerDigital = 16

  /** Same as the production integrator's (TileLink.scala:45). */
  private val rxQueueParams = AsyncQueueParams(depth = 32)

  val io = IO(new Bundle {
    // Per-die drive (poked by the testbench).
    val lpStateReq = Input(Vec(2, RDIStateReq()))
    val swStartLinkTraining = Input(Vec(2, Bool()))
    val pwrGood = Input(Vec(2, Bool()))

    // Primary observation: the 26-state debug LTSM state and the coarse state.
    val ltsmState = Output(Vec(2, LTSMState()))
    val ltState = Output(Vec(2, LTState()))

    // Transition evidence and failure early-detection.
    val trainingTimedout = Output(Vec(2, Bool()))
    val negotiatedParamsValid = Output(Vec(2, Bool()))
    val plStateSts = Output(Vec(2, RDIState()))
    val plInbandPres = Output(Vec(2, Bool()))
    val plTrainError = Output(Vec(2, Bool()))
    val plPhyInRecenter = Output(Vec(2, Bool()))
    val plSpeedmode = Output(Vec(2, SpeedMode()))
    val sbFaultSeen = Output(Vec(2, Bool()))

    // Same port-count hazard as LogPhyLoopbackHarness.scala:73-78: extra
    // observation PORTS (not width) have made the Verilator model segfault at
    // time zero in VerilatedScope registration. Everything new here is packed
    // into one word per die and elaborated only when asked for.
    val lpData =
      Option.when(exposeDataPath)(Input(Vec(2, UInt((rdiParams.nBytes * 8).W))))
    val lpValid = Option.when(exposeDataPath)(Input(Vec(2, Bool())))
    val lpIrdy = Option.when(exposeDataPath)(Input(Vec(2, Bool())))
    val plTrdy = Output(Vec(2, Bool()))
    val plValid = Output(Vec(2, Bool()))
    val plData =
      Option.when(exposeDataPath)(Output(Vec(2, UInt((rdiParams.nBytes * 8).W))))

    /** Packed PHY-level probe, for proving the clock tree and the bumps move
      * before a rung is blamed on training. Bit map in `probeWord`.
      */
    val phyProbe = Option.when(exposeProbe)(Output(Vec(2, UInt(128.W))))
  })

  // Phy reset stretcher. ChiselSim asserts the harness reset for ONE clock
  // step; at 16 steps per digital cycle that is a fraction of a ucieClk period,
  // and ucie_rst_sync.v:10-14 needs three edges of the target clock to release.
  private val phyResetSteps = 8 * cyclesPerDigital
  private val rstCnt = RegInit(0.U(log2Ceil(phyResetSteps + 1).W))
  when(rstCnt =/= phyResetSteps.U) { rstCnt := rstCnt + 1.U }
  private val phyReset = reset.asBool || rstCnt =/= phyResetSteps.U

  private def txCtlDefault: TxLaneDigitalCtlIO = {
    val w = Wire(new TxLaneDigitalCtlIO)
    w.dll_reset := false.B // MANDATORY, see header
    w.driver.pu_ctl := 63.U
    w.driver.pd_ctl := 63.U
    w.driver.en := true.B
    w.driver.en_b := false.B
    // Analog trim; tx_lane.v declares every one of these and reads none.
    // Copied from the register-block reset values (TileLink.scala:233-245).
    w.skew.dll_en := false.B
    w.skew.ocl := false.B
    w.skew.delay := 0.U
    w.skew.mux_en := "b00000011".U
    w.skew.band_ctrl := "b01".U
    w.skew.mix_en := 0.U
    w.skew.nen_out := 20.U
    w.skew.pen_out := 22.U
    for (i <- 0 until 32) { w.shuffler(i) := i.U(5.W) }
    w.sample_negedge := false.B
    w.delay := 0.U
    w
  }

  private def rxCtlDefault: RxLaneDigitalCtlIO = {
    val w = Wire(new RxLaneDigitalCtlIO)
    // rx_data_lane.v / rx_clock_lane.v read none of these; register-block
    // reset values (TileLink.scala:250-262) so the harness matches production.
    w.zen := false.B
    w.zctl := 0.U
    w.vref_sel := 63.U
    w.afeBypassEn := false.B
    w.afeOpCycles := 16.U
    w.afeOverlapCycles := 2.U
    w.afeBypass.aEn := false.B
    w.afeBypass.aPc := true.B
    w.afeBypass.bEn := false.B
    w.afeBypass.bPc := true.B
    w.afeBypass.selA := false.B
    w.sample_negedge := false.B
    w.delay := 0.U
    w
  }

  private def pllCtlDefault: PllCtlIO = {
    val w = Wire(new PllCtlIO)
    w.dref_low := 30.U
    w.dref_high := 98.U
    w.dcoarse := 15.U
    w.d_kp := 50.U
    w.d_ki := 4.U
    w.d_clol := true.B
    w.d_ol_fcw := 0.U
    w.d_accumulator_reset := "h8000".U
    w.vco_reset := true.B
    w.digital_reset := true.B
    w
  }

  val phys = Seq.fill(2)(Module(new Phy(numLanes)(true)))

  // Per-die PHY inputs that are NOT crossed (TileLink.scala:82).
  for (i <- 0 until 2) {
    val top = phys(i).io.top
    top.bypassClkP := clock
    top.bypassClkN := (!clock.asBool).asClock
    top.digitalBypassClk := phys(i).io.clkRst.txDivClk
    // The PLL leg of the clock mux is unconnected at both ends, so refClk and
    // pllRdacVref reach nothing observable.
    top.refClkP := false.B.asClock
    top.refClkN := true.B.asClock
    top.pllRdacVref := false.B

    phys(i).io.clkRst.reset := phyReset
    phys(i).io.clkRst.divResetb := true.B.asAsyncReset // never pulsed, see header

    phys(i).io.regs.pllBypassEn := true.B // MANDATORY, see header
    phys(i).io.regs.pllCtl := pllCtlDefault
    for (l <- 0 until numLanes + 4) {
      phys(i).io.regs.txctl(l) := txCtlDefault
      phys(i).io.regs.rxctl(l) := rxCtlDefault
    }
  }

  // Bump crossover: straight, lane i to lane i, both directions. Identical to
  // UcieBumpsIO.connect (TileLink.scala:85-99), which is the authored recipe.
  // A straight crossing is what makes MBINIT.REVERSALMB resolve to "no
  // reversal", same as the direct harness.
  for (i <- 0 until 2) {
    val me = phys(i).io.top
    val peer = phys(1 - i).io.top
    me.rxData := peer.txData
    me.rxValid := peer.txValid
    me.rxTrack := peer.txTrack
    me.rxClkP := peer.txClkP
    me.rxClkN := peer.txClkN
    me.sbRxClk := peer.sbTxClk
    me.sbRxData := peer.sbTxData
  }

  val duts = phys.map { p =>
    withClockAndReset(p.io.clkRst.ucieClk, p.io.clkRst.ucieRst) {
      Module(new LogicalPhy(
        afeParams = afeParams,
        sbParams = sbParams,
        rdiParams = rdiParams,
      ))
    }
  }

  /** Free-running counter in each die's ucieClk domain. The only way to see the
    * digital clock from the testbench: everything else visible there is a state
    * register that does not move during RESET. P0 uses it to prove both that
    * ucieClk runs and that it runs at exactly 1/cyclesPerDigital of the harness
    * clock. Probe-only.
    */
  private val digiCnts: Seq[Option[UInt]] = phys.map { p =>
    Option.when(exposeProbe) {
      withClockAndReset(p.io.clkRst.ucieClk, p.io.clkRst.ucieRst) {
        val c = RegInit(0.U(8.W))
        c := c + 1.U
        c
      }
    }
  }

  for (i <- 0 until 2) {
    val dut = duts(i).io
    val peer = duts(1 - i).io
    val phy = phys(i).io

    // Sideband: a zero-latency wire through the PHY (tx_driver.v:87 is
    // `assign dout = din`, Phy.scala:161-179 adds no register), so this is
    // bit-identical to the direct crossover it replaces.
    phy.sb.txClk := dut.analog.sidebandLink.out.fwClock.asBool.asClock
    phy.sb.txData := dut.analog.sidebandLink.out.bits.asBool
    dut.analog.sidebandLink.in.bits := phy.sb.rxData.asUInt
    dut.analog.sidebandLink.in.fwClock := phy.sb.rxClk.asUInt

    // Mainband TX into the 32:1 serializers.
    phy.tx.data := dut.analog.mainband.tx.bits.data
    phy.tx.valid := dut.analog.mainband.tx.bits.valid
    phy.tx.clkp := dut.analog.mainband.tx.bits.clkP
    phy.tx.clkn := dut.analog.mainband.tx.bits.clkN
    phy.tx.track := dut.analog.mainband.tx.bits.trk
    // One word per digital cycle is exactly the serializer rate; no hardware
    // behind it though (PhyIO has no handshake).
    dut.analog.mainband.tx.ready := true.B

    // Mainband RX out of the 1:32 deserializers, across the rxDivClk -> ucieClk
    // boundary. The AsyncQueue is copied from TileLink.scala:700-707 and it is
    // NOT optional:
    //   - rxDivClk is the PEER's forwarded clock / 16 (Phy.scala:210-213,
    //     ucie_clk_dist_network.sv:32), so it is a genuinely foreign domain and
    //     it does not run at all until the peer transmits a clock-bearing
    //     pattern. ucieClk cannot sample it: both are bit-clock/16, so a
    //     2-flop edge detector in the ucieClk domain sees a constant.
    //   - deq.valid is the only per-WORD receive strobe available anywhere.
    //     PhyIO has no such signal (io.rx is a bare Output with no handshake)
    //     and RxIO.valid is the deserialized valid LANE, not a word strobe.
    //     PatternReader.scala:246 gates its phase counter on mbRxValid, and its
    //     comment at :233-245 records what happens when that counts clocks
    //     instead of words: "every iteration came back dirty, and
    //     MBINIT.REPAIRCLK reported failure".
    // The queue is necessary but NOT sufficient: measured, it makes mbRxValid a
    // real per-word strobe and S3 still stops in exactly the same place, because
    // the serializer emits a word per slot whether or not one was sent. See the
    // note on (2) in the header.
    val rxFifo = Module(new AsyncQueue(new RxIO(numLanes), rxQueueParams))
    rxFifo.io.enq.bits := phy.rx
    rxFifo.io.enq.valid := true.B
    rxFifo.io.enq_clock := phy.clkRst.rxDivClk
    rxFifo.io.enq_reset := phy.clkRst.rxDivRst
    rxFifo.io.deq_clock := phy.clkRst.ucieClk
    rxFifo.io.deq_reset := phy.clkRst.ucieRst
    rxFifo.io.deq.ready := dut.analog.mainband.rx.ready
    dut.analog.mainband.rx.bits.data := rxFifo.io.deq.bits.data
    dut.analog.mainband.rx.bits.valid := rxFifo.io.deq.bits.valid
    dut.analog.mainband.rx.bits.trk := rxFifo.io.deq.bits.track
    dut.analog.mainband.rx.valid := rxFifo.io.deq.valid
    if (rxClkLanesFromPeer) {
      // D-15 escape hatch: the two clock lanes, and only those, bypass the PHY.
      dut.analog.mainband.rx.bits.clkP := peer.analog.mainband.tx.bits.clkP
      dut.analog.mainband.rx.bits.clkN := peer.analog.mainband.tx.bits.clkN
    } else {
      // What the only production integrator does (TileLink.scala:735-736).
      dut.analog.mainband.rx.bits.clkP := "h55555555".U
      dut.analog.mainband.rx.bits.clkN := "haaaaaaaa".U
    }

    // PHY status: no lock or ungate source exists anywhere in PhyIO.
    dut.analog.status.pllLock := true.B
    dut.analog.status.clocksUngatedAndStable := true.B

    // ------------------------------------------------------------------
    // Everything below is copied verbatim from LogPhyLoopbackHarness so any
    // difference in the ladder is attributable to the PHY.
    // ------------------------------------------------------------------
    dut.ctrl.pwrGood := io.pwrGood(i)
    dut.ctrl.retryTrainingAmt := 0.U
    dut.ctrl.swStartLinkTraining := io.swStartLinkTraining(i)
    dut.ctrl.maxErrorThresholdPerLane := 0.U
    dut.ctrl.changeInRuntimeLinkCtrlRegsDetected := false.B
    dut.ctrl.runtimeLinkCtrlBusyBit := false.B
    dut.ctrl.runtimeRequestForRepair := false.B

    dut.ctrl.localPhyParamSettings.valid := true.B
    dut.ctrl.localPhyParamSettings.bits.voltageSwing := 0.U
    dut.ctrl.localPhyParamSettings.bits.maxDataRate := 0.U // speed4
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
    dut.rdi.lpIrdy := io.lpIrdy.map(_(i)).getOrElse(false.B)
    dut.rdi.lpValid := io.lpValid.map(_(i)).getOrElse(false.B)
    dut.rdi.lpData := io.lpData.map(_(i)).getOrElse(0.U)
    dut.rdi.lpLinkError := false.B
    dut.rdi.lpWakeReq := false.B
    dut.rdi.lpCfg := 0.U
    dut.rdi.lpCfgVld := false.B
    dut.rdi.lpCfgCrd := false.B

    io.ltsmState(i) := dut.status.currentState
    io.ltState(i) := dut.status.ltState
    io.trainingTimedout(i) := dut.status.trainingTimedout
    io.negotiatedParamsValid(i) := dut.status.negotiatedPhyParamSettings.valid
    io.plStateSts(i) := dut.rdi.plStateSts
    io.plInbandPres(i) := dut.rdi.plInbandPres
    io.plTrainError(i) := dut.rdi.plTrainError
    io.plPhyInRecenter(i) := dut.rdi.plPhyInRecenter
    io.plSpeedmode(i) := dut.rdi.plSpeedmode
    // sbFirstFaultValid is deliberately excluded: it is the OR of the same
    // seven terms, so it adds no detection.
    io.sbFaultSeen(i) :=
      dut.status.sideband.sbParityErrSeen ||
      dut.status.sideband.sbRxPriorityQueuesFullSeen ||
      dut.status.sideband.sbDeserializerTimedoutSeen ||
      dut.status.sideband.sbInvalidRouteUpperSeen ||
      dut.status.sideband.sbInvalidRouteCurrSeen ||
      dut.status.sideband.sbInvalidRouteLowerSeen ||
      dut.status.sideband.sbUnhandledCurrentLayerMsgSeen
    io.plTrdy(i) := dut.rdi.plTrdy
    io.plValid(i) := dut.rdi.plValid
    io.plData.foreach(p => p(i) := dut.rdi.plData)

    io.phyProbe.foreach { p => p(i) := probeWord(i) }
  }

  /** Bit map, per die:
    *   0 ucieRst          1 txDivClk         2 txClkP bump    3 txClkN bump
    *   4 txValid bump     5 txData(0) bump   6 sbTxClk bump   7 sbTxData bump
    *   8 rxClkP bump      9 rxDivRst        10 mbRxValid     11 mb tx.valid
    *  12 mb tx.ready     13 rxDivClk        14 rx.valid word nonzero
    *  15 rx.track nonzero
    *  23:16 ucieClk-domain free-running counter
    *  31:24 rx.data(0)[7:0]
    *  63:32 rx.track at the PHY output, pre-AsyncQueue
    *  95:64 mainband tx.bits.trk -- the word this die HANDS to the serializer
    * 127:96 mainband rx.bits.trk -- the word PatternReader actually COMPARES
    *        (post-AsyncQueue, PatternReader.scala:310)
    *
    * bits 8-10 and 13 are the mainband RX story: is a forwarded clock arriving
    * at all (8, 13), has its reset domain released (9), and is any WORD being
    * delivered to PatternReader (10, which is exactly its mbRxValid).
    *
    * 95:64 against 127:96 is the CLKREPAIR round-trip: same lane, same pattern,
    * one serializer and one deserializer apart. If the set of words sent is not
    * the set of words received, the serdes is lossy for this pattern.
    */
  private def probeWord(i: Int): UInt = {
    val phy = phys(i).io
    val dut = duts(i).io
    Cat(
      dut.analog.mainband.rx.bits.trk,
      dut.analog.mainband.tx.bits.trk,
      phy.rx.track,
      phy.rx.data(0)(7, 0),
      digiCnts(i).getOrElse(0.U(8.W)),
      phy.rx.track =/= 0.U,
      phy.rx.valid =/= 0.U,
      phy.clkRst.rxDivClk.asBool,
      dut.analog.mainband.tx.ready,
      dut.analog.mainband.tx.valid,
      dut.analog.mainband.rx.valid,
      phy.clkRst.rxDivRst,
      phy.top.rxClkP.asBool,
      phy.top.sbTxData,
      phy.top.sbTxClk.asBool,
      phy.top.txData(0),
      phy.top.txValid,
      phy.top.txClkN.asBool,
      phy.top.txClkP.asBool,
      phy.clkRst.txDivClk.asBool,
      phy.clkRst.ucieRst,
    )
  }
}
