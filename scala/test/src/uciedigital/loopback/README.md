# LTSM Loopback

Two-`LogicalPhy` loopback harness (`LogPhyLoopbackHarness`) exercising real
link-training bring-up, plus a full-stack variant with D2D adapters on top
(`FullStackLoopbackHarness`, not yet used by a test). Tests pay the real
3.2M-cycle RESET minimum wait and observe training only through `LogicalPhy`'s
ports (`status.currentState`/`ltState`/`trainingTimedout`, `rdi.pl*`,
`analog.sidebandLink.out.{bits,fwClock}`).

## `LogPhyStagedBringupTest` — the ladder

Nine rungs, one milestone each, from RESET to protocol-free data traffic. **All
nine pass as of 2026-08-13** (9/9, 6 min 17 s):

| | Milestone |
|:--|---|
| S0 | asymmetric start: die 0 triggered, die 1 wakes on the remote clock pattern |
| S1 | SBINIT completes → `sMBINIT` |
| S2 | MBINIT.PARAM negotiated |
| S3 | MBINIT completes → `sMBTRAIN` |
| S4 | MBTRAIN completes → `sLINKINIT` |
| S5 | `plInbandPres` asserted |
| S6 | RDI reaches `RDIState.active` |
| S7 | both dies `sACTIVE`, no sideband fault anywhere in training, 20k-cycle dwell |
| S8 | RDI data crosses byte-exact both ways: bursts and simultaneous transmit |

A rung going red is now a **regression**. Each `Stage` carries a `blocker`
string naming the defects that used to live there and what to re-check; read it
before debugging. Seventeen RTL defects were fixed to get here — see
`docs_personal/FIX_LOG.md` for the per-defect history.

S8 elaborates the RDI data ports only for itself (`exposeDataPath`): left
connected they cost every other rung several times its wall clock, because the
transmit pack and receive unpack stop folding away.

## What this loopback does NOT verify

Worth knowing before trusting a green run:

- **`analog.ctrl` is never read.** The harness crosses the mainband regardless
  of the lane enables, `freqSel` and `clockPhaseSelect` that training computes,
  so the lane-repair and tristate logic is generated but unexercised.
- **`analog.status` is faked.** `pllLock` and `clocksUngatedAndStable` are tied
  high, so no PLL-not-locked or clock-unstable path is ever taken.
- **Verification layers are stripped**, so no SVA runs.
- **The channel is ideal**: zero skew, zero delay, one clock for both dies.
- MBTRAIN runs with the trainer completing immediately (no calibration
  hardware in this PHY), so the D2C link operations are not executed.

## Unit-level pins

`logphy/D2CMessageMismatchTest` pins MBTRAIN-chain message-name defects that the
ladder does not reach: the TX eye-sweep END mismatch
(`TxD2CEyeWidthSweep.scala:179/336`), the RX eye-sweep START mismatch
(`RxD2CEyeWidthSweep.scala:139/317`), and the `RxD2CPointTestRequester` start
deadlock (`req.valid` never driven, `RxD2CPointTest.scala:117-123`). The
SPEEDIDLE pair in that file is now a pass-direction regression: that mismatch
was fixed.

`sideband/SidebandRawParityRetryTest` pins the RAW-mode parity defect, which only
appears on a retry (a cold start passes by coincidence).

## Simulation notes

The suites strip the firtool verification layers
(`--disable-layers=Verification,...`) and use Verilator
`OptimizeForSimulationSpeed` (`--x-initial fast`) — the default
`x-initial=unique` segfaults on the layer-stripped model with the local
verilator 5.048 build. Clean `build/chiselsim/<SuiteName>` before running;
stale svsim workdirs produce broken binaries.
