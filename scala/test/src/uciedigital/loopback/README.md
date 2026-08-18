# LTSM Loopback

Two staged bring-up ladders. `LogPhyStagedBringupTest` cross-wires two
`LogicalPhy` instances (`LogPhyLoopbackHarness`) and exercises real
link-training bring-up; `UcieDigitalStagedBringupTest` stacks a full
{ProtocolLayer + D2DAdapter + LogicalPhy} per die (`UcieDigitalLoopbackHarness`)
and takes the same link to protocol-level data. Tests pay the real
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
before debugging.

S8 elaborates the RDI data ports only for itself (`exposeDataPath`): left
connected they cost every other rung several times its wall clock, because the
transmit pack and receive unpack stop folding away.

## `UcieDigitalStagedBringupTest` — one level up

Eleven rungs (U0–U10) over the full per-die stack, cross-wired at the analog
boundary. **All eleven pass as of 2026-08-13** (11/11, 10 min 44 s). It does
not re-derive the training milestones: the LogPhy ladder owns S0..S7, and U1
collapses them into a single floor rung with a real D2DAdapter on the RDI
instead of the stub (lp_clk_ack registered, lp_stall_ack through an FSM, real
cfg credits, lp_wake_req hardwired true). Above the floor: U0 checks the RDI
wake handshake with no reset wait, U2–U6 walk ADV_CAP → protocol negotiation →
FDI Active, U7 opens the chip interface, U8/U9 prove protocol beats cross the
link byte-exact — one each way, then simultaneous 4-beat bursts in order —
and U10 checks that no sideband fault bit latched while carrying the adapter's
own traffic.

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

## Simulation notes

The suites strip the firtool verification layers
(`--disable-layers=Verification,...`) and use Verilator
`OptimizeForSimulationSpeed` (`--x-initial fast`) — the default
`x-initial=unique` segfaults on the layer-stripped model with the local
verilator 5.048 build. Clean `build/chiselsim/<SuiteName>` before running;
stale svsim workdirs produce broken binaries.
