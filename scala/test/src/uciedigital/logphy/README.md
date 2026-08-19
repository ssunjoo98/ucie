# LogPHY Tests

Upstream suites (LFSR/Pattern golden-model tests) plus two new RDI suites:

| File | What it simulates | Coverage |
|------|-------------------|----------|
| `RdiStateMachineTest.scala` (17, new) | Single-DUT `RDIController`/`RDIStateMachine` with injected `{LinkMgmt.RDI.*}` sideband responses | Reset→Active bring-up handshake, Active→Retrain/LinkReset/Disabled transitions, PM L1/L2 PMNAK rejection (local + remote paths), trainingTimeout→linkError round trip, `plInbandPres` LTSM set/hold/clear tracking, documented findings: stall-ack not awaited on local exits, remote exits skip upper-layer stall, sticky stall after framing error |
| `RdiHandshakeTest.scala` (7, new) | Single-DUT clk/wake/stall handshake modules | 4-phase clock req/ack rule, wake + clock-ungate ordering, stall trigger/release conditions, fatal-assertion window (documented via intercept) |
