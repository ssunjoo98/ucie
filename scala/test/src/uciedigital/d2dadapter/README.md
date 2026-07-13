# D2D Adapter Tests

New test suites added on top of upstream `ucie-verif` (none existed for this layer before).
All pass under ChiselSim/Verilator — see repo-root `SETUP.md`-equivalent env notes; run with:

```bash
./mill test.testOnly 'edu.berkeley.cs.uciedigital.d2dadapter.*'
```

| File | What it simulates | Coverage |
|------|-------------------|----------|
| `D2DAdapterTest.scala` (8 tests) | Single DUT, directed pokes on FDI/RDI | Reset semantics (`plStateSts==reset`, RDI `lpIrdy` gating during PHY reset), wake/clk one-cycle ack handshakes, hardwired raw-format/streaming constants, FDI→RDI `lpLinkError` forwarding, PHY-linkError entry + recovery, bring-up front half (RDI active request + first ADV_CAP serialization onto `rdi.lpCfg`) |
| `AdapterLoopbackHarness.scala` | Not a test — hardware harness: two `D2DAdapter`s with sideband config lanes cross-wired (`rdi.lpCfg → peer rdi.plCfg`), logphy stubbed via pokes | Used by the two suites below |
| `D2DAdapterBringupTest.scala` (7 tests) | Two-adapter loopback; real sideband serialization/parity/credits | Full bring-up choreography to `active` on both dies, skew tolerance (1/5 cycles), **documented bug: PARAM_EXCH race** (skew ≥ 7 → permanent hang), **documented bug: retrain trap** (no retrain→active exit), teardown to disabled + second bring-up |
| `D2DAdapterStateTest.scala` (5 tests) | Single DUT + hand-built 128-bit sideband message injection (4-beat, CP parity) | LSM transitions reset→{disabled, linkReset} and recovery back to reset, remote-originated REQ/RSP exchanges, **documented bug: linkError oscillation**, PM (L1/L2) unimplemented negatives |

"Documented bug" tests assert the *current* (buggy) behavior with a `// BUG:` comment —
they pass today and will fail the day the RTL is fixed, flagging the assertion for flip.
