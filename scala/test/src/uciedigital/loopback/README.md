# LTSM Loopback (WIP — parked)

Two-`LogicalPhy` loopback harness targeting real link-training progression
(`sRESET → sSBINIT → sMBINIT → ... → sACTIVE`) plus a full-stack variant with
D2D adapters on top. **Work in progress**: the tests are `ignore`d and parts are
commented out because they depended on LogicalPhy observability hooks
(`ltsmTimeoutCycles`, `phyTrainerAutoRespond`, LTSM state export) that were
prototyped on modified RTL and reverted to keep `src/` pristine.

Blocked on (per analysis): PhyLaneTrainer stub responses for MBTRAIN, LTSM
state observability. Expected reachable without RTL changes: up to
`sMBINIT_PARAM`. See `VERIFICATION_PLAN.md` (LT- scenarios).
