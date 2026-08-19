package edu.berkeley.cs.uciedigital

import chisel3.simulator.HasSimulator

/** Mix into a test suite so it honors UCIE_SIM_BACKEND=vcs (the coverage flow,
  * scala/verdi_coverage/run_verdi_coverage.sh). Without the env var the default
  * stays Verilator, so plain `./mill test` behavior is unchanged.
  */
trait UcieSimPrefs {
  implicit val ucieSimulator: HasSimulator =
    if (sys.env.get("UCIE_SIM_BACKEND").contains("vcs")) UcieSimBackend.fromEnv
    else HasSimulator.simulators.verilator()
}
