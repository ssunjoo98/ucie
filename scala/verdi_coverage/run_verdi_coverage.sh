#!/bin/bash
# VCS coverage run + merge for ucie tests. Lives in scala/verdi_coverage/;
# all merged outputs stay in this directory.
#
# Usage:
#   ./run_verdi_coverage.sh                                            # full suite
#   ./run_verdi_coverage.sh --clean                                    # clean and run full suite
#   ./run_verdi_coverage.sh edu.berkeley.cs.uciedigital.logphy.UcieLFSRTest ...
#                                                                      # specific suites
#   ./run_verdi_coverage.sh --merge-only                               # just re-merge existing vdbs
#   ./run_verdi_coverage.sh --make-shim                                # build the VCS C++17 shim (once per host)
#   ./run_verdi_coverage.sh --check                                    # verify the toolchain in seconds, before a long run
#
# Per-test coverage DBs land in build/chiselsim/<Test>/<scenario>/workdir-vcs/simulation.vdb
# (fixed by ChiselSim). The merge stage produces, in this directory:
#   suites/<Suite>.vdb    <- ACCURATE per-suite merged DB (open these in Verdi)
#   reports/<Suite>/      <- per-suite urg report (all modules, line-annotated source
#                            in modinfo.txt / mod*.html)
#   modules_summary.txt   <- per-module scores of every suite, in one file
#   area_summary.txt      <- per-subsystem map: every RTL module, tested or not
#   merged.vdb, urgReport <- global all-suite merge: single trend number ONLY.
#                            Each suite elaborates a DIFFERENT design under the same
#                            top name (svsimTestbench/dut), so urg's cross-design
#                            merge silently drops non-matching module definitions.
#                            For per-module / per-line analysis use suites/ and
#                            reports/, never the global DB.
#
# View:  ./view_coverage.sh <Suite>    (no arg: global merged.vdb)

set -u
COV_DIR="$(cd "$(dirname "$0")" && pwd)"
# guard: COV_DIR feeds rm -rf paths below — never proceed with an empty value
[ -n "$COV_DIR" ] || { echo "ERROR: cannot resolve script directory"; exit 1; }
SCALA_DIR="$(dirname "$COV_DIR")"
cd "$SCALA_DIR" || exit 1

if [ "${1:-}" = "--clean" ]; then
  shift
  echo "Cleaning build/chiselsim and previous coverage outputs"
  rm -rf build/chiselsim
  # also drop every generated output, so nothing stale survives an aborted run
  rm -rf "$COV_DIR/suites" "$COV_DIR/reports" "$COV_DIR/merged.vdb" "$COV_DIR/urgReport" \
         "$COV_DIR/modules_summary.txt" "$COV_DIR/area_summary.txt"
fi

# --- compiler for the VCS csrc build ------------------------------------------
#
# csrc/Makefile drives BOTH halves of the build with one compiler (VCS_CC=gcc):
# svsim's generated C++ harness (generated-sources/*.cpp, needs C++17) and VCS's
# own generated legacy C (rmapats.c). That pins the usable compiler from both
# ends -- g++ 4.8 is too old for the harness, and gcc >= 14 rejects rmapats.c,
# which calls vcs_simpSetEBlkEvtID() with no declaration (an error since GCC 14,
# and csrc's own -w does not downgrade it). The gcc wrapper below re-adds
# -Wno-implicit-function-declaration so a modern compiler covers both.
#
# Setting PATH here does not reach VCS: svsim invokes $VCS_HOME/bin/vcs directly
# and mill's test fork rebuilds the environment. VCS_HOME is the one variable
# svsim is guaranteed to read, so the hook goes there -- a shim VCS_HOME of
# symlinks to the real one whose bin/vcs restores the real VCS_HOME and puts the
# wrappers on PATH. Build it with `--make-shim`; it lands under $UCIE_VCS_SHIM
# (default $HOME/.cache/ucie/vcs-home-shim) and is picked up automatically.
VCS_SHIM="${UCIE_VCS_SHIM:-$HOME/.cache/ucie/vcs-home-shim}"

cxx_is_cxx17() {  # $1 = compiler
  command -v "$1" >/dev/null 2>&1 || return 1
  printf '#include <string_view>\nint main(){return 0;}\n' > /tmp/.ucie_cxx17_$$.cc
  "$1" -std=c++17 -c /tmp/.ucie_cxx17_$$.cc -o /tmp/.ucie_cxx17_$$.o >/dev/null 2>&1
  local rc=$?
  rm -f /tmp/.ucie_cxx17_$$.cc /tmp/.ucie_cxx17_$$.o
  return $rc
}

find_cxx17() {  # prints the first C++17-capable g++ found, or nothing
  local c
  # $CONDA_PREFIX is the ACTIVE env, which is often the base rather than the one
  # holding the toolchain, so the envs are globbed too.
  for c in "${UCIE_CXX:-}" \
           "${CONDA_PREFIX:-}/bin/g++" \
           "${CONDA_PREFIX:-}"/envs/*/bin/g++ \
           "$HOME"/miniforge3/envs/*/bin/g++ \
           "$HOME"/miniconda3/envs/*/bin/g++ \
           g++ g++-15 g++-13 g++-11 g++-9; do
    [ -n "$c" ] || continue
    if cxx_is_cxx17 "$c"; then command -v "$c" || echo "$c"; return 0; fi
  done
  return 1
}

make_shim() {
  local real="${VCS_HOME:?VCS_HOME must point at a real VCS install}"
  local cxx; cxx="$(find_cxx17)" || {
    echo "ERROR: no C++17-capable g++ found. Set UCIE_CXX=/path/to/g++ and retry." >&2
    exit 1
  }
  local ccdir ccroot; ccdir="$(dirname "$cxx")"; ccroot="$(dirname "$ccdir")"
  echo "Building VCS shim at $VCS_SHIM"
  echo "  real VCS_HOME : $real"
  echo "  compiler      : $cxx"
  rm -rf "$VCS_SHIM"; mkdir -p "$VCS_SHIM/bin" "$VCS_SHIM/cc"
  local e b
  for e in "$real"/*; do b="$(basename "$e")"; [ "$b" = bin ] && continue; ln -sfn "$e" "$VCS_SHIM/$b"; done
  for e in "$real"/bin/*; do b="$(basename "$e")"; [ "$b" = vcs ] && continue; ln -sfn "$e" "$VCS_SHIM/bin/$b"; done
  # csrc calls plain `gcc` for both the C++ harness and rmapats.c, so the flag
  # that makes the legacy C acceptable is added here rather than in csrc/Makefile
  # (which VCS regenerates on every compile).
  cat > "$VCS_SHIM/cc/gcc" <<EOF
#!/bin/bash
# GCC 14 turned these long-standing C warnings into errors; VCS's generated C
# still relies on all of them. Only rmapats.c's implicit declaration is known to
# bite here, but the rest cost nothing and cover other VCS versions. They are C
# only, so C++ sources are passed through untouched. An older gcc silently
# ignores the -Wno-* names it does not know.
for a in "\$@"; do case "\$a" in *.cpp|*.cc|*.cxx|*.C) exec "$ccdir/gcc" "\$@";; esac; done
exec "$ccdir/gcc" -Wno-implicit-function-declaration -Wno-implicit-int \\
     -Wno-int-conversion -Wno-incompatible-pointer-types -Wno-return-mismatch "\$@"
EOF
  chmod +x "$VCS_SHIM/cc/gcc"
  ln -sfn "$cxx" "$VCS_SHIM/cc/g++"          # linker only (csrc LD=g++)
  cat > "$VCS_SHIM/bin/vcs" <<EOF
#!/bin/bash
# Generated by run_verdi_coverage.sh --make-shim. Restores the real VCS_HOME and
# puts the wrapped compilers ahead of the system ones for the csrc build.
export VCS_HOME="$real"
export PATH="$VCS_SHIM/cc:$ccdir:\$PATH"
# VCS prepends \$LDFLAGS to its link line, ahead of -lsnpsmalloc and friends, so
# a conda env's activation LDFLAGS reaches the simv link. Its --as-needed then
# drops libsnpsmalloc.so from DT_NEEDED -- nothing in simv references it, only
# libvcsucli.so does -- and simv dies at startup with
#   symbol lookup error: libvcsucli.so: undefined symbol: snpsReallocFunc
# --gc-sections and -z now are hostile here for the same reason. Keep only the
# rpath, which is what lets simv find the compiler's own libstdc++ at runtime.
export LDFLAGS="-Wl,--no-as-needed -Wl,-rpath,$ccroot/lib -L$ccroot/lib"
unset CFLAGS CXXFLAGS CPPFLAGS DEBUG_CFLAGS DEBUG_CXXFLAGS
exec "\$VCS_HOME/bin/vcs" "\$@"
EOF
  chmod +x "$VCS_SHIM/bin/vcs"
  echo "Done. Re-run without --make-shim."
}

if [ "${1:-}" = "--make-shim" ]; then make_shim; exit 0; fi

if [ -d "$VCS_SHIM" ]; then
  export VCS_HOME="$VCS_SHIM"
elif ! cxx_is_cxx17 "${UCIE_CXX:-g++}"; then
  echo "WARNING: the default g++ cannot compile C++17, so the svsim harness will" >&2
  echo "         fail to build under VCS. Run: $0 --make-shim" >&2
fi

# Toolchain smoke test. Every environment failure seen on this host shows up in
# a trivial VCS build too, so `--check` reproduces all of them in seconds
# instead of after a multi-minute mill run:
#   compile  -- the C++17 probe stands in for svsim's generated harness
#   compile  -- VCS's own generated rmapats.c, which modern gcc rejects
#   link+run -- simv resolving snpsReallocFunc out of libsnpsmalloc.so
run_check() {
  local d; d="$(mktemp -d)" || return 1
  printf 'module top;\n  initial begin $display("UCIE_CHECK_OK"); $finish; end\nendmodule\n' > "$d/top.sv"
  printf '#include <string_view>\nextern "C" int ucie_probe(){ std::string_view s="ok"; return (int)s.size(); }\n' > "$d/probe.cpp"
  echo "Toolchain check (VCS_HOME=$VCS_HOME)"
  if ! ( cd "$d" && "$VCS_HOME/bin/vcs" -full64 -sverilog -licqueue -o simv top.sv probe.cpp ) > "$d/vcs.log" 2>&1; then
    echo "FAIL: compile/link. Last lines of $d/vcs.log:" >&2
    grep -iE "error|undefined" "$d/vcs.log" | tail -5 >&2
    echo "Hint: '-std=c++17' unrecognized -> compiler too old; 'implicit declaration'" >&2
    echo "      in rmapats.c -> gcc >= 14 without the wrapper. Run: $0 --make-shim" >&2
    return 1
  fi
  if ! ( cd "$d" && ./simv ) > "$d/run.log" 2>&1 || ! grep -q UCIE_CHECK_OK "$d/run.log"; then
    echo "FAIL: simv built but did not run. Last lines of $d/run.log:" >&2
    tail -5 "$d/run.log" >&2
    echo "Hint: 'undefined symbol' -> a linker flag (--as-needed/--gc-sections) from" >&2
    echo "      the active conda env reached the link. Run: $0 --make-shim" >&2
    return 1
  fi
  echo "PASS: C++17 harness, VCS legacy C, and simv link+startup all work."
  rm -rf "$d"
}

if [ "${1:-}" = "--check" ]; then run_check; exit $?; fi

if [ "${1:-}" != "--merge-only" ]; then
  # --no-daemon: the mill daemon caches its startup environment, so
  # UCIE_SIM_BACKEND only reliably reaches the forked test JVM without it.
  if [ $# -gt 0 ]; then
    UCIE_SIM_BACKEND=vcs ./mill --no-daemon test.testOnly "$@" \
      || echo "WARN: some tests failed (continuing to merge coverage)"
  else
    UCIE_SIM_BACKEND=vcs ./mill --no-daemon test \
      || echo "WARN: some tests failed (continuing to merge coverage)"
  fi
fi

if [ ! -d build/chiselsim ]; then
  echo "ERROR: build/chiselsim not found — did the VCS run produce coverage?"
  exit 1
fi

# --- Stage 1: accurate per-suite merges + reports ----------------------------
# Merge unit: scenarios of a suite that elaborate the SAME design. Most suites
# use one design for every scenario and merge as a single unit, but a suite can
# drive several different DUTs (e.g. SidebandChannelRandomTest tests the
# Protocol/D2D/LogPhy channels in one class); merging those together makes urg
# silently drop every design but the first. Scenarios are therefore grouped by
# a design fingerprint (the sorted list of generated .sv names), and each group
# gets its own <Suite>__<scenario-prefix> entry.
rm -rf "$COV_DIR/suites" "$COV_DIR/reports"
mkdir -p "$COV_DIR/suites" "$COV_DIR/reports"
TOTAL=0
SUITES=()
for sd in build/chiselsim/*/; do
  s=$(basename "$sd")
  declare -A GROUP_VDBS=()
  declare -A GROUP_LABEL=()
  n=0
  while IFS= read -r v; do
    scen="${v%/workdir-vcs/simulation.vdb}"
    # fingerprint by CONTENT, not just file names: scenarios with the same module
    # set but different parametrization (e.g. 8 vs 16 lanes) are different designs,
    # and merging them makes urg drop the extra instances' data (CMR-VCINF).
    fp=$(find "$scen/primary-sources" -maxdepth 1 -name "*.sv" 2>/dev/null | sort \
         | xargs cat 2>/dev/null | md5sum | cut -d' ' -f1)
    GROUP_VDBS[$fp]="${GROUP_VDBS[$fp]:-}$SCALA_DIR/$v"$'\n'
    # remember one scenario name per group for labeling
    [ -z "${GROUP_LABEL[$fp]:-}" ] && GROUP_LABEL[$fp]=$(basename "$scen" | cut -c1-32)
    n=$((n + 1))
  done < <(find "$sd" -type d -name simulation.vdb 2>/dev/null | sort)
  [ "$n" -eq 0 ] && continue
  TOTAL=$((TOTAL + n))

  ngroups=${#GROUP_VDBS[@]}
  for fp in "${!GROUP_VDBS[@]}"; do
    # short fingerprint suffix keeps labels unique when scenario-name prefixes collide
    if [ "$ngroups" -eq 1 ]; then label="$s"; else label="${s}__${GROUP_LABEL[$fp]}-${fp:0:6}"; fi
    URG_ARGS=()
    g=0
    while IFS= read -r vv; do
      [ -n "$vv" ] && { URG_ARGS+=(-dir "$vv"); g=$((g + 1)); }
    done <<< "${GROUP_VDBS[$fp]}"
    SUITES+=("$label")
    echo "[$label] merging $g vdbs"
    # urg mishandles -dbname paths containing directories (it collapses them to
    # the parent name), so run from suites/ with a plain db name instead.
    ( cd "$COV_DIR/suites" \
      && urg -full64 "${URG_ARGS[@]}" -dbname "$label" \
          -report "$COV_DIR/reports/$label" -format both > /dev/null ) \
      || echo "WARN: urg failed for $label"
  done
  unset GROUP_VDBS GROUP_LABEL
done

if [ "$TOTAL" -eq 0 ]; then
  echo "ERROR: no simulation.vdb found under build/chiselsim — did the VCS run produce coverage?"
  exit 1
fi

# --- Combined per-module summary across all suites ---------------------------
{
  echo "Per-suite module coverage (accurate; generated $(date '+%Y-%m-%d %H:%M'))"
  echo "(each section carries its own column header; ASSERT column appears only where assertions exist)"
  for s in "${SUITES[@]}"; do
    echo
    echo "===== $s ====="
    # module table of the suite report, without the leading blurb
    awk '/^-{10,}/{on=1; next} on' "$COV_DIR/reports/$s/modlist.txt" 2>/dev/null
  done
} > "$COV_DIR/modules_summary.txt"

# --- Area verification map (per-subsystem tested/untested module table) ------
"$COV_DIR/area_summary.sh" > /dev/null 2>&1 \
  && echo "area map    -> $COV_DIR/area_summary.txt" \
  || echo "WARN: area_summary.sh failed"

# --- Stage 2: global merge (trend number only — see header caveat) -----------
URG_ARGS=()
while IFS= read -r v; do URG_ARGS+=(-dir "$SCALA_DIR/$v"); done \
  < <(find build/chiselsim -type d -name simulation.vdb 2>/dev/null | sort)
echo "[global] merging $TOTAL vdbs (trend number only)"
( cd "$COV_DIR" \
  && urg -full64 "${URG_ARGS[@]}" -dbname merged -report urgReport -format both > /dev/null ) \
  || echo "WARN: global urg merge failed"

echo ""
echo "Done. Outputs in $COV_DIR:"
echo "  reports/<Suite>/dashboard.txt|.html   per-suite coverage (accurate)"
echo "  modules_summary.txt                   all suites' module scores in one file"
echo "  area_summary.txt                      per-subsystem tested/untested module map"
echo "  ./view_coverage.sh <Suite>            Verdi GUI on an accurate suite DB"
echo "  ./view_coverage.sh                    Verdi GUI on the global merged DB (trend only)"
