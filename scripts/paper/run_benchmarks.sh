#!/usr/bin/env bash
# Re-run every benchmark the RSS paper reports, into one results folder.
#
#   bash scripts/paper/run_benchmarks.sh all          # everything, in order
#   bash scripts/paper/run_benchmarks.sh check        # fast sanity tests only
#   bash scripts/paper/run_benchmarks.sh complexity   # per-stage cost vs N, A, K
#   bash scripts/paper/run_benchmarks.sh scaling      # X parallel M-Blocks demos
#   bash scripts/paper/run_benchmarks.sh robustness   # success rates, perturbed starts
#   bash scripts/paper/run_benchmarks.sh plot         # figure from the complexity data
#
# Environment:
#   OUT=results/paper-...   reuse an existing results folder (default: a new one)
#   JOBS=4                  parallel runs for the robustness studies
#   DRY=1                   print the commands without running them
#
# Timing benchmarks are sensitive to other load: close other apps, keep the
# laptop on power, and do not run two timing sections at once.

set -euo pipefail
cd "$(dirname "$0")/../.."

PY=${PY:-.venv/bin/python}
JOBS=${JOBS:-4}
REV=$(git rev-parse --short HEAD 2>/dev/null || echo nogit)
DIRTY=$(git diff --quiet 2>/dev/null && echo "" || echo "-dirty")
OUT=${OUT:-results/paper-$(date +%Y%m%d)-${REV}${DIRTY}}

run() {
  echo "+ $*"
  if [[ -z "${DRY:-}" ]]; then "$@"; fi
}

record_environment() {
  mkdir -p "$OUT"
  {
    echo "date: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
    echo "git: $(git rev-parse HEAD 2>/dev/null || echo none)${DIRTY}"
    echo "machine: $(uname -mrs)"
    echo "cpu: $(sysctl -n machdep.cpu.brand_string 2>/dev/null || grep -m1 'model name' /proc/cpuinfo | cut -d: -f2)"
    echo "cores: $(sysctl -n hw.ncpu 2>/dev/null || nproc)"
    "$PY" -c "import sys, mujoco, numpy; print('python:', sys.version.split()[0]); print('mujoco:', mujoco.__version__); print('numpy:', numpy.__version__)"
  } > "$OUT/environment.txt"
  echo "results -> $OUT"
}

check() {
  # The tests that cover what the benchmarks exercise (about 3-5 minutes).
  run "$PY" -m pytest -q --no-cov \
    tests/test_backend_neutral_config.py tests/test_mujoco_wrenches.py \
    tests/test_mujoco_pool_allocation.py tests/test_world_state_incremental_adjacency.py \
    tests/test_momentum_pivot.py tests/test_mblocks_online_planning.py
}

complexity() {
  # Per-stage cost vs module count N, assembly size A and candidate pairs K.
  # Events are timed on mock, MuJoCo (delayed, the default) and MuJoCo
  # immediate, including the extra cost of the step after each event.
  run "$PY" scripts/complexity/bench_complexity.py --out "$OUT/complexity"
}

scaling() {
  local s="$OUT/scaling"
  run mkdir -p "$s"
  # Completion: does every copy finish, and how long does it take? One fresh
  # process per X; shared session, then the isolated-sessions control.
  run "$PY" examples/scenarios/mblocks_parallel_scaling.py \
    --sweep 1,2,4,8,16,32 --csv "$s/completion_shared.csv"
  run "$PY" examples/scenarios/mblocks_parallel_scaling.py \
    --sweep 1,2,4,8,16,32 --isolated --csv "$s/completion_isolated.csv"
  # Per-copy detail for the headline point.
  run "$PY" examples/scenarios/mblocks_parallel_scaling.py \
    --copies 8 --json "$s/completion_shared_x8.json"
  # Throughput: real-time factor over a fixed 5 s of simulated time.
  run "$PY" examples/scenarios/mblocks_parallel_scaling.py \
    --sweep 1,2,4,8,16,32,64 --duration 5 --csv "$s/throughput_shared.csv"
  run "$PY" examples/scenarios/mblocks_parallel_scaling.py \
    --sweep 1,2,4,8,16,32,64 --duration 5 --isolated --csv "$s/throughput_isolated.csv"
}

robustness() {
  local r="$OUT/robustness"
  run mkdir -p "$r"
  run "$PY" scripts/regression/robustness.py --runs 24 --jobs "$JOBS" \
    --out "$r/mblocks_online_lattice_large" -- \
    runtime examples/robot_packs/mblocks_3d --demo mblocks_online_lattice_large --no-viewer
  run "$PY" scripts/regression/robustness.py --runs 20 --jobs "$JOBS" \
    --out "$r/mblocks_online_lattice" -- \
    runtime examples/robot_packs/mblocks_3d --demo mblocks_online_lattice --no-viewer
  run "$PY" scripts/regression/robustness.py --runs 12 --jobs "$JOBS" \
    --out "$r/smores_physical_driver_to_snake" -- \
    runtime examples/robot_packs/smores_ep --backend mujoco \
    --demo smores_physical_driver_to_snake --model-view smores_topology --no-viewer
}

plot() {
  run "$PY" scripts/complexity/plot_complexity.py \
    "$OUT/complexity/measurements.csv" "$OUT/complexity/fig_complexity.pdf"
}

case "${1:-}" in
  check) check ;;
  complexity) record_environment; complexity ;;
  scaling) record_environment; scaling ;;
  robustness) record_environment; robustness ;;
  plot) plot ;;
  all) record_environment; check; complexity; plot; scaling; robustness ;;
  *) sed -n '2,20p' "$0"; exit 2 ;;
esac
echo "done: $OUT"
