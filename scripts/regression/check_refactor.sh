#!/usr/bin/env bash
# A/B regression suite: does the working tree behave exactly like a baseline ref?
#
# Every case runs twice with the same interpreter, examples and Robot Packs:
# once importing the baseline's src/ (extracted with `git archive`, so your
# checkout and .git are untouched) and once importing the working tree's src/.
# Outputs are compared bit-for-bit: world-state traces (every pose, velocity,
# joint, connector frame, connection and event, as float.hex), Runtime Inspector
# frames, CLI stdout/JSON, planner snapshots and exit codes. Only wall-clock
# fields are ignored.
#
# Usage:
#   scripts/regression/check_refactor.sh [quick|full|static|self-test] [BASE_REF]
#
#   quick      static checks + every case tagged quick           (~5-10 min)
#   full       static checks + every case, incl. long physics     (~45-90 min)
#   static     ruff, ruff format, pyright (3 configs), pytest only
#   self-test  proves the suite catches a 1e-15 relative change in one qpos value
#   BASE_REF   git ref holding the pre-change code. Default: where the current
#              branch left main (git merge-base HEAD main); on main itself, HEAD,
#              i.e. your uncommitted working tree is the candidate.
#
# Environment:
#   PYTHON=path    interpreter (default: .venv/bin/python, venv/bin/python, python3)
#   OUT=dir        where results go (default: $TMPDIR/modsim-regression-<time>)
#   SKIP_STATIC=1  skip lint/type/pytest in quick/full
#   ONLY=regex     run only cases whose name matches (grep -E)
set -uo pipefail

MODE="${1:-quick}"
REPO="$(git rev-parse --show-toplevel 2>/dev/null)" || { echo "Run inside the ModSim repository." >&2; exit 2; }
cd "$REPO"
if [[ -n ${2:-} ]]; then
  BASE_REF="$2"
elif git rev-parse --verify -q main >/dev/null && [[ "$(git rev-parse main)" != "$(git rev-parse HEAD)" ]]; then
  BASE_REF="$(git merge-base HEAD main)"   # the commit this branch started from
else
  BASE_REF=HEAD
fi
HERE="$REPO/scripts/regression"

if [[ -z "${PYTHON:-}" ]]; then
  for candidate in "$REPO/.venv/bin/python" "$REPO/venv/bin/python" "$(command -v python3)"; do
    if [[ -x "$candidate" ]]; then PYTHON="$candidate"; break; fi
  done
fi
TMP_ROOT="${TMPDIR:-/tmp}"
OUT="${OUT:-${TMP_ROOT%/}/modsim-regression-$(date +%Y%m%d-%H%M%S)}"
OUT="${OUT%/}"
BASE_SRC="$OUT/baseline-src"
mkdir -p "$OUT/base" "$OUT/new" "$OUT/logs"

bold() { printf '\033[1m%s\033[0m\n' "$*"; }
pass() { printf '  \033[32mPASS\033[0m  %s\n' "$*"; }
fail() { printf '  \033[31mFAIL\033[0m  %s\n' "$*"; FAILURES+=("$*"); }
FAILURES=()

# --------------------------------------------------------------------------
# Case table: tier|name|kind|arguments   (@OUT@ = this case's output dir)
#   demo   -> modsim CLI args for `runtime` / `run --gui` (run_demo.py)
#   cli    -> modsim CLI args (run_traced.py cli)
#   script -> python script + args (run_traced.py script)
# --------------------------------------------------------------------------
read -r -d '' CASES <<'EOF'
quick|cli_backends|cli|backends
quick|cli_inspect_generic|cli|pack inspect examples/robot_packs/generic_cube
quick|cli_inspect_smores|cli|pack inspect examples/robot_packs/smores_ep
quick|cli_inspect_mblocks|cli|pack inspect examples/robot_packs/mblocks_3d
quick|cli_validate_generic_sim|cli|pack validate examples/robot_packs/generic_cube --profile simulation
quick|cli_validate_smores_sim|cli|pack validate examples/robot_packs/smores_ep --profile simulation
quick|cli_validate_mblocks_sim|cli|pack validate examples/robot_packs/mblocks_3d --profile simulation
quick|cli_views_generic|cli|views examples/robot_packs/generic_cube
quick|cli_views_mblocks_lattice|cli|views examples/robot_packs/mblocks_3d --view mblocks_lattice --count 5 --spacing 0.05 --output json
quick|cli_dock_mock|cli|dock examples/robot_packs/generic_cube --count 3 --output json
quick|cli_dock_mujoco|cli|dock examples/robot_packs/generic_cube --count 3 --backend mujoco --output json
quick|cli_dock_undock_mock|cli|dock examples/robot_packs/generic_cube --count 3 --undock --output json
quick|cli_run_generic_mock|cli|run examples/robot_packs/generic_cube --backend mock --output json
quick|cli_run_generic_mujoco|cli|run examples/robot_packs/generic_cube --backend mujoco --output json
quick|cli_run_generic_mujoco_3_gravity|cli|run examples/robot_packs/generic_cube --backend mujoco --count 3 --gravity --ground --output json
quick|cli_run_smores_undock|cli|run examples/robot_packs/smores_ep --backend mujoco --fixed-connector pan --moving-connector pan --undock-at 4 --output json
quick|demo_generic_dock_mujoco|demo|runtime examples/robot_packs/generic_cube --demo dock --backend mujoco --no-viewer --fixed-connector front --moving-connector front --duration 4
quick|demo_generic_dock_mock|demo|runtime examples/robot_packs/generic_cube --demo dock --backend mock --no-viewer --fixed-connector front --moving-connector front --duration 4
quick|demo_smores_dock|demo|run --gui examples/robot_packs/smores_ep --backend mujoco --demo dock --fixed-connector pan --moving-connector pan --model-view smores_topology --connector-gap 0.02 --approach 0.03 --duration 4.0 --dt 0.002 --no-gravity --no-viewer
quick|demo_smores_dock_undock|demo|run --gui examples/robot_packs/smores_ep --backend mujoco --demo dock_undock --fixed-connector pan --moving-connector pan --model-view smores_topology --connector-gap 0.02 --approach 0.03 --retract 0.03 --duration 6.0 --dt 0.002 --no-gravity --no-viewer
quick|demo_smores_diff_drive|demo|runtime examples/robot_packs/smores_ep --backend mujoco --demo smores_diff_drive_dock_undock --model-view smores_topology --no-viewer
quick|demo_smores_driver_to_snake|demo|run --gui examples/robot_packs/smores_ep --backend mujoco --demo smores_driver_to_snake --model-view smores_topology --duration 14.0 --dt 0.002 --connector-gap 0.02 --approach 0.03 --no-gravity --no-viewer
quick|demo_mb_five_pivot_mujoco|demo|runtime examples/robot_packs/mblocks_3d --backend mujoco --demo mblocks_five_module_pivot --no-viewer
quick|demo_mb_five_pivot_mock|demo|runtime examples/robot_packs/mblocks_3d --backend mock --demo mblocks_five_module_pivot --no-viewer
quick|demo_mb_momentum_pivot|demo|runtime examples/robot_packs/mblocks_3d --demo mblocks_momentum_pivot --no-viewer
quick|demo_mb_twelve_line|demo|runtime examples/robot_packs/mblocks_3d --backend mujoco --demo mblocks_twelve_module_line --no-viewer
quick|demo_mb_staircase_mock|demo|runtime examples/robot_packs/mblocks_3d --backend mock --demo mblocks_twelve_module_staircase --no-viewer
quick|demo_mb_staircase_mujoco|demo|runtime examples/robot_packs/mblocks_3d --backend mujoco --demo mblocks_twelve_module_staircase --no-viewer
quick|demo_mb_online_lattice|demo|runtime examples/robot_packs/mblocks_3d --demo mblocks_online_lattice --no-viewer
quick|script_mb_plan_only_large|script|examples/scenarios/mblocks_online_planning.py --preset large --plan-only
quick|script_mb_online_small|script|examples/scenarios/mblocks_online_planning.py --preset small --snapshot @OUT@/snapshot.json
quick|script_scaling_shared_x2|script|examples/scenarios/mblocks_parallel_scaling.py --copies 2 --duration 3 --progress 0 --json @OUT@/result.json
quick|script_scaling_isolated_x2|script|examples/scenarios/mblocks_parallel_scaling.py --copies 2 --duration 3 --progress 0 --isolated --json @OUT@/result.json
full|demo_mb_physical_twelve_line|demo|runtime examples/robot_packs/mblocks_3d --demo mblocks_physical_twelve_module_line --no-viewer
full|demo_mb_physical_staircase|demo|runtime examples/robot_packs/mblocks_3d --backend mujoco --demo mblocks_physical_twelve_module_staircase --no-viewer
full|demo_smores_physical_d2s|demo|runtime examples/robot_packs/smores_ep --backend mujoco --demo smores_physical_driver_to_snake --model-view smores_topology --no-viewer
full|demo_smores_online_assembly|demo|runtime examples/robot_packs/smores_ep --demo smores_online_assembly --no-viewer
full|demo_smores_online_d2s|demo|runtime examples/robot_packs/smores_ep --demo smores_online_driver_to_snake --no-viewer
full|demo_mb_online_lattice_large|demo|runtime examples/robot_packs/mblocks_3d --demo mblocks_online_lattice_large --no-viewer
full|demo_mb_online_lattice_elbow|demo|runtime examples/robot_packs/mblocks_3d --demo mblocks_online_lattice_elbow --no-viewer
full|script_mb_online_large|script|examples/scenarios/mblocks_online_planning.py --preset large --snapshot @OUT@/snapshot.json
full|script_smores_online_assembly|script|examples/scenarios/smores_online_planning.py --demo smores_online_assembly --snapshot @OUT@/snapshot.json
full|script_scaling_shared_x8_full|script|examples/scenarios/mblocks_parallel_scaling.py --copies 8 --progress 0 --json @OUT@/result.json
EOF

# --------------------------------------------------------------------------
prepare_baseline() {
  bold "Baseline: $BASE_REF ($(git rev-parse --short "$BASE_REF"))   Candidate: working tree"
  rm -rf "$BASE_SRC"; mkdir -p "$BASE_SRC"
  # examples/ comes along because the runtime demos locate their plan modules
  # relative to the installed source tree (src/../examples).
  git archive "$BASE_REF" src examples | tar -x -C "$BASE_SRC" \
    || { echo "Cannot read src/ and examples/ at $BASE_REF" >&2; exit 2; }
  local changed untracked
  changed="$(git diff --stat "$BASE_REF" -- src ':(exclude)*.DS_Store' | sed '$d')"
  untracked="$(git ls-files --others --exclude-standard -- src | grep -v -e __pycache__ -e '\.DS_Store$' || true)"
  if [[ -z $changed && -z $untracked ]]; then
    echo "ERROR: src/ at $BASE_REF ($(git rev-parse --short "$BASE_REF")) is identical to your working tree," >&2
    echo "       so every case would compare the code with itself. Pass the commit from" >&2
    echo "       before the change, e.g.:  $0 $MODE main" >&2
    exit 2
  else
    echo "Library changes being tested ($BASE_REF -> working tree):"
    [[ -n $changed ]] && printf '%s\n' "$changed" | sed 's/^/  /'
    [[ -n $untracked ]] && printf '%s\n' "$untracked" | sed 's/^/     new file: /'
  fi
}

run_case() {  # side name kind args
  local side=$1 name=$2 kind=$3 args=$4 src case_dir
  case_dir="$OUT/$side/$name"
  if [[ $side == base ]]; then src="$BASE_SRC/src"; else src="$REPO/src"; fi
  args="${args//@OUT@/$case_dir}"
  mkdir -p "$case_dir"
  # shellcheck disable=SC2086  # word-splitting of the case arguments is intended
  if [[ $kind == demo ]]; then
    PYTHONPATH="$src" "$PYTHON" "$HERE/run_demo.py" --out "$case_dir" -- $args \
      >"$OUT/logs/$side-$name.log" 2>&1
  else
    PYTHONPATH="$src" "$PYTHON" "$HERE/run_traced.py" --out "$case_dir" "$kind" -- $args \
      >"$OUT/logs/$side-$name.log" 2>&1
  fi
  local status=$?
  local imported
  imported="$(cat "$case_dir/modsim_path.txt" 2>/dev/null)"
  local src_real
  src_real="$(cd "$src" && pwd -P)"  # macOS: $TMPDIR is under /var -> /private/var
  if [[ $imported != "$src"/* && $imported != "$src_real"/* ]]; then
    echo "    $side imported modsim from '${imported:-?}', expected $src (see logs/$side-$name.log)"
    return 1
  fi
  return $status
}

run_cases() {
  local tiers=$1 selected=0
  bold "Running A/B cases (results in $OUT)"
  printf '  %-36s %9s %9s  %s\n' CASE BASE NEW RESULT
  while IFS='|' read -r tier name kind args; do
    [[ -z $tier ]] && continue
    [[ $tiers == *"$tier"* ]] || continue
    if [[ -n ${ONLY:-} ]] && ! [[ $name =~ $ONLY ]]; then continue; fi
    selected=$((selected + 1))
    local ok=1
    run_case base "$name" "$kind" "$args" || ok=0
    run_case new "$name" "$kind" "$args" || ok=0
    local tb tn verdict
    tb="$(cat "$OUT/base/$name/wall_seconds.txt" 2>/dev/null || echo '?')"
    tn="$(cat "$OUT/new/$name/wall_seconds.txt" 2>/dev/null || echo '?')"
    if [[ $ok == 0 ]]; then
      verdict="ERROR"
      FAILURES+=("$name: harness error")
      for side in base new; do
        if [[ -f "$OUT/logs/$side-$name.log" ]]; then
          echo "      --- last lines of logs/$side-$name.log"
          tail -n 12 "$OUT/logs/$side-$name.log" | sed 's/^/      /'
        fi
      done
    elif report="$("$PYTHON" "$HERE/compare_runs.py" "$OUT/base" "$OUT/new" "$name" 2>&1)"; then
      verdict="identical"
      if grep -q '"raised": "' "$OUT/base/$name/summary.json" 2>/dev/null; then
        verdict="identical (both raise: $(sed -n 's/.*"raised": "\([^:]*\.\)\{0,1\}\([A-Za-z]*Error\).*/\2/p' "$OUT/base/$name/summary.json"))"
      fi
    else
      verdict="DIFFERENT"
      FAILURES+=("$name")
      printf '%s\n' "$report" | sed '1d; s/^/        /'
    fi
    printf '  %-36s %8ss %8ss  %s\n' "$name" "$tb" "$tn" "$verdict"
  done <<<"$CASES"
  [[ $selected -gt 0 ]] || echo "  (no cases selected)"
}

run_static() {
  bold "Static checks and unit tests (working tree)"
  local tool
  run_step() {  # label command...
    local label=$1; shift
    local log="$OUT/logs/static-${label// /_}.log"
    if "$@" >"$log" 2>&1; then
      pass "$label"
    else
      fail "$label (log: $log)"
      grep -E "^(FAILED|ERROR) |[0-9]+ (failed|passed|error)|^E  " "$log" | head -n 25 | sed 's/^/        /'
    fi
  }
  if "$PYTHON" -m ruff --version >/dev/null 2>&1; then
    run_step "ruff check" "$PYTHON" -m ruff check .
    run_step "ruff format --check" "$PYTHON" -m ruff format --check .
  else echo "  skip  ruff (not installed for $PYTHON)"; fi
  if tool="$(command -v pyright || true)"; [[ -n $tool ]] || "$PYTHON" -m pyright --version >/dev/null 2>&1; then
    local pyright=("$PYTHON" -m pyright); [[ -n $tool ]] && pyright=("$tool")
    run_step "pyright (strict core)" "${pyright[@]}"
    [[ -f pyright-mujoco.json ]] && run_step "pyright (mujoco)" "${pyright[@]}" --project pyright-mujoco.json
    [[ -f pyright-studio.json ]] && run_step "pyright (studio)" "${pyright[@]}" --project pyright-studio.json
  else echo "  skip  pyright (not installed)"; fi
  # Keep going past import errors in optional-dependency test modules so one
  # broken extra (e.g. Studio's pyvista/pyvistaqt) does not hide every other result.
  run_step "pytest (full suite)" "$PYTHON" -m pytest -q -p no:cacheprovider --continue-on-collection-errors
  run_step "lazy-sampling exactness tests" "$PYTHON" -m pytest -q -p no:cacheprovider tests/test_mujoco_lazy_sampling.py
}

self_test() {
  bold "Self-test: a 1e-15 relative change to one qpos entry must be caught"
  prepare_baseline >/dev/null
  local name=selftest_diff_drive args="runtime examples/robot_packs/smores_ep --backend mujoco --demo smores_diff_drive_dock_undock --model-view smores_topology --no-viewer"
  run_case base "$name" demo "$args"
  MODSIM_REGRESSION_PERTURB=1e-15 run_case new "$name" demo "$args"
  # The comparator exits 1 when it finds a difference, which is the goal here.
  "$PYTHON" "$HERE/compare_runs.py" "$OUT/base" "$OUT/new" "$name" >"$OUT/logs/selftest.txt" 2>&1
  if grep -q "^FAIL" "$OUT/logs/selftest.txt"; then
    echo "  Expected: the perturbed run must differ. The comparator reported:"
    sed '1s/^FAIL /DIFF /; s/^/    /' "$OUT/logs/selftest.txt" | head -8
    pass "perturbation detected - the comparison is sensitive at the last-bit level"
  else
    fail "perturbation NOT detected - do not trust identical results from this suite"
  fi
}

summary() {
  echo
  bold "Summary"
  if [[ ${#FAILURES[@]} -eq 0 ]]; then
    printf '  \033[32mAll checks passed.\033[0m Results: %s\n' "$OUT"
  else
    printf '  \033[31m%d problem(s):\033[0m\n' "${#FAILURES[@]}"
    printf '    - %s\n' "${FAILURES[@]}"
    echo "  Results and logs: $OUT"
  fi
  echo "  Manual visual checks: scripts/regression/MANUAL_CHECKS.md"
}

echo "Python: $PYTHON ($("$PYTHON" -c 'import sys, mujoco; print(sys.version.split()[0], "mujoco", mujoco.__version__)' 2>/dev/null || echo 'mujoco not importable'))"
case "$MODE" in
  static) run_static ;;
  quick) prepare_baseline; [[ -n ${SKIP_STATIC:-} ]] || run_static; run_cases quick ;;
  full) prepare_baseline; [[ -n ${SKIP_STATIC:-} ]] || run_static; run_cases quick,full ;;
  self-test) self_test ;;
  *) sed -n '2,32p' "$0"; exit 2 ;;
esac
summary
[[ ${#FAILURES[@]} -eq 0 ]]
