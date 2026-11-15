# A/B regression suite

Checks that a change to ModSim's library code leaves behaviour **bit-for-bit
identical**. It was written for the lazy kinematic sampling change, but it works
for any refactor under `src/` that should not change results.

```bash
scripts/regression/check_refactor.sh self-test    # 1. prove the suite catches a 1e-15 change (~10 s)
scripts/regression/check_refactor.sh quick        # 2. lint, types, pytest + 33 A/B cases (~10 min)
scripts/regression/check_refactor.sh full         # 3. + long physics demos and scaling run (~1 h)
```

Each command takes an optional `BASE_REF`, the pre-change code (default `HEAD`).
With the change still uncommitted, `HEAD` is right. Once you have committed it,
pass the commit before it:

```bash
scripts/regression/check_refactor.sh quick main     # change committed on a branch off main
scripts/regression/check_refactor.sh quick HEAD~1   # change is the latest commit
```

Useful environment variables:

- `PYTHON=.venv/bin/python` selects the interpreter.
- `ONLY='^demo_smores'` runs a subset of cases.
- `SKIP_STATIC=1` skips lint, type checks and pytest.
- `OUT=dir` keeps the results somewhere specific.

## How each case runs

- **Two runs per case.** Each case runs once with the baseline's `src/` and
  `examples/` (extracted with `git archive` into the results folder) and once
  with your working tree. Both runs use the same interpreter, the same Robot
  Packs and the same arguments. Your checkout and `.git` are never modified.
- **Import check.** Each run records which `modsim` it actually imported, and the
  case is marked `ERROR` if that is not the expected tree.
- **What a case can drive:**
  - `modsim runtime …` / `modsim run --gui …` demos go through the real CLI,
    including every per-demo default. Only the Qt launch is intercepted, and the
    same Qt-free `RuntimeInspectorRunner` the GUI worker uses drives the demo.
    Inspector frames are published on a fixed simulated-time cadence.
  - Other CLI commands: `pack`, `views`, `dock`, headless `run`.
  - Example scripts.

## What is compared

| File | Contents |
|---|---|
| `trace.txt` | Every 250 steps and at shutdown: every module pose, velocity, link pose and joint state; every connector's lifecycle, frame, axes, point velocity and `resolved` flag; connections, assemblies and the full event log. Floats are written as `float.hex`. |
| `frames.jsonl` | Every Runtime Inspector frame the GUI would have displayed. |
| `summary.json` | Final scenario phase and detail, step count, metrics and connections. |
| `stdout.txt`, `exit_code.txt` | CLI and script output. JSON output is compared structurally. |
| `snapshot.json`, `result.json` | Planner snapshots and scaling-benchmark results. |

Wall-clock fields (`planning_ms` and the scaling script's timing columns) are
the only things ignored. On a difference, the report names the first differing
line of the trace, or the JSON path, for example
`$.view.nodes[1].world_orientation_wxyz[1]`.

`MANUAL_CHECKS.md` lists what still needs eyes on screen: windows, rendering,
pause/resume and Studio.

## Files

| Script | Purpose |
|---|---|
| `check_refactor.sh` | Orchestrator and case table. Add a case by adding one line. |
| `run_demo.py` | Runs one Runtime Inspector demo headlessly, with tracing. |
| `run_traced.py` | Runs a CLI command or script, with tracing. |
| `state_trace.py` | The exact world-state dump. |
| `compare_runs.py` | The A/B comparator: `compare_runs.py BASE_DIR NEW_DIR [CASE…]`. |
