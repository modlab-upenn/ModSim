# Manual regression checks: lazy kinematic sampling

`check_refactor.sh` proves that simulation results are bit-identical: world
state, Runtime Inspector frames, CLI output and planner snapshots. These manual
checks cover what it cannot see: windows, rendering, live panels and
interactive controls.

## Running one demo side by side with the baseline

Every command below can be run against the pre-change code by putting the
extracted baseline first on `PYTHONPATH`. `check_refactor.sh` prints where it
put it; any run leaves it under `$OUT/baseline-src/src`.

```bash
# candidate (your working tree)
modsim runtime examples/robot_packs/mblocks_3d --demo mblocks_online_lattice

# baseline (same command, old library code)
PYTHONPATH=/tmp/modsim-regression-<time>/baseline-src/src \
  python -m modsim runtime examples/robot_packs/mblocks_3d --demo mblocks_online_lattice
```

To confirm which code a window is running, start Python the same way and
`import modsim; print(modsim.__file__)`.

## 1. Runtime Inspector with the MuJoCo companion viewer

Run each of these and tick the boxes. The final status text and event counts
should match what `check_refactor.sh` recorded in
`$OUT/new/<case>/summary.json`.

| Demo | Command |
|---|---|
| Four-cube online lattice | `modsim runtime examples/robot_packs/mblocks_3d --demo mblocks_online_lattice` |
| Six-cube online lattice | `modsim runtime examples/robot_packs/mblocks_3d --demo mblocks_online_lattice_large --speed 0.5` |
| Momentum pivot | `modsim runtime examples/robot_packs/mblocks_3d --demo mblocks_momentum_pivot` |
| Physical twelve-module line | `modsim runtime examples/robot_packs/mblocks_3d --demo mblocks_physical_twelve_module_line --speed 4` |
| SMORES differential-drive dock/undock | `modsim runtime examples/robot_packs/smores_ep --backend mujoco --demo smores_diff_drive_dock_undock --model-view smores_topology` |
| SMORES dock/undock (`run --gui`) | `modsim run --gui examples/robot_packs/smores_ep --backend mujoco --demo dock_undock --fixed-connector pan --moving-connector pan --model-view smores_topology --connector-gap 0.02 --approach 0.03 --retract 0.03 --duration 6.0 --dt 0.002 --no-gravity` |

For each one:

- [ ] Both windows open: the Qt Runtime Inspector and the MuJoCo viewer.
- [ ] Cubes and wheels move smoothly in the viewer. No module jumps, jitters at rest, or freezes while time advances.
- [ ] **Connector markers / frames** (where the view draws them) sit on the module faces and follow them through each pivot or approach. A stale-frame bug would show up as markers left behind for a frame or more.
- [ ] The topology/lattice graph adds and removes edges at the same moments the viewer shows faces latching and releasing.
- [ ] The Event log lists `DockCommitted` / `UndockCommitted` / `AssemblyMerged` / `AssemblySplit` events in a sensible order. After completion, the count matches `events` in `summary.json`.
- [ ] Telemetry panels (flywheel RPM, pivot angle, landing effort, settled error; wheel speeds for SMORES) update continuously and are non-zero during motion.
- [ ] The final status reads the same as in `summary.json` (for example *Target reached: 6 blocks, 21 verified pivots* for the six-cube lattice).
- [ ] **Pause / Resume** (the button and the Space key in the MuJoCo window): everything freezes. The view can still be rotated while paused. Resume continues without a jump.
- [ ] Stop and closing either window shut both down cleanly, with no traceback in the terminal.

## 2. Inspector without the native viewer

```bash
modsim runtime examples/robot_packs/mblocks_3d --backend mock --demo mblocks_twelve_module_staircase --no-viewer
modsim runtime examples/robot_packs/mblocks_3d --demo mblocks_online_lattice --no-viewer
```

- [ ] The semantic view and panels update without a MuJoCo window (worker-thread path).
- [ ] The mock-backend staircase completes; the lattice view shows each module's cell change.

## 3. Headless MuJoCo passive viewer (`run --view`)

```bash
modsim run examples/robot_packs/generic_cube --backend mujoco --count 2 --duration 8 --view
modsim run examples/robot_packs/smores_ep --backend mujoco --fixed-connector pan --moving-connector pan --undock-at 4 --view
```

- [ ] The modules approach, latch (they then move as one) and, in the second command, separate after about 4 s.
- [ ] The terminal summary after the window closes matches the `cli_run_*` case output in `$OUT/new/.../stdout.txt`.

This path uses passive docking, which reads every connector frame on every
step. It is the most demanding consumer of the lazily resolved frames.

## 4. Studio (pack authoring; no runtime state involved)

```bash
modsim studio examples/robot_packs/mblocks_3d
modsim studio examples/robot_packs/smores_ep
```

- [ ] The packs open, the 3D viewport renders modules and connector frames, and validation shows no new issues.

## 5. Scaling benchmark sanity

```bash
python examples/scenarios/mblocks_parallel_scaling.py --sweep 1,4,16 --duration 2 --csv /tmp/scaling.csv
```

- [ ] `sync_wall_s` is much smaller than before the change. The physics and controller columns are about the same.
- [ ] `completed` / `failed` per X match a baseline run of the same command (run it with the baseline `PYTHONPATH` shown above).

## If something looks different

1. Re-run that case on its own:
   `ONLY='^demo_mb_online_lattice$' scripts/regression/check_refactor.sh quick`
2. Open `$OUT/base/<case>/trace.txt` and `$OUT/new/<case>/trace.txt`. Each
   `=== session N step K` header marks one recorded sample, and
   `compare_runs.py` names the first line that differs.
3. Compare `$OUT/{base,new}/<case>/frames.jsonl` to see what the Inspector would
   have displayed at each publish.
