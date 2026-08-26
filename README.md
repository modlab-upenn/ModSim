# ModSim

ModSim is a Python-first, backend-agnostic framework for modular and multi-robot
systems. Robot Packs describe hardware and docking semantics; ModSim owns the
canonical world state and event log; backend adapters own physics.

The pre-alpha project currently supports eight core workflows:

- import a local URDF into a self-contained Robot Pack;
- author and validate Robot Packs with strict split-YAML schemas;
- edit packs in the PySide6/PyVista Studio application;
- run docking and undocking through a backend-neutral runtime;
- simulate fixed connections and bounded joint-effort commands with MuJoCo;
- run physical SMORES-EP differential-drive docking and seven-module
  reconfiguration on a ground plane;
- generate immutable model-view snapshots; and
- inspect live topology, events, status, and metrics in the Runtime Inspector.

Isaac Sim integration, actuator/transmission catalogs, autonomous
reconfiguration planning, and non-fixed MuJoCo connections are not yet
implemented. The `simulation` profile checks structural readiness; it does not
launch a simulator.

## Install

ModSim requires Python 3.11 or newer. From a source checkout, install the full
development environment with `uv`:

```bash
uv sync --locked --extra dev --extra studio --extra mujoco --python 3.12
uv run --no-sync modsim --version
```

Or use a virtual environment and pip:

```bash
python3.12 -m venv .venv
.venv/bin/python -m pip install -e ".[studio,mujoco,dev]"
```

The extras are independent: `studio` adds the desktop applications, `mujoco`
adds physics and native-viewer support, and `dev` adds repository tooling. The
base install contains the Robot Pack, model-view, mock-backend, and CLI APIs.

## Robot Packs and CLI

Inspect the minimal example and verify that it is structurally ready to run:

```bash
modsim pack inspect examples/robot_packs/generic_cube
modsim pack validate examples/robot_packs/generic_cube --profile simulation
```

Create a new draft from a concrete URDF (Xacro must first be expanded):

```bash
modsim pack init --from-urdf path/to/module.urdf --out path/to/new_pack
```

Use repeated `--asset-root` options when mesh references cannot be resolved
relative to the URDF. The importer copies resolved local assets and rewrites
their paths so the resulting pack is self-contained.

Other useful command surfaces are:

```bash
modsim views examples/robot_packs/generic_cube
modsim dock examples/robot_packs/generic_cube --count 3
modsim run examples/robot_packs/generic_cube --backend mujoco
modsim backends
```

`dock` exercises connector semantics with the mock backend by default. `run`
executes a scripted lifecycle without a GUI unless `--view` is supplied.

## Studio

Install the `studio` extra, then open a pack in the Robot Pack Builder:

```bash
modsim studio examples/robot_packs/generic_cube
```

Studio supports URDF visualization, semantic property editing, validation,
atomic Save, non-overwriting Export As, and canonical YAML preview. URDF
remains the source of mechanical geometry and kinematics; Robot Pack YAML adds
modular-robot semantics.

See [Studio](docs/studio.md) for the authoring workflow and current UI limits.

## Runtime Inspector

Install both the `studio` and `mujoco` extras, then run the live two-module
docking demonstration:

```bash
modsim runtime examples/robot_packs/generic_cube \
  --fixed-connector front \
  --moving-connector front \
  --duration 4
```

The Runtime Inspector displays immutable topology and event snapshots while a
single authoritative process owns the runtime. With MuJoCo, a separate native
3D viewer opens by default; `--no-viewer` suppresses it while retaining the Qt
inspector. Use `modsim run` for a fully non-GUI execution.

The larger `examples/robot_packs/smores_ep` pack includes `dock`,
`dock_undock`, `smores_diff_drive_dock_undock`,
`smores_driver_to_snake`, and `smores_physical_driver_to_snake`. The physical
demonstrations use measured joint feedback and bounded MuJoCo effort actuators;
their platform-specific controller parameters and routes live under
`examples/scenarios/`.

See [Runtime Inspector](docs/runtime_inspector.md) for the execution boundary,
controls, and named demonstrations.

## Reproducible SMORES-EP examples

Run these commands from the repository root after installing the `studio` and
`mujoco` extras. If the virtual environment is not activated, replace `modsim`
with `uv run --no-sync modsim` or `.venv/bin/modsim`.

Inspect the pack, then apply both validation profiles:

```bash
modsim pack inspect examples/robot_packs/smores_ep
modsim pack validate examples/robot_packs/smores_ep
modsim pack validate \
  examples/robot_packs/smores_ep \
  --profile simulation
```

Run a two-module docking demonstration using the `pan` face on each module:

```bash
modsim runtime examples/robot_packs/smores_ep \
  --backend mujoco \
  --demo dock \
  --fixed-connector pan \
  --moving-connector pan \
  --model-view smores_topology \
  --connector-gap 0.02 \
  --approach 0.03 \
  --duration 4.0 \
  --dt 0.002 \
  --no-gravity
```

Run the physical two-module demonstration first when evaluating SMORES
locomotion and docking:

```bash
modsim runtime examples/robot_packs/smores_ep \
  --backend mujoco \
  --demo smores_diff_drive_dock_undock \
  --model-view smores_topology
```

That command supplies the physics defaults: MuJoCo, gravity, the ground plane,
a 5 cm initial root height, a 2 cm connector gap, a 2 ms solver step, and a
12-second run. The moving module drives its `pan`/TOP face into the target's
rear `bottom` face through the left/right wheel joints. The graph follows
`0 → 1 → 0` edges while the event log records candidate, dock, merge, undock,
and split events. After an 80 ms release delay, the moving module reverses away
under wheel effort. Both windows retain the final state until Stop or close.
The generic connector acceptance region remains 6 mm, but this physical
controller continues driving until the measured face-frame separation is at
most 1 mm before creating the ideal weld, avoiding a visibly floating latch.

The pack-local MJCF uses Fusion-derived 40 mm tire radius and 67.2 mm track
geometry. The published wheel-speed cap is 90°/s; the controller clips to that
limit. Tire friction, actuator effort limits and gains, joint damping/armature,
and the small rear support skid are explicitly provisional simulation
parameters, not measured hardware constants. Locomotion and contact are
dynamic, but EP-face magnetic attraction is not yet modeled: once measured
acceptance and the 1 mm near-contact gate succeed, the latch is represented by
an ideal fixed weld. Keep `--dt` at or below 0.005 s for this tuned model. See the
[SMORES-EP project](https://www.modlabupenn.org/smores-ep/),
the [assembly controller paper](https://www.modlabupenn.org/wp-content/uploads/2022/03/liu_smores_assembly_2020.pdf),
and the [EP-face characterization](https://www.modlabupenn.org/wp-content/uploads/tosun2016epface.pdf)
for the hardware/control basis.

This first dynamics slice starts the two modules in a deterministic, already
heading-aligned approach. It demonstrates differential-drive actuation and
contact, not yet robust navigation or recovery from arbitrary lateral/yaw
errors.

Run the older kinematic pair through docking, undocking, and visible
retraction:

```bash
modsim runtime examples/robot_packs/smores_ep \
  --backend mujoco \
  --demo dock_undock \
  --fixed-connector pan \
  --moving-connector pan \
  --model-view smores_topology \
  --connector-gap 0.02 \
  --approach 0.03 \
  --retract 0.03 \
  --duration 6.0 \
  --dt 0.002 \
  --no-gravity
```

Finally, run the seven-module Driver-to-Snake demonstration:

```bash
modsim runtime examples/robot_packs/smores_ep \
  --backend mujoco \
  --demo smores_driver_to_snake \
  --model-view smores_topology \
  --duration 14.0 \
  --dt 0.002 \
  --connector-gap 0.02 \
  --approach 0.03 \
  --no-gravity
```

Do not add `--ground` to Driver-to-Snake. It begins with seven nodes and six
connections, performs four `6 → 5 → 6` edge transitions, and ends in the chain
`module_1–module_3–module_2–module_4–module_5–module_6–module_7`. See the
[Runtime Inspector examples](docs/runtime_inspector.md#reproducible-smores-ep-workflows)
for expected events, metrics, connector substitutions, viewer behavior, and a
fully headless two-module alternative.

Run the corresponding wheel-driven physics realization with gravity and the
ground enabled by its demo defaults:

```bash
modsim runtime examples/robot_packs/smores_ep \
  --backend mujoco \
  --demo smores_physical_driver_to_snake \
  --model-view smores_topology \
  --speed 4
```

The initial Driver tree is placed upright once at simulation time zero. Each
published connector replacement then releases the existing weld, drives either
one module or its connected three-module component through an authored route,
and commits the replacement only after measured connector acceptance. No root
pose, root velocity, or external wrench is injected after initialization. The
viewer therefore shows wheel/contact dynamics throughout all four
`6 → 5 → 6` graph transitions.

The command reserves 210 simulated seconds. `--speed 4` requests four simulated
seconds per wall-clock second without changing the physics timestep, motor
limits, controller targets, or event timestamps. The tuned reference run still
completes at about 177 simulated seconds—nominally about 44 seconds of wall time
at 4×—and the full display budget is nominally 52.5 seconds. Omit `--speed` for
real-time pacing, or choose any positive factor such as `2`, `3`, or `0.5` for
slow motion. Pacing is best-effort when the CPU/GPU cannot keep up. Both windows
hold the final chain until Stop or close.

This is a ModSim physics realization of the topology plan in Liu, Whitzer, and
Yim's [2019 Driver-to-Snake example](https://www.modlabupenn.org/wp-content/uploads/2019/08/chao_smores_reconfiguration_2019.pdf),
not a reproduction of published hardware trajectories or an autonomous path
planner. The collision-clearance routes, contact parameters, control gains,
and ideal post-capture weld are provisional simulation choices.

## Python API

```python
from pathlib import Path

from modsim import MockBackendAdapter, RobotPackLoader, RuntimeSession, SceneSpec
from modsim.robot_packs import RobotPackValidator, ValidationProfile

loaded = RobotPackLoader().load(Path("examples/robot_packs/generic_cube"))
report = RobotPackValidator().validate(
    loaded,
    profile=ValidationProfile.SIMULATION,
)
report.raise_for_errors()

scene = SceneSpec.grid("generic_cube", count=2, spacing_m=0.1)
session = RuntimeSession.create(loaded.pack, scene, MockBackendAdapter())
session.step(0.01)
```

The backend contract ensures that logical connections are committed only after
the backend confirms their physical constraint. Generated model views remain
derived snapshots rather than canonical state.

## Documentation

- [Robot Pack format](docs/robot_pack_spec.md) — normative split-YAML schema
- [Docking semantics](docs/docking_semantics.md) — acceptance and lifecycle
- [Backend adapters](docs/backends.md) — runtime/backend responsibilities
- [Model views](docs/model_views.md) — recipes and immutable generated views
- [Runtime Inspector](docs/runtime_inspector.md) — live visualization contract
- [Studio](docs/studio.md) — desktop authoring workflow
- [Archived design history](docs/archive/) — dated roadmaps and handoff records

## Development checks

```bash
uv lock --check
uv run --no-sync ruff check .
uv run --no-sync ruff format --check .
uv run --no-sync pyright
uv run --no-sync pytest
git diff --check
```

GUI tests require a display or Xvfb. MuJoCo tests require the `mujoco` extra.
