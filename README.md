# ModSim

ModSim is a Python-first, backend-agnostic framework for modular and multi-robot
systems. Robot Packs describe hardware and docking semantics; ModSim owns the
canonical world state and event log; backend adapters own physics.

The pre-alpha project currently supports seven core workflows:

- import a local URDF into a self-contained Robot Pack;
- author and validate Robot Packs with strict split-YAML schemas;
- edit packs in the PySide6/PyVista Studio application;
- run docking and undocking through a backend-neutral runtime;
- simulate fixed connections with the optional MuJoCo adapter;
- generate immutable model-view snapshots; and
- inspect live topology, events, status, and metrics in the Runtime Inspector.

An experimental [SMORES spatial handoff](#experimental-3d-smores-reconfiguration)
adds support-mode planning and bounded joint actuation in MuJoCo. General
actuator APIs, arbitrary autonomous reconfiguration, Isaac Sim integration,
and non-fixed MuJoCo connections remain unimplemented. The `simulation`
profile checks structural readiness; it does not launch a simulator.

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
modsim run --gui examples/robot_packs/generic_cube \
  --fixed-connector front \
  --moving-connector front \
  --duration 4
```

The Runtime Inspector displays immutable topology and event snapshots while a
single authoritative process owns the runtime. With MuJoCo, a separate native
3D viewer opens by default; `--no-viewer` suppresses it while retaining the Qt
inspector. Use `modsim run` for a fully non-GUI execution.

The larger `examples/robot_packs/smores_ep` pack includes the `dock`,
`dock_undock`, and `smores_driver_to_snake` demonstrations. Its geometry and
connector values are example data, not certified hardware specifications. The
platform-specific Driver-to-Snake plan lives in
`examples/scenarios/smores_driver_to_snake.py`; all three demonstrations use
the same generic runtime scenario engine.

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
modsim run --gui examples/robot_packs/smores_ep \
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

Run the same pair through docking, undocking, and visible retraction:

```bash
modsim run --gui examples/robot_packs/smores_ep \
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
modsim run --gui examples/robot_packs/smores_ep \
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

## Experimental 3D SMORES reconfiguration

A five-module benchmark lifts a payload, docks it to a folded receiving
chain, transfers support, and unfolds into a **four-module vertical tower**:
`receiver → arm → upper → payload`. The helper parks separately. Two bases are
explicitly fixed to the environment; this is an experimental spatial handoff,
not a demonstration of general free-standing 3D reconfiguration.

```bash
.venv/bin/modsim run examples/robot_packs/smores_ep --demo smores_spatial_handoff --gui
.venv/bin/modsim run examples/robot_packs/smores_ep --demo smores_spatial_handoff --output json
```

The planner searches support-preserving connector changes and collision-checked
joint paths. ModSim owns the searches, feedback controller, measured world state,
and docking lifecycle. MuJoCo provides mechanical queries and executes
force-limited servos under gravity. The native viewer preserves the original
CAD materials and shows live bonds, fixed supports, the planned payload path,
legends, and completion status. Space pauses; `S` stops. The final view stays
open for inspection. The Studio inspector uses the planar demo's three themes
and shared layout: **Runtime state** shows live and target topology side by
side; **Planning** shows projected 3D paths, waypoints, measured trails, action
progress/history, and diagnostics; **Event log** shows simulation events and
planner decisions. Pause/Resume controls the same simulation as the native
window. Legends explain colors, symbols, and time navigation.

The nominal run completes in about 31 simulated seconds. Actuator settings,
collision proxies, and ideal connector welds remain provisional. Read the
[paper-style mathematical review (PDF)](docs/papers/smores_3d_reconfiguration.pdf)
([editable source](docs/papers/smores_3d_reconfiguration.md)) for equations,
proofs, pseudocode, measured plots, and open questions. The
[algorithm and ModSim implementation](docs/smores_3d_algorithm.md) covers the exact
searches, pseudocode, execution checks, and architecture. See the
[theory proposal and benchmark](docs/smores_spatial_planning.md) for the review
of Chao Liu's work, the proposed support-aware 3D formulation, the target
topology, measured results, and the assumptions still needing validation.

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
- [SMORES spatial planning](docs/smores_spatial_planning.md) — research proposal,
  elevated handoff demo, and physical limits
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
