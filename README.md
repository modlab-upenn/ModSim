# ModSim

ModSim is a Python-first, backend-agnostic framework for modular and multi-robot
systems. ModSim owns Robot Pack definitions and modular-robot semantics;
simulator adapters such as MuJoCo and Isaac Sim will own physics.

The current implementation provides the Robot Pack foundation and the first
native ModSim Studio vertical slice:

- strict, typed schemas for assets, modules, joints, connectors, capabilities,
  and backend mappings;
- split-YAML loading and deterministic, atomic export;
- actionable structural, filesystem, completeness, and cross-reference
  validation;
- typed URDF import and self-contained draft Robot Pack generation;
- a standalone PySide6/PyVistaQt Studio for URDF visualization and Robot Pack
  authoring;
- multi-module scene composition, canonical world state, and a derived assembly
  index;
- executable docking and undocking: compatibility, acceptance regions, guards,
  two-phase commit against a backend, and an append-only event log;
- a backend adapter contract and a dependency-free kinematic mock backend;
- modular-robot runtime metrics;
- the `modsim` command-line interface;
- a simulator-neutral generic Robot Pack and test suite.

The current version does **not** provide `modsim run`, a MuJoCo or Isaac Sim
adapter, or any real physics. Docking executes end to end against the mock
backend only. The `simulation` validation profile is a stricter structural
readiness check; it does not launch a simulator.

## Install and use the package

ModSim requires Python 3.11 or newer. The distribution/install name is
`modsim-robotics`; the Python import and console command are both `modsim`.

From a source checkout:

```bash
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install .
modsim --version
```

This installs ModSim and all Phase 1 runtime dependencies into the virtual
environment. The GUI is intentionally optional. Install Studio and development
dependencies with:

```bash
python -m pip install -e ".[studio,dev]"
modsim studio examples/robot_packs/generic_cube
pytest
ruff check .
ruff format --check .
pyright
```

With `uv`, the equivalent locked development setup is:

```bash
uv sync --locked --extra dev --extra studio
uv run --no-sync modsim --help
uv run --no-sync modsim studio examples/robot_packs/generic_cube
uv run --no-sync pytest
```

An installed package can launch the same application with either
`modsim studio` or `modsim-studio`.

Before a public package release, install a locally built wheel in a clean
virtual environment:

```bash
python -m pip install dist/modsim_robotics-0.1.0-py3-none-any.whl
modsim pack validate /absolute/path/to/a/robot_pack
```

Do not substitute `pip install modsim`; that is not this distribution.

## Use from a source checkout

The example and repository documentation paths below exist in a source checkout,
not in an installed wheel:

```bash
modsim pack inspect examples/robot_packs/generic_cube
modsim pack validate examples/robot_packs/generic_cube
modsim pack validate examples/robot_packs/generic_cube --profile simulation
modsim pack validate examples/robot_packs/generic_cube --output json
modsim dock examples/robot_packs/generic_cube --count 3
modsim studio examples/robot_packs/generic_cube
```

A Robot Pack is a self-contained directory. Its root `robot_pack.yaml` refers to
typed specification and backend-mapping documents and keyed local assets. Paths
are portable, relative to the pack, and cannot escape it.

The implemented format is documented in
`docs/robot_pack_spec.md`. `docs/AGENTS.md` and `docs/HANDOFF.md` describe the
broader architecture and future milestones.

## Create a Robot Pack from URDF

From the command line:

```bash
modsim pack init \
  --from-urdf /path/to/module.urdf \
  --out /path/to/new_robot_pack

modsim studio /path/to/new_robot_pack
```

Use one or more `--asset-root` options when a URDF uses `package://` paths that
cannot be resolved relative to the URDF:

```bash
modsim pack init \
  --from-urdf /path/to/module.urdf \
  --asset-root /path/to/robot_description_package \
  --out /path/to/new_robot_pack
```

The importer extracts links, joints, joint limits, inertial masses, primitive
geometry, and mesh references. It copies the URDF and resolved meshes into a
self-contained pack, rewrites mesh paths, generates module and URDF mapping
documents, and leaves modular connector/capability semantics for the user.

Studio can also perform this workflow through **File → Create from URDF**. Its
initial editor provides:

- project, module, link, joint, and connector trees;
- Robot Pack metadata editing through the Properties panel;
- visual/collision geometry toggles;
- link frames, joint axes, connector frames, docking axes, and approach axes;
- selection from the tree or 3D meshes;
- numeric connector placement and joint metadata editing;
- live authoring/simulation validation and a read-only canonical YAML preview;
- atomic Save and non-overwriting Export As.

**Save** writes the edited Robot Pack metadata and semantic specifications back
to the canonical split-YAML documents. The YAML preview is for inspection only;
make changes through the Properties panel and other editors.

URDF remains the source of geometry, mesh references, and kinematics. For
example, changing a cube's shape or dimensions requires editing its URDF and
reimporting it; Robot Pack metadata does not replace that mechanical source.
Connector placement currently uses numeric pose fields. Studio does not yet
provide a translation or rotation gizmo.

Each Studio launch rewrites a local session log. With a pack supplied, the
default sidecar is
`<pack-parent>/.modsim/logs/<pack-folder-name>/studio.log`; without a pack, it
is `<current-working-directory>/.modsim/logs/studio.log`. Set
`MODSIM_STUDIO_LOG` to override the path. The **Session Log** tab mirrors the
same file while Studio is running.

See `docs/studio.md` for the exact workflow and current limitations. Do not add
SMORES CAD, URDF, or mesh assets to this repository until their redistribution
terms are confirmed.

## Python API

```python
from pathlib import Path

from modsim.robot_packs import (
    RobotPackLoader,
    RobotPackValidator,
    RobotPackWriter,
    ValidationProfile,
)

loaded = RobotPackLoader().load(Path("/path/to/robot_pack"))
report = RobotPackValidator().validate(
    loaded,
    profile=ValidationProfile.AUTHORING,
)
report.raise_for_errors()

# Export atomically to a new path.
exported = RobotPackWriter().write(
    loaded,
    Path("/path/to/new_robot_pack_export"),
)

# An explicit editor-style Save stages and validates a complete replacement,
# then atomically swaps the existing pack directory.
saved = RobotPackWriter().update(loaded)
```

## Run a docking session

The quickest check is the `modsim dock` command, which runs a mock docking
session and reports whether a pack's connectors actually mate:

```bash
modsim dock examples/robot_packs/generic_cube --count 3
modsim dock examples/robot_packs/generic_cube --count 3 --undock
modsim dock examples/robot_packs/generic_cube --output json
```

It places `--count` modules `--spacing` metres apart, runs `--steps` steps of
`--dt` seconds, and prints the event log, the resulting assemblies, and the
runtime metrics. `--latch` (the default) issues a dock command for every pair
whose geometry already satisfies acceptance; `--no-latch` leaves docking to each
connector type's own `auto_latch` policy. When nothing docks, the command
reports which criterion or guard blocked each pair.

This exercises the real semantic pipeline against the kinematic mock backend. It
runs no physics, so it answers "is this pack authored such that these connectors
would latch?" rather than "will this robot work?".

The same thing from Python, which is what to build on:

```python
from pathlib import Path

from modsim import MockBackendAdapter, RobotPackLoader, RuntimeSession, SceneSpec
from modsim.core.ids import ConnectorInstanceId

pack = RobotPackLoader().load(Path("examples/robot_packs/generic_cube")).pack
scene = SceneSpec.grid("generic_cube", 3, spacing_m=0.1)
session = RuntimeSession.create(pack, scene, MockBackendAdapter())

session.request_dock(
    ConnectorInstanceId("generic_cube_0/front"),
    ConnectorInstanceId("generic_cube_1/rear"),
)
for event in session.step(0.01):
    print(event.kind, event.sequence)

print(session.world.assemblies.assemblies)
print(session.metrics().as_dict())
```

ModSim decides whether two connectors may mate and what it means when they do;
the backend decides only whether the requested physical constraint exists. A
logical connection is created only after the backend confirms the constraint, so
semantic state can never claim a connection that no physics engine is
enforcing.

Connector types can declare a `docking_policy` controlling auto-latching,
measured or nominal alignment, redock cooldown, and break force. See
`docs/docking_semantics.md` for the full pipeline, the acceptance criteria, and
the notes a MuJoCo adapter should follow.

## Design boundaries

- URDF and meshes are imported mechanical assets, not canonical ModSim state.
- Robot Pack YAML is the modular-robot semantic layer.
- Graphs and matrices will be generated views, not canonical state.
- Disconnected modules remain first-class entities.
- Docking and undocking are explicit lifecycle events, and the event log is the
  only path that mutates world state.
- Assembly identity is derived from membership, not from a counter, so the same
  configuration produces the same identifiers across runs and replays.
- GUI and simulator dependencies stay outside the core package. Studio is a
  separate optional package and consumes the same public core API that future
  web or native C++ clients can target.

The package is pre-release. A software license, publication channel, public
project URLs, and redistribution policy for robot assets must be chosen before
public distribution.
