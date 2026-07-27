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
- the `modsim` command-line interface;
- a simulator-neutral generic Robot Pack and test suite.

The current version does **not** provide `modsim run`, docking execution,
MuJoCo, Isaac Sim, or a physics runtime. The `simulation` validation profile is
a stricter structural readiness check; it does not launch a simulator.

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

## Design boundaries

- URDF and meshes are imported mechanical assets, not canonical ModSim state.
- Robot Pack YAML is the modular-robot semantic layer.
- Graphs and matrices will be generated views, not canonical state.
- Disconnected modules remain first-class entities.
- Docking and undocking will be explicit lifecycle events.
- GUI and simulator dependencies stay outside the core package. Studio is a
  separate optional package and consumes the same public core API that future
  web or native C++ clients can target.

The package is pre-release. A software license, publication channel, public
project URLs, and redistribution policy for robot assets must be chosen before
public distribution.
