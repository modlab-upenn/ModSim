# AGENTS.md — ModSim Implementation Instructions v0.3

## Project goal

Implement ModSim, a Python-first backend-agnostic framework for modular robotics, with ModSim Studio as a standalone desktop PySide6/PyVistaQt application.

ModSim should let users:

1. start from CAD-generated URDF/meshes;
2. import those assets into ModSim;
3. interactively build a Robot Pack in ModSim Studio, a standalone desktop GUI built with PySide6/Qt and PyVistaQt/PyVista;
4. define hardware descriptions, joints, connectors, docking interfaces, capabilities, and constraints;
5. validate and save Robot Pack YAML;
6. generate mathematical model views;
7. run a simulation where MuJoCo handles physics and ModSim manages modular-robot state;
8. view modular-robot model views, events, and metrics in ModSim Studio Runtime Inspector using Qt panels and PyQtGraph plots.

## Core design rules

- ModSim is not a physics engine.
- URDF is an imported mechanical asset, not the canonical ModSim state.
- CAD or external tooling should generate URDF/meshes.
- Robot Pack YAML is the modular-robot semantic layer.
- Canonical state is `HardwareCatalog + CapabilityCatalog + WorldState + EventLog`.
- Graphs are generated views, not the foundation.
- All modules remain first-class even when disconnected.
- Assemblies are derived connected components.
- Cohorts are logical groups that may contain disconnected modules.
- Docking and undocking are first-class hybrid events.
- MuJoCo and Isaac Sim are backend adapters, not core dependencies.
- SMORES-like robots are examples only; keep the framework general.
- ModSim Studio is a standalone desktop app, not a web GUI for MVP.
- The primary Studio GUI stack is PySide6/Qt + PyVistaQt/PyVista + PyQtGraph.
- Do not make the primary GUI an Isaac Sim extension. Isaac integration comes later through an adapter or optional extension.
- Do not embed MuJoCo/Isaac viewer into Studio for MVP; use simulator viewer separately and Studio for semantic/model-view inspection.

## Language and style

- Use Python 3.11+ or 3.12+.
- Use type hints everywhere.
- Use Pydantic v2 for validated configuration schemas.
- Use dataclasses or Pydantic models for runtime data structures.
- Use `Enum` for lifecycle states.
- Use `Protocol` or abstract base classes for interfaces.
- Avoid unstructured dictionary-heavy core code.
- Keep `modsim-core` independent from GUI and backend packages.
- Add tests with every substantive change.

## Recommended packages

Core:

```text
pydantic
ruamel.yaml
orjson
numpy
scipy
networkx
typer
rich
platformdirs
```

Geometry/import:

```text
yourdfpy
trimesh
lxml
meshio
```

GUI / ModSim Studio:

```text
PySide6            # main desktop app and editor widgets
pyvista            # 3D mesh/model visualization
pyvistaqt          # embedded PyVista viewport inside Qt
pyqtgraph          # runtime metrics, matrices, plots
qtawesome optional # icons/theme only
pytest-qt          # GUI tests after shell stabilizes
```

Backend:

```text
mujoco optional
Isaac Sim later, inside Isaac's own Python environment
```

Dev:

```text
uv
ruff
pyright or mypy
pytest
pytest-cov
hypothesis
pre-commit
```

## First implementation order

1. Project skeleton and tooling.
2. Robot Pack schema, loader, writer, validator.
3. URDF importer and draft Robot Pack builder.
4. ModSim Studio shell using PySide6 with an embedded PyVistaQt viewport.
5. URDF/mesh visualization, link/joint/frame tree, and selection model.
6. Connector, docking-interface, joint, and hardware constraint editors.
7. ModelViewRegistry and first view previews inside Studio.
8. Runtime core and mock backend.
9. Runtime Inspector mode in Studio with PyQtGraph metrics.
10. MuJoCo backend MVP.
11. Docking/undocking with backend bridge.

## ModSim Studio implementation rules

- Keep GUI code in `modsim_studio`; never import PySide6, PyVista, PyVistaQt, PyQtGraph, VTK, MuJoCo, or Isaac from `modsim_core`.
- Treat Robot Pack YAML as the editable document model.
- Use view-model or controller classes between widgets and core schema objects.
- Make user edits explicit where practical: `AddConnector`, `UpdateJointSpec`, `SetAcceptanceRegion`, `SetConnectorCompatibility`, etc.
- The main layout should start with: left project/URDF tree, center PyVistaQt viewport, right property editor, bottom validation/YAML/event panel.
- PyVistaQt overlays should show link frames, joint axes, connector frames, docking approach directions, and acceptance regions.
- Runtime Inspector should show ModSim semantics, not replace the simulator viewer: WorldState, assemblies, connections, docking lifecycle, ModelViews, metrics, and backend status.
- Use PyQtGraph for live metrics and simple matrix/model-view panels.
- Keep Studio optional through packaging extras: `modsim[studio]`.

## Do not build first

- Do not build a full physics engine.
- Do not build CAD-to-URDF generation.
- Do not implement full Isaac Sim adapter yet.
- Do not implement a web GUI first.
- Do not implement an Isaac Sim extension as the primary UI.
- Do not build a custom CAD-to-URDF generator.
- Do not implement full behavior editor yet.
- Do not implement formal mission planning yet.
- Do not make graph the canonical state.
- Do not make the framework SMORES-specific.

## Quality bar

Every milestone should include:

```text
unit tests
clear schemas
validation errors with useful messages
sample data/fixtures
minimal CLI path
no hidden global state
```

Use this preferred Codex starting prompt:

```text
Read AGENTS.md and HANDOFF.md. Implement Milestone 0 and 1 only: Python project skeleton, pyproject.toml, package layout, CLI shell, Robot Pack schema models, YAML loader/writer, validator skeleton, and example generic Robot Pack. Add tests for schema validation and YAML round-trip. Do not implement PySide6/PyVistaQt GUI, MuJoCo, or docking yet.
```


Use this preferred Codex prompt for the first GUI milestone:

```text
Read AGENTS.md and HANDOFF.md. Implement Milestone 3 only: create the ModSim Studio PySide6 shell with dockable panels, project open/save, URDF link/joint tree, embedded PyVistaQt viewport, validation panel, and a basic connector editor. Keep GUI code in modsim-studio and do not import PySide6/PyVista from modsim-core. Add smoke tests for view models and skip rendering-heavy tests unless stable in CI.
```
