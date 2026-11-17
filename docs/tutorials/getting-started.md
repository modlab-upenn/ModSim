---
title: Getting started
sidebar_label: Getting started
---

Use ModSim Studio to explore the bundled **Generic Cube** RobotPack, validate
its description, and save your first edit. Then watch two cubes dock and undock
in a MuJoCo simulation.

Studio has two applications: the **Robot Pack Builder** for inspecting and
authoring packs, and the **Runtime Inspector** for watching simulations. This
tutorial uses their graphical controls; terminal commands launch each window.

## What you will learn

- Find a module's mechanical asset and connector definitions in the Builder.
- Validate a RobotPack and save an edited working copy.
- Watch docking change connections and assemblies in the Runtime Inspector.
- Distinguish physical geometry, connector semantics, and live runtime state.

## Prerequisites

Complete [Installation](../how-to/installation.md), including the Studio
dependencies and `modsim-robotics setup`. Keep that Python environment active.
You can add MuJoCo when you reach the simulation step below.

## Explore and simulate Generic Cube

### 1. Open Generic Cube in Studio

From the terminal with your ModSim environment active, run:

```bash
modsim-robotics studio generic_cube
```

The Builder opens with **Generic Cube Module** selected in the **Project** tree,
pack metadata in the right-hand **Inspector**, and a cube in the 3D viewport.
The pack describes one reusable module type; it does not yet contain a running
scene with several cubes.

The name `generic_cube` selects the starter from your installed RobotPack
library. The configuration locates the library, while this launch command
chooses the robot. If you have installed several versions, use
`generic_cube@0.1.0` to select this tutorial's version.

### 2. Explore the module and its connectors

Use the project tree to connect the visual model with its description:

| Select in Project | What to look for |
| --- | --- |
| **Module Types → Generic Cube** | The module's name, 1 kg mass, URDF asset, and `base_link` root link. |
| **Assets → generic_cube** | The URDF asset referenced by the module. |
| **Module Types → Generic Cube → Connectors → front** | A connector 0.05 m along the module's positive X axis. |
| **Module Types → Generic Cube → Connectors → rear** | A connector 0.05 m along negative X. Compare its orientation and docking axis with `front`. |
| **Connector Types → Fixed Face Connector** | The shared compatibility, acceptance tolerances, and fixed-connection rules used by both connectors. |

The mechanical asset describes a 0.1 m cube. Expand **Links** under
**Module Types → Generic Cube** to find its single `base_link`; the **Joints**
branch has no entries.

Use **Fit** above the viewport if you need to recenter the model. Inspect the
connector frames and axes alongside their position and orientation fields in
the Inspector.

The **URDF** supplies the mechanical description: links, geometry, and inertia.
The **RobotPack YAML** adds module identity, connectors, and docking rules.
Selecting a connector shows one placed instance; selecting its connector type
shows the rules shared by those instances.

Open **YAML Preview** in the bottom **Document** panel to see the corresponding
documents. This preview is read-only; use the Inspector to edit the pack.

### 3. Validate the pack

Choose **Validate → Authoring Profile** (`F6`), then
**Validate → Simulation Readiness** (`F7`). Open the bottom **Validation** tab to
read the results. Generic Cube should pass both profiles with
**0 errors · 0 warnings**.

Authoring validation checks the description and its references. Simulation
readiness applies stricter structural checks. Neither action starts a simulator;
the docking example will exercise the description in a runtime session.

### 4. Save an edited working copy

Try a small edit in a separate workspace so that the installed starter stays
available for the simulation:

1. Choose **File → Export As** (`Ctrl+Shift+S`).
2. In **Export to a new Robot Pack folder**, browse to the `workspaces` directory
   inside your ModSim home and enter a **new** folder name, `practice_cube`.
   With the default home, the destination is
   `~/.modsim/workspaces/practice_cube`. Browse to your actual home directory
   rather than relying on the dialog to expand `~`.
3. Confirm the export. The exported copy becomes the current project.
4. Select **Module Types → Generic Cube**, change **Name** to `Practice Cube`,
   and click **Apply module metadata**. The tree label updates, and the project
   shows **Unsaved changes**.
5. Choose **File → Save** (`Ctrl+S`), then run **Simulation Readiness** (`F7`)
   again. The edited copy should still pass.

Use the workspace destination instead of the export dialog's suggested sibling
folder in the RobotPack library. This exercise changes a display name and keeps
the same pack ID and version; storing the copy outside the library avoids an
ambiguous `generic_cube` selection. It also leaves the installed cube unchanged.

### 5. Run the docking example

Close the Builder to return to its terminal, or use another terminal with the
same Python environment active. Follow
[Install MuJoCo](../how-to/installation.md#install-mujoco) in that environment,
then return here.

Launch the Runtime Inspector using the original installed Generic Cube:

```bash
modsim-robotics runtime generic_cube --backend mujoco --demo dock_undock --fixed-connector front --moving-connector front --duration 6 --no-gravity --speed 0.25
```

The simulation starts automatically. Two windows open: Studio's **Runtime
Inspector**, showing connections and events, and a companion **MuJoCo viewer**,
showing physical motion. Both show the same runtime session. The Builder and
Runtime Inspector are separate applications; launch the Inspector with the
command above.

This demonstration creates two cubes and aligns their `front` connector frames
for a scripted approach, docking, and release. Gravity is disabled to focus on
the connection lifecycle. `--speed 0.25` slows playback to a quarter of real
time so you can inspect the intermediate state. The six-second duration is a
simulation-time budget; the example can finish earlier once release completes.

In **Runtime state**, watch a connection edge appear between the module nodes
when docking commits, then disappear when the connection is released. An
**assembly** is a group of modules joined by committed connections:

| Stage | Modules | Active connections | Assemblies |
| --- | --- | --- | --- |
| Before docking | 2 | 0 | 2 |
| Docked | 2 | 1 | 1 |
| After undocking | 2 | 0 | 2 |

Open **Event log** to find `DockCommitted` and `AssemblyMerged`, followed by
`UndockCommitted` and `AssemblySplit`. Use **Pause** and **Resume** to inspect
the intermediate state, or **Stop** to end the run early. A completed run shows
**Simulation complete** and keeps its final state available for inspection.
Run the launch command again to replay it.
Close the Inspector and its companion viewer when you have finished.

The topology graph shows logical connections; use the MuJoCo viewer to observe
physical positions and motion. MuJoCo computes the mechanics and physical
constraints. ModSim maintains the canonical `WorldState`, including committed
connections, events, and assembly membership. Studio generates its graph from
that state, so nearby cubes only gain an edge after a connection commits.

## Continue exploring

Return to your workspace copy in the Builder to inspect more fields. Your
display-name edit changed its saved module metadata; running the example by
`generic_cube` still selects the original installed starter.

For terminal inspection, validation, mock docking, headless MuJoCo runs, and
JSON output, see the separate [CLI guide](../how-to/cli.md).
