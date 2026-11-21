---
title: Getting started
sidebar_label: Getting started
---

Use ModSim Studio to explore the bundled **Generic Cube** RobotPack, validate
its description, and save your first edit. Then watch two cubes dock and undock
in a MuJoCo simulation.

Studio has two workspaces: **Robot Pack** for inspecting and authoring packs,
and **Runtime** for configuring a simulation and watching it in the embedded
Inspector. This tutorial launches Studio once and uses its graphical controls.

## What you will learn

- Find a module's mechanical asset and connector definitions in the Builder.
- Validate a RobotPack and save an edited working copy.
- Watch docking change connections and assemblies in the Runtime Inspector.
- Distinguish physical geometry, connector semantics, and live runtime state.

## Prerequisites

Complete [Installation](../how-to/installation.md) and `modsim-robotics setup`.
Studio is included in the default installation. Keep that Python environment active.
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
**Validate → Simulation Metadata (Structural)** (`F7`). Open the bottom
**Validation** tab to read the results. Generic Cube should pass both profiles with
**0 errors · 0 warnings**.

Authoring validation checks the description and its references. Simulation
metadata validation applies stricter structural checks. Neither action starts
a simulator or checks every Inspector launch requirement. The Runtime workspace
checks the selected backend, demonstration, and view separately.

### 4. Save an edited working copy

Try a small edit in a separate workspace so that the installed starter stays
available while you experiment:

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
5. Choose **File → Save** (`Ctrl+S`), then run
   **Simulation Metadata (Structural)** (`F7`) again. The edited copy should still pass.

Use the workspace destination instead of the export dialog's suggested sibling
folder in the RobotPack library. This exercise changes a display name and keeps
the same pack ID and version; storing the copy outside the library avoids an
ambiguous `generic_cube` selection. It also leaves the installed cube unchanged.

### 5. Run the docking example

Follow [Install MuJoCo](../how-to/installation.md#install-mujoco) in the same
Python environment if it is not installed yet. If you install it while Studio
is open, restart Studio with the saved workspace copy so it uses the updated
environment:

```bash
modsim-robotics studio "~/.modsim/workspaces/practice_cube"
```

Otherwise keep the edited pack open. Switch to **Runtime** in Studio and set:

| Setting | Value |
| --- | --- |
| **Backend** | **MuJoCo** |
| **Demonstration** | **Dock undock** (`dock_undock`) |
| **Module type** | `generic_cube` |
| **Model view** | **Module Topology (module_topology_graph)**, the Generic Cube runtime topology recipe. |
| **Native 3D viewer** | Enabled |
| **Duration (s)** | `6` |
| **Playback speed** | `0.25` |

In **Advanced runtime settings**, select `front` for both the fixed and moving
connectors and disable gravity and ground. Click **Validate runtime**, correct
any reported issue, then click **Start simulation**. If prompted about unsaved
edits, choose **Save and start** to launch the current document.

The bundled cube already contains its topology recipe. For a new pack without
one, **Add topology view** creates an unsaved recipe that is saved with the
pack before starting. **Validate → Runtime Launch** (`F8`) is a shortcut to
open Runtime and check the selected launch.

The Runtime workspace shows the **Runtime Inspector**, and a companion
**MuJoCo viewer** opens to show physical motion. Both observe the same runtime
session. You can return to **Robot Pack** to inspect the description without
closing either view.

This demonstration creates two cubes and aligns their `front` connector frames
for a scripted approach, docking, and release. Gravity is disabled to focus on
the connection lifecycle. A playback speed of `0.25` requests quarter speed
so you can inspect intermediate state. The six-second duration is a
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
If the MuJoCo viewer is holding the completed scene, use **Stop** or close
that companion to release the runtime. Once **Back to setup** is enabled,
choose it, then **Start simulation** to replay. Closing Studio also closes
its companion viewer.

The topology graph shows logical connections; use the MuJoCo viewer to observe
physical positions and motion. MuJoCo computes the mechanics and physical
constraints. ModSim maintains the canonical `WorldState`, including committed
connections, events, and assembly membership. Studio generates its graph from
that state, so nearby cubes only gain an edge after a connection commits.

## Continue exploring

Return to **Robot Pack** to inspect more fields in your workspace copy. Your
display-name edit changed its saved module metadata; the installed starter is
still available separately by the name `generic_cube`. Edits made while a run
is active affect a subsequent launch after saving, rather than changing the
running simulation.

Switching **Backend** preserves your current scenario settings. Choosing a
different **Demonstration** or **Reset demonstration settings** restores its
preset values. Backend, duration, and other launch settings are session choices;
they are not written into the RobotPack when you save.

For terminal inspection, validation, mock docking, headless MuJoCo runs, and
JSON output, see the separate [CLI guide](../how-to/cli.md).
