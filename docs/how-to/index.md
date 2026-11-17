---
title: Build and run your own RobotPack
sidebar_label: How-to guides
---

Use these guides to start with a module's URDF and build a RobotPack, author its docking
interfaces in Studio, and run a two-module simulation. The example is a
standalone 0.1 m cube named `workshop_cube`; you can create it with an installed
ModSim package and a text editor.

If you are opening ModSim for the first time, complete
[Installation](installation.md) and [Getting started](../tutorials/getting-started.md)
first.

## The five ideas you need to work with ModSim

ModSim adds a few concepts that a physics engine does not provide on its own.
The guides below build on these five ideas:

| | |
| --- | --- |
| **Module** | One physical unit of your robot. A system contains instances of one or more module types. |
| **Connector** | A spot on a module where it can attach to another module: a magnetic face, a latch, a hinge port, etc. |
| **Connector type** | The shared rules for a kind of connector: what it can mate with, how precisely two must line up, and what joint forms when they do. |
| **RobotPack** | A folder holding your URDF plus a few YAML files that describe your modules and connectors. It's the only thing ModSim needs from you. |
| **Assembly** | A group of modules that are currently connected. A module on its own counts as an assembly of one. |

ModSim decides **whether two connectors should dock** and tracks their
connections and assemblies. The **backend** handles motion and physical
constraints: `mock` provides deterministic kinematic checks without physical
dynamics, while `mujoco` simulates rigid-body physics and contacts. A connection
is committed only after the backend confirms that its constraint was created. Support for IssacSim is coming soon.

## Follow the authoring workflow

| Task | Result |
| --- | --- |
| [Prepare a URDF](prepare-urdf.md) | A local mechanical model with geometry, collision shape, mass, and inertia. |
| [Import the model](import-urdf.md) | A self-contained draft RobotPack open in the Builder. |
| [Author connectors](author-connectors.md) | One reusable fixed connector type and two placed interfaces. |
| [Validate and save](validate-and-save.md) | A saved, structurally valid pack with a default runtime view. |
| [Run the pack](run-robot-pack.md) | A docking and undocking demonstration in the Runtime Inspector and MuJoCo viewer. |


## Keep source, workspace, and library separate

Save your source `workshop_cube.urdf` in an working folder. Import it into `<ModSim home>/workspaces/workshop_cube`, a **new** destination. With the
default home, this is `~/.modsim/workspaces/workshop_cube`.

The configured RobotPack library contains installed packs such as
`generic_cube`. A workspace pack is opened by its explicit path. Naming its
folder `workshop_cube` does not register it in the library, and copying a folder
does not change the pack ID inside it.

The commands in these guides use the default home. If setup prepared another
home, substitute its actual absolute path. In file dialogs, browse to that
directory rather than relying on `~` or an environment-variable name to expand.

## Extend the workflow

- [Configure joint control](configure-joints.md) for a model with actuated joints.
- [Write a runtime experiment](write-runtime-experiment.md) to create your own scene
  and react to docking events in Python.
- [Use the CLI](cli.md) for terminal import, validation, mock checks, and headless runs.
- [Troubleshoot a pack](troubleshoot-robot-pack.md) when import, validation, or docking
  does not produce the expected result.

For the reasoning behind the workflow, continue with the
[architecture explanations](../explanation/index.md).
