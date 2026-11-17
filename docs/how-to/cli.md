---
title: Work with a RobotPack from the terminal
sidebar_label: CLI tasks
slug: /cli
---

Use these tasks for terminal import, structural checks, mock docking, generated
JSON, and headless physics. The parallel
[Studio workflow](index.md) follows the same pack through graphical authoring.
Complete [Installation](installation.md) and keep its Python environment active.

Commands below use the workshop pack at the default ModSim home. Substitute
its actual absolute path if your home is elsewhere. To check the installed
starter instead, replace the pack selector with `generic_cube`.

## Select a pack

List the configured library's installed identities and paths:

```bash
modsim-robotics pack list
modsim-robotics pack inspect generic_cube
modsim-robotics pack inspect generic_cube@0.1.0
```

A bare ID selects an unambiguous installed pack. `ID@VERSION` requests that
version; duplicate matches still need an explicit path. `--home PATH` changes
the library home for a command, and `--pack` can supply a selector instead of
the positional argument. Pack-reading and runtime commands report missing or
ambiguous selectors. `modsim-robotics studio` can instead open a blank Builder
for importing a URDF; supplying a selector opens that pack directly.

Workspace directories are opened explicitly:

```bash
modsim-robotics pack inspect "~/.modsim/workspaces/workshop_cube"
modsim-robotics pack inspect --pack ./my_robot_pack
```

The selector resolver expands `~`. A relative path beginning with `./` bypasses
library lookup. Configuration locates a library; it does not choose a robot.

## Import a URDF without the Builder

Save the complete model from [Prepare a URDF](prepare-urdf.md), then create a
new draft destination. Replace the source path below with your file:

```bash
modsim-robotics pack init --from-urdf /absolute/path/to/workshop_cube.urdf --out "$HOME/.modsim/workspaces/workshop_cube"
```

The destination must not already exist. `--from-urdf` and `--out` are filesystem
arguments, so use expanded or absolute paths. For models with local meshes,
repeat `--asset-root /path/to/assets` for each required search root.

This imports mechanics and creates the split YAML documents; it does not infer
connectors, capabilities, or a runtime view. Continue in Studio with
[Author connectors](author-connectors.md) and [Validate and save](validate-and-save.md),
or author the same semantic documents directly if that is your intended
workflow. Do not run import again over an already-created pack.

## Validate saved files

Run both profiles against the saved directory:

```bash
modsim-robotics pack validate "~/.modsim/workspaces/workshop_cube" --profile authoring
modsim-robotics pack validate "~/.modsim/workspaces/workshop_cube" --profile simulation
```

After completing the workshop authoring steps, expect:

```text
VALID workshop_cube@0.1.0 (authoring structural validation): 0 error(s), 0 warning(s), 0 info message(s)
VALID workshop_cube@0.1.0 (simulation structural validation): 0 error(s), 0 warning(s), 0 info message(s)
```

These commands read disk rather than pending Builder edits. Authoring permits
some missing simulation metadata as warnings; simulation readiness applies
stricter structural checks. Neither command starts physics or proves the
connectors can reach acceptance.

## Check fixed docking with the mock backend

Place three cube centers one module width apart:

```bash
modsim-robotics dock "~/.modsim/workspaces/workshop_cube" --backend mock --count 3 --spacing 0.1
modsim-robotics dock "~/.modsim/workspaces/workshop_cube" --backend mock --count 3 --spacing 0.1 --undock
```

The first run should commit two front-to-rear connections and produce one
assembly. The second creates a fresh session, docks the same pairs, then
releases both connections, ending with three assemblies and zero active
connections. Look for `DockCommitted`/`AssemblyMerged` followed by
`UndockCommitted`/`AssemblySplit`.

`dock` requests latching for geometrically accepted pairs by default. Add
`--no-latch` to observe only automatic latching; the workshop type's omitted
policy defaults to explicit requests, so that variation will not dock it.
The mock is kinematic: these checks do not simulate gravity, contacts, or
articulated joint motion.

## Run the lifecycle headlessly

Install [MuJoCo](installation.md#install-mujoco), then run a scripted
two-module approach, dock, release, and retraction:

```bash
modsim-robotics run "~/.modsim/workspaces/workshop_cube" --backend mujoco --fixed-connector front --moving-connector rear --undock-at 2.5 --duration 4 --no-gravity --no-ground
```

Expect one successful dock, one successful undock, zero failures, zero final
connections, and two final assemblies. `--undock-at` sets the release time;
`--duration` is the simulated-time budget. The connector-pair options must be
supplied together and require two modules.

For the graphical demonstration with automatic `dock_undock` timing and
synchronized playback controls, follow [Run your RobotPack](run-robot-pack.md).
The headless command above requires no Qt window or display.

## Read JSON and generated views

Add `--output json` when collecting a machine-readable result:

```bash
modsim-robotics dock "~/.modsim/workspaces/workshop_cube" --backend mock --count 2 --undock --output json
modsim-robotics views "~/.modsim/workspaces/workshop_cube" --view module_topology --count 2 --output json
```

The docking result contains `connections`, `assemblies`, `events`, and
`metrics`. Historical success counts describe the whole run; the active
connection count describes its final state.

Each command creates a fresh state. The separate `views` command therefore
produces two nodes and zero edges, independently of the preceding docking run.
It derives the view from that command's scene; it does not reopen a previous
live session.

## Look up options or diagnose a result

```bash
modsim-robotics pack init --help
modsim-robotics pack validate --help
modsim-robotics dock --help
modsim-robotics run --help
modsim-robotics backends
```

Use [Troubleshoot a RobotPack](troubleshoot-robot-pack.md) for import errors,
acceptance failures, backend availability, or a missing runtime view. Continue
with [Write a runtime experiment](write-runtime-experiment.md) to control a
scene and handle its events directly in Python.
