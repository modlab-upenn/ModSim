---
title: Work with a RobotPack from the terminal
sidebar_label: CLI tasks
slug: /cli
---

Use these tasks for terminal import, structural checks, mock docking, generated
JSON, and headless physics. The primary
[Studio workflow](index.md) follows the same pack through graphical authoring
and runtime inspection in one window.
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

## Choose a launch workflow

| Entry point | Behavior |
| --- | --- |
| `modsim-robotics studio PACK` | Open the combined application. Configure and validate in **Runtime**, then click **Start simulation**. Opening the pack does not start physics. |
| `modsim-robotics runtime PACK` | Open a standalone Runtime Inspector and automatically start its configured demonstration. |
| `modsim-robotics run PACK --gui` | Use the same standalone Inspector path and demo defaults as `runtime`. |
| `modsim-robotics run PACK` | Execute the command's scene without an Inspector and report results in the terminal. |
| `modsim-robotics run PACK --backend mujoco --view` | Show only the native MuJoCo viewer for the non-Inspector scene. |

Inspector launches from Studio or the CLI require a supported runtime model-view
recipe; opening Studio for authoring does not. Headless runs, `dock`, and the
native-viewer-only `run --view` path do not require one. `--no-viewer` on an
Inspector launch suppresses its native 3D
companion; it does not make the Qt Inspector headless or remove its recipe
requirement. `run --gui` and `run --view` are separate launch choices and cannot
be combined.

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
some missing simulation metadata as warnings; the simulation profile applies
stricter structural checks. Neither command starts physics, checks Inspector
readiness, or proves the connectors can reach acceptance. In Studio these are
**Authoring Profile** (`F6`) and **Simulation Metadata (Structural)** (`F7`);
**Validate → Runtime Launch** (`F8`) checks the selected graphical launch
separately. Generic pack validation does not require a model-view recipe.

## Enter physical quantities and choose output units

Physical options accept a number followed by a compatible unit. Quote a value
containing spaces so the shell passes it as one argument:

```bash
modsim-robotics dock "~/.modsim/workspaces/workshop_cube" --backend mock --count 3 --spacing "100 mm" --dt "10 ms"
modsim-robotics runtime "~/.modsim/workspaces/workshop_cube" --backend mock --orientation "90 deg" --duration "6 s" --no-viewer
```

The second command also needs a supported runtime model-view recipe. Quantity
conversion does not relax the chosen demo's backend or connector requirements.

| Option | Equivalent alias | Unit for a bare number |
| --- | --- | --- |
| `--spacing` | `-s` | m |
| `--connector-gap` | `--gap` | m |
| `--orientation` | — | rad |
| `--approach` | `--approach-speed` | m/s |
| `--retract` | `--retract-speed` | m/s |
| `--height` | `--initial-height` | m |
| `--dt` | `--timestep` | s |
| `--duration`, `--undock-at` | — | s |
| `--publish-hz` | — | Hz |

Options apply to the commands shown by each command's `--help`. Supply a
physical option only once; conflicting aliases are rejected. Existing bare
numbers keep their SI meaning regardless of display preferences. Thus
`--spacing 0.1`, `--spacing "10 cm"`, and `--spacing "100 mm"` mean the same
physical separation.

Inspect or save the local profile:

```bash
modsim-robotics units show --supported
modsim-robotics units set --length-unit mm --angle-unit deg --time-unit ms
modsim-robotics units set --mass-unit g --torque-unit 'N*mm'
modsim-robotics units set --angular-velocity-unit rpm
modsim-robotics units set --angular-velocity-unit auto
modsim-robotics units set --reset
```

`units show` does not create files. `units set` explicitly saves
`~/.modsim/units.toml`, honoring `--home`, `MODSIM_PATH`, and `MODSIM_HOME`.
`--reset` starts from defaults; `auto` restores derived speed/acceleration
choices. Other `units set` flags cover force, inertia, stiffness, damping,
frequency, and velocity. Use `units set --help` for their exact spellings.

For one command, `--length-unit`, `--angle-unit`, and `--time-unit` override
human-readable output or the launched Inspector without saving the profile:

```bash
modsim-robotics dock generic_cube --backend mock --spacing "100 mm" --length-unit cm --angle-unit deg --time-unit ms
modsim-robotics units show --output json
```

The display options do not change numerical inputs. Simulation, saved Robot
Packs, and physical `--output json` results remain canonical SI. The JSON from
`units show` and `units set` instead describes the profile and effective unit
symbols. Raw event JSON is not rewritten by guessing the meaning of numbers. The
[units chapter](../explanation/units-and-orientation.md) defines each quantity
and the separate orientation convention.

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

## Launch the standalone Inspector

When a terminal command is useful for reproducing a graphical run, use:

```bash
modsim-robotics runtime "~/.modsim/workspaces/workshop_cube" --backend mujoco --demo dock_undock --fixed-connector front --moving-connector rear --duration 6 --speed 0.25
```

Replacing `runtime` with `run` and adding `--gui` selects the same Inspector
workflow. The named demo controls scene setup; ordinary headless `run` keeps
its own defaults, so removing `--gui` is not a promise to replay an identical
scene.

The Inspector selects a default runtime recipe unless you pass
`--model-view RECIPE_ID`. An explicit recipe must exist in the pack and enable
runtime mode, but need not be the default. Use `--backend mock` for supported
kinematic demos without MuJoCo; the native viewer is then disabled by default.
MuJoCo opens its companion viewer by default, and `--no-viewer` suppresses it.

This standalone command starts execution when the Inspector appears; it does
not open Studio's editable setup form. To configure before starting, use
`modsim-robotics studio` and the **Runtime** workspace instead.

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
modsim-robotics runtime --help
modsim-robotics backends
```

Use [Troubleshoot a RobotPack](troubleshoot-robot-pack.md) for import errors,
acceptance failures, backend availability, or a missing runtime view. Continue
with [Write a runtime experiment](write-runtime-experiment.md) to control a
scene and handle its events directly in Python.
