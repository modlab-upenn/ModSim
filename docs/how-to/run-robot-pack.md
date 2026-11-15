---
title: Run a RobotPack in the Runtime Inspector
sidebar_label: Run your RobotPack
---

Load the saved workshop cube into a two-module docking and release
demonstration. The Runtime Inspector shows committed connections and events;
the MuJoCo companion viewer shows the corresponding physical scene.

Complete [Validate and save](validate-and-save.md), including the default
`module_topology` recipe. Follow
[Install MuJoCo](installation.md#install-mujoco) in the same Python environment.
Both windows need a working desktop display.

## Launch the saved workspace pack

Save your edits, then close the Builder to return to its terminal or open
another terminal with the same Python environment active. The Builder and
Runtime Inspector are separate applications; launch the runtime from the
terminal.

For the default ModSim home, run:

```bash
modsim-robotics runtime "~/.modsim/workspaces/workshop_cube" --backend mujoco --demo dock_undock --fixed-connector front --moving-connector rear --duration 6 --no-gravity --no-ground --speed 0.25
```

ModSim expands `~` in the explicit pack selector. If your ModSim home is
elsewhere, replace the selector with the saved pack's absolute path. Use that
path rather than the bare name `workshop_cube`: workspace directories are
outside the installed RobotPack library.

The options make the demonstration reproducible:

| Option | Purpose |
| --- | --- |
| `--backend mujoco` | Execute the mechanics and physical constraints with MuJoCo. |
| `--demo dock_undock` | Approach, dock, and release using the scripted connector-pair scenario. |
| `--fixed-connector front --moving-connector rear` | Select module-local connector IDs explicitly. |
| `--duration 6` | Set a six-second simulated-time budget; the demo supplies a release time within it. |
| `--no-gravity --no-ground` | Isolate connector behavior without falling or floor contact. |
| `--speed 0.25` | Request quarter-speed playback so the intermediate state is easier to inspect. |

The runtime creates two instances of `workshop_cube`, stages the chosen frames,
and starts automatically. This demonstration scripts the approach and sends
docking and release requests. It is a connector lifecycle check, not an
autonomous controller or a validation of hardware locomotion.

## Observe docking and release

In **Runtime state**, look for two module nodes. They should first become
connected by an edge, then separate logically after release. The node layout
is a connectivity presentation; use the MuJoCo viewer to observe positions
and motion.

In **Event log**, inspect this sequence:

```text
DockCandidateDetected
DockCommitted
AssemblyMerged
UndockCommitted
AssemblySplit
```

An assembly is a connected group of modules, including a single disconnected
module. The expected counts through this run are:

| Stage | Modules | Active connections | Assemblies |
| --- | --- | --- | --- |
| Before docking | 2 | 0 | 2 |
| Docked | 2 | 1 | 1 |
| Released | 2 | 0 | 2 |

The edge appears only after a committed connection. Candidate detection or
close physical proximity alone does not establish it. Releasing the constraint
does not itself push bodies apart. The Inspector can stop at scenario completion
before a further retraction step produces a visible gap. Confirm release from
`UndockCommitted`, the removed edge, and the two final assemblies.

## Inspect and finish the run

Use **Pause** and **Resume** in the Inspector to stop and continue at simulation
step boundaries. Pressing **Space** in the native MuJoCo window controls the
same playback state. Pausing freezes simulated time and lifecycle processing
while leaving camera and graph interaction available.

When the scenario finishes, the **Simulation complete** banner and final
state remain available. A time limit or failure banner is a different outcome;
check the events instead of treating elapsed time as success. **Stop** ends
execution cooperatively and retains the last semantic state for inspection.
Close the Inspector to finish. To replay, run the launch command again.

Add `--no-viewer` to the same command for the Qt Inspector alone. It still
needs a display. For a fully headless execution, use the corresponding
[CLI workflow](cli.md#run-the-lifecycle-headlessly).

If the view is missing or docking fails, use
[Troubleshoot a RobotPack](troubleshoot-robot-pack.md). Recheck the saved path,
runtime/default view flags, connector IDs, axes, acceptance values, and
undocking support before widening tolerances.

Previous: [Validate and save](validate-and-save.md).
Continue with [a Python runtime experiment](write-runtime-experiment.md), or
[configure joint control](configure-joints.md) for a module with actuators.
