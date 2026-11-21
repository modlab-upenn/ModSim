---
title: Run a RobotPack in the Runtime Inspector
sidebar_label: Run your RobotPack
---

Load the saved workshop cube into a two-module docking and release
demonstration. The Runtime Inspector shows committed connections and events;
the MuJoCo companion viewer shows the corresponding physical scene.

Complete [Validate and save](validate-and-save.md), including the
`module_topology` runtime recipe used by this Inspector workflow. Follow
[Install MuJoCo](installation.md#install-mujoco) in the same Python environment.
Both windows need a working desktop display.

## Configure the run in Studio

Keep the saved workshop pack open and switch from **Robot Pack** to
**Runtime**. If you closed Studio, reopen that workspace by its explicit path:

```bash
modsim-robotics studio "~/.modsim/workspaces/workshop_cube"
```

ModSim expands `~` in the explicit pack selector. If your ModSim home is
elsewhere, substitute the saved pack's absolute path. Workspace directories
are outside the installed RobotPack library, so a bare ID does not select them.

Choose these settings:

| Setting | Value and purpose |
| --- | --- |
| **Backend** | **MuJoCo** executes mechanics and physical constraints. |
| **Demonstration** | **Dock undock** (`dock_undock`) approaches, docks, and releases a connector pair. |
| **Module type** | `workshop_cube` uses the imported module definition. |
| **Model view** | **Module Topology (module_topology_graph)** selects the `module_topology` recipe that displays committed connections. |
| **Native 3D viewer** | Enabled to show the physical scene in the companion window. |
| **Duration** | `6 s` gives the demonstration a six-second simulated-time budget. |
| **Playback speed** | `0.25` requests quarter-speed playback. |

Expand **Advanced runtime settings**, select `front` as the fixed connector
and `rear` as the moving connector, and disable gravity and ground. These
settings isolate connector behavior without falling or floor contact.

Physical fields show their selected units. For example, enter `6 s` or
`6000 ms` for the same duration, and use suffixes such as `2 ms`, `10 mm`,
`90 deg`, or `20 mm/s` in the relevant advanced fields. A bare number uses
that field's displayed unit. Empty **Connector gap**, **Undock at**, and
**Retraction speed** retain the demonstration's automatic behavior.
**View → Display units** changes the profile without changing the launch's
physical values or pending valid edits. See
[Units and orientation](../explanation/units-and-orientation.md).


Click **Validate runtime** and inspect the result. This checks the actual
backend, demo settings, and selected Inspector view in addition to structural
pack validation. It does not step physics or prove the faces will mate.
A missing view can be added explicitly with **Add topology view**, then selected.
That action changes the in-memory pack and needs saving.
**Validate → Runtime Launch** (`F8`) opens this workspace and runs the same
launch check.

Click **Start simulation**. Studio repeats readiness checks and, if the pack
has unsaved edits, offers **Save and start** or **Cancel**. Execution uses the
saved pack after a second check of its files. The Runtime workspace displays
the embedded Inspector, and the MuJoCo companion opens beside Studio.

The runtime creates two instances of `workshop_cube`, stages the chosen frames,
and starts the demonstration. This scripts the approach and sends docking and
release requests. It is a connector lifecycle check, not an autonomous
controller or a validation of hardware locomotion.

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

Select a module in **Runtime state** to read its measured world XYZ and
fixed-axis roll/pitch/yaw. The runtime retains a quaternion; the readout converts
that orientation for inspection. Inspector times, tolerances, event quantities,
and planning plots follow the same profile. Lattice axes remain **cells**.
The standalone Inspector exposes its own **Display units** button.

Use **Pause** and **Resume** in the Inspector to stop and continue at simulation
step boundaries. Pressing **Space** in the native MuJoCo window controls the
same playback state. Pausing freezes simulated time and lifecycle processing
while leaving camera and graph interaction available.

When the scenario finishes, the **Simulation complete** banner and final
state remain available. A time limit or failure banner is a different outcome;
check the events instead of treating elapsed time as success. **Stop** ends
execution cooperatively and retains the last semantic state for inspection.
If the MuJoCo companion is holding the completed scene, use **Stop** or close
that companion to release its execution resources. Once **Back to setup** is
enabled, choose it to change settings or start another run. You can inspect
**Robot Pack** during execution, but saving,
exporting, opening, and creating packs are blocked until the run has stopped.
In-memory edits apply to a later run after saving. Closing Studio stops its
active execution and closes its companion viewer.

Changing **Backend** preserves your duration, playback speed, timestep,
connector selections, and other tuning. It changes the native-viewer
availability and default: enabled for MuJoCo, disabled for other backends.
Choose another **Demonstration**, or click **Reset demonstration settings**, to
restore that demonstration's preset values before tuning it. Validate again
after either change; a physics-specific demonstration still requires MuJoCo.

Launch settings belong to the current Studio session. **Save** persists RobotPack
edits, including added view recipes; it does not save duration, backend choice,
or other launch settings as a reusable preset.

Disable **Native 3D viewer** in setup for the Qt Inspector alone. It still
needs a display. The standalone `modsim-robotics runtime` launcher remains
available for repeatable terminal launches; for fully headless execution, use
the corresponding [CLI workflow](cli.md#run-the-lifecycle-headlessly).

If the view is missing or docking fails, use
[Troubleshoot a RobotPack](troubleshoot-robot-pack.md). Recheck the saved path,
selected runtime view, connector IDs, axes, acceptance values, and
undocking support before widening tolerances.

Previous: [Validate and save](validate-and-save.md).
Continue with [a Python runtime experiment](write-runtime-experiment.md), or
[configure joint control](configure-joints.md) for a module with actuators.
