---
title: Configure joint control in a RobotPack
sidebar_label: Configure joint control
---

Use this optional task when your URDF contains actuated joints, such as wheels
or a module articulation. The workshop cube has no joints, so its docking
workflow does not need this step.

The URDF supplies mechanical joint names, types, parent/child links, and axes.
RobotPack metadata declares which command modes a joint accepts and the limits
ModSim checks before forwarding a command to a backend.

## Inspect the imported joint

Import your model using [Import a URDF](import-urdf.md), or open its saved pack
by explicit path. In the Builder, expand **Module Types → your module → Joints**
and select the joint you intend to command.

The Inspector shows **Joint ID**, **Source joint**, **Type**, and
**Parent → child**. The source name identifies the mechanical joint in the
URDF; the joint ID is its semantic name in the pack. The importer normalizes
IDs, so read the displayed ID before using it in a script. An imported URDF
joint named `wheel_joint` will normally retain `wheel_joint` as its ID when
that name is valid and unique.

Check the link association and axis in the mechanical source. Studio does not
convert a fixed URDF joint into a wheel by adding a control-mode name. Correct
mechanical changes in the URDF and deliberately import/update the model.

## Declare an effort-controlled joint

MuJoCo currently executes scalar **effort** commands. For a continuous wheel
joint, an example configuration is:

| Inspector field | Example value |
| --- | --- |
| **Control modes** | `effort` |
| **Max velocity (rad/s)** | `12` |
| **Max effort (Nm)** | `0.8` |

These are teaching values; choose appropriate limits for your model. Leave
position bounds absent for a continuous joint. Revolute joints expose
**Lower position (rad)** and **Upper position (rad)**; prismatic joints use
metres, metres per second, and newtons instead of angular units.

Click **Apply joint metadata**. The importer may already have copied positive
velocity and effort limits from URDF, but it does not infer that a joint should
be driven. Verify the values rather than assuming a declared control mode was
created automatically.

An effort-controlled angular joint needs a finite, positive maximum effort
in newton-metres. For a linear joint, it needs maximum effort in newtons.
ModSim validates each submitted command against the declared mode and bound.
The MuJoCo adapter creates effort actuators for supported declared joints when
it loads the pack.

`position` and `velocity` are schema-level modes, but MuJoCo does not implement
those command modes in this release. Writing them into **Control modes** does
not add a position or velocity controller. Similarly, a maximum-velocity
metadata value does not turn an effort command into a speed controller.

## Validate, save, and reload

Run **Authoring Profile** (`F6`) and **Simulation Metadata (Structural)** (`F7`). Fix missing
mode-specific limit metadata, click **Apply joint metadata**, and rerun the
profile. Then **Save** (`Ctrl+S`). Follow
[Validate and save](validate-and-save.md) for the complete handoff.

A backend reads the declaration when constructing its model. Stop an existing
run and create a new runtime after changing control metadata; an already
loaded backend does not acquire a new actuator from an unsaved Builder edit.

## Send commands from your experiment

The current Runtime Inspector does not provide manual joint-control widgets.
Use a Python runtime to submit `JointCommand` values through
`RuntimeSession.set_joint_commands()`. Identify the target with its runtime
ID, `<module instance>/<joint ID>`, rather than a backend-native actuator name.

Commands persist until replaced or cleared. Use `clear_joint_commands()` to
clear selected targets or all targets, then release the backend in a
`finally` block. MuJoCo clears effort targets to zero; it does not reset joint
position or velocity. The mock backend does not implement joint commands.

Continue with [Write a runtime experiment](write-runtime-experiment.md) for
scene creation and cleanup, and use the
[Python API reference](../reference/python-api.mdx) for command signatures and
the errors raised for unsupported modes or invalid targets.

Return to [Author connectors](author-connectors.md) or the
[workflow overview](index.md).
