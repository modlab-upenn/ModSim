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
| **Max velocity** | `12 rad/s` |
| **Max effort** | `0.8 N*m` |

These are teaching values; choose appropriate limits for your model. Leave
position bounds absent for a continuous joint. Revolute joints expose
**Lower position** and **Upper position**, for example `-90 deg` and `90 deg`.
Prismatic joints use length, linear velocity, and force, for example `25 mm`,
`100 mm/s`, and `5 N`. Each field displays the unit selected in
**View → Display units** and accepts an explicit compatible suffix. For a
rotating joint `60 rpm` is equivalent to `2π rad/s`; the same suffix is rejected
for a prismatic joint. Continuous joint angles retain multiple revolutions.

Click **Apply joint metadata**. The importer may already have copied positive
velocity and effort limits from URDF, but it does not infer that a joint should
be driven. Verify the values rather than assuming a declared control mode was
created automatically.

An effort-controlled angular joint needs a finite, positive maximum effort
in canonical newton-metres. For a linear joint, it needs canonical newtons.
Entering `800 N*mm` or `0.8 N*m` declares the same rotating-joint limit.
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

For a readable command value, normalize its declared quantity once:

```python
from modsim_robotics.units import joint_value_to_si

wheel_effort_nm = joint_value_to_si(
    800, "N*mm", joint_type="continuous", control_mode="effort"
)
assert wheel_effort_nm == 0.8
```

The result is an ordinary SI number for the existing `JointCommand.value`.
See [Units and orientation](../explanation/units-and-orientation.md) for the
shared quantity contract.

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
