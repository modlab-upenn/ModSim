---
title: Author connector types and placements
sidebar_label: Author connectors
---

Give the imported cube two docking interfaces. A **connector type** holds the
shared mating and physical-connection rules; a **connector instance** places
one of those interfaces on a module link. Create the type first, then place
`front` and `rear` on `base_link`.

Start with the pack from [Import a URDF](import-urdf.md) open in the Builder.
The values below define a teaching example using fixed connections. They are
not measured limits or tolerances for a physical connector.

## Create the reusable type

Select **Connector Types** in Project and click **Add connector type**.
In **Add connector type**, enter:

| Field | Value |
| --- | --- |
| **Connector type ID** | `fixed_face` |
| **Name** | `Fixed Face Connector` |
| **Active connector** | Checked |
| **Gender** | `hermaphroditic` |
| **Custom metadata** | Leave empty |

Click **OK**. Studio selects the new **Fixed Face Connector** row. Its
self-compatibility entry is created automatically. The remaining semantic
sections still need values; creation does not supply a ready-to-run type.

**Active connector** describes powered hardware. It does not turn automatic
latching on. This example uses explicit docking requests from the runtime.

## Fill in the mating rules

In the selected type's Inspector, complete the following fields. Scroll down
to reach each section, then click **Apply connector type** after entering all
the values.

| Section and field | Value |
| --- | --- |
| **Identity & compatibility → Compatible with** | `fixed_face` |
| **Allowed orientations → Orientation mode** | `discrete` |
| **Allowed values (rad)** | `0, 1.5707963267948966, 3.141592653589793, 4.71238898038469` |
| **Acceptance region → Acceptance shape** | `box` |
| **Position tolerance (m)** | `0.006` |
| **Orientation tolerance (rad)** | `0.13962634015954636` |
| **Max relative velocity (m/s)** | `0.05` |
| **Physical connection → Physical constraint** | `fixed` |
| **Load limits → Max normal force (N)** | `90` |
| **Max shear force (N)** | `40` |
| **Max bending moment (Nm)** | `1.8` |
| **Docking policy → Supports undocking** | Checked |
| **Declare docking policy** | Unchecked |

The angles allow quarter-turn mating orientations. The acceptance values allow
6 mm positional error, about 8° angular error, and 0.05 m/s relative speed.
Fill all three acceptance numbers together; a partially populated acceptance
region cannot be applied.

A `fixed` connection asks MuJoCo for a rigid weld after acceptance. Its
compliance and hinge controls remain disabled. The teaching load ratings make
the declared interface complete; they are not general force limits enforced
by the runtime. Automatic overload release uses a separately declared
**Break force (N)** policy and backend force feedback.

Leaving **Declare docking policy** unchecked uses the current defaults:
automatic latching off, measured alignment, and no declared cooldown or break
force. The docking demonstration sends the required explicit request.

Click on Apply Connector Type to save the changes. 

## Place the front and rear interfaces

Expand **Module Types → Workshop Cube → Links** and select **base_link**. If
you kept the importer's display name, the module row is `workshop_cube` instead.
Click **Add connector to this link**.

In **Add connector to base_link**, enter the first column below and click
**OK**. Then select `base_link` again, reopen the dialog, and add the second.

| Field | Front interface | Rear interface |
| --- | --- | --- |
| **Connector ID** | `front` | `rear` |
| **Connector type** | `fixed_face` | `fixed_face` |
| **Position xyz (m)** | `0.05, 0, 0` | `-0.05, 0, 0` |
| **Rotation rpy (rad)** | `0, 0, 0` | `0, 0, 3.141592653589793` |
| **Docking axis** | `1, 0, 0` | `-1, 0, 0` |
| **Approach axis** | `1, 0, 0` | `-1, 0, 0` |
| **Custom fields** | Empty | Empty |

Positions and orientations are local to the parent link. The rear frame is
rotated by π about Z. Both axes are unit vectors expressed in the
**parent-link frame**, independently of the connector's rotation. Point each
docking axis outward from its face; the rear axis therefore remains
`-1, 0, 0` even after setting its RPY.

The two connector rows now appear under the module's **Connectors** branch.
Select each to inspect its location and axes. Existing connectors use
**Numeric pose**, **Position XYZ (m)**, and **Rotation RPY (rad)** controls;
keep **Numeric pose** checked and **Named frame** empty. After editing an
existing connector, click **Apply connector**.

The viewport draws placed connector frames and axes after an edit is applied.
Connector selection and placement use the tree and numeric fields; there is
no interactive connector translation/rotation gizmo.

## Adapt the type to different hardware

Keep the example values while completing this workflow. For another system,
choose rules based on its intended interface behavior:

| Hardware behavior | Corresponding authoring choice |
| --- | --- |
| Two unlike mating parts | Create male and female types, each naming the other in **Compatible with**. Compatibility is required in both directions. |
| Latches when faces meet | Enable **Declare docking policy** and **Automatic latching** on both mating types. |
| Waits for a controller request | Leave automatic latching off and send a runtime docking request. |
| Accepts any mating roll | Use `continuous` orientations and leave the discrete value list empty. |
| Allows a physical pivot | Use `hinge` with a unit hinge axis in each connector frame and a positive anchor separation; matching types need matching hinge geometry. Test with MuJoCo. |
| Requires nominal alignment | Select `nominal` docking alignment deliberately; this can snap the mating pose. |
| Needs time before recapture | Declare a positive redock cooldown. |
| Must remain permanently joined | Leave **Supports undocking** unchecked. |

MuJoCo currently executes fixed and hinge connections. The mock backend checks
fixed-style connection semantics and refuses hinges. Compliant, ball, and
custom connections do not have MuJoCo execution support in this release.

The imported cube's capability catalog can stay empty for this demonstration.
Adding a connector does not require inventing `dock` or `undock` capability
entries; the runtime consumes its connector types and docking policies directly.

Previous: [Import a URDF](import-urdf.md).
Next: [Validate and save](validate-and-save.md).
