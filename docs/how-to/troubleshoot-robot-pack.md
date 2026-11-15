---
title: Troubleshoot a RobotPack
sidebar_label: Troubleshoot a RobotPack
---

Start with the first stage that differs from the expected result: import,
authoring, structural validation, backend startup, or docking. Keep the pack
path and the current error or event visible while checking it. Changing several
tolerances at once makes it harder to identify the cause.

## Import or authoring fails

| Symptom | Check and correction |
| --- | --- |
| **Could not inspect URDF** | Open the exact source file in a text editor. It must be concrete XML with a `<robot>` root, named links, valid parent/child references, and supported joint/geometry data. Expand Xacro first. |
| A package or mesh root is requested | Select a directory that resolves the local mesh references. The importer does not download HTTP assets. A model with unresolved meshes cannot become a self-contained pack. |
| **Could not create Robot Pack** with an existing destination | Choose a new folder name in **Choose a new Robot Pack folder**. Browse to its parent and enter the new name; do not create that folder first. |
| A mesh is missing or a material looks different | Verify local files, scale, and visual origins. Solid URDF colors are supported. Textures and indirect OBJ/DAE material dependencies need separate review. |
| **Create a connector type first** | Add the reusable type under **Connector Types**, then select a link and add its connector instances. |
| The ID is rejected | Use lowercase snake case, such as `fixed_face` or `front`. Put capitalization and spaces in **Name**, not the ID. |
| Acceptance fields cannot be applied | Fill position tolerance, orientation tolerance, and maximum relative velocity together. Numeric values must be finite and use the displayed units. |
| A property change disappears | Click its **Apply** button, then **Save**. Editing a text field alone does not persist the document. Check which copy is currently open after **Export As**. |

Changing the module's metadata does not resize its mechanical asset. For a
different shape or inertia, edit the URDF and use a deliberate new import.
Use **Fit** or the **View** menu to restore a hidden or misplaced authoring
panel before assuming the model was lost.

## Validation does not pass

Run **Authoring Profile** (`F6`) and **Simulation Readiness** (`F7`) and read
the **Validation** table's code, location, and message. Find the referenced
entity, apply one correction, and rerun the profile.

| Finding | Where to look |
| --- | --- |
| Missing allowed orientations | Select the concrete connector type and complete **Orientation mode** and its discrete values, if applicable. |
| Missing acceptance or physical connection | Complete the type's acceptance fields and choose a supported physical constraint. |
| Unknown or missing load limit | Check **Max normal force**, **Max shear force**, and **Max bending moment**. The workshop values are teaching data; use appropriate values for another robot. |
| Missing docking or approach axis | Select the placed connector and provide a unit vector in its parent-link frame. |
| Unknown connector type or capability | Repair the referenced catalog entry or remove the incorrect reference. A name in a module must identify a declared entry. |
| Missing joint-control limit | Check the joint's **Control modes** and the bound required for each mode, as described in [Configure joint control](configure-joints.md). |

A clean structural report does not prove that a default runtime view exists
or that connector geometry will mate. Verify those parts explicitly using
[Validate and save](validate-and-save.md), then execute a runtime check.

## The runtime cannot start

**The workspace name is not found.** A bare ID searches the configured installed
library. Open a workspace by its saved absolute path, or use the explicit
default-home path shown in [Run your RobotPack](run-robot-pack.md). Check that
the Builder saved the same directory the runtime is loading.

**No default runtime model-view recipe.** Add or select `module_topology` in
**Model Views**. Use builder `module_topology_graph`, enable **Available at
runtime**, check **Default view**, keep configuration empty, and save. A
runtime view is generated from state; no nodes or edges need to be authored.

**Backend unavailable or a native window fails.** Install MuJoCo into the same
Python environment that launches ModSim. The Runtime Inspector requires Studio
and a display. Add `--no-viewer` if only the native 3D window is unavailable;
Qt still requires a display. Use the [headless CLI task](cli.md#run-the-lifecycle-headlessly)
when no desktop is available. The runtime launcher selects its MuJoCo viewer
interpreter automatically on macOS.

## The modules do not dock or release

| Event detail or symptom | Likely cause and next check |
| --- | --- |
| No nearby candidates | Verify face positions and scene spacing. For the 0.1 m workshop cube, the face centers are at ±0.05 m and a touching row has 0.1 m center spacing. |
| `axis_alignment` fails | Check the parent-link-frame docking axes. Front and rear point in opposite directions; connector RPY does not replace the axis declaration. |
| `orientation` fails | Check the connector-frame orientation and allowed roll values on both mating types. Use radians. |
| `relative_velocity` fails | Check the actual relative speed and the type's declared maximum. Correct the controller or intended tolerance deliberately. |
| Incompatible types | Both types must list each other in **Compatible with**, and their genders must form a supported pair. |
| `command_required` | Automatic latching is off on at least one type. The scripted demo sends a request; a custom experiment must call `request_dock()` when appropriate. **Active connector** does not enable auto-latching. |
| Undocking is refused | Enable **Supports undocking** on both mating types for a releasable interface, apply the edit, save, and start a new run. |
| Bodies remain close after release | Removing a constraint does not apply a separating force. Check `UndockCommitted` and the connection count, then supply retraction or other motion. |
| A hinge is refused by mock | Use MuJoCo for hinge physics. The mock deliberately refuses to approximate a hinge as a rigid connection. |

Inspect **Event log** before judging connectivity from the 3D image.
`DockCommitted` establishes a logical connection; `UndockCommitted` removes it.
The topology edge follows those commitments. A nearby candidate after release
can be reported without creating another edge.

## Preserve useful diagnostics

The Builder's **Session Log** tab records import, editing, validation, and
persistence diagnostics, including the log path chosen at startup. Logs are
local and rewritten at each launch, so copy the relevant details before restarting.
Importing another pack within the same Builder session does not relocate its
already-open log.

When reporting a problem, include the ModSim version, explicit pack path or
identity, backend, launch command, validation code/location or runtime event,
and the last action before the failure. For a graphical issue, describe the
selected tree row and panel. Keep the smallest model or sequence that reproduces
it so the mechanical and semantic causes can be distinguished.

Return to [the authoring workflow](index.md), [connector authoring](author-connectors.md),
or [the runtime experiment](write-runtime-experiment.md).
