# ModSim Studio MVP

ModSim Studio is the optional standalone Robot Pack Builder. It is implemented
with PySide6/Qt and an embedded PyVistaQt/VTK viewport. Physics and simulator
backends are not part of this milestone.

## Install and launch

From a source checkout:

```bash
python -m pip install -e ".[studio,dev]"
modsim studio
```

Open a pack directly:

```bash
modsim studio /absolute/path/to/robot_pack
```

The standalone `modsim-studio` command and `python -m modsim_studio` are
equivalent.

## Session logging

Each Studio launch starts a new local session log and rewrites the previous log
rather than appending to it. When a pack is supplied at launch, the log is kept
in a sidecar directory outside the atomically replaced pack folder:

```text
<pack-parent>/.modsim/logs/<pack-folder-name>/studio.log
```

When Studio is launched without a pack, the default is:

```text
<current-working-directory>/.modsim/logs/studio.log
```

Set `MODSIM_STUDIO_LOG` to override the log path:

```bash
MODSIM_STUDIO_LOG=/path/to/studio.log modsim studio /path/to/robot_pack
```

The **Session Log** tab mirrors the same file during the running session.
It records startup context, open/import operations, edits, validation issues,
save/export operations, and full Python exception traces. The `.modsim`
directory is local runtime state and is ignored by Git. Keeping the active log
beside, rather than inside, the Robot Pack ensures that atomic Save operations
cannot replace it.

## URDF-to-Robot-Pack workflow

1. Select **File → Create from URDF**.
2. Choose an expanded `.urdf` or XML file.
3. If meshes cannot be resolved, select the package or mesh asset root.
4. Choose a new Robot Pack directory. Existing destinations are refused.
5. Inspect imported links, joints, visual geometry, collision geometry, link
   frames, and joint axes.
6. Select a link and choose **Add connector to this link**.
7. Enter the connector ID/type, local pose, docking axis, and approach axis.
8. Select joints to add control modes and unit-bearing limits.
9. Select the Robot Pack or module item to edit its exposed metadata in the
   Properties panel.
10. Run authoring validation with `F6` and simulation-readiness validation with
   `F7`.
11. Use **Save** to update the open pack or **Export As** to create a new pack.

Save stages a complete copy, reloads it, compares its semantic model, and then
atomically swaps the pack directory. It writes metadata and semantic edits to
the canonical split-YAML documents. Export As never overwrites its destination.
The YAML preview is read-only; use the Properties panel and dedicated editors
to make changes.

## Editing boundaries

URDF remains the source of link geometry, mesh references, and kinematics.
Robot Pack YAML adds modular-robot metadata and semantics but does not replace
that mechanical source. Changing a primitive cube's shape or dimensions, for
example, requires editing the URDF and reimporting it.

Connector placement currently uses numeric pose fields in the Properties panel.
Studio does not yet provide an interactive translation or rotation gizmo.

## Viewport conventions

- Red, green, and blue frame arrows are local X, Y, and Z.
- Yellow arrows are imported joint axes.
- Bright green arrows are connector docking axes.
- Blue arrows are connector approach axes.
- Collision geometry is a translucent red wireframe.
- Connector docking and approach axes are expressed in their parent-link
  frame, matching Robot Pack format 0.1.

The viewport uses the URDF zero joint configuration. Joint animation and 3D
translation/rotation gizmos are not implemented yet; numeric connector pose
changes are immediately redrawn.

## Import support and limitations

The importer currently supports:

- links and tree-structured joints;
- fixed, revolute, continuous, prismatic, floating, and planar joints;
- origins, axes, scalar limits, and link masses;
- box, cylinder, sphere, and local mesh geometry;
- relative, `file://`, and locally resolvable `package://` mesh references;
- self-contained asset copying and URDF mesh-path rewriting.

It intentionally does not:

- execute Xacro;
- fetch HTTP or HTTPS assets;
- import transmissions or simulator-specific XML extensions;
- infer connectors, compatibility, acceptance regions, or capabilities;
- copy every external texture referenced indirectly by OBJ/DAE material files;
- modify CAD or generate URDF.

Mesh decoding depends on formats supported by the installed VTK/PyVista stack.
STL, OBJ, PLY, VTK-family, and common polygon formats are the safest first
targets. Verify materials and external textures after importing OBJ or DAE.

## Package boundary

`modsim` contains the backend-neutral schemas, importer, validation, and
persistence APIs. It does not import Qt, PyVista, VTK, or PyQtGraph.

`modsim_studio` is an optional client of those APIs. A future web frontend or
C++ visualization application can consume the same Robot Pack and importer
models without depending on Studio.
