# ModSim Studio MVP

ModSim Studio provides two optional native applications: the Robot Pack Builder
with its PyVistaQt/VTK authoring viewport, and a lightweight Runtime Inspector
with PyQtGraph topology/cubic-lattice views and an event table. The applications
share core Robot Pack and model-view contracts but remain separate windows in
the current slice. A MuJoCo Runtime Inspector launch also opens the backend's
native 3D viewer as a separate companion window by default; it is not embedded
in either Studio application.

Online SMORES demos add a Planning tab to the Runtime Inspector, with physical
XY routes, target topology, per-action status, an execution timeline, and planner
decisions. See [Online planar planning](planning.md) for launch commands and scope.

## Install and launch

Create a repository-local virtual environment and install the current checkout
in editable mode:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -e ".[studio,dev]"
.venv/bin/modsim studio
```

Open a pack directly:

```bash
.venv/bin/modsim studio /absolute/path/to/robot_pack
```

Install MuJoCo too, then launch the Runtime Inspector for a pack:

```bash
.venv/bin/python -m pip install -e ".[studio,mujoco,dev]"
.venv/bin/modsim runtime /absolute/path/to/robot_pack \
  --fixed-connector CONNECTOR_ID \
  --moving-connector CONNECTOR_ID
```

That one command opens both the Runtime Inspector and MuJoCo viewer. ModSim
automatically uses the active environment's `mjpython` for the viewer child on
macOS. Add `--no-viewer` for the semantic graph/event window alone, including
when the backend should run without a native 3D window. The Qt window still
requires a desktop display or Xvfb. `--backend mock` also opens only the
semantic window; explicit `--viewer` with that backend is rejected.

The standalone `modsim-studio` command and `python -m modsim_studio` are
equivalent. Running through `.venv/bin/python -m modsim_studio` is useful while
developing because the editable install uses the current source tree.

## Appearance and workspace

Both applications share a native Studio design with three built-in palettes:

- **Midnight Panels** (default): navy surfaces with teal accents;
- **Graphite Workbench**: charcoal surfaces with blue accents; and
- **Light Studio**: light surfaces with blue accents.

Use the **Theme** picker at the top right of either window to switch immediately.
The choice is stored in Qt user settings under organization `ModSim`, application
`Studio`, key `appearance/theme`. Both applications restore that preference on
their next launch. Windows within the same process update together; an already
running separate process keeps its current theme until changed or relaunched.
Appearance preferences are not written into Robot Packs or tracked project files.

The shared styling covers headers, menus, dialogs, fields, tables, tabs, and
scrollbars. The authoring viewport background/grid and runtime graph colors
also follow the palette. Theme changes preserve in-progress edits, document
selection, viewport cameras, and runtime presentation state. Robot materials,
geometry, physics, and the native MuJoCo companion viewer are unchanged.
The styling and small line icons are implemented in Studio itself using the
existing PySide6 dependency; no external theme or icon package is required.

The Builder has an Open/Import/Save/Validate toolbar, a searchable project tree,
and a scrollable Inspector. Connector pose inputs separate XYZ and RPY
components. Connector and connector-type fields are grouped by purpose;
connector custom metadata can be expanded when needed. The Validation tab
shows the actual profile result and issue counts, with a read-only issue table
when there are findings. Viewport layers and **Fit** are above the preview.
Dock panels remain movable and can be restored through the **View** menu.

Open, import, asset-root selection, and export use Qt's built-in file dialogs,
which follow the Studio theme. They bypass the native GTK file picker, avoiding
process aborts caused by incompatible GTK/pixbuf libraries inherited from a
Snap-packaged terminal (for example, a loader requiring a newer system glibc).
Native dialog helpers are disabled application-wide before dialogs are created;
setting only the individual file-dialog option is too late on some Qt/GTK paths.

## Current feature inventory

The current Studio MVP provides:

- a project tree for imported URDF assets, connector types, module types,
  links, joints, and connector instances;
- Properties editors for pack and module fields, custom pack/connector/type
  metadata, joint control modes and limits, connector pose and axes, and
  reusable connector-type semantics;
- explicit connector-type creation and reference-safe removal, plus connector
  reassignment to an existing type and an imported URDF body/link;
- connector-type fields for gender, compatibility, allowed orientations,
  acceptance tolerances, physical constraints, compliance, hinge axis/anchor
  geometry, load limits, and undocking support, plus optional runtime docking
  policy;
- a Model Views catalog for adding, editing, and removing named builder
  recipes, supported modes, default selection hints, and JSON configuration;
- a separate Runtime Inspector that runs named two-module or seven-module
  scenarios plus larger M-Blocks routes, draws the live module-topology or
  cubic-lattice view, supports module-label visibility and lattice pan/orbit/
  zoom controls, retains the ordered canonical event log, and can supervise a
  native MuJoCo companion window showing that same runtime;
- a read-only preview of the canonical split-YAML documents;
- authoring validation with `F6` and stricter structural
  simulation-readiness validation with `F7`;
- atomic in-place Save and non-overwriting Export As;
- an embedded viewport for URDF visual and collision geometry, link frames,
  joint axes, and connector frames and axes;
- resolved URDF solid-color materials, smooth PBR surface shading, scene
  lighting, a shadowed grid floor, and non-destructive link-selection bounds;
  and
- a local per-launch session log mirrored in the **Session Log** tab.

The authoring viewport renders one module type at a time in the URDF zero-joint
configuration. Simulation readiness is a validation profile; it does not start
a runtime by itself. `modsim runtime` is the explicit launch path. See
`runtime_inspector.md` for controls, process/thread ownership, and current
limitations.

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
save/export operations, and full Python exception traces. Studio dialogs show
concise field-oriented validation guidance, while the log retains the model,
field location, error type, rejected value, and traceback needed for debugging.
The `.modsim`
directory is local runtime state and is ignored by Git. Keeping the active log
beside, rather than inside, the Robot Pack ensures that atomic Save operations
cannot replace it.

For a dual-window runtime, the Qt process remains the sole session-log writer.
It captures diagnostics from the native-viewer child and mirrors them into the
same file instead of allowing the child to truncate or concurrently write the
log. Closing the native viewer is recorded as an ordinary runtime stop; an
unexpected child failure retains its detailed diagnostic in the log while the
Runtime Inspector shows a concise error.

For a repeatable current-source debugging session, launch from the repository
root and keep the log in the ignored repository-local `.modsim` directory:

```bash
mkdir -p .modsim/logs
MODSIM_STUDIO_LOG="$PWD/.modsim/logs/studio-dev.log" \
  .venv/bin/python -m modsim_studio /absolute/path/to/robot_pack
```

Reproduce one problem per launch because the file is truncated when the
session starts. Inspect the **Session Log** tab while reproducing it, then use
the file for the complete timestamped validation messages and Python
tracebacks. Record the selected tree entity and the action immediately before
an error; selection state is relevant to the known GUI issues below. Launching
with the pack path also makes the startup and open operation part of the same
log. If Studio is launched without a pack and the pack is opened later, the
session continues to use the working-directory log chosen at startup.

## URDF-to-Robot-Pack workflow

1. Select **File → Create from URDF**.
2. Choose an expanded `.urdf` or XML file.
3. If meshes cannot be resolved, select the package or mesh asset root.
4. Choose a new Robot Pack directory. Existing destinations are refused.
5. Inspect imported links, joints, visual geometry, collision geometry, link
   frames, and joint axes.
6. Select **Connector Types**, choose **Add connector type**, and define the
   reusable interface.
7. Select a link and choose **Add connector to this link**.
8. Enter the connector ID, select an existing type, and enter its local pose,
   axes, and optional custom fields.
9. Select an existing connector under the module's **Connectors** group to
   change its type, imported URDF body/link, frame/pose, axes, custom fields,
   or to remove it.
10. Select joints to add control modes and unit-bearing limits.
11. Select the Robot Pack to edit custom metadata in the Properties panel.
12. Select **Model Views** to add any model-view recipes that should be
    available during authoring or at runtime.
13. Run authoring validation with `F6` and simulation-readiness validation with
   `F7`.
14. Use **Save** to update the open pack or **Export As** to create a new pack.

Save stages a complete copy, reloads it, compares its semantic model, and then
atomically swaps the pack directory. It writes metadata and semantic edits to
the canonical split-YAML documents. Export As never overwrites its destination.
The YAML preview is read-only; use the Properties panel and dedicated editors
to make changes. Add, edit, and remove operations initially change only the
in-memory document. They are not persisted until **Save** or **Export As**
completes.

### Connector add, edit, and remove details

Create a type first by selecting the **Connector Types** catalog row and using
**Add connector type**. Creation requires an explicit lowercase snake_case ID
such as `ep` or `smores_ep`; put display capitalization such as `EP` in the
separate Name field. New types initially declare themselves compatible using
that validated ID. The dialog also exposes the active flag, gender, and custom
metadata. Select the new concrete type row to edit compatibility, orientation,
acceptance, physical, load, undocking, and custom-metadata fields. **Remove
connector type** refuses deletion while a connector instance uses the type.
When an unused type is removed, compatible type and capability references to
it are cleaned so the YAML remains valid.

The connector-type Properties panel also exposes an optional docking policy.
Enable **Declare docking policy** to edit automatic latching, measured versus
nominal alignment, redock cooldown, and break force. Disabling it removes the
explicit policy and restores the documented runtime defaults. Applying any
other connector-type edit preserves both a declared docking policy and custom
metadata.

Selecting `hinge` as the physical constraint enables **Hinge axis (x, y, z)**
and **Hinge anchor separation (m)**. The axis must be a unit vector and is
expressed in that connector's own frame, not the parent-link frame used by the
connector's docking and approach axes. Anchor separation must be positive.
Both compatible hinge types must define the same values. Selecting a different
constraint disables and removes hinge geometry when the edit is applied; a
hinge cannot be saved with either required field missing.

To add a connector, select a concrete link row under **Module Types → module →
Links**. The link's Properties panel contains **Add connector to this link**.
The dialog requires a schema-valid lower-case identifier, an existing type
selected from the catalog, numeric `xyz` and `rpy` triples, and unit-length
docking and approach axes. The selected imported URDF link becomes the
connector's initial parent body.

To edit or remove an instance, select its concrete row under **Module Types →
module → Connectors**. Its stable ID remains read-only. Its type and parent
body/link use dropdowns populated from the current connector-type catalog and
the module's imported URDF. The named frame, numeric local-pose mode, pose,
axes, and custom fields are editable. **Remove connector** removes the instance
and any backend connector-frame mapping for its ID from the in-memory document,
without a confirmation dialog or undo. Close and discard the document to
abandon an unsaved removal, or use **Save** to persist it.

### Custom metadata fields

The Robot Pack, connector-instance, and connector-type Properties panels each
contain a two-column custom metadata editor. Use **Add field** and **Remove
selected** to manage entries. Values use JSON syntax: strings need quotes,
while numbers, booleans, nulls, lists, and nested objects can be entered
directly. The schema permits at most 128 fields per metadata mapping and checks
field-name portability. Applying an editor changes the in-memory document;
**Save** or **Export As** writes the values to the appropriate canonical YAML
file.

### Model-view recipes

Select the **Model Views** catalog row and choose **Add model view**. Each
recipe has a stable lower-snake-case ID, optional display name, registered
builder ID, authoring/runtime mode checkboxes, a default-view hint, and a
JSON-compatible builder-configuration table. The initial builder is
`module_topology_graph`, which generates one node per module instance and one
edge per active docked connection. It always includes disconnected modules as
isolated nodes and currently accepts no configuration fields, so leave its
configuration table empty.

Select a concrete recipe to edit or remove it. As with other Studio edits, the
change is in memory until **Save** or **Export As**. Removing a recipe removes
only the presentation/generation configuration; it never removes modules,
connectors, runtime connections, or other canonical robot state.

Studio does not ask the user to enter graph nodes or edges. The model-view
factory derives them from the Robot Pack and current runtime `WorldState`.
Recipes say which views a robot platform recommends and how their builders are
configured. Additional generic or platform-specific builders may be
registered later while using the same recipe format.

## Editing boundaries

URDF remains the source of link geometry, mesh references, and kinematics.
Robot Pack YAML adds modular-robot metadata and semantics but does not replace
that mechanical source. Changing a primitive cube's shape or dimensions, for
example, requires editing the URDF and reimporting it.

Connector placement currently uses numeric pose fields in the Properties panel.
Studio does not yet provide an interactive translation or rotation gizmo.

## Viewport conventions

- Visuals use resolved global or inline URDF `rgba` material colors. Visuals
  without a supported color use the diagnostic link palette.
- Smooth surface normals, rough nonmetallic PBR shading, multiple lights, and
  shadows provide shape and depth cues even for untextured STL meshes.
- A large grid floor extends well beyond the robot and is enabled by default.
  Camera framing uses only the robot bounds, so the expanded floor does not
  shrink the model in the view. Toggle the floor with **Ground** in the
  viewport toolbar.
- Selecting a link draws gold bounds around it without replacing its material.
- Red, green, and blue frame arrows are local X, Y, and Z.
- Yellow arrows are imported joint axes.
- Bright green arrows are connector docking axes.
- Blue arrows are connector approach axes.
- Collision geometry is a translucent red wireframe.
- Connector docking and approach axes are expressed in their parent-link
  frame, matching Robot Pack format 0.1.

The viewport uses the URDF zero joint configuration. Link frames and joint axes
are available from the toolbar but default to hidden so they do not obscure the
robot. Joint animation and 3D translation/rotation gizmos are not implemented
yet; numeric connector pose changes are immediately redrawn.

## Known bugs and UX limitations

These are current-source limitations, not intended long-term behavior:

- **Connectors cannot be selected or moved in 3D.** Mesh picking selects links
  only. Connector selection is through the project tree, and placement is
  through numeric fields; there is no translation or rotation gizmo.
- **Named-frame-only connectors are not drawn.** Studio preserves named-frame
  and numeric-local-pose location modes, but the viewport does not yet resolve
  arbitrary named frames. A connector without `local_pose` therefore has no
  rendered connector glyph.
- **Mechanical-name validation is split.** Studio now verifies connector
  parent-body changes against the imported URDF link list. The standalone Robot
  Pack validator still does not parse referenced URDFs to cross-check module
  roots, connector parents, source joints, named frames, or backend mapping
  names after the URDF is changed externally.
- **Viewport coverage is intentionally limited.** Acceptance-region geometry
  and physical hinge axes/anchors are editable but not rendered. The viewport
  shows one module type rather than a multi-module assembly, has no joint
  animation, and does not display
  contacts, physics, docking execution, generated model-view previews, or
  runtime state. The separate Runtime Inspector renders the generated logical
  graph and events; its optional MuJoCo companion window renders 3D physics
  without embedding the backend viewer in Studio. A document edit rebuilds
  the scene, resets the camera, and currently returns multi-module documents
  to the first renderable module.
- **Native authoring regression coverage remains limited.** Focused Builder
  tests cover appearance, pending connector edits, project filtering, and file
  picker acceptance/cancellation with a stub viewport. The native launch smoke
  test checks camera/material preservation when switching themes. Full 3D
  interaction and all editing dialogs still need manual reproduction and session
  logs. Document-model tests separately cover connector/type mutation,
  URDF-body association, and metadata persistence.

## Import support and limitations

The importer currently supports:

- links and tree-structured joints;
- fixed, revolute, continuous, prismatic, floating, and planar joints;
- origins, axes, scalar limits, and link masses;
- named and inline URDF solid-color materials with validated `rgba` values;
- box, cylinder, sphere, and local mesh geometry;
- relative, `file://`, and locally resolvable `package://` mesh references;
- self-contained asset copying and URDF mesh-path rewriting.

It intentionally does not:

- execute Xacro;
- fetch HTTP or HTTPS assets;
- import transmissions or simulator-specific XML extensions;
- infer connectors, compatibility, acceptance regions, or capabilities;
- render or automatically copy texture-image material dependencies;
- copy every external texture referenced indirectly by OBJ/DAE material files;
- modify CAD or generate URDF.

Mesh decoding depends on formats supported by the installed VTK/PyVista stack.
STL, OBJ, PLY, VTK-family, and common polygon formats are the safest first
targets. Solid URDF colors work with any decoded geometry. Texture references
are retained in the imported model and reported as deferred, but texture images
are not yet bundled or displayed; verify all external assets after importing
OBJ or DAE.

## Package boundary

`modsim` contains the backend-neutral schemas, importer, validation,
persistence, runtime, immutable inspector transport, and model-view APIs. It
does not import Qt, PyVista, VTK, or PyQtGraph.

`modsim_studio` is an optional client of those APIs. A future web frontend or
C++ visualization application can consume the same Robot Pack and importer
models without depending on Studio.
