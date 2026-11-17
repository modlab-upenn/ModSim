# ModSim Studio MVP

ModSim Studio provides two optional native applications: the Robot Pack Builder
with its PyVistaQt/VTK authoring viewport, and a lightweight Runtime Inspector
with PyQtGraph topology/cubic-lattice views and an event table. The applications
share core Robot Pack and model-view contracts but remain separate windows in
the current slice. A MuJoCo Runtime Inspector launch also opens the backend's
native 3D viewer as a separate companion window by default; it is not embedded
in either Studio application.

Online SMORES demos show live and target topology side by side in **Runtime state**.
The **Planning** tab contains physical XY routes, per-action status, and a time-only
action-history chart. **Event log** has its own tab for simulation events and planner
decisions. Collapsible/hover legends explain the graphics, and a persistent result
banner distinguishes completion from failure, stopping, or an exhausted time budget.
See [Online planar planning](planning.md) for launch commands and scope.
The [M-Blocks planner](mblocks_planning.md) uses the same tabs with live/target
cubes, swept-cell and pivot overlays, generated moves, and flywheel telemetry.

## Install and launch

Create a repository-local virtual environment and install the current checkout
in editable mode:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -e ".[studio,dev]"
.venv/bin/modsim-robotics studio
```

With no pack selector, the Builder opens a blank **Build your Robot Pack**
welcome screen. Choose **Create from URDF** to author a new pack, or
**Open Robot Pack** to open an existing directory. Save and Export As are
disabled until a project exists. A blank launch does not require library setup
or select an implicit robot.

To explore the installed starter instead, prepare the library and select it:

```bash
.venv/bin/modsim-robotics setup
.venv/bin/modsim-robotics studio generic_cube
```

When supplied, `generic_cube` resolves by ID in the installed
library, whose directory is configured by `[paths].robot_packs` in the ModSim
home's `config.toml`. Use `modsim-robotics pack list` to see IDs, versions, and
paths, or qualify the name as `generic_cube@0.1.0`. Ambiguous names report an
error rather than selecting the newest version. There is no implicit default
robot, including when only the starter is installed.

Home selection uses `--home PATH`, then `MODSIM_PATH`, legacy `MODSIM_HOME`, and
finally `~/.modsim`. Setup prints guidance for persisting `MODSIM_PATH`; it does
not edit your shell profile. Launching Studio does not run setup implicitly.

Open a different pack directly, or use the equivalent named option:

```bash
.venv/bin/modsim-robotics studio /absolute/path/to/robot_pack
.venv/bin/modsim-robotics studio --pack /absolute/path/to/robot_pack
```

An absolute pack path or a relative path beginning with `./` bypasses library
configuration without changing it. `studio generic_cube --home PATH` instead
looks up the name in another ModSim home's library. `--pack generic_cube` is an
equivalent way to supply the selector.

Install MuJoCo too, then launch the Runtime Inspector for a pack:

```bash
.venv/bin/python -m pip install -e ".[studio,mujoco,dev]"
.venv/bin/modsim-robotics runtime /absolute/path/to/robot_pack \
  --fixed-connector CONNECTOR_ID \
  --moving-connector CONNECTOR_ID
```

That one command opens both the Runtime Inspector and MuJoCo viewer. ModSim
automatically uses the active environment's `mjpython` for the viewer child on
macOS. Add `--no-viewer` for the semantic graph/event window alone, including
when the backend should run without a native 3D window. The Qt window still
requires a desktop display or Xvfb. `--backend mock` also opens only the
semantic window; explicit `--viewer` with that backend is rejected.

The standalone `modsim-robotics-studio` command and `python -m modsim_robotics_studio` are
equivalent and accept the same positional selector, `--pack`, and `--home PATH`
selection. Both open the blank Builder when the selector is omitted. Supplied
names and paths use the same explicit resolution rules; neither opens a default.
Running through `.venv/bin/python -m modsim_robotics_studio` is useful while
developing because the editable install uses the current source tree.

## Appearance and workspace

Both applications use the supplied ModSim production branding, with three
theme packs:

- **ModSim Midnight** (default): deep navy surfaces and teal actions;
- **ModSim Dark**: charcoal surfaces, pale text, and teal actions; and
- **ModSim Light**: cool white surfaces, navy text, and deep teal actions.

Use the **Theme** picker at the top right of either window to switch immediately.
The choice is stored in Qt user settings under organization `ModSim`, application
`Studio`, key `appearance/theme`. Both applications restore that preference on
their next launch. Windows within the same process update together; an already
running separate process keeps its current theme until changed or relaunched.
Appearance preferences are not written into Robot Packs or tracked project files.
An existing `graphite` preference selects ModSim Dark without resetting other
settings. The canonical theme IDs are `light`, `dark`, and `midnight`.

The shared styling covers headers, menus, dialogs, fields, tables, tabs, and
scrollbars. The authoring viewport background/grid and runtime graph colors
also follow the palette. Theme changes preserve in-progress edits, document
selection, viewport cameras, and runtime presentation state. Robot materials,
geometry, physics, and the native MuJoCo companion viewer are unchanged.
Charts use separate categorical hues instead of variations of the UI accent.
The same planner state or assembly keeps its color meaning across themes, with
darker inks on the light background. Legends show the colors currently in use.
Headers use the original outlined ModSim wordmark. Window/application icons,
toolbar glyphs, and welcome artwork come from the matching production pack.
Palette JSON and curated assets are included under
`src/modsim_robotics_studio/assets/branding/`, with their provenance and hashes.
The installed wheel does not read the original branding folder. Supporting
text uses available system fonts; the outlined logo needs no font installation.
PySide6 renders these assets without an additional theme or icon dependency.

Studio preserves the operating system's body-font size and accessibility
settings. Headings, workspace labels, and loading text use proportional point
sizes. Combo boxes and spin boxes retain Fusion's native controls and arrows.
List, tree, and table selections use the theme's foreground/background pair;
inline editors use compact padding so their text fits inside the row.

Wordmarks are rendered from SVG for the widget's display scale and refreshed
when its device pixel ratio changes. Toolbar icons supply 1×, 2×, 3×, and 4×
representations. Application icons use the scalable SVG with original 256,
512, and 1024 px PNG representations. Splash and loading backgrounds are scaled
to physical display pixels using the current device pixel ratio. Their source
artwork is 1920×1080; displaying it on a 4K screen does not add image detail or
make it native 4K artwork.

### Linux dock and application launcher

The Builder and Runtime Inspector share the desktop identity
`io.github.modlab_upenn.ModSim`. Before creating their splash or main window,
both configure that identity and register a per-user application launcher and
icon. This lets Ubuntu and other Linux desktops associate the windows with
ModSim rather than the Python executable. The dock uses the supplied Midnight
1024 px app icon; window icons continue to follow the selected theme.

The registration files are:

```text
<desktop-data>/applications/io.github.modlab_upenn.ModSim.desktop
<desktop-data>/modsim-robotics/icons/modsim.png
```

`<desktop-data>` is the absolute `XDG_DATA_HOME` when configured, otherwise
`~/.local/share`. Startup updates these files only when their contents change.
Activating the application launcher opens the blank Builder using the Python
environment of the most recent Builder or Inspector launch, preserving its
virtual-environment executable path. Launch the Inspector with the runtime
command and explicit RobotPack selector described above.

If desktop registration cannot write its files, startup logs a warning and
continues with the window icon. Registration is Linux-only and leaves RobotPacks
and shell profiles untouched.

### Windows and macOS application identity

On Windows, both launchers set the process AppUserModelID to
`io.github.modlab_upenn.ModSim` before constructing `QApplication`. This gives
the running applications a shared taskbar identity. Qt supplies the running
application and window icons on Windows and macOS as well as Linux.

Persistent Windows launcher branding requires a native executable or shortcut
with matching identity and icon metadata. Finder and Launchpad branding on
macOS requires an `.app` bundle. The current Python installation does not create
these native packages or shortcuts. Native desktop validation currently covers
Linux; Windows and macOS integration still requires testing on those platforms.

### Startup and loading feedback

Both applications open a themed startup splash with an animated ModSim logo,
version, and current initialization stage. The supplied SVG symbol assembles,
the outlined wordmark appears, and the completed logo softly pulses. Normal
startup keeps this window visible for at least three seconds from its first
display. An approximately 1.1-second intro plays through the Qt event loop
before the heavier window and plotting modules are imported and constructed.
That intro and subsequent initialization count toward the three-second minimum;
only the remaining time is waited before showing the prepared main window.
Qt/VTK construction on the GUI thread can temporarily pause the animation.

Clicking the splash leaves it open. Closing it cancels startup and keeps any
prepared main window hidden. A startup failure closes it immediately. On a
successful blank launch, the Builder presents the same background artwork with
its two authoring actions. The pictured robot is illustrative brand art, not a
preview of the loaded RobotPack.

Opening, importing, saving, and exporting a pack show a modal loading panel.
URDF parsing and document reads/writes run in a temporary worker while the Qt
event loop keeps the panel responsive. Widget updates and VTK scene creation
remain on the GUI thread. A loading stage is painted before preview generation;
large VTK scene construction can still temporarily occupy that thread.
The panel prevents document edits or dismissal during persistence, closes on
success or failure, and leaves error reporting to the existing action handler.
Progress is indeterminate because these operations do not report a measurable
completion percentage. The startup splash's three-second minimum applies only
to initial application branding; these document operations finish when their
work completes. The Runtime Inspector's corresponding first-frame
loading behavior is described in [the runtime contract](runtime_inspector.md#appearance).

The Builder has an Open/Import/Save/Validate toolbar, a searchable project tree,
and a scrollable Inspector. Connector pose inputs separate XYZ and RPY
components. Connector and connector-type fields are grouped by purpose;
connector custom metadata can be expanded when needed. The Validation tab
shows the actual profile result and issue counts, with a read-only issue table
when there are findings. Viewport layers and **Fit** are above the preview.
Dock panels remain movable and can be restored through the **View** menu.

Both applications and their file/creation dialogs use a shared window-sizing
policy. New windows account for the completed layout and current font metrics,
with opening dimensions capped at 88% of the target screen's usable width and
86% of its usable height. They open as normal windows with visible desktop
margins. Dialogs use their visible parent's monitor; independent windows start
on the pointer's monitor, with the application screen as fallback. Qt handles
the operating system's high-DPI scaling; these dimensions are logical pixels,
not physical display pixels.

Normal window size and position, along with main-window dock arrangement, are
stored locally under `ModSim` / `Studio`, in `windows/<window-kind>/` preferences.
Reopening restores the previous normal dimensions even if the window was
closed while maximized. Remembered sizes may use more space than the new-window
defaults, but are constrained to the current usable desktop with a small margin.
Independent windows return to their saved monitor when it remains available;
dialog geometry is adjusted to its parent's monitor. Monitor changes or
disconnections keep the window within the available screen, subject to the
desktop compositor's placement rules. Startup and operation-loading windows
use the same bounds without persisting their temporary geometry.

The Builder starts with preferred Project and Inspector sidebar widths while
reserving space for the module preview. The welcome screen, Inspector forms,
and validation area scroll within their assigned space. Creation-dialog forms
wrap long rows and scroll while OK and Cancel remain outside the scroll area.
Welcome actions allow room for their text at the current font size; validation
and metadata table rows also follow their contents and font.

The Builder prepares visual/collision geometry and diagnostic layers when a
module is loaded, then retains their VTK graphics objects. Selecting links or
joints updates highlights and properties; layer toggles change visibility.
These interactions preserve the camera and draw at most once, without mesh
reads or scene reconstruction. Connector edits replace only connector overlays;
other semantic metadata edits leave mechanical geometry intact. New module
scenes are constructed with intermediate rendering suppressed. Imported mesh
data is cached for the active asset, copied before placement/scaling, and
invalidated when that asset is reopened or reimported. **Fit** explicitly refits
the camera. Mouse navigation continues to render interactively, while the
static authoring viewport no longer redraws periodically when idle.

Open, import, asset-root selection, and export use Qt's built-in file dialogs,
which follow the Studio theme. They bypass the native GTK file picker, avoiding
process aborts caused by incompatible GTK/pixbuf libraries inherited from a
Snap-packaged terminal (for example, a loader requiring a newer system glibc).
Native dialog helpers are disabled application-wide before dialogs are created;
setting only the individual file-dialog option is too late on some Qt/GTK paths.

Every picker includes a **Show hidden files and folders** checkbox, including
URDF selection, package/mesh-root selection, opening a pack, choosing a new
import folder, and Export As. Enable it to browse directories such as
`~/.modsim`, or clear it to hide those entries again. The choice is shared
between pickers and restored on the next launch as a local Studio preference;
it is not stored in a RobotPack. Toggling visibility keeps the current folder,
typed filename, and file-type filter.

The file browser uses the same readable colors and compact inline editors in
List, Detail, and icon presentations. **F2** starts a file or folder rename;
**Enter** commits it and **Escape** cancels the edit. Back, Forward, and Parent
navigation controls use palette-aware Qt arrows, including disabled states
and right-to-left layouts.

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
a runtime by itself. `modsim-robotics runtime` is the explicit launch path. See
`runtime_inspector.md` for controls, process/thread ownership, and current
limitations.

## Session logging

Each Studio launch starts a new local session log and rewrites the previous log
rather than appending to it. When a name or path is supplied, Studio resolves it
before configuring session logging. The log is kept in a sidecar directory
outside the atomically replaced pack folder:

```text
<pack-parent>/.modsim/logs/<pack-folder-name>/studio.log
```

A blank Studio launch uses the working-directory log:

```text
<current-working-directory>/.modsim/logs/studio.log
```

Set `MODSIM_STUDIO_LOG` to override the log path:

```bash
MODSIM_STUDIO_LOG=/path/to/studio.log modsim-robotics studio /path/to/robot_pack
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
  .venv/bin/python -m modsim_robotics_studio /absolute/path/to/robot_pack
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

1. Launch `modsim-robotics studio` and select **Create from URDF** on the welcome
   screen, or use **File → Create from URDF** in an open Builder.
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

`modsim_robotics` contains the backend-neutral schemas, importer, validation,
persistence, runtime, immutable inspector transport, and model-view APIs. It
does not import Qt, PyVista, VTK, or PyQtGraph.

`modsim_robotics_studio` is an optional client of those APIs. A future web frontend or
C++ visualization application can consume the same Robot Pack and importer
models without depending on Studio.
