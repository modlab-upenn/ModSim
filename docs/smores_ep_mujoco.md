# SMORES-EP MuJoCo docking demo

This is the reproducible local workflow for the private SMORES-EP Robot Pack at
`.modsim/robot_packs/smores_ep`. The Fusion-exported visual meshes remain
ignored by Git; do not publish them until redistribution is explicitly cleared.
The pre-integration pack is preserved at
`.modsim/backups/smores_ep-pre-mujoco-integration`.

## Authored connector semantics

The pack defines one active, genderless, self-compatible connector type:
`ep_face`. It supports fixed docking and undocking, four quarter-turn mating
orientations, explicit latch commands, nominal alignment, and a 0.1-second
redock cooldown.

The current tolerances and load limits are marked `provisional_demo` metadata.
They make the software path executable; they are not verified SMORES hardware
ratings and must be replaced from CAD, controller, and test data.

| Connector | Parent link | Local position (m) | Outward docking axis |
|---|---|---:|---:|
| `bottom` | `base_link` | `[-0.010741577148, 0, 0]` | `[-1, 0, 0]` |
| `pan` | `front_wheel_1` | `[0.005762, 0, 0]` | `[1, 0, 0]` |
| `left` | `left_wheel_1` | `[0, 0.04405, 0]` | `[0, 1, 0]` |
| `right` | `right_wheel_1` | `[0, -0.04405, 0]` | `[0, -1, 0]` |

The positions come from the exported mating surfaces in each URDF link frame.
In particular, the fixed SMORES `bottom` face is the rear `-X` plane of
`base_link`, opposite the `pan`/top face; it is not the module underside. Its
origin uses the dominant centered planar surface rather than four 0.3 mm mesh
protrusions extending slightly farther in `-X`. Studio should be used to verify
every glyph against the CAD before treating the pack as mechanically
authoritative.

## Collision proxies

The detailed Fusion STL files remain visual geometry. Detailed-mesh collision
was replaced by one 12-triangle OBJ box per connector-bearing face:

| Link/face | Proxy size (m) | Proxy center in link (m) |
|---|---:|---:|
| base/bottom | `[0.008, 0.0651, 0.0708856]` | `[-0.006741577148, 0, 0]` |
| front wheel/pan | `[0.008, 0.070, 0.070]` | `[0.001762, 0, 0]` |
| left wheel/left | `[0.075, 0.008, 0.075]` | `[0, 0.04005, 0]` |
| right wheel/right | `[0.075, 0.008, 0.075]` | `[0, -0.04005, 0]` |

The filenames are deliberately distinct. MuJoCo creates one mesh asset per
name; reusing one filename with several URDF scale values can collapse those
references onto one scale and create false initial contacts.

These face boxes are docking-demo proxies, not a complete collision model for
locomotion, stability, or manipulation.

The SMORES URDF explicitly retains its five detailed Fusion STL visual meshes
for MuJoCo. ModSim assigns the four collision proxies to MuJoCo geom group 3,
which the viewer hides initially while physics continues to use them. Toggle
**Group 3** in the native viewer to inspect the proxies. The visual meshes are
currently uniformly silver: STL contains geometry but no texture coordinates
or material graph, and the URDF assigns the same solid RGBA material to every
link.

## Setup and validation

```bash
UV_CACHE_DIR=.uv-cache UV_PYTHON_INSTALL_DIR=.uv-python \
  uv sync --locked --extra dev --extra studio --extra mujoco --python 3.12

uv run --no-sync modsim backends
uv run --no-sync modsim pack validate \
  .modsim/robot_packs/smores_ep --profile simulation
```

## Headless docking and undocking

```bash
uv run --no-sync modsim run .modsim/robot_packs/smores_ep \
  --backend mujoco \
  --fixed-connector pan \
  --moving-connector pan \
  --connector-gap 0.02 \
  --approach 0.03 \
  --duration 1.5 \
  --dt 0.002 \
  --undock-at 1.0 \
  --retract 0.03
```

A successful run reports, in order:

```text
DockCandidateDetected
DockCommitted
AssemblyMerged
UndockCommitted
AssemblySplit
```

Use `bottom`, `left`, or `right` for both connector options to exercise the
other same-face pairs. All four were verified headlessly with zero gravity.

## Coupled Runtime Inspector and MuJoCo viewer

Launch the same real MuJoCo scenario with its native 3D view, live semantic
graph, and event log:

```bash
uv run --no-sync modsim runtime .modsim/robot_packs/smores_ep \
  --backend mujoco \
  --fixed-connector pan \
  --moving-connector pan \
  --connector-gap 0.02 \
  --approach 0.03 \
  --duration 2.0 \
  --dt 0.002
```

The graph starts with `smores_ep_0` and `smores_ep_1` as isolated nodes. The
edge `smores_ep_0/pan<->smores_ep_1/pan` appears after MuJoCo accepts the weld,
and the table shows `DockCandidateDetected`, `DockCommitted`, and
`AssemblyMerged`. Add `--undock-at 1.0 --retract 0.03` to watch the edge be
removed and the assembly split. The local pack's default `smores_topology`
recipe supplies the graph model.

That one command opens two separate windows backed by one authoritative
`RuntimeSession`: the Runtime Inspector shows ModSim semantics and the native
MuJoCo window shows meshes, contacts, and collision-debug groups. On macOS the
launcher automatically uses the active environment's `mjpython` for the native
child; do not start a second `modsim run` command. Add `--no-viewer` for the
semantic window and a headless MuJoCo backend. The Qt window still needs a
desktop display or Xvfb; use `modsim run` without `--view` for a completely
non-GUI run.

### Named docking-and-undocking demonstration

Use the named preset when the two-module demonstration should always dock and
then undock without calculating `--undock-at` manually:

```bash
uv run --no-sync modsim runtime .modsim/robot_packs/smores_ep \
  --backend mujoco \
  --demo dock_undock \
  --fixed-connector pan \
  --moving-connector pan \
  --connector-gap 0.02 \
  --approach 0.03 \
  --retract 0.03 \
  --duration 6.0 \
  --dt 0.002 \
  --no-gravity
```

The graph progresses from two isolated nodes to one edge and back to two
isolated nodes. The expected committed lifecycle is
`DockCandidateDetected`, `DockCommitted`, `AssemblyMerged`,
`UndockCommitted`, and `AssemblySplit`; the moving module then retracts. The
Runtime Inspector and native MuJoCo windows remain views of the same session.

### Seven-module Driver-to-Snake demonstration

Run the paper-backed multi-module demonstration with:

```bash
uv run --no-sync modsim runtime .modsim/robot_packs/smores_ep \
  --backend mujoco \
  --demo smores_driver_to_snake \
  --duration 14.0 \
  --dt 0.002 \
  --connector-gap 0.02 \
  --approach 0.03 \
  --no-gravity
```

Do not add `--ground`; gravity and the ground plane must both remain disabled
for this scripted staging demonstration. The graph begins with seven nodes and
six edges in the Driver topology. Each of four actions removes one connection
and commits its replacement, producing four visible `6 → 5 → 6`
edge-count transitions. The final graph is the chain:

```text
module_1 — module_3 — module_2 — module_4 — module_5 — module_6 — module_7
```

The four staged replacements are:

| Action | Undock | Dock |
|---:|---|---|
| 1 | `module_1/bottom ↔ module_2/pan` | `module_1/pan ↔ module_3/bottom` |
| 2 | `module_7/pan ↔ module_5/bottom` | `module_7/bottom ↔ module_6/pan` |
| 3 | `module_2/right ↔ module_4/left` | `module_2/pan ↔ module_4/bottom` |
| 4 | `module_5/left ↔ module_4/right` | `module_5/bottom ↔ module_4/pan` |

The topology and action pairs come from Figure 16 and Table III of Chao Liu,
Michael Whitzer, and Mark Yim,
[*A Distributed Reconfiguration Planning Algorithm for Modular Robots*](https://www.modlabupenn.org/wp-content/uploads/2019/08/chao_smores_reconfiguration_2019.pdf),
IEEE Robotics and Automation Letters, 2019,
DOI [`10.1109/LRA.2019.2930432`](https://doi.org/10.1109/LRA.2019.2930432).
The paper calls the connectors `TOP`, `BOTTOM`, `LEFT`, and `RIGHT`; this pack
maps `TOP` to `pan` and otherwise keeps the face names.

The paper is the provenance for the named configurations and connector action
pairs. ModSim's current runtime executes the pairs sequentially in Table III
order and moves whole components through deterministic kinematic staging. This
is a visualization and semantic-lifecycle demonstration, not autonomous
reconfiguration planning, collision-free path planning, wheel/joint control,
or a physically supported SMORES locomotion reproduction. The event log and
metrics are ModSim runtime output, not measurements reported by the paper.

## Standalone viewer on macOS

The older `modsim run --view` workflow opens only MuJoCo's passive viewer. It
still must own the main thread on macOS, so that standalone command requires
`mjpython` explicitly:

```bash
.venv/bin/mjpython -m modsim run .modsim/robot_packs/smores_ep \
  --backend mujoco \
  --fixed-connector pan \
  --moving-connector pan \
  --connector-gap 0.02 \
  --duration 4 \
  --undock-at 2.5 \
  --view
```

The scenario drives the moving module's root free joint. It demonstrates
backend loading, measured connector frames, acceptance, weld creation, logical
assembly changes, release, and retraction. It does not yet command SMORES
wheels, pan, or tilt actuators; model contact exclusion and MuJoCo
constraint-force reporting also remain future work.
