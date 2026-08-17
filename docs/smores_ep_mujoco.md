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
| `bottom` | `base_link` | `[0.034458, 0, -0.0396]` | `[0, 0, -1]` |
| `pan` | `front_wheel_1` | `[0.005762, 0, 0]` | `[1, 0, 0]` |
| `left` | `left_wheel_1` | `[0, 0.04405, 0]` | `[0, 1, 0]` |
| `right` | `right_wheel_1` | `[0, -0.04405, 0]` | `[0, -1, 0]` |

The positions come from the exported mesh bounds in each URDF link frame.
Studio should be used to verify every glyph against the CAD before treating the
pack as mechanically authoritative.

## Collision proxies

The detailed Fusion STL files remain visual geometry. Detailed-mesh collision
was replaced by one 12-triangle OBJ box per connector-bearing face:

| Link/face | Proxy size (m) | Proxy center in link (m) |
|---|---:|---:|
| base/bottom | `[0.075, 0.075, 0.008]` | `[0.034458, 0, -0.0356]` |
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

## Viewer on macOS

The passive MuJoCo viewer must own the main thread on macOS:

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
