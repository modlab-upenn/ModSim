# SMORES-EP end-to-end workflow: URDF to MuJoCo

This document walks the committed SMORES-EP Robot Pack at
`examples/robot_packs/smores_ep` through the full pipeline: importing the
Fusion-exported URDF, authoring the four electropermanent-face connector
semantics, validating, and running fixed docking under MuJoCo. It applies the
generic six-stage path in `docs/WORKFLOW.md` to one real module and uses that
pack's actual link, joint, and connector values.

The pack already exists in the source checkout, so this doc serves two
purposes: it records how the pack was built from URDF and how to author the
same semantics onto a fresh SMORES import, and it is a reproduction-and-run
guide for the pack as shipped. The run reference in `docs/smores_ep_mujoco.md`
is the authority for the exact demo commands, connector-frame table, collision
proxies, and provenance; this document is the load-in-to-simulation narrative
around it.

The commands below assume the virtual environment is active
(`source .venv/bin/activate`), so `modsim ...` is on the path. Equivalent forms:
`.venv/bin/modsim ...` without activation, or `uv run --no-sync modsim ...`
under `uv`. Every command accepts either the pack **directory**
(`examples/robot_packs/smores_ep`) or the manifest path
(`examples/robot_packs/smores_ep/robot_pack.yaml`) — they are interchangeable.

> **Connector names — read this first.** The SMORES-EP module has exactly four
> connectors: **`bottom`, `pan`, `left`, `right`**. There is no `front`
> connector — that name belongs to the `generic_cube` example. Passing
> `--fixed-connector front` fails with *"module type 'smores_ep' has no
> connector(s) front"*. Use the same valid face for both `--fixed-connector` and
> `--moving-connector`, because a face is only self-compatible with its own type
> (`ep_face`).

> **Asset note:** repository inclusion of the SMORES-EP V4.2.0 export was
> authorized for collaborator access only; no separate asset license currently
> grants broader reuse. The connector frames, tolerances, and load limits are
> `provisional_demo` data, not verified SMORES-EP hardware specifications.

## What the SMORES-EP module is, as a pack

The module type `smores_ep` has five links, four joints, and four connectors:

- **Links:** `base_link` (root), `left_wheel_1`, `right_wheel_1`, `tilt_body_1`,
  `front_wheel_1`. Each visual is a detailed Fusion STL in millimetres
  (`scale="0.001 0.001 0.001"`), all assigned one solid silver material.
- **Joints:** `joint_left_wheel` and `joint_right_wheel` (continuous, base to
  each wheel, Y axis); `joint_tilt` (revolute, base to tilt body, Y axis,
  ±1.570796 rad); `joint_pan` (continuous, tilt body to front wheel, X axis).
  The kinematic chain matters for connectors: `pan` rides on `front_wheel_1`,
  which is the child of `tilt_body_1`, so it belongs to an articulated child
  link, not the root.
- **Connectors:** four `ep_face` faces — `bottom`, `pan`, `left`, `right` — each
  on a different link. See Stage 2 for the exact poses.

Total authored module mass is ~0.4386 kg. The four EP faces are all the same
connector *type*, `ep_face`, which is genderless and self-compatible, so any
face can mate with any other face on another module.

## Stage 0 — Install and confirm MuJoCo

From the repository root, into a virtual environment with the Studio and MuJoCo
extras:

```bash
python3.12 -m venv .venv
PIP_NO_CACHE_DIR=1 .venv/bin/python -m pip install --upgrade pip
PIP_NO_CACHE_DIR=1 .venv/bin/python -m pip install -e ".[studio,mujoco,dev]"
source .venv/bin/activate
```

The locked `uv` equivalent is:

```bash
UV_CACHE_DIR=.uv-cache UV_PYTHON_INSTALL_DIR=.uv-python \
  uv sync --locked --extra dev --extra studio --extra mujoco --python 3.12
```

Confirm the backend is present — `mujoco` must appear as installed before Stage
6 can step physics:

```bash
modsim backends
```

## Stage 1 — Load in the URDF

The pack's geometry and kinematics come entirely from the Fusion-exported URDF.
To build the self-contained pack from that source, the importer is run against
the raw URDF and its mesh directory:

```bash
modsim pack init \
  --from-urdf /path/to/smores_ep.urdf \
  --asset-root /path/to/smores_meshes \
  --out examples/robot_packs/smores_ep
```

Use one or more `--asset-root` options because the export references its STL
meshes by relative path; point them at the directory holding
`base_link.stl`, `left_wheel_1.stl`, `right_wheel_1.stl`, `tilt_body_1.stl`,
and `front_wheel_1.stl`. The importer:

- extracts the five links, the four tree joints, joint origins and axes, the
  `joint_tilt` scalar limits, and per-link inertial masses;
- copies the resolvable STL meshes into `assets/meshes/`, rewrites their paths,
  and keeps the millimetre scale;
- resolves the single inline `silver` solid-color material for Studio preview;
  and
- writes `robot_pack.yaml`, `specs/module_types.yaml` (with an empty
  `connectors` list), and `mappings/urdf_mapping.yaml`.

It accepts URDF, not Xacro — expand any Xacro source to a concrete URDF first.
It refuses to overwrite an existing destination or to create a pack with
unresolved mesh references. STL geometry carries no texture coordinates or
material graph, which is why every SMORES visual renders uniformly silver.

> The committed pack already contains the imported URDF at
> `examples/robot_packs/smores_ep/assets/urdf/smores_ep.urdf`; there is no
> separate raw export in the repository. Re-run Stage 1 only when starting from a
> fresh Fusion export, pointing `--from-urdf` and `--asset-root` at that source.

Two SMORES-specific details in the URDF are deliberate and are preserved
through import and into MuJoCo:

- The `<mujoco><compiler discardvisual="false"/></mujoco>` element keeps the
  five detailed STL visuals in the MuJoCo model instead of discarding them.
- Each connector-bearing link carries a lightweight collision proxy — a
  12-triangle OBJ box (`bottom_face_box.obj`, `pan_face_box.obj`,
  `left_face_box.obj`, `right_face_box.obj`) — rather than using the detailed
  mesh for contact. The filenames are intentionally distinct so MuJoCo does not
  collapse several scaled references onto one mesh asset and create false
  contacts. These proxies are docking-demo geometry, not a full collision model.
  Their exact sizes and centers are tabulated in `docs/smores_ep_mujoco.md`.

At the end of Stage 1 the pack loads and previews, but it has no docking
semantics yet — the importer leaves connectors and capabilities to the author.

## Stage 2 — Author the electropermanent-face semantics

This is the modular layer the URDF cannot express. Author it in Studio
(recommended, so each connector glyph can be checked against the CAD) or in the
YAML documents directly. Open Studio on the pack with:

```bash
modsim studio examples/robot_packs/smores_ep
```

### The `ep_face` connector type

One reusable type describes every SMORES face:

```yaml
# specs/connector_types.yaml
connector_types:
  ep_face:
    name: SMORES-EP Electropermanent Face
    active: true
    gender: genderless
    compatible_with: [ep_face]
    allowed_orientations:
      mode: discrete
      values_rad: [0.0, 1.5707963267948966, 3.141592653589793, 4.71238898038469]
    acceptance_region:
      shape: cylinder
      position_tolerance_m: 0.006
      orientation_tolerance_rad: 0.17453292519943295
      max_relative_velocity_m_s: 0.05
    physical_connection:
      constraint: fixed
      compliance: null
    limits:
      max_normal_force_n: 100.0
      max_shear_force_n: 50.0
      max_bending_moment_nm: 2.0
    supports_undocking: true
    docking_policy:
      auto_latch: false
      alignment: nominal
      redock_cooldown_s: 0.1
      break_force_n: null
```

Key choices for SMORES:

- **Genderless and self-compatible** (`compatible_with: [ep_face]`): any EP
  face can dock to any other, which is what makes the seven-module chain
  possible.
- **Four quarter-turn orientations**: an electropermanent face mates at 0, π/2,
  π, or 3π/2.
- **`alignment: nominal`**: on commit, the connector frames snap coincident at
  the matched discrete orientation rather than freezing the observed pose. This
  prevents pose drift accumulating across the repeated dock/undock cycles in the
  reconfiguration demo.
- **`redock_cooldown_s: 0.1`**: a face stays free for 0.1 s after undocking or a
  failed dock before it can latch again.
- **`auto_latch: false`**: docking is by explicit command, as for a commanded
  electropermanent magnet.

The load limits and tolerances are simulation placeholders — replace them from
CAD, controller, and test data before treating the pack as authoritative.

### The four connector instances

Each face is one instance on a different link, positioned from the exported
mating surface in that link's frame:

| Connector | Parent link | Local position (m) | Yaw (rad) | Docking / approach axis |
|---|---|---|---|---|
| `bottom` | `base_link` | `[-0.010741577148, 0, 0]` | π | `[-1, 0, 0]` |
| `pan` | `front_wheel_1` | `[0.005762, 0, 0]` | 0 | `[1, 0, 0]` |
| `left` | `left_wheel_1` | `[0, 0.04405, 0]` | π/2 | `[0, 1, 0]` |
| `right` | `right_wheel_1` | `[0, -0.04405, 0]` | −π/2 | `[0, -1, 0]` |

```yaml
# specs/module_types.yaml (connectors excerpt)
connectors:
  - id: bottom
    connector_type: ep_face
    parent_link: base_link
    local_pose:
      xyz_m: [-0.010741577148, 0.0, 0.0]
      rpy_rad: [0.0, 0.0, 3.141592653589793]
    docking_axis: [-1.0, 0.0, 0.0]
    approach_axis: [-1.0, 0.0, 0.0]
  # ... pan, left, right follow the table above
```

The `bottom` face is the important subtlety: it is the rear **−X** plane of
`base_link`, directly opposite the `pan`/top face — not the module underside.
Its origin uses the dominant centered planar surface rather than the small mesh
protrusions extending slightly farther in −X. Getting this right is what makes
the seven-module MuJoCo chain form a straight horizontal nose-to-tail line
instead of a vertically kinked shape. In Studio, select `bottom`, enable the
connector and collision overlays, and confirm the glyph sits on the rigid rear
base opposite `pan`, both arrows pointing outward along −X, with a thin vertical
rear plate for the proxy.

### Capabilities and the model view

Advertise the two primitive actions and declare the runtime graph recipe:

```yaml
# specs/capabilities.yaml
capabilities:
  dock:   { name: Dock,   kind: primitive_action, required_connector_types: [ep_face] }
  undock: { name: Undock, kind: primitive_action, required_connector_types: [ep_face] }
```

```yaml
# robot_pack.yaml (excerpt)
model_views:
  - id: smores_topology
    name: SMORES Topology
    builder: module_topology_graph
    modes: [runtime]
    default: true
    configuration: {}
```

`smores_topology` is the default recipe, so the runtime uses it without an
explicit `--model-view`. The mapping document (`mappings/urdf_mapping.yaml`)
maps all five ModSim link IDs and four joint IDs to their identical URDF names;
the simulation profile requires that coverage.

## Stage 3 — Validate

```bash
modsim pack inspect examples/robot_packs/smores_ep
modsim pack validate examples/robot_packs/smores_ep --profile simulation
```

The simulation profile promotes completeness warnings to errors and checks that
every module type, root link, joint, and named-frame connector has mapping
coverage. It does not launch a simulator or parse the URDF — it proves the pack
is structurally ready to hand to a backend. Get it to zero errors before Stage
6. Invalid packs exit 1; valid packs exit 0.

## Stage 4 — Generate the topology view

List the pack's recipes, then generate one immutable snapshot. Either the pack
directory or the `robot_pack.yaml` path works:

```bash
modsim views examples/robot_packs/smores_ep/robot_pack.yaml

modsim views examples/robot_packs/smores_ep/robot_pack.yaml \
  --view smores_topology --count 3 --output json
```

With no docking yet, the snapshot is `--count` isolated module nodes — the
correct base state before any welds.

## Stage 5 — Semantic dry run on the mock backend

Confirm the EP faces are authored so they actually latch, without any physics:

```bash
modsim dock examples/robot_packs/smores_ep --count 3
modsim dock examples/robot_packs/smores_ep --count 3 --undock
modsim dock examples/robot_packs/smores_ep --output json
```

This runs the real semantic pipeline against the kinematic mock backend and
prints the event log, resulting assemblies, and metrics. When a pair does not
dock, it names the criterion or guard that blocked it — the fastest way to tune
the `ep_face` acceptance cylinder or a connector pose from Stage 2. It answers
"is this pack authored so these faces would latch?", not "will the robot
physically work?".

## Stage 6 — Simulate under MuJoCo

MuJoCo composes the module assets, measures each connector frame after load,
steps rigid-body physics, and executes the **fixed** EP-face connections using a
pool of reserved weld constraints. Fixed docking is the only connection type
MuJoCo runs today; wheel/pan/tilt actuation, contact exclusion, and
constraint-force reporting remain future work. Gravity is off by default for
`modsim run` (a docking demo should not also be a falling demo); pass
`--no-gravity` explicitly to the `runtime` demos.

### 6a. Headless docking and undocking (`modsim run`)

Fully non-GUI, scriptable, good for CI and quick checks. This does **not** need
`mjpython`:

```bash
modsim run examples/robot_packs/smores_ep \
  --backend mujoco \
  --fixed-connector pan --moving-connector pan \
  --connector-gap 0.02 --approach 0.03 \
  --duration 1.5 --dt 0.002 \
  --undock-at 1.0 --retract 0.03 \
  --output json
```

A successful run reports, in order:

```text
DockCandidateDetected
DockCommitted
AssemblyMerged
UndockCommitted
AssemblySplit
```

Swap both connector values for `bottom`, `left`, or `right` to exercise the
other same-face pairs; all four were verified headlessly at zero gravity.
`--retract` backs the driven module away after release so the undock is visible
— without it the two welded modules coast along together still touching.

### 6b. Live Runtime Inspector plus native viewer (`modsim runtime`)

The coupled GUI: one authoritative `RuntimeSession` backs both the semantic
graph/event Inspector and the native MuJoCo 3D view, so they cannot diverge. Use
the `dock_undock` preset so the two modules always dock then undock:

```bash
modsim runtime examples/robot_packs/smores_ep \
  --backend mujoco --demo dock_undock \
  --fixed-connector pan --moving-connector pan \
  --connector-gap 0.02 --approach 0.03 \
  --retract 0.03 --duration 6.0 --dt 0.002 --no-gravity
```

The graph starts with `smores_ep_0` and `smores_ep_1` isolated, gains the edge
`smores_ep_0/pan<->smores_ep_1/pan` after MuJoCo accepts the weld, then returns
to two isolated nodes after the release. The collision proxies sit in MuJoCo
geom group 3, hidden initially; toggle **Group 3** in the native viewer to
inspect them while physics keeps using them. On macOS the launcher finds and
uses the environment's `mjpython` for the native child automatically — run the
command exactly as written; do not prefix it with `mjpython`.

For a single dock with no scripted release, drop `--demo dock_undock` and the
undock flags:

```bash
modsim runtime examples/robot_packs/smores_ep \
  --backend mujoco \
  --fixed-connector pan --moving-connector pan \
  --connector-gap 0.02 --approach 0.03 --duration 4
```

Add `--no-viewer` to either form to keep only the semantic Qt window with MuJoCo
stepping headlessly; Qt still needs a display or Xvfb.

### 6c. Standalone passive viewer, row mode (macOS `mjpython`)

The older `modsim run --view` workflow opens only MuJoCo's passive viewer, and
because that viewer must own the main thread on macOS it requires `mjpython`
explicitly. With no connector options it uses **row mode**: it places `--count`
modules in a row and drives the last one along world −X into the first:

```bash
mjpython -m modsim run examples/robot_packs/smores_ep --backend mujoco \
  --count 2 --duration 8 --undock-at 4 --view
```

`--view` paces the run to wall clock and holds the window open at the end. To
target a specific face pair instead of row mode, add
`--fixed-connector pan --moving-connector pan` (or another valid face).

### 6d. Seven-module Driver-to-Snake demonstration

The paper-backed multi-module preset. It needs no connector options — its
actions are scripted — and both gravity and the ground plane must stay off:

```bash
modsim runtime examples/robot_packs/smores_ep \
  --backend mujoco --demo smores_driver_to_snake \
  --duration 14.0 --dt 0.002 \
  --connector-gap 0.02 --approach 0.03 --no-gravity
```

Do **not** add `--ground`. The graph begins with seven nodes and six edges in
the Driver topology. Four scripted replacements each remove one connection and
commit its replacement, producing four visible `6 → 5 → 6` edge transitions:

| Action | Undock | Dock |
|---:|---|---|
| 1 | `module_1/bottom ↔ module_2/pan` | `module_1/pan ↔ module_3/bottom` |
| 2 | `module_7/pan ↔ module_5/bottom` | `module_7/bottom ↔ module_6/pan` |
| 3 | `module_2/right ↔ module_4/left` | `module_2/pan ↔ module_4/bottom` |
| 4 | `module_5/left ↔ module_4/right` | `module_5/bottom ↔ module_4/pan` |

The final graph is the chain:

```text
module_1 — module_3 — module_2 — module_4 — module_5 — module_6 — module_7
```

With the flags above, the verified final metrics are six active connections,
one assembly, ten successful docks (six initial plus four replacements), four
successful undocks, and zero dock/undock failures. Because `bottom` is on the
rear base, the final MuJoCo module roots form a straight horizontal nose-to-tail
chain in that order. The topology and action pairs come from Figure 16 and
Table III of Liu, Whitzer, and Yim,
[*A Distributed Reconfiguration Planning Algorithm for Modular Robots*](https://www.modlabupenn.org/wp-content/uploads/2019/08/chao_smores_reconfiguration_2019.pdf)
(IEEE RA-L, 2019); the paper's `TOP` maps to `pan`. ModSim executes those pairs
by deterministically staging whole components through the ordinary dock/undock
pipeline — this is a semantic-lifecycle and visualization demonstration, not
autonomous reconfiguration planning, collision-free path planning, or SMORES
locomotion.

## Command quick reference

Every command takes the pack directory or its `robot_pack.yaml` path. Valid
connectors: `bottom`, `pan`, `left`, `right`.

| Goal | Command |
|---|---|
| Confirm MuJoCo installed | `modsim backends` |
| Author / preview in GUI | `modsim studio examples/robot_packs/smores_ep` |
| Validate for simulation | `modsim pack validate examples/robot_packs/smores_ep --profile simulation` |
| List / generate a view | `modsim views examples/robot_packs/smores_ep/robot_pack.yaml --view smores_topology --count 3 --output json` |
| Mock dry run (no physics) | `modsim dock examples/robot_packs/smores_ep --count 3 --undock` |
| Headless MuJoCo dock/undock | `modsim run ... --backend mujoco --fixed-connector pan --moving-connector pan --undock-at 1.0 --retract 0.03 --output json` |
| Coupled GUI Inspector | `modsim runtime ... --backend mujoco --demo dock_undock --fixed-connector pan --moving-connector pan --no-gravity` |
| Standalone viewer (macOS) | `mjpython -m modsim run ... --backend mujoco --count 2 --duration 8 --undock-at 4 --view` |
| Seven-module reconfiguration | `modsim runtime ... --backend mujoco --demo smores_driver_to_snake --no-gravity` |

## Reproduce-and-verify checklist

1. Install with the `mujoco` extra and confirm `modsim backends` lists it.
2. `modsim pack validate examples/robot_packs/smores_ep --profile simulation`
   reports zero errors.
3. In Studio, verify each of the four EP-face glyphs against the CAD, especially
   `bottom` on the rear −X base opposite `pan`.
4. `modsim run ... --fixed-connector pan --moving-connector pan ...` prints the
   five-event dock/undock sequence; repeat for `bottom`, `left`, `right`.
5. The seven-module `smores_driver_to_snake` demo ends in the nose-to-tail chain
   with six connections, one assembly, ten docks, four undocks, zero failures.

## Caveats specific to this pack

- Connector frames, acceptance tolerances, and load limits are `provisional_demo`
  data, not certified SMORES-EP hardware ratings.
- MuJoCo executes fixed EP-face docking only; wheel, pan, and tilt actuation are
  not commanded, and welded-contact exclusion and constraint-force feedback are
  not implemented.
- Visual meshes are uniformly silver (STL has no materials); the four collision
  proxies are docking-demo geometry, not a locomotion-grade collision model.
- Do not add proprietary or redistribution-restricted CAD, URDF, or mesh assets
  beyond what is already authorized for this repository.

## See also

- `docs/smores_ep_mujoco.md` — the SMORES run reference: exact demo commands,
  the connector-frame and collision-proxy tables, and full provenance.
- `docs/WORKFLOW.md` — the generic, pack-agnostic end-to-end workflow.
- `docs/robot_pack_spec.md` — the Robot Pack format 0.1 contract.
- `docs/docking_semantics.md` — how acceptance, alignment, and cooldown are
  evaluated at runtime.
- `docs/runtime_inspector.md` — the live graph/event workflow.
