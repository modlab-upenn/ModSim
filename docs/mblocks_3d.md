# 3D M-Blocks integration

ModSim's second real-platform Robot Pack is `examples/robot_packs/mblocks_3d`.
It combines a user-provided Fusion 360 export with published M-Blocks geometry
and semantics. The current result supports Robot Pack authoring, connector
validation, generated topology/lattice views, a two-module momentum-driven
physics pivot, a composed twelve-module one-plane physics route, and matched
kinematic/physical 2×6-mat-to-staircase demonstrations. It remains a
deterministic physics bootstrap, not a hardware-validated full 3D M-Block
model.

## Research basis

The pack targets the mechanical generation described by Romanishin, Gilpin,
Claici, and Rus in [*3D M-Blocks: Self-reconfiguring Robots Capable of
Locomotion via Pivoting in Three Dimensions*](https://doi.org/10.1109/ICRA.2015.7139450),
ICRA 2015. The discrete reconfiguration model follows Sung, Bern, Romanishin,
and Rus, [*Reconfiguration Planning for Pivoting Cube Modular
Robots*](https://doi.org/10.1109/ICRA.2015.7139451), ICRA 2015.

The larger benchmark is motivated by Romanishin et al.,
[*Decentralized Control for 3D M-Blocks for Path Following, Line Formation,
and Light Gradient Aggregation*](https://doi.org/10.1109/IROS40897.2019.8967810), IROS 2019,
which reports physical decentralized line formation but does not provide an
exact replayable action trace for that experiment.

Generated 2D plans and the initial physical square-to-line benchmark are now
available separately; see [Online M-Blocks planning](mblocks_planning.md).

The published robot is a nominal 50 mm cube that reconfigures through 90- and
180-degree rotations about shared cube edges. Passive permanent magnets form
face bonds and temporary edge hinges. One internal flywheel produces reaction
torque; in the published 3D mechanism its carrier changes between three
orthogonal actuation planes.

The papers use cube-on-lattice diagrams and ordered snapshots to communicate
configurations and motion primitives, but do not define a standard interactive
visualizer or UI protocol. ModSim therefore follows their discrete cubic-cell
model while treating the projection controls, diagnostics, and interaction
described below as ModSim presentation design.

## Pack contents

```text
examples/robot_packs/mblocks_3d/
├── robot_pack.yaml
├── assets/
│   ├── README.md
│   ├── urdf/mblocks_3d.urdf
│   ├── mujoco/mblocks_3d.xml
│   └── meshes/
├── specs/
│   ├── module_types.yaml
│   ├── connector_types.yaml
│   └── capabilities.yaml
└── mappings/urdf_mapping.yaml
```

The Robot Pack contains hardware assets and semantics, not executable demo
code. The reference routes live beside it in `examples/scenarios/`, including
`mblocks_twelve_module_line.py` for the kinematic benchmark and
`mblocks_twelve_module_physics.py` for the physical sequence.
`mblocks_twelve_module_staircase.py` contains the shared topology and route for
both staircase executors. The named
Runtime Inspector entries therefore require a source checkout that contains
those example files.

The expanded URDF is the Studio/imported mechanical source. It preserves the
three source meshes while replacing the detailed collision meshes with one
nominal 50 mm cube. Its inertials and +Y flywheel axis are normalized for the
same one-plane physics bootstrap as the pack-local MJCF: 0.150 kg total mass
and `8.4e-6 kg m^2` flywheel axial inertia. The MJCF uses the same link and
joint names, renders the detailed meshes, gives contact only to a simple box,
and exposes the flywheel through the ordinary Robot Pack effort-command
boundary.

The module has six passive, genderless `mblock_magnetic_face` connector
instances: `pos_x`, `neg_x`, `pos_y`, `neg_y`, `pos_z`, and `neg_z`. Each is
centered 25 mm from the root origin and accepts the four quarter-turn mating
orientations. Passive capture is represented by `auto_latch: true`; committed
faces use measured alignment and an ideal fixed weld.

Eight additional directed ports represent the four XZ corners approached
through either an X-normal or Z-normal face. Co-located variants are separate
because connector acceptance needs to know which outward normal opposes the
other endpoint at each quarter turn. All eight use
`mblock_magnetic_edge_hinge`, whose physical connection is a two-point hinge
with connector-local axis `[0, 1, 0]` and 40 mm anchor separation. Its
provisional 3 mm position envelope permits the next hinge to engage after a
measured face landing without silently snapping the cube; orientation remains
limited to 5 degrees and point speed to 0.2 m/s. This is the complete directed
catalog needed to repeat pivots in the current +Y plane; it is not an
enumeration of all cube edges and all three carrier planes.

The physical magnets are deliberately not represented as separate logical
connectors. One connected face is one ModSim connection and one topology edge.

## Generated views

The pack declares three runtime recipes:

- `mblocks_topology` uses the generic `module_topology_graph` builder and is
  the explicit connectivity-only alternative.
- `mblocks_lattice` uses the generic `cubic_lattice` builder with a 50 mm
  pitch. It derives integer cells, one of the 24 proper cube orientations,
  lattice residuals, off-lattice state, occupancy conflicts, assemblies, and
  face-labelled connections from `WorldState`. It is the pack's default
  runtime view.
- `mblocks_physics_lattice` uses the same builder with its lattice origin
  raised 25 mm so grounded cube centers occupy integer Z cells. The physical
  momentum demonstrations and staircase reference select it automatically unless `--model-view`
  overrides the choice.

The lattice view is derived data, not a second configuration store. Module
poses and committed connections remain canonical in `WorldState`.

In the Runtime Inspector, the lattice result is rendered as solid cubes at
measured poses plus dashed ghost cubes at their nearest cells. The default
isometric view can be switched to XY, XZ, or YZ; the Z-layer selector filters
the occupied layers. Global lattice axes, optional measured local axes,
face-labelled connection markers, and stable module/connection/warning-cell
selection make orientation and connectivity visible at the same time. Amber
identifies an off-lattice pose, red identifies an occupancy conflict, and
normal cube colours identify assemblies. These are display diagnostics only;
they never snap or edit the simulation.

Generate both views without launching a simulator:

```bash
modsim views examples/robot_packs/mblocks_3d \
  --view mblocks_topology \
  --count 3 \
  --spacing 0.05 \
  --output json

modsim views examples/robot_packs/mblocks_3d \
  --view mblocks_lattice \
  --count 3 \
  --spacing 0.05 \
  --output json
```

## Inspect and validate

```bash
modsim pack inspect examples/robot_packs/mblocks_3d
modsim pack validate examples/robot_packs/mblocks_3d
modsim pack validate examples/robot_packs/mblocks_3d --profile simulation
modsim studio examples/robot_packs/mblocks_3d
```

Both validation profiles are expected to report zero issues. Studio should
load three links, two joints, three visual meshes, the primitive collision
box, and all fourteen connector frames without unresolved-asset warnings.

The existing generic two-module lifecycle can exercise any face before the
pivot scenario is used:

```bash
modsim runtime examples/robot_packs/mblocks_3d \
  --backend mujoco \
  --demo dock_undock \
  --fixed-connector pos_x \
  --moving-connector pos_x \
  --no-gravity
```

## Five-module kinematic pivot

Run the authored traversal with the Runtime Inspector and MuJoCo mesh viewer:

```bash
modsim runtime examples/robot_packs/mblocks_3d \
  --backend mujoco \
  --demo mblocks_five_module_pivot
```

The demo stages four blocks as a horizontal base and docks `block_5` above
`block_1`. It then moves `block_5` across the base through three 90-degree arcs
about successive positive-y edge axes. Each action uses the ordinary canonical
lifecycle: release one face weld, publish the assembly split, move along the
authored arc, request the accepted target face, create its MuJoCo weld, and
publish the assembly merge. The lattice view therefore shows the solid
`block_5` cube moving continuously around each pivot while its dashed nearest
cell changes, and the connection overlay shows three `4 → 3 → 4` edge
transitions. It finishes with five modules, four connections, and one assembly.

The nominal 50 mm lattice cells progress from `(0, 0, 1)` to `(3, 0, 1)` for
`block_5`; the four base cells remain `(0, 0, 0)` through `(3, 0, 0)`. Final
metrics are seven successful docks—four initial connections and three
replacements—three successful undocks, and no failures. The eight-second CLI
display budget includes initial and connected holds; the scripted movement
normally completes near 6.0 simulated seconds.

This demo is backend-neutral and can be checked quickly without a native 3D
window:

```bash
modsim runtime examples/robot_packs/mblocks_3d \
  --backend mock \
  --demo mblocks_five_module_pivot
```

Both commands use `mblocks_lattice` because it is the default runtime recipe.
Add `--model-view mblocks_topology` to either command when only the stable
module/connection graph is wanted.

`KinematicPivotScenario` preserves every member of a detached assembly under
one rigid transform and evaluates the reference frame at every step. It never
edits graph state directly: all edges still come from committed connections in
`WorldState`. The scenario does write module root poses, so it is a semantic and
visual integration demonstration rather than a physics experiment.

## Two-module momentum-pivot physics

Run the first physics path with its demo-aware defaults:

```bash
modsim runtime examples/robot_packs/mblocks_3d \
  --demo mblocks_momentum_pivot
```

The CLI selects MuJoCo, gravity, the ground plane, a 25 mm root height, a
0.00025 s solver/controller step, a five-second display budget, and
`mblocks_physics_lattice`. Use `--speed FACTOR` to change wall-clock pacing
without changing the physics. `0 < --dt <= 0.0005` is accepted; larger steps
are rejected because the short braking impulse becomes under-resolved.

`moving_block` starts on top of `support_block`. At time zero the scenario
stages the initial `pos_z ↔ neg_z` fixed face and coincident
`edge_pos_x_pos_z ↔ edge_pos_x_neg_z` hinge. It then:

1. settles and spins the moving flywheel toward 9,000 RPM using bounded effort;
2. releases the fixed face while retaining the +Y hinge;
3. brakes the flywheel toward zero and lets MuJoCo integrate shell rotation,
   support/ground contact, and the 180-degree edge roll;
4. commits the measured `pos_x ↔ pos_x` target face inside a 1 mm capture gate;
   and
5. releases the temporary hinge and holds one connected two-cube assembly.

Only initialization may stage module roots. After scenario creation, all
motion comes from the internal joint effort, gravity, contact, and hinge; no
root pose, root velocity, or external wrench is written. `DockCommitted`,
`ConnectionRuntime`, the event table, and topology/lattice edges preserve
whether each connection is `fixed` or `hinge`.

The fixed face and pre-engaged edge hinge are both active during settling and
spin-up. This intentional redundant-constraint interval prevents solver drift
from moving the very narrow hinge capture frames before release, but it can
introduce preload and is part of the deterministic approximation rather than a
claim about independently measured magnetic engagement forces.

The physical reference ceilings are:

| Parameter | Value | Use in the bootstrap |
|---|---:|---|
| Module mass | 0.150 kg | normalized MJCF total |
| Flywheel axial inertia | `8.4e-6 kg m^2` | MJCF and controller impulse calculation |
| Flywheel speed ceiling | 20,000 RPM | joint limit and overspeed guard |
| Ordinary spin-up effort | 0.03 N m | maximum positive effort |
| Mechanical-brake effort | 2.6 N m | maximum short reverse pulse |

The 9,000 RPM scenario target and 1 mm capture gate are reproducible ModSim
bootstrap choices below those published ceilings, not reported hardware
setpoints. The regression convergence trace is:

| Step | Completion time | Final moving-root x | Brake impulse |
|---:|---:|---:|---:|
| 0.5 ms | ~0.8585 s | within 0.0502–0.0507 m | within 0.00763–0.00767 N m s |
| 0.25 ms | ~0.8595 s | within 0.0502–0.0507 m | within 0.00763–0.00767 N m s |
| 0.1 ms | ~0.8596 s | within 0.0502–0.0507 m | within 0.00763–0.00767 N m s |

All three runs end with the support at lattice cell `(0, 0, 0)`, the moving
cube at `(1, 0, 0)`, one fixed target connection, no temporary hinge, and one
assembly. These are simulator regression results, not experimental hardware
measurements.

The magnetic behavior is deliberately approximate. The controller performs a
deterministic face-to-hinge-to-face lifecycle; it does not integrate a
magnetic force field or let measured force decide which magnets engage or
release. The two-point hinge retains body collision, but it idealizes the
temporary magnetic edge contact.

## Twelve-module structure-to-line benchmark

Run the larger view/event demonstration:

```bash
modsim runtime examples/robot_packs/mblocks_3d \
  --backend mujoco \
  --demo mblocks_twelve_module_line
```

Eleven connected blocks form a horizontal substrate and `block_12` begins
above `block_1`. Ten authored quarter turns move it across the top surface;
one final half-turn places it beside `block_11`, producing one connected line
occupying cells `(0, 0, 0)` through `(11, 0, 0)`. The route contains 11
connection replacements and finishes near 21.30 simulated seconds inside the
default 24-second display run. The final state has 12 modules, 11 connections,
and one assembly.

For a faster semantic check without the native mesh viewer, use:

```bash
modsim runtime examples/robot_packs/mblocks_3d \
  --backend mock \
  --demo mblocks_twelve_module_line
```

This benchmark is inspired by the 2019 physical line-formation work, which
does not publish an exact replayable per-module move trace. The chosen module,
route order, and analytical arcs are ModSim-authored. The demonstration is
kinematic, gravity-free, and non-autonomous; it is not the exact 2019 sequence
and does not claim momentum-driven execution of twelve modules.

## Twelve-module one-plane physics sequence

Run the separate physics realization with the Runtime Inspector and native
MuJoCo viewer:

```bash
modsim runtime examples/robot_packs/mblocks_3d \
  --demo mblocks_physical_twelve_module_line \
  --speed 4
```

The endpoint topology is the same as the kinematic benchmark, but the
execution boundary is different. At simulation time zero, eleven fixed face
connections form the substrate and attach `block_12` above `block_1`.
`MomentumPivotSequenceScenario` then composes eleven independently checked
`MomentumPivotScenario` actions: ten 90-degree traverses across successive
support blocks followed by one 180-degree roll beside `block_11`.

For every action, the scenario:

1. pre-engages the coincident directed +Y edge hinge while the current face
   still holds the geometry;
2. settles and spins `block_12`'s flywheel toward 6,000 RPM for a quarter turn
   or 9,000 RPM for the final half-turn, using at most 0.03 N m;
3. releases the current fixed face and applies at most 2.6 N m of flywheel
   braking while MuJoCo integrates shell rotation, contact, gravity, and the
   retained hinge;
4. commits the authored target face from measured connector frames inside the
   1 mm position gate and within 20 degrees of the authored positive turn (the
   sign is equivalent at exactly 180 degrees); and
5. releases the temporary hinge, holds the new face, and activates the next
   action.

The moving face IDs cycle through `neg_z`, `pos_x`, `pos_z`, and `neg_x` as
the cube orientation advances. Its directed edge-hinge IDs cycle through the
four corresponding X- and Z-normal variants. This is why the one-plane pack
needs eight edge ports even though only one temporary hinge is active at a
time.

The scene may place roots only during initialization. Once the sequence is
created, it sends bounded flywheel effort and ordinary dock/undock requests;
it does not set module root poses or velocities and does not apply root
wrenches. `WorldState` remains canonical, and the physics lattice plus event
table expose the intermediate fixed/hinge topology. A successful complete run
has eleven final fixed connections, one assembly, and cells `(0, 0, 0)`
through `(11, 0, 0)`. Its event accounting is 33 successful docks (11 initial
faces plus one hinge and one target face per action) and 22 successful undocks
(one old face and one hinge per action).

The maintained end-to-end real-MuJoCo regression at the 0.0005 s demo default
verifies all 11 pivots, the exact 11 fixed-connection final topology, one
twelve-module assembly, those 33 dock and 22 undock events, cells `x = 0..11`,
and no node outside the physics lattice's configured 3 mm / 3 degree
tolerances. It replaces backend root-pose, root-twist, and root-wrench controls
with failing guards immediately after sequence creation, confirming that none
is used during the physical route. A separate 0.00025 s three-block regression
covers the first surface traversal; no full-sequence pose-residual or
repeatability range is claimed yet.

The CLI selects MuJoCo, gravity, an infinite ground plane, the grounded
physics lattice, a 0.0005 s solver/controller step, and a 12-second simulated
display budget. `--speed 4` changes only wall-clock pacing; it does not scale
the flywheel, controller, solver step, or event timestamps. The demo rejects a
non-MuJoCo backend, disabled gravity/ground, a root height below 25 mm, a step
above 0.0005 s, or generic connector/root-motion overrides.

This path demonstrates compositional physics execution, not full M-Blocks
hardware fidelity. The action order is ModSim-authored, target capture is a
deterministic constraint transition rather than a magnetic field, and every
pivot remains in one +Y plane. It neither reconstructs the unpublished 2019
hardware trace nor implements decentralized/autonomous planning.

## Twelve-module mat-to-staircase benchmark

The staircase adds the cyclic and multi-module operations that the line demo
does not exercise. Start with the deterministic reference:

```bash
modsim runtime examples/robot_packs/mblocks_3d \
  --backend mock \
  --demo mblocks_twelve_module_staircase \
  --no-viewer
```

The same reference route can use MuJoCo only as a mesh renderer and endpoint
constraint backend:

```bash
modsim runtime examples/robot_packs/mblocks_3d \
  --backend mujoco \
  --demo mblocks_twelve_module_staircase
```

Run the dynamics version with:

```bash
modsim runtime examples/robot_packs/mblocks_3d \
  --demo mblocks_physical_twelve_module_staircase \
  --speed 4
```

Both commands begin with the exact cells

```text
z=0:  A B C D E F       at y=0
      A B C D E F       at y=1
```

where each letter is a two-module Y slab. The persistent `pos_y ↔ neg_y`
connection within each slab and both row-matched X connections between
neighboring slabs give the mat 16 fixed face bonds. The goal is

```text
z=2:          C
z=1:        B A
z=0:      D E F
```

duplicated at `y=0` and `y=1`. Its exact topology has 18 fixed bonds: six
within slabs, four across the ground tier, two across the middle tier, and six
vertical bonds. The occupied goal cells are `(3,y,0)`, `(4,y,0)`, `(5,y,0)`,
`(4,y,1)`, `(5,y,1)`, and `(5,y,2)` for both Y rows.

The route contains eleven collision-checked positive rotations about +Y:

1. slab A makes one 180-degree lift and four 90-degree traverses;
2. slab B makes one 180-degree lift and two 90-degree traverses; and
3. slab C makes two 180-degree lifts and one 90-degree traverse.

At actions 8 and 9, a landing creates both bottom and side face bonds. Action
10 releases four faces at once before C climbs to the top tier. This requires
`CoordinatedPivotPlan`, whose topology validator accepts cycles but proves for
each authored action that the released connections detach exactly the declared
moving slab, leave the stationary structure connected, use each connector at
most once, and restore one connected structure at the target.

`CoordinatedKinematicPivotScenario` is the phase-one oracle. It releases every
cut face through the normal docking lifecycle, applies one analytical rigid
arc to both detached roots, and commits every landing face through the same
lifecycle. It completes near 25.22 simulated seconds with 42 dock commits, 24
undock commits, 11 runtime assembly splits, 11 landing merges, and the exact
18-bond goal. The initial construction contributes a further 11 assembly-merge
events.

`CoordinatedMomentumPivotScenario` is the phase-two executor. It pre-engages
one row-0 two-point hinge while the take-off faces still hold the slab, spins
both moving flywheels with independently bounded effort, releases the complete
face cut, and brakes both wheels synchronously. MuJoCo alone integrates the two
module roots. Once every authored landing pair simultaneously satisfies normal
connector acceptance and the 2.8 mm near-contact gate, all fixed faces commit
and the hinge releases. One hinge is deliberate: the retained Y-face weld
makes the slab rigid, while two collinear hinges would redundantly constrain
the MuJoCo system.

Quarter turns target 6,000 RPM. Ground-level half turns target 15,000 RPM, and
the elevated C-over-B half turn targets 19,500 RPM; all remain under the
published 20,000 RPM ceiling. Spin-up stays at or below 0.03 N m per flywheel
and the sampled mechanical-brake pulse stays at or below 2.6 N m per
flywheel. The backend reserves 24 weld slots and two hinge slots; at most 18
welds and one hinge are active.

The maintained real-MuJoCo regression at 0.5 ms completes the 11 actions near
9.50 simulated seconds without calling root-pose, root-twist, or root-wrench
controls. It ends with 18 fixed connections, one assembly, 53 dock commits
(16 initial faces, 11 hinges, and 26 landing faces), and 35 undock commits (24
take-off faces and 11 hinges). These timings and tuned targets are simulator
regression values, not hardware measurements.

The nearest lattice cells are correct at completion, but fixed-world residuals
in the reference run reached approximately 6.8 mm and 4.5 degrees because
measured welds preserve small capture and whole-assembly drift. The strict
`mblocks_physics_lattice` recipe therefore honestly marks some cubes amber.
This is a useful dynamics diagnostic, not a second source of configuration
truth; the measured poses and 18 committed bonds in `WorldState` remain
canonical.

The demonstration is inspired by the 2015 M-Blocks motion primitives and
pivoting-cube planning model. No primary paper publishes this exact mat-to-
staircase trace. It should be described as a ModSim-designed, paper-inspired
demonstration, not a reproduced experiment or an autonomous reconfiguration
result. Although the final object is two cells deep and three cells tall, all
motions are an extrusion of a single +Y actuation plane; arbitrary 3-axis
reconfiguration still requires the carrier model and additional directed
edges.

## Fidelity and known limitations

The source export and the published robot are not identical:

- The original Fusion export reported a 0.1628758277 kg CAD-derived total. The
  self-contained URDF and physics-oriented MJCF are both normalized to the
  published 0.150 kg reference; the original value remains pack metadata for
  provenance rather than an active inertial model.
- The detailed base mesh has a 53 mm visual envelope around a nominal 50 mm
  body. Two copies at 50 mm pitch can therefore show up to 3 mm of visual
  overlap, even though the collision boxes and connector frames meet exactly.
- The Fusion export's flywheel axis was diagonal and its source axial inertia
  was about `6e-6 kg m^2`. Both the pack URDF and MJCF deliberately replace
  those properties with a +Y axis and the published `8.4e-6 kg m^2` reference
  for one validated plane. The real 3D mechanism rotates its carrier among
  three actuation planes; that carrier motion remains absent.
- The declared `2.6 N m` effort is a hard peak bound corresponding to a short
  mechanical-brake impulse, not a continuous motor rating. The controller
  separately caps ordinary spin-up at 0.03 N m and disables braking at the
  sampled zero crossing.
- Connector tolerances, shear/bending limits, contact friction, joint damping,
  and armature are provisional simulation mappings.
- Face capture and edge transition are ideal runtime constraints, not a
  continuous permanent-magnet force model. Equality-row forces are available
  as scalar net-force estimates, but full connector moments/wrenches and
  force-selected magnetic breakaway are not modeled.
- The eight directed edge ports cover repeated rotations in the +Y plane only.
  The other two actuation planes and their directed edge approaches cannot yet
  request the corresponding physical pivots.
- Co-located X-normal and Z-normal ports at one modeled corner are distinct
  logical connector resources. The authored controller activates only one, but
  the schema does not yet provide an exclusivity-group field that prevents a
  generic caller from commanding both simultaneously.
- The opaque shell hides the internal flywheel in the ordinary external view.
- The source ROS package declares no usable redistribution license. The files
  were copied into this working tree at the user's direction, but public
  redistribution must be confirmed before they are published.

## Physics development gates

The current extension supplies the complete directed-edge catalog for the +Y
plane, a real three-cube 90-degree surface-traverse regression at 0.25 ms, a
passing eleven-action line regression at 0.5 ms, and a passing eleven-action
two-module-slab staircase regression at 0.5 ms. The important next validation
gate is broader full-route convergence, with recorded pose residuals, brake
impulses, contact sensitivity, and repeatability under small initial-pose
perturbations. Only the two-module primitive currently has 0.5, 0.25, and 0.1
ms convergence coverage. Until the same evidence exists for both composed
routes, the commands should be treated as deterministic engineering
demonstrations rather than quantitative evidence of hardware performance.

After that gate, useful fidelity increments are a real three-plane carrier,
identified flywheel/motor and contact parameters, a continuous or calibrated
magnetic capture/breakaway model, full force-and-moment telemetry, recovery
from failed capture, and a planner that schedules physically executable
primitives. The kinematic twelve-module command should remain as the fast
semantic reference even as the separate physics route matures.
