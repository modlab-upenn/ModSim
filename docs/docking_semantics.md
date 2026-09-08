# Docking semantics 0.1

This document is the implemented contract for docking execution. `archive/HANDOFF.md`
is a roadmap; when the two differ, this document and the code under
`src/modsim/core`, `src/modsim/connectors`, `src/modsim/backends`, and
`src/modsim/runtime` describe what actually runs.

Format 0.1 covers the full semantic pipeline against the mock backend and fixed
plus hinge connection execution under the optional MuJoCo adapter. Section 9
records the implemented reserved-constraint bridge and its remaining physics
limitations.

## 1. What ModSim owns and what a backend owns

ModSim decides *whether* two connectors should mate and *what it means* when
they do. A backend decides only whether the requested constraint physically
exists.

```text
ModSim owns:                       Backend owns:
  connector compatibility            rigid body integration
  acceptance-region evaluation       contacts and collision
  docking guards and policy          constraint solving
  lifecycle state machine            constraint forces
  logical connections                body and joint state
  assembly membership                rendering
  event log and metrics
```

A backend is never asked "is this a valid dock?" It is asked "create this
constraint," and it answers yes or no. That boundary is what keeps modular
semantics identical across the mock backend, MuJoCo, and anything later.

## 2. The per-step pipeline

`RuntimeSession.step` runs one pass:

```text
1. adapter.step(dt)                 backend advances physics
2. adapter.snapshot()               backend reports link poses, twists, forces
3. world.ingest(snapshot)           resolve connector world frames
4. overload evaluation              release connections past their break force
5. docking.run(world, adapter)      releases, then detection, then commits
```

Docking decisions are always made against the state the backend just reported,
never against a stale snapshot. Requested releases are processed before new
docks so that a connection cannot break and re-form within one step.

An authored kinematic transit can call
`RuntimeSession.step(dt, process_connectors=False)`. The session still owns the
backend step, snapshot ingestion, connector-frame resolution, and overload
releases, but it defers requested and passive connector processing until a
later pass. This is used when a route has an explicit docking endpoint and a
passive connector encountered slightly earlier along the analytical path must
not latch opportunistically. Ordinary simulation loops use the default
`process_connectors=True` pipeline above.

### 2.1 Connector frame resolution

A connector's world frame is resolved in this order:

1. `snapshot.connector_frames[module][connector]`, when the backend materialises
   connector frames natively (MuJoCo sites, USD prims);
2. otherwise the parent link's world pose composed with the authored
   `local_pose`.

Preferring a measured frame keeps ModSim and the backend from disagreeing about
where a connector is. The `connector_frame_map` in a backend mapping document is
what a backend uses to populate that field.

Docking and approach axes are authored in the **parent-link** frame, not the
connector frame, so they are rotated by the link's world rotation rather than by
the connector pose.

This convention does not apply to `physical_connection.hinge.axis`. A hinge
axis is expressed in **each connector's own frame**, because it describes a
line belonging to the mating interface rather than an approach direction
belonging to a link. Compatible hinge types must provide an identical axis and
`anchor_separation_m`; their connector poses determine the corresponding world
hinge line.

A connector's velocity includes the lever-arm term:

```text
v_connector = v_link + omega_link x (p_connector - p_link)
```

Dropping that term would make a spinning module look stationary at its
connectors and would let docks commit that should not.

### 2.2 Broad phase

Free connector positions go into a uniform spatial hash whose cell size is the
largest `position_tolerance_m` declared by any connector type in the pack. Only
connectors that are resolved and not engaged participate. Explicitly commanded
pairs are always evaluated, even when far outside the radius, so that a rejected
command produces an informative `DockFailed` rather than silence.

Candidate pairs are emitted in sorted order, so the same world state always
evaluates candidates in the same sequence.

## 3. Compatibility

Compatibility is a type-level question, answered before any geometry, because it
is far cheaper than acceptance and rejects most pairs outright.

- `compatible_with` must name the other type **in both directions**. A one-sided
  declaration is treated as incompatible: in a heterogeneous pack the omission is
  far more likely to be an authoring mistake than a deliberate asymmetry.
- Genders pair as `male`↔`female`, `hermaphroditic`↔`hermaphroditic`, and
  `genderless`↔`genderless`. Nothing else mates.

## 4. Acceptance

`evaluate_acceptance` returns a structured `AcceptanceResult`, not a boolean,
because both the Studio overlay and the `DockFailed` event need to say *which*
criterion failed and by how much. Four criteria are measured.

| Criterion | Measured | Tolerance |
|---|---|---|
| `position` | distance between connector origins | `position_tolerance_m` |
| `axis_alignment` | angle between docking axis A and **minus** docking axis B | `orientation_tolerance_rad` |
| `orientation` | roll error against the nearest allowed orientation | `orientation_tolerance_rad` |
| `relative_velocity` | speed difference at the two connector points | `max_relative_velocity_m_s` |

### 4.1 Effective tolerances

When two connector types declare different tolerances, the **stricter** value
governs each of the three quantities. A connector cannot become more forgiving
by being presented with a sloppier mate.

### 4.2 Acceptance shapes

`shape` is honoured rather than ignored, and is evaluated in each connector's own
frame. Both sides must be satisfied.

```text
sphere    |offset| <= tolerance
box       max(|x|, |y|, |z|) <= tolerance
cylinder  |axial| <= tolerance and radial <= tolerance
```

### 4.3 Axis convention

Two connectors mate when their docking axes are **antiparallel** — each points
into the other. The measured quantity is therefore the angle between `axis_a`
and `-axis_b`, which is zero for a correct mate.

### 4.4 Roll and orientation labelling

Roll is the signed angle about the shared docking axis, measured between
in-plane reference directions taken from each connector frame. The connector's
own x axis is preferred; when x is parallel to the docking axis the projection
degenerates and y, then z, are tried in turn.

The roll is snapped to the nearest value in `allowed_orientations.values_rad`.
Both sides are checked — side A against `roll`, side B against `-roll` — and the
larger error governs. A connector type with no declared orientation set, or a
`continuous` set, accepts any roll and reports zero error.

The matched **orientation index** is recorded on the committed connection.
Configuration recognition, topology equivalence, and reconfiguration planning all
depend on knowing which discrete mating state was used, and reconstructing it
after the fact is lossy.

## 5. Guards

Guards are the policy layer between "the geometry works" and "we should do it."
They are separate from acceptance so that a Studio preview can show a pair as
geometrically valid while the runtime still refuses to latch.

| Code | Meaning |
|---|---|
| `self_pair` | a connector cannot dock to itself |
| `same_module` | both connectors belong to one module |
| `already_engaged` | one connector already holds a connection |
| `cooldown` | `redock_cooldown_s` has not elapsed |
| `physical_connection_undefined` | neither type declares a connection, or the two conflict |
| `physical_connection_unsupported` | `ball` or `custom` — parameters not modelled in 0.1 |
| `command_required` | not both types are auto-latching and no command was issued |

Conflicting `physical_connection` intent is rejected rather than resolved. A pair
where one end wants a rigid weld and the other wants compliance has no defensible
default, so it is surfaced as an authoring error. Hinge ends must also declare
the same complete `HingeConstraintSpec`; a disagreement about axis or anchor
separation is rejected.

**Loop closure** — docking two modules that already share an assembly — is
*allowed* but flagged with a warning on the `GuardResult`. Closed kinematic
chains are legitimate modular robot configurations, and refusing them would rule
out rings and lattices; but a redundant constraint inside an already-rigid
assembly is also a common mistake, so it is reported.

Undocking has one guard: both connector types must declare
`supports_undocking: true`. A permanent connector stays permanent.

## 6. Two-phase commit

A logical connection is created **only after** the backend confirms the physical
constraint exists.

```text
1. re-check guards and acceptance against the current snapshot
2. build a ConnectionRequest
3. outcome = adapter.create_physical_connection(request)
4. success -> DockCommitted, then AssemblyMerged if two components joined
   failure -> DockFailed with reason BACKEND_REFUSED and the backend's detail
```

The resolved physical constraint type travels through this transaction. A
successful `DockCommitted` records it, `ConnectionRuntime` preserves it, and
topology/lattice view edges copy it into their immutable DTOs. Event replay and
generated graphs therefore retain whether an edge was fixed or hinged instead
of reconstructing that meaning from connector catalogs later.

If the backend reports `supports_runtime_constraints: false`, the attempt is
refused before it is made. ModSim degrades explicitly rather than silently
claiming connections that no physics engine is enforcing.

Failure sets both connectors to `FAILED` and starts their cooldown. `FAILED` is
cleared at the next `ingest`, so a rejected attempt can never strand a connector.

Within one pass a connector may commit at most once. Because proposals are
evaluated in sorted order, which pair wins is deterministic.

### 6.1 Event-only mutation

`WorldState` is mutated exclusively by applying events. This makes the event log
a complete and replayable description of what happened, and it means "why is this
module attached to that one?" is answerable from the log alone. Transient
detection states (`CANDIDATE_DETECTED`, `IN_ACCEPTANCE_REGION`) may be marked
without an event; anything that implies a logical connection may not.

Opportunistic near-misses are not logged. They occur every step for every nearby
pair and would swamp the log. Only commanded attempts report failure.

## 7. Alignment policy: measured or nominal

On latch, the relative pose frozen into the connection is either:

- **`measured`** (default) — the relative transform exactly as observed. This is
  physically honest and is what a docking-controller experiment wants.
- **`nominal`** — connector frames coincident, docking axes exactly antiparallel,
  roll set to the matched discrete orientation. Snapping keeps repeated
  reconfiguration from accumulating pose drift, at the cost of a small
  discontinuity at latch. This is what lattice and self-reconfiguration work
  wants.

Snapping is asymmetric on purpose: if *either* connector type asks for nominal
alignment, the connection snaps. A type authored for drift-free reconfiguration
is better honoured than overridden by a permissive mate.

The `snap_to_nominal` flag travels on the `ConnectionRequest`, so the backend
performs the actual repositioning. The world sees the snapped pose on the next
ingest, not in the step that committed the dock.

## 8. Undocking, assemblies, and load

Undocking removes the backend constraint first, then applies `UndockCommitted`,
then recomputes the affected component. Only that one component is walked, so
the cost is proportional to the assembly rather than to the whole world.

Assembly identifiers are **derived from membership**, not from a counter:

```text
AssemblyId = "assembly:" + min(member module ids)
```

The same physical configuration therefore always produces the same identifier
across runs, replays, and processes, which is what makes assembly identity
comparable in reconfiguration experiments that replay logs.

Every module always belongs to exactly one assembly, and a disconnected module is
a valid assembly of size one.

**Overload.** When a backend reports constraint forces and a connector type
declares `break_force_n`, a measured force above the limit emits
`ConnectorOverloaded` and releases the connection. The *lower* of the two
declared break forces governs: a joint is only as strong as its weakest half.

## 9. The MuJoCo backend

`modsim_backend_mujoco` implements fixed and hinge execution. See
`docs/backends.md` for scene composition, naming, and options. What matters for
docking semantics:

**Fixed connections.** A commit atomically claims one reserved weld and one
reserved contact-exclusion entry, re-points them at the mating body pair, and
activates the weld. Release returns both entries to their pools. Authored
static exclusions remain present, and the combined signature array stays
sorted for MuJoCo's collision-filter lookup. The dynamic exclusion covers the
exact two bodies constrained by the weld, not every articulated body in their
modules or assemblies.

**Hinge connections.** A commit claims one hinge slot containing two reserved
`mjEQ_CONNECT` equalities. Their point anchors are centred on each measured
connector origin and separated by the authored distance along the connector-
local hinge axis. Constraining two separated corresponding points removes
translation and all rotation except rotation about their common line. Unlike a
weld, a hinge does **not** claim a contact exclusion: collision remains active
so a pivoting M-Block can interact with its support and the ground. The
`hinge_pool_size` adapter option controls the number of simultaneously active
hinges; each slot reserves two equality rows. Pool exhaustion refuses the
connection instead of faking a latch.

**Constraint forces.** Snapshots now report scalar force magnitudes for every
active weld and hinge through `constraint_forces_n`. For a weld this is the
Euclidean norm of its three translational equality rows; rotational rows are
not mixed into a value whose unit is newtons. For a two-point hinge it is the
norm of the vector sum of the two three-axis point-force vectors. Their
difference also represents a moment, which is deliberately not folded into
this scalar field. These conventions feed the backend-neutral overload path;
they are constraint-solver reactions, not yet a complete connector wrench.

**Still deferred.** `constraint: compliant` needs an explicit conversion from
physical stiffness units into MuJoCo `solref`/`solimp`. Long weld chains also
remain softer than equivalent compiled rigid bodies; an optional `mjSpec`
recompile/fusion path is a later optimization. Ball/custom constraints and
full connector wrench telemetry remain unimplemented.

Two things real dynamics exposed that the mock hid. The relative-velocity
criterion now matters: an approach faster than the connector type's
`max_relative_velocity_m_s` is rejected, which is correct but is the most likely
cause of a scenario that "won't dock". And an auto-latching pair with no
`redock_cooldown_s` re-latches in the *same step* it is released, because a
docking pass evaluates releases before detection and the two halves are still
touching. Authoring a cooldown is the intended remedy.

### 9.1 Physical differential-drive demonstration

The SMORES-EP two-module physics demonstration is the first controller that
moves modules through articulated joints instead of writing root poses during
the approach. It starts two free modules once, settles them under gravity on a
ground plane, brakes the fixed module's wheels, holds pan/tilt, and drives the
moving module's left/right tire joints toward `bottom ↔ pan`. Measured connector
frames still pass through the ordinary compatibility, acceptance, guard, and
two-phase commit pipeline. A successful latch activates the same ideal fixed
weld as other MuJoCo demos. A controller-specific 1 mm near-contact gate keeps
the broad 6 mm acceptance tolerance from creating a visibly floating weld.
After an 80 ms face-deactivation delay, the weld is released and wheel effort
reverses the moving module.

The pack-local MJCF supplies simple tire and rear-skid contacts plus provisional
friction, damping, armature, effort limits, and controller gains. These are
stable bootstrap values, not identified SMORES-EP motor or tire parameters.
Magnetic attraction/capture forces and electrical EP-face state are not yet
modelled. See `runtime_inspector.md` for the command and `backends.md` for the
joint-command boundary.

The physical Driver-to-Snake demonstration extends the same boundary to seven
modules. It stages the initial six-edge tree once, then drives the four
published replacement actions solely through bounded joint efforts. The first
two actions move leaf modules; the last two allocate a common planar twist over
the wheels of a connected three-module component. Ordinary undock/dock requests
remain the only way graph edges and assemblies change. The authored navigation
routes are deterministic demo inputs, not autonomous planning or published
hardware trajectories.

### 9.2 M-Blocks face-to-hinge-to-face demonstration

The two-module M-Blocks physics bootstrap exercises both constraint kinds. At
time zero it stages a fixed face bond and a coincident +Y edge hinge. After
spin-up, it releases only the face, brakes the moving module's internal
flywheel, lets MuJoCo integrate the shell around the retained hinge, commits
the measured target face, then releases the hinge. Core event, connection, and
assembly state remain canonical throughout.

The twelve-module one-plane route composes that lifecycle rather than adding a
second docking mechanism. Its complete eleven-face starting tree is committed
in one docking pass at simulation time zero. For each of ten quarter turns and
one final half-turn, `MomentumPivotSequenceScenario` activates exactly one
directed edge hinge while the current fixed face still holds its geometry,
then delegates spin-up, fixed-face release, braking, measured target capture,
hinge release, and connected hold to `MomentumPivotScenario`. Capture requires
both ordinary connector acceptance and the authored pivot-angle/direction gate.
The target face
from one action is required to be the directed initial face of the next action.
Every explicit hinge/face commit and release verifies the complete expected
topology immediately, so a passive face that latches during the same docking
pass cannot remain hidden until the end of the action. A failed physical commit
or topology mismatch terminates the sequence instead of advancing to a
plausible-looking later state.

Only initialization may establish module root poses. After sequence creation,
the controller uses joint effort plus ordinary dock/undock requests; MuJoCo
owns every resulting root motion. A successful complete lifecycle contains 11
initial fixed commits and, per action, one hinge commit, one fixed-face
release, one target-face commit, and one hinge release. The canonical end state
therefore has 11 fixed connections and no active hinge.

Failure is diagnostic rather than transactional: the controller immediately
zeros flywheel effort and stops advancing, but preserves whatever physical
constraints and canonical events existed at the failure instant. This lets the
Runtime Inspector show the failed mechanical state. Session shutdown remains
the cleanup boundary; automatic rollback or recovery planning is not yet
implemented.

This deterministic constraint transition approximates the effect of passive
edge magnets; ModSim does not yet integrate a magnetic force-versus-distance
field or let loads choose which magnetic contacts break. No module root pose,
twist, or wrench is written after initialization. All current physical pivots
remain in the +Y plane; the published carrier's other two planes are not
modeled. See `mblocks_3d.md` for the reference parameters, commands, and
fidelity boundary.

## 10. Mock backend

`MockBackendAdapter` is deliberately kinematic, not dynamic. Bodies move at
whatever twist they are given; nothing falls and nothing collides. What it does
model faithfully is the part ModSim depends on: a physical connection makes two
modules move as one rigid body, and removing it lets them move independently.

The mock explicitly refuses `constraint: hinge`; treating it as a weld would
hide the rotational degree of freedom and make a physics test misleading. Use
MuJoCo for hinge execution. The refusal passes through ordinary two-phase
commit as `DockFailed(BACKEND_REFUSED)`.

Every link of a module reports the module pose, because the mock has no
articulated kinematics. Modules with internal joints therefore need a real
backend before their non-root connectors mean anything.

It also exposes `fail_next_connection` and `set_constraint_force` so the failure
and overload paths are testable without a physics engine.

## 11. Running a session

### 11.1 From the command line

```bash
modsim dock path/to/pack --count 3 --spacing 0.1
modsim dock path/to/pack --count 3 --undock
modsim dock path/to/pack --no-latch          # honour each type's auto_latch only
modsim dock path/to/pack --output json
modsim dock path/to/pack --backend mujoco --no-gravity
```

`modsim dock` is a one-shot authoring check: it places modules already in range
and asks whether they would latch. `modsim run` is the generic scripted
root-motion simulation; modules approach, latch, and release on a schedule:

```bash
modsim run path/to/pack --backend mujoco --count 2 --duration 6 --undock-at 4
modsim run path/to/pack --backend mujoco \
  --fixed-connector front --moving-connector rear --connector-gap 0.03
modsim run path/to/pack --backend mujoco --view          # real-time viewer
modsim run path/to/pack --gravity --ground --height 0.2  # let them settle first
modsim run path/to/pack --undock-at 4 --retract 0        # release without retracting
```

On macOS, `--view` must run under `mjpython` rather than `python`.

For the physical wheel/contact path, use the Runtime Inspector demonstration:

```bash
modsim runtime examples/robot_packs/smores_ep \
  --backend mujoco \
  --demo smores_diff_drive_dock_undock \
  --model-view smores_topology
```

### 11.2 Releasing is not separating

Worth stating plainly, because it surprises everyone once: **removing a
constraint does not push anything apart.** Two welded modules share a velocity,
so the instant the weld is released they keep coasting side by side, in contact,
at the same speed. The connection is gone — `UndockCommitted` and
`AssemblySplit` are both logged, and the assembly index shows two components —
but nothing moves relative to anything.

Separation is an *actuation* step, in simulation exactly as it would be on real
hardware. `modsim run` therefore backs the driven module away after release, at
`--retract` (defaulting to the approach speed). Passing `--retract 0` reproduces
the coast-together behaviour, which is itself worth seeing once.

`--latch` (the default) issues a dock command for every pair whose geometry
already satisfies acceptance, which is what makes the command useful on a pack
whose connectors are commanded rather than auto-latching. When nothing docks the
command reports whether the pairs were never within the detection radius, failed
a criterion, or were blocked by a guard.

### 11.3 From Python

```python
from pathlib import Path

from modsim import MockBackendAdapter, RobotPackLoader, RuntimeSession, SceneSpec
from modsim.core.ids import ConnectorInstanceId

pack = RobotPackLoader().load(Path("examples/robot_packs/generic_cube")).pack
scene = SceneSpec.grid("generic_cube", 3, spacing_m=0.1)
session = RuntimeSession.create(pack, scene, MockBackendAdapter())

session.request_dock(
    ConnectorInstanceId("generic_cube_0/front"),
    ConnectorInstanceId("generic_cube_1/rear"),
)
for event in session.step(0.01):
    print(event.kind, event.sequence)

print(session.world.assemblies.assemblies)
print(session.metrics().as_dict())
```

## 12. Not yet implemented

```text
an Isaac Sim adapter
articulated kinematics in the mock backend
ALIGNING and LOAD_BEARING lifecycle transitions driven by the engine
ball and custom physical connections
compliant connection translation in the MuJoCo adapter
connector load estimates beyond a single constraint-force magnitude
constraint moment/wrench reporting beyond scalar equality-force conventions
position/velocity joint-command implementations and actuator catalogs
general or autonomous docking, approach, and reconfiguration planning
richer Studio lifecycle, contact-force, and metric-plot panels
```
