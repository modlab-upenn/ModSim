# Docking semantics 0.1

This document is the implemented contract for docking execution. `archive/HANDOFF.md`
is a roadmap; when the two differ, this document and the code under
`src/modsim/core`, `src/modsim/connectors`, `src/modsim/backends`, and
`src/modsim/runtime` describe what actually runs.

Format 0.1 covers the full semantic pipeline against the mock backend and fixed
connection execution under the optional MuJoCo adapter. Section 9 records the
implemented weld-pool bridge and its remaining physics limitations.

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
| `physical_connection_unsupported` | `hinge`, `ball`, or `custom` — not modelled in 0.1 |
| `command_required` | not both types are auto-latching and no command was issued |

Conflicting `physical_connection` intent is rejected rather than resolved. A pair
where one end wants a rigid weld and the other wants compliance has no defensible
default, so it is surfaced as an authoring error.

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

`modsim_backend_mujoco` implements the fixed-connection execution path. See
`docs/backends.md` for scene composition, naming, and options. What matters for
docking semantics:

**Done.** Connector frames are materialised as MuJoCo sites and reported in
`snapshot.connector_frames`, so acceptance is evaluated against frames measured
by the engine. Docking and undocking execute: a committed connection atomically
claims a reserved weld and a reserved contact-exclusion entry, re-points both at
the mating body pair, and activates the weld. Release deactivates the weld and
returns both entries to their pools. Authored static exclusions remain present,
and the combined signature array stays sorted for MuJoCo's collision-filter
lookup. The dynamic exclusion covers the exact two bodies constrained by the
weld, not every articulated body in their modules or assemblies. Pool
exhaustion refuses the connection rather than faking a latch.

**Not done**, in the order I would take them:

- **Constraint forces.** Report them in `snapshot.constraint_forces_n` and
  break-force release plus connector-load metrics — already implemented in core
  — start working with no further change.
- **Compliance.** `constraint: compliant` maps to the weld's `solref`/`solimp`,
  not to a separate constraint type. The schema's stiffness values are in
  physical units and `solref` is in time-constant form, so the conversion must be
  explicit and documented.
- **Weld chains are soft.** A long chain is measurably less stiff and slower
  than the equivalent compiled rigid body. An optional `mjSpec` recompile path
  that fuses a stable assembly into a real kinematic tree is worth adding *after*
  the weld path works, not instead of it.

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

## 10. Mock backend

`MockBackendAdapter` is deliberately kinematic, not dynamic. Bodies move at
whatever twist they are given; nothing falls and nothing collides. What it does
model faithfully is the part ModSim depends on: a physical connection makes two
modules move as one rigid body, and removing it lets them move independently.

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
hinge, ball, and custom physical connections
compliant connection translation in the MuJoCo adapter
connector load estimates beyond a single constraint-force magnitude
MuJoCo equality-constraint force reporting
position/velocity joint-command implementations and actuator catalogs
general or autonomous docking, approach, and reconfiguration planning
richer Studio lifecycle, contact-force, and metric-plot panels
```
