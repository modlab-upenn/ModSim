# Continuous docking 0.1

**Status: Phase 1 implemented; later phases are draft.** The schema, validator,
core semantics and mock backend (§14.4) run and are tested. Sections and
bullets marked *(Phase 2)* to *(Phase 5)* are still draft contract. **§14
records the review decisions. Where §14 and an earlier section differ, §14
wins;** amendments that are implemented have been folded into the sections
they amend. The implemented parts are also stated in `docking_semantics.md`,
`backends.md`, `robot_pack_spec.md` and `model_views.md`.

Implemented in Phase 1:

| Area | Code |
|---|---|
| Schema and validator rules (§3) | `robot_packs/schema.py`, `robot_packs/validator.py` |
| Surface geometry and attachment state (§4, §6) | `core/surface.py`, `WorldState.surface_attachment` |
| Point-to-surface acceptance and broad phase (§4) | `connectors/surface.py` |
| Multi-occupancy, guards, side ordering (§5) | `core/entities.py`, `core/state.py`, `connectors/guards.py`, `connectors/docking.py` |
| Realisation selection and retarget contract (§7) | `connectors/docking.py`, `backends/base.py`, `backends/mock.py` |
| Per-axis break limits (§8.1) | `runtime/session.py` |
| View fields and protocol version 4 (§9) | `model_views/`, `runtime/inspection_protocol.py` |
| Slide driving (§11) | `runtime/surface_slide.py` |

Tests: `tests/test_continuous_docking.py`,
`tests/test_continuous_docking_schema.py`, `tests/test_surface_slide.py`,
`tests/test_break_limits.py`, `tests/test_mock_constraint_types.py` and
`tests/test_studio_surface_round_trip.py` (Studio edit, save and reload keep
the new fields), using the synthetic pack in `tests/surface_fixtures.py`.

Continuous (freeform) docking means a point connector can attach **anywhere**
on a surface connector, several point connectors can share one surface, and an
attached point connector can keep sliding over the surface. The first target is
FreeSN: a strut's magnet array attaches anywhere on a node's steel sphere.

Physical figures were checked against both FreeSN papers during review
(§14.1). The magnet force curve F(h) appears only as plots, so it still has to
be digitised before Phase 5.

## 1. Summary of the verdicts

| Hypothesis | Verdict | Where it changes |
|---|---|---|
| `interface: point \| surface`, sphere shape, `capacity`, `min_separation_rad` | **Holds** | §3 |
| Position error is the magnet height h = \|p − c\| − R | **Revised.** Acceptance uses the geometric *seat gap*. Magnet height is a separate quantity taken from the lift joint. | §4.1, §8 |
| Axis error is measured against the outward normal | **Holds**, but stated as *antiparallel* to keep the convention in §4.3 of `docking_semantics.md` | §4.1 |
| No roll criterion, because `continuous` already exists | **Holds** | §4.1 |
| Nominal snap projects onto the surface and keeps tangent position and spin | **Holds**, and the existing `nominal_relative_transform` already does it | §4.2 |
| `already_engaged` becomes capacity plus separation | **Holds, but larger than stated.** One-connection-per-connector is assumed in 14 places. | §5 |
| Attachment coordinates are measured state, not events | **Holds.** Stored as a frame, not as (θ, φ, ψ). | §6 |
| Broad phase on sphere centres, O(S + N) | **Holds.** It needs a separate point-to-surface query. | §4.3 |
| Per-axis overload from the connector-frame wrench | **Holds, but must be opt-in.** Existing packs already declare `limits`. | §8 |
| Topology view unchanged; 80-triangle quantisation | **Revised.** Edges gain an optional attachment field, which needs protocol version 4. Quantisation becomes its own builder. | §9 |
| Physics models live in core; backends see generic requests | **Holds.** A surface connection reaches the backend as an ordinary fixed or ball request. | §7 |
| Retarget primitive | **Holds**, as an optional protocol in the style of `SupportsJointCommands` | §7.2 |
| `ball` constraint | **Holds.** Implement the deferred `ball` type in general. | §7.3 |
| Link-wrench primitive | **Deferred** to `physical` mode (Phase 5) | §10 |
| Runtime contact-filter primitive | **Revised.** It belongs in the connection request, not in a separate call. Deferred until Phase 3 shows it is needed. | §7.4 |
| Contact forces per body pair "exist" | **Wrong.** `supports_contact_forces` is advertised, but no snapshot field or API carries contact forces. | §7.5 |
| `stepwise_fixed` fallback (re-create every k steps) | **Rejected.** It either breaks event-only mutation or spams dock, undock, split and merge events. It is replaced by `static_fixed`. | §7.1 |

Phase 0 found two existing defects. Both are fixed:

- The mock advertised `BALL`, `CUSTOM` and `COMPLIANT` but welded every one of
  them rigidly. Because the guards accept `compliant`, a compliant dock on the
  mock was silently faked as a weld (A6). The mock now advertises `{FIXED}` and
  refuses every other type with `DockFailed(BACKEND_REFUSED)`.
- `docking_semantics.md` §6 said a commit re-checks guards. The code reuses the
  proposal from the same pass, which is equivalent for point connectors only.
  Surface pairs are now re-guarded at commit (§5.2), and the contract says so.

## 2. What does not change

- **Event-only mutation.** `WorldState` changes only through events and
  `ingest`. Sliding writes no events (§6.3).
- **Two-phase commit.** A logical surface connection exists only after the
  backend accepts an ordinary fixed or ball request.
- **Deterministic ordering.** Candidate pairs are evaluated in sorted order.
  Several struts competing for one surface resolve in that order (§5.2).
- **Explicit degradation.** The runtime chooses a realisation from the
  backend's capabilities and records which one it used. A backend that lacks a
  primitive refuses it rather than faking it (§7.1).
- **Membership-derived assembly ids** and pair-derived connection ids
  (`connection_id(a, b)` in [ids.py](../src/modsim/core/ids.py)). A point
  connector holds at most one connection, so the pair id is unique while the
  connection is live, exactly as today.
- **Core imports no simulator.** Surface geometry, slide integration, the
  differential-drive model and break limits are all in `modsim`.
- **Point-to-point docking** keeps its current behaviour bit for bit. SMORES-EP
  and M-Blocks packs declare no surface connectors, and every new field is
  optional with a default that reproduces today's behaviour.

## 3. Schema

### 3.1 Connector types

```yaml
connector_types:
  freesn_shell:
    interface: surface                # new; default "point"
    gender: genderless
    compatible_with: [freesn_magnet]
    allowed_orientations: {mode: continuous, values_rad: []}
    surface:                          # required iff interface: surface
      shape: sphere                   # only value in 0.1; plane/cylinder reserved
      center_m: [0.0, 0.0, 0.0]       # in the connector frame
      radius_m: 0.060
      capacity: 12                    # both papers
      min_separation_rad: 1.0471975511965976   # 60 deg; see 14.1
    acceptance_region:
      shape: sphere
      position_tolerance_m: 0.003     # seat-gap tolerance, see 4.1
      orientation_tolerance_rad: 0.10 # tilt from the normal
      max_relative_velocity_m_s: 0.05
    supports_undocking: true

  freesn_magnet:
    gender: genderless
    compatible_with: [freesn_shell]
    allowed_orientations: {mode: continuous, values_rad: []}
    acceptance_region: {...}
    physical_connection:
      constraint: fixed
      surface_realizations: [driven_fixed, ball, static_fixed]   # new, ordered
    limits: {...}
    docking_policy:
      auto_latch: true
      alignment: nominal
      redock_cooldown_s: 0.2
      break_limits:                   # new, opt-in, see 8 and 14.1
        max_normal_force_n: 79.0      # seated on wheels (T-RO); NOT the 140-160 N contact pull
        max_shear_force_n: 49.0       # T-RO side direction (front: 54 N)
        max_bending_moment_nm: 1.6    # T-RO front direction (side: 1.8 N m)
        max_torsion_moment_nm: 1.3    # T-RO
```

Additions to `schema.py`:

| Model | Field | Rule |
|---|---|---|
| `ConnectorTypeSpec` | `interface: ConnectorInterface = POINT` | new `StrEnum` with values `point` and `surface` |
| `ConnectorTypeSpec` | `surface: SurfaceSpec \| None = None` | required iff `interface: surface`, forbidden otherwise (same pattern as `hinge`) |
| `SurfaceSpec` | `shape`, `center_m`, `radius_m > 0`, `capacity >= 1`, `min_separation_rad` in [0, π] | strict |
| `PhysicalConnectionSpec` | `surface_realizations: tuple[SurfaceRealization, ...] = ()` | non-empty and duplicate-free when present; allowed only with `constraint: fixed` |
| `PhysicalConnectionSpec` | `ball: BallConstraintSpec \| None` | required iff `constraint: ball` (§7.3) |
| `ConnectorLimits` | `max_torsion_moment_nm` | new and optional. The model has no torsion field today ([schema.py:385](../src/modsim/robot_packs/schema.py#L385)). |
| `DockingPolicySpec` | `break_limits: ConnectorLimits \| None = None` | opt-in per-axis breaking (§8) |
| `ConnectorSpec` | `magnet_height_joint: Identifier \| None` and `magnet_height_offset_m` (h₀, default 0) | optional prismatic joint whose position gives the magnet height h = h₀ − q (§8.2, A8) |
| `ConnectorSpec` | `surface_contact_links: tuple[str, ...] = ()` | links that touch a mated surface; reserved for contact exclusion (A4, Phase 2) |
| `ConnectorSpec` | `surface_drive: SurfaceDriveSpec \| None` | `model: differential`, `left_joint`, `right_joint`, `wheel_radius_m`, `track_m` (§11) |

**`ball` parameters are optional while authoring.** A bare `constraint: ball`
was already valid authoring data, and an existing test relies on it, so
`ball:` is not required by the schema. The simulation profile reports a ball
without its anchor as `connector.physical_connection_unsupported`, and the
guards refuse it, exactly as before. `ball:` is forbidden on any other
constraint.

**On a pair that includes a surface connector, the authored `constraint` must
be `fixed`** (A7). The constraint that actually runs comes from the chosen
entry in `surface_realizations` and is recorded on `DockCommitted.constraint`.
A pair that declares no `surface_realizations` is realised as `static_fixed`.

`SurfaceRealization` takes the values `driven_fixed`, `ball`, `static_fixed`
and `physical`. In Phase 1, `physical` is accepted by the schema but rejected
by the simulation profile.

### 3.2 Validator rules

- A surface type must declare `allowed_orientations` as `continuous`, or omit
  it.
- Two surface types are never compatible. Point-to-surface and point-to-point
  pairs are allowed.
- `surface_realizations` is meaningful only on a pair that includes a surface
  connector. On a point-only pair it is a warning in the authoring profile and
  an error in the simulation profile.
- Any authored constraint other than `fixed` on a surface pair is an error
  (`connector.surface_constraint_invalid`, A7). That includes `hinge`, whose
  line on a sphere is undefined.
- `physical` in `surface_realizations` is a warning in the authoring profile and
  an error in the simulation profile (`connector.surface_realization_unsupported`).
- `magnet_height_joint` must name a prismatic joint of the module, and the
  `surface_drive` joints must be revolute or continuous joints of the module.
- `max_torsion_moment_nm` is not a completeness requirement, so existing packs
  that declare normal, shear and bending only stay valid for simulation.
- A surface connector must be on the module's **root link**, or the simulation
  profile reports that the mock backend cannot place it. This is the existing
  mock limitation ([mock.py](../src/modsim/backends/mock.py) reports every link
  at the module pose), stated for this case.
- When both connector types declare `surface_realizations`, the lists must be
  identical. Otherwise the pair is rejected as conflicting, the same rule as
  for hinge parameters in `resolve_physical_connection`.

### 3.3 Where the realisation is authored (open question 4)

The preference list belongs on `physical_connection`, not on the connector type
body and not in backend options.

- It describes how the physical hold is realised. That is what
  `physical_connection` already describes, and the hinge precedent put its
  parameters there, together with the conflict rule.
- It is a hardware fact. FreeSN's worm-gear wheels cannot be back-driven, which
  is why `driven_fixed` is physically reasonable. A pack author knows this; a
  backend does not.
- Backend options are engine-specific tuning and must not change semantics.

*(Not in Phase 1.)* Experiments may override the list per scene with an
optional `SceneSpec.surface_realizations`, following the precedent of
`connection_budget`. §14.4 did not include it, so it is not implemented yet.
The runtime selects the first entry the backend supports and records it
(§7.1).

## 4. Point-to-surface geometry

### 4.1 Acceptance

Let the surface connector's world frame be S, the sphere centre
c = S·`center_m`, and the radius R. Let the point connector's origin be p and
its world docking axis â.

```text
n        = (p − c) / |p − c|          outward normal at the closest point
q        = c + R n                    attachment point on the surface
gap      = |p − c| − R                signed seat gap (geometric)
```

| Criterion | Measured | Tolerance |
|---|---|---|
| `position` | the seat gap, evaluated with each side's acceptance shape (below) | `position_tolerance_m` |
| `axis_alignment` | angle between â and **−n** (antiparallel, as in `docking_semantics.md` §4.3) | `orientation_tolerance_rad` |
| `orientation` | always zero error, because both types are `continuous`. Spin ψ is recorded (§6.1) | n/a |
| `relative_velocity` | \|v_p − v_S(q)\|, where v_S(q) is the surface body's velocity **at q**, including the lever arm | `max_relative_velocity_m_s` |

The point side applies its acceptance shape to the offset q − p, expressed in
its own frame, exactly as `_position_satisfied` does today. The surface side's
offset is purely radial by construction, so its shape reduces to
\|gap\| ≤ tolerance. The stricter-tolerance rule (§4.1 of
`docking_semantics.md`) is unchanged.

**Revision.** The hypothesis used the magnet height h as the position error.
For FreeSN, a strut is *seated* when its wheels and casters rest on the shell.
The magnets can then be anywhere in their −15 to +10 mm stroke. Two quantities
are therefore needed:

- the **seat gap**: geometric, measured at the point connector, and used for
  acceptance; and
- the **magnet height**: taken from the lift joint, and used for force and
  release (§8.2).

Authoring the FreeSN point connector on `strut_body` at the shell-contact apex
(`contact_upper` in the CAD notes) rather than on `magnet_array_*` has three
benefits. The seat gap becomes the true standoff. The connector works on the
mock, which has no articulated links. And a weld on `strut_body` leaves the
prismatic lift joint free, so the magnet height stays observable.

**Relative velocity must use v_S(q).** Today the connector velocity is the
parent body's velocity at the connector origin
([state.py:386](../src/modsim/core/state.py#L386)). For a shell connector that
origin is the sphere centre. A spinning node would look stationary there and
could latch a strut against a moving surface. This is the same defect that
`docking_semantics.md` §2.1 warns about for the lever-arm term.

### 4.2 Nominal snapping

The core builds a *virtual attachment frame* A_q at q. Its z axis is n, and its
x axis is the point connector's x axis projected onto the tangent plane, so the
measured spin is kept. A_q is expressed in the surface connector's **link**
frame and is used in place of the surface connector's `local_pose` everywhere
in the request (§7). The point side and A_q then form an ordinary point-to-point
pair:

- `measured` alignment freezes `pose_q.relative_to(pose_p)`;
- `nominal` alignment calls the existing `nominal_relative_transform` with
  A_q as the surface side. Because both types are `continuous`,
  `orientation_rad` is the measured roll, so snapping removes gap and tilt and
  keeps the tangent position and spin. **No new snapping code is needed.**

### 4.3 Broad phase

The existing hash ([broadphase.py](../src/modsim/connectors/broadphase.py))
pairs *point origins* within `position_tolerance_m`. A shell connector's origin
is the sphere centre, about 60 mm from any magnet, so the existing hash never
pairs them. Surface connectors are therefore removed from the point hash, and
`candidate_surface_pairs` is added:

- insert the centre of every surface connector that has spare capacity, with
  cell size max(R) + max(`position_tolerance_m`) over the pack;
- query the 27 neighbouring cells for every resolved free point connector whose
  type lists a surface type in `compatible_with`, and keep the pairs with
  \|p − c\| ≤ R + tolerance;
- emit the pairs in sorted order and merge them with the point pairs and the
  requested pairs before evaluation.

The expected cost is O(S + N) per step, for S point connectors and N surfaces.
Explicitly requested pairs are still always evaluated.

## 5. Multi-occupancy

### 5.1 Every place that assumes one connection per connector (open question 3)

| # | Location | Assumption | Change |
|---|---|---|---|
| 1 | `ConnectorInstance.connection_id` ([entities.py:240](../src/modsim/core/entities.py#L240)) | single slot | add `connection_ids: tuple[ConnectionId, ...]` (sorted). `connection_id` becomes a read-only property that returns the sole id or `None`. |
| 2 | `ConnectorInstance.is_engaged` / `ENGAGED_STATES` ([entities.py:63](../src/modsim/core/entities.py#L63), [358](../src/modsim/core/entities.py#L358)) | engaged means unavailable | keep `is_engaged` meaning "holds at least one connection". Add `has_capacity`, which is true for a free point connector or for a surface connector below capacity. Rows 3 to 9 switch to `has_capacity`. |
| 3 | `_apply_dock_committed` raises if either side is engaged ([state.py:460](../src/modsim/core/state.py#L460)) | | raise if `not has_capacity` |
| 4 | `_apply_dock_committed` sets `connection_id` and `DOCKED` | | append to `connection_ids`. A surface connector is `DOCKED` while it holds at least one connection. |
| 5 | `_apply_undock_committed` frees both ends and starts both cooldowns ([state.py:486-495](../src/modsim/core/state.py#L486-L495)) | | remove the id. A surface connector returns to `FREE` only at zero connections. **The cooldown applies to the point side only**, so one strut leaving does not lock out the other eleven. |
| 6 | `_apply_dock_failed` sets `FAILED` and the cooldown on both ends ([state.py:505](../src/modsim/core/state.py#L505)) | | point side only, for the same reason |
| 7 | `mark_lifecycle` ignores engaged connectors ([state.py:559](../src/modsim/core/state.py#L559)) | | unchanged. A surface connector with connections shows `DOCKED` and does not flicker through detection states. |
| 8 | `evaluate_dock_guards`, `already_engaged` ([guards.py:100](../src/modsim/connectors/guards.py#L100)) | | split into `already_engaged` (point side) and the new codes `surface_full` and `surface_separation` (§5.2). The cooldown guard is checked on the point side only. |
| 9 | `DockingManager.detect` broad-phase filter ([docking.py:146](../src/modsim/connectors/docking.py#L146)) | | point hash uses point connectors only; surface hash uses `has_capacity` (§4.3) |
| 10 | `DockingManager.run` "one commit per connector per pass" ([docking.py:419](../src/modsim/connectors/docking.py#L419)) | | still one per point connector. A surface connector accepts several per pass, each re-guarded against commits earlier in the pass (§5.2). |
| 11 | `DockingManager._is_stale` drops a request if either end is engaged ([docking.py:435](../src/modsim/connectors/docking.py#L435)) | | stale if the point end is engaged or the surface is full |
| 12 | `free_connectors` ([docking.py:485](../src/modsim/connectors/docking.py#L485)) | | returns connectors with `has_capacity` |
| 13 | `collect_metrics`, `connector_count_docked` ([metrics.py:85](../src/modsim/runtime/metrics.py#L85)) | counts lifecycle states | unchanged in meaning (connectors, not slots). Add `surface_slots_occupied`. |
| 14 | Lattice view face labels use the connector's docking axis ([lattice.py](../src/modsim/model_views/lattice.py)) | | a surface endpoint gets no face label. The surface has no docking axis of its own. |

Places where the assumption does **not** appear: `connection_id()`, the
`_link`/`_unlink` neighbour counts (parallel edges between two modules are
already supported), `AssemblyIndex`, the topology view (it keys edges by
connection id and already allows parallel edges), the online planners (they
key on connection ids), and the session's overload loop.

### 5.2 Capacity and separation guards

For a surface connector C and a candidate attachment normal n:

```text
surface_full        len(C.connection_ids) >= capacity
surface_separation  min over existing connections k on C of angle(n, n_k) < min_separation_rad
```

Here n_k is the **measured** attachment normal of connection k from the current
sample (§6). Proposals from one pass are evaluated in sorted order. Before a
surface pair commits, `commit` re-evaluates both guards against
`connection_ids` as they stand, including commits made earlier in the same pass.
Of two struts that reach the same spot in the same step, the one first in
sorted order wins, and the other is refused with `surface_separation`. That
refusal is logged only if the dock was commanded, as for every other near-miss.

### 5.3 Side ordering

For a surface connection, **side A is always the point connector**. This holds
in `DockProposal`, `ConnectionRequest`, `DockCommitted` and `ConnectionRuntime`.
The connection id is still sorted (`connection_id` sorts internally), so ids
are unaffected. The reasons:

- The wrench is reported about connector A's origin
  (`ConstraintWrench` docstring; MuJoCo stores that origin once at commit in
  `_wrench_reference`, [adapter.py:840](../src/modsim_backend_mujoco/adapter.py#L840)).
  With the point side as A, the reference point is fixed in the strut and
  never moves when the strut slides.
- `wrench.resolved_in(point_connector_frame)` then gives normal, shear,
  bending and torsion directly in the magnet frame, which is the frame the
  break limits are written in.

The views already sort endpoints themselves
([topology.py `_connection_edge`](../src/modsim/model_views/topology.py)).
Phase 1 confirmed that no consumer requires `connector_a < connector_b`: the
only ordering comparison is `DockingManager._canonical`, which keys requested
pairs, and planners key on connection ids. The requested-pair and detection
sets keep canonical sorted keys; only the proposal and everything after it use
point-first order.

**Snapping moves side A for a surface pair.** Both backends' nominal snap moved
module B, which for a surface pair is the node and would drag every other strut
on it. `ConnectionRequest.snap_side` (default `"b"`) is set to `"a"` for surface
pairs, so the strut moves. The MuJoCo adapter refuses side-A snapping until
Phase 3 implements it, rather than moving the wrong module.

**A surface snap happens only when the strut is alone.** Moving side A is safe
only when side A's assembly is the strut by itself. If the strut already
belongs to a larger assembly (for example it is docked to another node), the
snap would drag that structure or, when the dock closes a loop such as a
triangle of nodes and struts, could not be done at all. In that case the
commit keeps the **measured** pose: `DockCommitted.alignment` is `measured` and
`DockCommitted.nominal_alignment_withheld` is true. Every `DockCommitted` now
records the alignment it actually used. `alignment` describes the committed
pose; `nominal_alignment_withheld` is true exactly when nominal was requested
and the pose is measured, so it is never true together with
`alignment: nominal`. `docking_semantics.md` §7 tabulates the three cases. (The mock still refuses a side-A snap
that would move a weld loop, as a defence; the core no longer asks for one.)

## 6. Attachment state

### 6.1 Representation

```python
@dataclass(frozen=True, slots=True)
class SurfaceAttachment:
    normal_local: Vec3  # n in the surface connector frame
    frame_local: Transform  # A_q in the surface connector frame (spin included)
    seat_gap_m: float
    magnet_height_m: float | None  # from magnet_height_joint, if declared
```

`polar_rad`, `azimuth_rad`, `spin_rad` and `spin_reference_degenerate` are
derived properties of `SurfaceAttachment` (`modsim.core.surface`).

(θ, φ, ψ) are **derived display coordinates**, not canonical state. The
hypothesis stored (θ, φ, ψ). That chart is singular at the poles. Worse, by
the hairy-ball theorem *no* tangent reference for ψ is continuous over the
whole sphere. A unit normal plus a frame has no singularity. The view DTO
reports θ and φ, and reports ψ against a documented reference (the surface
frame's x axis projected onto the tangent plane, falling back to y near ±x),
together with a flag when that reference is near-degenerate.

### 6.2 Where it lives (open question 2)

Attachment state lives in **`WorldState`**, as a lazily resolved accessor over
the current sample: `world.surface_attachment(connection_id)`. It is computed
from the two connector frames already resolved lazily, and cached per sample
stamp, so a step that reads no attachment pays nothing. It is not stored on
`ConnectionRuntime` (a plain dataclass whose fields come only from events) and
not on the snapshot (backends must not need surface semantics).

`ConnectionRuntime.relative_transform` keeps its current meaning: the pose
committed by `DockCommitted`. It is **not** rewritten as the strut slides,
because only events may write it. The live relative pose is the measured
attachment.

The attachment reaches the Runtime Inspector through the view DTO (§9), not
through a new frame field. That keeps the rule that the inspector shows
generated views of canonical state.

### 6.3 Events (open question 5)

Sliding writes **no** events: no per-step events, and no slide start or stop
events.

- The attachment comes from samples. Like a module pose, it can be reproduced
  from events plus samples, which is the replay model today.
- A slide command is controller intent, like a joint command, and joint
  commands are not logged either. The precedent from the planning work is that
  intent travels separately from canonical state (protocol version 3 carries
  planner intent beside the event log, not inside it).
- "Slide started" or "slide stopped" would have to be inferred from a velocity
  threshold, which is a presentation choice. A view or the scenario status can
  show it without changing the log.

`DockCommitted` gains `realization: SurfaceRealization | None = None` (§7.1)
and `attachment: SurfaceAttachment | None = None`, holding the attachment at
commit time. `ConnectionRuntime` copies them as `realization` and
`committed_attachment`. `UndockCommitted` is unchanged. The attachment at
commit is measured before any nominal snap; the snapped pose appears in the
live attachment from the next sample.

## 7. Backend contract

### 7.1 Realisations and degradation

| Realisation | Backend constraint | Capability required | Behaviour |
|---|---|---|---|
| `driven_fixed` | `FIXED` + retarget | `FIXED` in `supported_constraint_types` and `supports_connection_retarget` | Core integrates slide commands into a target attachment each step and sends one batched retarget. |
| `ball` | `BALL` anchored at the sphere centre | `BALL` in `supported_constraint_types` | Passive slide and spin about the centre under external load. Not wheel-driven in 0.1 (A3). |
| `static_fixed` | `FIXED` | `FIXED` | No sliding. Slide commands for this connection are refused with `SurfaceSlideError`. |
| `physical` | none (core force field) | `supports_link_wrench` | Phase 5 (§10) |

At commit, the runtime tries the realisations in the resolved list whose
capability the backend advertises, in order. A backend refusal falls through
to the next one (A4), and only the last refusal fails the dock, with every
refusal in the detail. If no listed realisation is advertised, the dock fails
with `DockFailed(BACKEND_REFUSED)` and a detail naming the list and the
backend.
The chosen realisation is recorded on `DockCommitted` and `ConnectionRuntime`.
`DockCommitted.constraint` records the executed constraint type (`fixed` or
`ball`), so views and replay see what physically existed.

**Why `stepwise_fixed` is dropped.** Re-creating the constraint every k steps
changes its backend handle. A handle that changes outside an event breaks
event-only mutation. A handle change made through events produces
`UndockCommitted`, `AssemblySplit`, `DockCommitted` and `AssemblyMerged` every
k steps, which floods the log and changes the topology revision. Between the
remove and the create, nothing holds the strut. `static_fixed` is the honest
fallback, and the mock implements retarget cheaply (§7.2), so the slide
semantics can still be tested without a physics engine.

### 7.2 Retarget

This follows the existing pattern: a capability flag plus an optional protocol,
as for `SupportsJointCommands` and `SupportsModuleKinematics`. The seven-method
`BackendAdapter` is unchanged.

```python
@dataclass(frozen=True, slots=True)
class ConnectionRetarget:
    handle: ConstraintHandle
    connector_a_local: Transform
    connector_b_local: Transform  # for surfaces: the new A_q in link B
    relative_transform: Transform


@runtime_checkable
class SupportsConnectionRetarget(Protocol):
    def retarget_physical_connections(
        self, updates: tuple[ConnectionRetarget, ...]
    ) -> tuple[bool, ...]: ...
```

- **One call per step** for every sliding connection.
- **Generic.** It retargets any fixed connection, not only surface
  connections.
- **Timing.** A retarget is a connection change under rule 5. It acts within
  `connection_change_delay_substeps` of the next step.
- **Atomic, as joint commands are.** The whole batch is validated before any
  update is applied. Each update gets a flag; `False` marks an update whose
  handle is not held or that the backend refused, and then **no** update in the
  batch was applied.
- **Refusal handling.** The slide driver drops the refused updates, resends the
  rest, stops sliding the refused connections and reports them in
  `RuntimeSession.surface_slide_report.refusals`. It does not raise: a refused
  loop retarget on the mock is a legitimate limitation, not an adapter bug. A
  batch whose result count does not match raises `BackendError`.

**MuJoCo** *(Phase 3)*. Rewrite the weld's `eq_data` relpose, which is
`body_relative_transform` of the three new transforms. This is O(1) per
connection, needs no recompile, and sets `_rows_stale`. The wrench reference is
unchanged because side A is the point side (§5.3).

**Mock.** The mock moves the connected component that contains **side A** (the
sliding strut), once this weld is ignored, as a rigid body so the commanded
relative pose holds. Side B and everything else welded to it stay put, so the
node and its other struts do not move (A1). It refuses a retarget when both
sides stay connected without this weld (a loop), and
`MockBackendAdapter.refusal_reason` explains why.

### 7.3 Ball (open question 6)

Implement the deferred `ball` type in general, rather than adding a
surface-only primitive:

```yaml
physical_connection:
  constraint: ball
  ball:
    anchor_m: [0.0, 0.0, 0.0]  # in connector A's frame
```

`anchor_m` is the pivot in **connector A's frame**; connector B's anchor is the
same point carried by the committed relative transform. The draft said "in
each endpoint connector frame", which is ambiguous when the two frames are not
coincident. For a surface realisation, the core builds the spec itself: the
anchor is the sphere centre expressed in the point connector's frame at commit.
The backend therefore sees an ordinary ball request.

**`ball` is free-sliding only in 0.1** (A3). A ball holds the radial distance,
so nothing presses the wheels onto the shell, and wheel friction has no normal
load. A `ball` connection slides and spins passively under external load;
`set_surface_slide` on it raises `SurfaceSlideError`. Wheel-driven `ball` needs
the magnet preload and therefore the link-wrench primitive (Phase 5).

**MuJoCo** *(Phase 3)*. A new pool of single `mjEQ_CONNECT` equalities, sized by the
already-existing `ConnectionBudget.ball`. Wrenches reuse the connect-row
capture used for hinges. Like hinges, ball constraints keep body collision,
which the wheels need for traction.

**Mock.** Refuse `ball`, as it refuses `hinge`. A kinematic backend cannot
represent the free rotation.

The other existing types are not used here. `compliant` remains deferred: it
needs the `solref` conversion described in `docking_semantics.md` §9. `custom`
is not used.

### 7.4 Contact exclusion *(Phase 2; decided by A4)*

Under `driven_fixed`, any wheel contact that does not match the odometry loads
the weld, and that load feeds the per-axis break check. So:

- `ConnectionRequest.extra_contact_exclusions: tuple[tuple[str, str], ...]` is
  added in Phase 2, defaulting to `()`;
- for a `driven_fixed` surface connection, the core fills it with every
  (point-side link, surface link) pair whose point-side link is listed in
  `ConnectorSpec.surface_contact_links` (FreeSN: the four wheels and the strut
  body). The schema field exists from Phase 1; nothing consumes it yet;
- a backend that cannot honour the exclusions refuses the request, and the
  runtime falls through to the next realisation (already implemented, §7.1);
- Phase 3 measures the residual weld wrench during a steady slide as a
  regression check.

### 7.5 Contact forces

`BackendCapabilities.supports_contact_forces` exists and MuJoCo sets it, but
`BackendStateSnapshot` deliberately carries no contacts
([snapshot.py](../src/modsim/core/snapshot.py)). The hypothesis's "load on a
connection that has no constraint" would need a new snapshot field. It is
needed only in `physical` mode and is deferred to Phase 5. Until then the flag
should be documented as "reserved", because no current consumer can read
contact forces.

### 7.6 Changes to `ConnectionRequest`

These are defaulted fields after the existing ones, as was done for hinges:

```python
surface_realization: SurfaceRealization | None = None
surface_center_link_b: Vec3 | None = None  # informational; ball anchors are in the spec
```

`connector_b_local` carries A_q, `physical_connection` carries the realised
spec (`fixed`, or `ball` with its anchor), and A is the point side. A backend
that ignores the two new fields still builds the correct constraint. A third
field, `snap_side: Literal["a", "b"] = "b"`, says which side a nominal snap
moves (§5.3); a backend that cannot snap side A must refuse such a request.

**Open question 1, answered.** Reuse `ConnectionRequest`. The hinge
precedent added only a sub-spec under `physical_connection` and one defaulted
field on the event and on the runtime record. It did not add a sibling request
type. A surface connection reduces to a fixed or ball request between a real
connector frame and a virtual one. A sibling type would make every backend
branch on it for no physical difference.

### 7.7 Capabilities

| Field | mock | MuJoCo |
|---|---|---|
| `supports_connection_retarget` (new) | yes | not yet (Phase 3) |
| `supported_constraint_types` | `{FIXED}` | `{FIXED, HINGE}`; `BALL` in Phase 3 |
| `supports_link_wrench` (new, Phase 5) | no | Phase 5 |

`BackendRequirements` gains `surface_slide`, which only
`supports_connection_retarget` satisfies (A3).

## 8. Load and release

### 8.1 Per-axis break limits

`DockingPolicySpec.break_limits` is a new, **opt-in** field. The existing
`ConnectorLimits` is not reused for breaking, because the generic cube,
SMORES-EP and M-Blocks packs already declare `limits` as datasheet
documentation. M-Blocks declares a 23 N normal limit, and consuming it could
break the existing hinge pivots. This has not been measured. With `break_limits`
declared, `RuntimeSession._evaluate_overloads` decomposes the wrench about
connector A's **world docking axis** â (the axis is authored in the parent-link
frame and need not be the connector frame's z):

```text
normal   |F·â|            shear    |F − (F·â)â|
torsion  |M·â|            bending  |M − (M·â)â|
```

The whole-force `break_force_n` check runs first, as before. For each axis
quantity, the lower limit of the two sides governs. The first exceeded quantity
is reported as `ConnectorOverloaded(quantity=...)` with values in that
quantity's unit, and the connection is released. The wrench is read only when
`break_limits` are declared, because a backend may compute moments lazily.
`ConnectorOverloaded.quantity` defaults to `"force_magnitude"`, so existing
events keep their meaning, and a per-axis event does not overwrite
`ConnectionRuntime.measured_force_n`.

Checking a moment limit needs `supports_constraint_torques`. On a backend
without it, a moment limit is a `BackendError` from `RuntimeSession.create`,
raised before the backend loads, not a silently skipped check. The mock now
reports `torque_nm=None`, matching its capabilities; `set_constraint_force`
also accepts a world-frame force vector.

**Every exceeded axis releases the connection; that is an approximation**
(A5). Real FreeSN slips when shear or torsion exceeds its limit and detaches
only under excess normal force or bending. Slip under load is out of scope
until `physical` mode.

### 8.2 Height-dependent hold (optional in Phase 3, required in Phase 5)

If the point connector declares `magnet_height_joint`, the magnet height
h = h₀ − q_lift is read from that joint's state, with h₀ the defaulted
`magnet_height_offset_m` (0 for FreeSN, whose CAD zero pose has h = 0). Phase 1
implements the reading (`WorldState.magnet_height_m` and
`SurfaceAttachment.magnet_height_m`); the force curve below is Phase 5. The sign follows the CAD
notes: positive lift protrudes the magnet. The axial limit becomes F(h), and
the shear limit becomes μ·F(h). Retracting the magnets therefore releases the
connection through the ordinary overload path. In constrained realisations the
geometric gap is pinned by the constraint, so the joint is the only valid
source. F(h) is a tabulated curve on the connector type
(`magnet_force_curve: [[h_m, force_n], ...]`, interpolated monotonically). The
values must be digitised from the force plots (ICRA Fig. 3, T-RO Fig. 2). The
text gives only anchors: about 140–160 N with the magnets touching the shell,
about 79 N normal strength when seated on the wheels, and under 30 N by about
8 mm (§14.1).

## 9. Views and the Runtime Inspector

- `DockedConnectionEdge` gains `attachment: SurfaceAttachmentDTO | None = None`
  (normal in the surface frame, θ and φ, ψ with a degeneracy flag, seat gap,
  magnet height) and `realization: SurfaceRealization | None = None`. Both are
  filled for topology and lattice edges. A lattice edge's surface endpoint has
  no face label (`source_face`/`target_face` and their residuals are `None`). Edge and node sets are
  unchanged. The topology builder's cache token already includes
  `sample_sequence`, so a sliding attachment refreshes the view without a
  topology revision.
- Model-view DTOs and protocol messages are `extra="forbid"`
  ([models.py](../src/modsim/model_views/models.py),
  [inspection_protocol.py](../src/modsim/runtime/inspection_protocol.py)), so
  adding these fields changes the wire format. **The Runtime Inspector protocol
  moves to version 4.** The framing tests in
  `tests/test_runtime_inspection_protocol.py` are updated accordingly.
- *(Deferred to Phase 2.)* Discrete identity is a separate builder,
  `spherical_attachment`, configured
  with `subdivision: icosphere_2`. A frequency-2 icosphere has 42 vertices and
  80 faces, which matches the T-RO paper's 42 sensors and 80 spherical
  triangles (confirmed, §14.1).
  The builder reports each attachment's triangle index and residual and derives
  its result without writing to `WorldState`, as the cubic lattice does.
- Rendering the attachment in the inspector (a sphere glyph with markers) is
  outside Phases 1 to 3. Until then the edge tooltip shows the DTO fields.

## 10. `physical` mode (Phase 5, needs its own review)

With no backend constraint, the two-phase commit has nothing to confirm. The
proposal: the logical connection is backed by a core-owned force element whose
handle is `field:<connection_id>`. Commit succeeds only if the backend
advertises `supports_link_wrench` and accepts the first wrench batch. Each step,
the core applies F(h) along −n at the magnet link through one batched
`apply_link_wrenches` call. Load needs either contact forces (§7.5) or an
estimate from the applied field. Because a force is held constant across a
step's substeps, this mode is valid only below a timestep bound that the
runtime checks at session creation. This section is a placeholder that marks
where the contract tension lies, not a design.

## 11. Slide driving

The core holds a `SurfaceSlideDriver` beside the `DockingManager` in
`RuntimeSession`. It is not part of `WorldState`.

- `session.set_surface_slide(connection_id, SurfaceSlideCommand)` takes a
  backend-neutral command: a tangential velocity in the attachment frame (m/s)
  and a spin rate (rad/s). It is validated like a joint command. An unknown
  connection, a non-sliding realisation or non-finite values raise
  `SurfaceSlideError`.
- Only `driven_fixed` connections accept commands. `static_fixed` and `ball`
  raise `SurfaceSlideError` (A3), as does a backend without
  `supports_connection_retarget`. Commands persist until
  `clear_surface_slide` or until the connection is released. A command
  belongs to the connection it was set for, not to the connector pair: a
  release followed by a re-dock of the same pair ends it, even within one
  docking pass. Connection ids and backend handles are both derived from the
  pair, so the driver drops a command when an `UndockCommitted` for its id is
  among the step's events, or when the connection now present has a different
  `created_at_s` (a release outside `step`, such as a direct
  `process_docking`).
- A drive model on the point `ConnectorSpec` turns wheel joint rates into that
  command: `surface_drive: {model: differential, left_joint, right_joint,
  wheel_radius_m, track_m}`. Phase 1 provides it as the pure function
  `differential_slide_command`; nothing feeds it measured wheel rates until
  Phase 3. The FreeSN geometry is set in Phase 4.
- Step order: `adapter.step` → `snapshot` → `ingest` → overloads → docking pass
  → **slide update**. The slide update integrates each `driven_fixed` command
  on the sphere (a rotation about the centre plus a spin about the normal) and
  sends one retarget batch. Running after the docking pass means connections
  released in this step are skipped, and connections committed in this step
  start sliding at the next one. The world measures each retarget at the next
  ingest. The commanded target is integrated from the measured attachment at
  the time the command was set.

**Sliding is guarded** (A2). Before sending a batch, the update checks each
target against the other connections on the same surface: their new targets if
they were processed earlier in this step, otherwise their measured attachments.

- A target inside `min_separation_rad` of another connection is clamped to the
  boundary along the commanded great circle (found by bisection, so it never
  lands inside). The spin part of the command still applies.
- An attachment already inside the boundary (for example after physics pushed
  it there) may only move away; a move that would bring it closer is dropped.
- Commands are processed in sorted connection-id order, so two struts closing
  on each other in one step give the same result on every run.

Clamps are reported in `RuntimeSession.surface_slide_report.clamps`, not as
events; like the command, a clamp is controller-side (§6.3).

## 12. Test plan

**Phase 1: schema, core and mock** (`tests/test_continuous_docking.py`, with a
synthetic in-memory pack: a sphere module with one root-link `shell`, and a
strut module with two root-link point connectors)

- schema and validator: every rule in §3.2, both profiles, YAML round-trip
  through `RobotPackWriter`;
- acceptance: seat gap on both sides of zero, tilt at the tolerance, velocity
  using v_S(q) with a spinning sphere (it must reject when the origin velocity
  alone would accept), and the stricter tolerance applied across mixed types;
- nominal snap keeps the tangent position and spin to 1e-9;
- broad phase: point-to-surface pairs found, far pairs not found, requested
  pairs always evaluated, deterministic order, and a scaling check that per-step
  work grows linearly with the number of struts;
- multi-occupancy: fill to capacity, then `surface_full`; separation just
  inside and just outside the limit; two struts on one spot in one pass, where
  the first in sorted order wins; undocking one of three leaves the shell
  `DOCKED`; cooldown on the strut only; `DockFailed` does not cool the shell
  down;
- realisation selection and degradation: a list with no supported entry gives
  `DockFailed(BACKEND_REFUSED)`; `static_fixed` refuses slide commands; the
  chosen realisation appears on the event, the runtime record and the view
  edge;
- side ordering: A is the point side even when its id sorts after the shell's;
- event log: a 1,000-step slide adds zero events; replaying the events plus the
  samples reproduces the attachments;
- the mock no longer advertises `ball`, `custom` or `compliant`. No existing
  test relied on those flags, so none was changed for A6.

All of the Phase 1 items above, and the amendment tests listed in §14.4, are
implemented.

**Phase 2: contract and conformance** (`tests/test_backend_conformance.py`)

- a second fixture: the synthetic sphere/strut pack. Both backends agree on the
  surface-pair proposals, the committed sequence, the realisation chosen (when
  their capabilities match) and the attachment at commit, to 1e-6;
- retarget: both backends move the strut to the same commanded attachment
  after one step (gravity off);
- a backend without `supports_connection_retarget` falls back to the next
  realisation, and the conformance test asserts the recorded fallback rather
  than equality.

**Phase 3: MuJoCo**

- the retarget and the reported wrench agree with `J.T @ f`, as in
  `tests/test_mujoco_wrenches.py`;
- ball pool exhaustion refuses the connection; ball wrenches;
- per-axis overload: a load applied along each axis breaks only on that axis's
  limit;
- side-A nominal snapping: a lone strut's nominal surface dock moves the strut
  (root free joint) and not the node, replacing the current refusal;
- the `driven_fixed` slide follows wheel odometry within a tolerance, and the
  weld wrench stays below a bound during a steady slide (this measurement
  decides §7.4).

**Throughout:** ruff, all three pyright projects and the full pytest suite
pass. SMORES-EP and M-Blocks tests are untouched.

The only existing test changed in Phase 1 is
`tests/test_runtime_inspection_protocol.py`, whose framing cases name the
protocol version and were moved to version 4 (14.2 item 3).

## 13. Open items for the reviewer

1. Approve dropping `stepwise_fixed` in favour of `static_fixed` plus a mock
   retarget (§7.1).
2. Approve putting the FreeSN point connector on `strut_body` and separating
   the seat gap from the magnet height (§4.1). This affects how the Phase 4
   pack is authored.
3. Approve the protocol bump to version 4 in Phase 1, or defer the edge
   attachment field until the inspector renders it.
4. Approve fixing the mock's over-advertised constraint types in Phase 1. Some
   existing tests may assert the current set.
5. Paper figures (capacity 12, 60° separation, 160/45–49 N, 1.5/1.3 N·m, F(h),
   the 42/80 division) need checking against the PDFs. They could not be
   extracted here because `poppler`/`pypdf` are not installed.

All five items were resolved in review. See §14.

## 14. Review decisions (2026-11-25)

The user reviewed this draft. Every verdict in §1 is accepted except where an
amendment below changes it. Where this section and an earlier one differ, this
section wins. When an amendment is implemented, fold it into the section it
amends and keep a one-line note here.

### 14.1 Paper figures, checked

Sources: Tu et al., ICRA 2022 (the FreeSN connector) and T-RO 2023
(configuration identification). Use the T-RO value where they differ. Local
copies are in `Claude outputs/freesn_reference/papers/` (gitignored).

| Quantity | Value to use | Source and notes |
|---|---|---|
| Shell diameter / thickness | 120 mm / 0.8 mm | both papers |
| Capacity | 12 | both papers |
| Minimum separation | 60° | Both papers give the minimum central angle as just under 60°. Twelve struts at icosahedron vertices are 63.4° apart, so a 60° guard still admits a full node. |
| Discrete grid | 42 vertices, 80 spherical triangles | T-RO: sensors sit at the icosahedron's vertices and edge midpoints. This confirms the `icosphere_2` inference in §9. |
| Magnet pull with the magnets touching the shell | about 140 N (T-RO), 160 N (ICRA) | Wheels lifted. This is **not** the normal strength of a seated strut. |
| Normal strength, seated on wheels | **about 79 N** | T-RO. The magnetic force is shared with the wheels. Shear, bending and torsion reach their maxima in this state. |
| Shear | 49 N side, 54 N front | T-RO (ICRA: 45 N). Use 49 N as the single isotropic value in 0.1. |
| Bending | 1.6 N·m front, 1.8 N·m side, at about a 9 cm arm | T-RO (ICRA: 1.5 N·m). Use 1.6 N·m in 0.1. |
| Torsion | 1.3 N·m | T-RO (ICRA: 1.5 N·m) |
| F(h) | plots only | ICRA Fig. 3, T-RO Fig. 2. Text anchors: maximum at contact, under 30 N by about 8 mm. A strut can pull off once h > 3 mm (ICRA). The curve must be digitised. It is not needed until Phase 5. |

### 14.2 Open items in §13, resolved

1. Drop `stepwise_fixed` for `static_fixed` plus a mock retarget: **approved**.
2. FreeSN point connector on `strut_body` at the shell-contact apex, with seat
   gap separate from magnet height: **approved**. This matches the T-RO
   physics, where the magnetic force is shared with the wheels once they touch
   the shell.
3. Runtime Inspector protocol version 4 in Phase 1: **approved**. Add the
   edge's attachment and realisation fields now, not when rendering lands.
4. Stop the mock advertising constraint types it does not implement:
   **approved for Phase 1**. See also A6.
5. Paper figures: **resolved** by §14.1, except for digitising F(h).

### 14.3 Amendments

Implemented amendments are folded into the sections they amend; each keeps a
one-line note here.

- **A1** (§7.2): the mock retarget moves side A's component, not side B's.
  Implemented.
- **A2** (§5.2, §11): separation is enforced while sliding, by clamping at the
  boundary. Implemented.
- **A3** (§7.1, §7.3, §7.7): `ball` is free-sliding only; only retarget
  satisfies `surface_slide`. Implemented.
- **A4** (§7.4): contact exclusion under `driven_fixed` is decided. The schema
  field and the fall-through on refusal are implemented; the request field and
  its use are Phase 2.
- **A5** (§8.1): every exceeded axis releases; documented as an approximation.
  Implemented.
- **A6** (§1): the mock advertises and realises `fixed` only. Implemented as its
  own commit; no existing test relied on the old set.
- **A7** (§3.1, §3.2): a surface pair must author `constraint: fixed`.
  Implemented, and stated in `robot_pack_spec.md`.
- **A8** (§8.2): h₀ is the defaulted `magnet_height_offset_m`. Implemented.

### 14.4 Phase 1 scope

**Delivered**, with these differences from the list below:

- the `spherical_attachment` builder (item 9) is deferred to Phase 2, as this
  section allows;
- `ball:` parameters are optional while authoring (§3.1), the ball anchor is
  expressed in connector A's frame (§7.3), and `ConnectionRequest` gained
  `snap_side` so a surface snap moves the strut (§5.3);
- a surface snap is applied only when the strut is alone in its assembly;
  otherwise the commit keeps the measured pose and records that on
  `DockCommitted` (§5.3);
- the MuJoCo adapter changed in one place only: it refuses side-A snapping
  rather than moving the wrong module.

**Deferred:**

- `SceneSpec.surface_realizations`, the per-scene override of the realisation
  list (§3.3). It was not in the Phase 1 list. Schedule it with the Phase 2
  contract work or later.
- **Side-A snapping in MuJoCo is a Phase 3 item.** Until then the adapter
  refuses any request with `snap_side="a"`, so on MuJoCo a nominal surface
  dock by a lone strut is refused through ordinary two-phase commit (and falls
  through to the next realisation, which is refused the same way). Measured
  alignment is unaffected. Phase 3 must move the strut's root free joint so
  the requested pose holds, as `_snap_to_nominal` already does for side B.
- The `spherical_attachment` builder (Phase 2, see above).

Phase 1 = schema, validator, core semantics and the mock backend. It runs
without MuJoCo. Implement, in this order, and stop for review at the end:

1. **Mock capability fix (A6).** Do this first, as its own commit, so any
   fallout is separate from the new feature.
2. **Schema and validator (§3, A7, A8).** `ConnectorInterface`, `SurfaceSpec`,
   `SurfaceRealization`, `surface_realizations`, `BallConstraintSpec` (schema
   only; no backend runs it in Phase 1), `break_limits`,
   `max_torsion_moment_nm`, `magnet_height_joint` with h₀, and
   `surface_contact_links` (A4; schema only in Phase 1). Every §3.2 rule, in
   both profiles, plus YAML round-trip through `RobotPackWriter`.
3. **Point-to-surface geometry (§4).** Acceptance with the seat gap and v_S(q),
   the virtual attachment frame A_q, nominal snapping through the existing
   `nominal_relative_transform`, and `candidate_surface_pairs`.
4. **Multi-occupancy (§5).** All 14 rows of §5.1, the `surface_full` and
   `surface_separation` guards, re-checking at commit, and side A = point side
   (§5.3), including the check that no consumer requires
   `connector_a < connector_b`.
5. **Attachment state and events (§6).** `SurfaceAttachment`,
   `world.surface_attachment(...)`, and the new defaulted fields on
   `DockCommitted` and `ConnectionRuntime`.
6. **Realisation selection (§7.1, A3).** Choose the realisation at commit and
   record it. Add `supports_connection_retarget` and the
   `SupportsConnectionRetarget` protocol (§7.2) to `backends/base.py` and
   implement it in the mock (A1). The mock refuses `ball`.
7. **Slide driving (§11, A2, A3).** `SurfaceSlideDriver`, `set_surface_slide`,
   the separation clamp, and the step-order slot after the docking pass. The
   `surface_drive` wheel model is schema plus a pure function, tested on its
   own; nothing drives it from wheel joints until Phase 3.
8. **Per-axis overload (§8.1, A5).** Opt-in `break_limits`, the
   `ConnectorOverloaded.quantity` field, the session-creation error when a
   moment limit meets a backend without torque support, and the mock reporting
   `torque_nm=None`.
9. **Views and protocol version 4 (§9, 14.2 item 3).** The edge's
   `attachment` and `realization` fields, the protocol bump, and updated
   framing tests. The `spherical_attachment` builder may wait for Phase 2 if
   Phase 1 runs long; say so at the review.
10. **Contracts.** Update `docking_semantics.md`, `backends.md` and
    `robot_pack_spec.md` for everything implemented, and move the implemented
    parts of this document from "draft" to "implemented".

**Not in Phase 1:** MuJoCo changes, `extra_contact_exclusions` on the request
(Phase 2), the FreeSN example pack (Phase 4), F(h) and `physical` mode
(Phase 5).

**Tests to add for the amendments**, on top of the §12 Phase 1 plan:

- A1: a retarget on the mock moves the strut's component, and the node and the
  other struts on it do not move; a retarget that closes a loop is refused with
  a message.
- A2: a slide toward a neighbour stops at `min_separation_rad` (to 1e-9); a
  strut already inside the boundary can move away but not closer; two struts
  closing on each other in one step give the same result on every run.
- A3: `set_surface_slide` on a `ball` connection raises `SurfaceSlideError`;
  `SURFACE_SLIDE` is not satisfied by `BALL` alone.
- A6: the mock refuses `compliant`, `ball` and `custom` with
  `BACKEND_REFUSED`.
- A7: a surface pair authored with `constraint: ball` or `hinge` fails
  validation.

**Done for Phase 1** means: all §12 Phase 1 tests and the tests above pass;
ruff, all three pyright projects and the full pytest suite pass; SMORES-EP and
M-Blocks tests are unchanged apart from any updated only because of A6, each
named in the review summary; and the contracts are updated.
