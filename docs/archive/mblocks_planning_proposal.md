# M-Blocks online planning proposal

Status: historical proposal. The initial 2D implementation is documented in
[Online M-Blocks planning](../mblocks_planning.md); this file is not a current contract.
The current behavior remains documented in [M-Blocks integration](../mblocks_3d.md),
[planning](../planning.md), and [Runtime Inspector](../runtime_inspector.md).

## Recommendation and scope

Build a centralized, feedback-driven lattice reconfiguration planner. Start with
connected configurations, sequential single-cube pivots, and the one actuation
plane supported by the current physical pack. Target shapes are scenario inputs;
the planner chooses the moving cubes, support edges, connector changes, and routes.

Use the 2015 geometric planning algorithm as the reference and add a ModSim
execution layer that checks actual landings before continuing. Keep geometric
reachability, physical capability, and observed execution success distinct.
The first release should demonstrate generated plans involving several different
moving modules, not merely replay the existing twelve-module traversal.

Full three-plane physical execution, simultaneous pivots, disconnected aggregation,
and automatically selected multi-cube moving groups are subsequent milestones.

## Research basis

| Paper | Relevance and intended use |
| --- | --- |
| Sung, Bern, Romanishin, Rus, [Reconfiguration Planning for Pivoting Cube Modular Robots, ICRA 2015](https://doi.org/10.1109/ICRA.2015.7139451) | Main planning reference: admissibility, boundary traversal, and conversion through a common line. Its guarantees apply to a restricted geometric model; the 3D treatment excludes non-convex holes. |
| Romanishin, Gilpin, Claici, Rus, [3D M-Blocks, ICRA 2015](https://people.csail.mit.edu/sclaici/pdf/romanishin20153d.pdf) | Hardware reference for flywheel actuation, magnetic attachment, and switching between three orthogonal actuation axes. |
| Feshbach, Sung, [Reconfiguring Non-Convex Holes in Pivoting Modular Cube Robots, RA-L 2021](https://danielfeshbach.github.io/publication/pivoting-cubes-ral-2021/) | Later extension to admissible 3D configurations with non-convex holes, using revised slice/branch removal ordering. A subsequent geometric-planning milestone. |
| Claici et al., [Distributed Aggregation for Modular Robots in the Pivoting Cube Model, ICRA 2017](https://people.csail.mit.edu/sclaici/pdf/claici2017blocks.pdf) | Distributed light-guided aggregation. Relevant if we later start with separated modules; it addresses a different objective from a prescribed target shape. |
| Romanishin, Mamish, Rus, [Decentralized Control for 3D M-Blocks for Path Following, Line Formation, and Light Gradient Aggregation, IROS 2019](https://doi.org/10.1109/IROS40897.2019.8967810) | Neighbor identification and distributed behaviors; useful for a later local-policy comparison. Only its publication record and [author-posted abstract](https://independent.academia.edu/JMamish) were accessible in this review, so this proposal does not claim an implementation-level review of that controller. |

The first four full papers were consulted. The 2019 DOI currently appears in
`mblocks_3d.md` under a different title; correct that citation during implementation.
Implement the algorithms originally in ModSim, with citations and without importing
research-repository code or adding a planner dependency.

## What the repository already provides

| Existing component | Reuse and required change |
| --- | --- |
| `model_views/lattice.py` | Already derives cells, 24 cube orientations, residuals, and conflicts. Share its pure geometry conventions with planning; `WorldState` remains authoritative. |
| `runtime/momentum_pivot.py` | Effort-driven single-pivot controller and activation from an existing session. Reuse measured capture and bounded actuation. |
| `runtime/momentum_sequence.py` | Authored sequences require the same moving module and one replaced connection per action. A general planner needs a different orchestration layer. |
| `runtime/coordinated_pivot.py` | Existing handling of multiple released/target bonds and cyclic assemblies informs the new action executor. Group motion remains outside the initial planner. |
| `planning/models.py`, `runtime/online_planning.py` | SMORES-specific tree goals, planar poses, and wheel execution. Preserve these APIs; introduce lattice-specific goal/action models. |
| `runtime/inspection.py`, `runtime/inspector_runner.py` | Existing planner telemetry is typed and selected around SMORES. Add an explicit lattice-planner payload and a common optional scenario capability. |
| M-Blocks line and staircase scenarios | Useful regression fixtures and execution references, but their move sequences are authored. |

The current pack has a fixed local +Y flywheel axis and directed hinge ports for
the XZ plane. The published carrier's plane-switching mechanism is absent.
Capability checks must use measured module orientation, not assume every cube
can pivot about every world axis. Current fixed welds and temporary hinges also
approximate magnetic attachment; they do not model a magnetic force field.

## Current attachment model and recommended first demonstration

The M-Blocks pack models attachment as a connection lifecycle. Accepted face
alignment activates a measured-pose weld; a temporary edge uses a two-point hinge.
There is no attraction before capture. Both connector types set
`docking_policy.break_force_n: null`, so neither releases automatically under load.
The listed 23 N normal, 18 N shear, and 0.45 N m bending limits are declared
ratings, not enforced magnetic breakaway criteria.

The runtime supports an optional scalar overload threshold using backend-reported
constraint reactions. Those reactions are not magnetic attraction measurements,
and the scalar does not represent peeling moment or separate tensile/shear loads.
Simply copying the normal rating into `break_force_n` would not establish a
validated magnetic model.

Existing momentum demos explicitly schedule weld/hinge transitions, and suspend
ordinary connector processing between those transitions. Their target capture
checks measured alignment, relative speed, and additional controller gates.
Flywheel effort, inertia, gravity, contact, and resulting motion are simulated;
overcoming the magnetic attachment itself is not.

For the first paper-based demonstration, use a six-module planar cluster forming
a line, followed by a second admissible target such as an L shape. A 2-by-3
rectangle is a candidate starting fixture; validate the exact fixtures against
the paper's admissibility rules before adopting them. Scale to twelve modules
after primitive execution and multiple-moving-module orchestration are reliable.

Prefer a horizontal tabletop configuration for this baseline: initialize the
cubes so their local +Y flywheel/hinge axes point along world Z. This uses one
actuation plane throughout, with no carrier switching. The geometric workspace
then lies above the floor, avoiding vertical routes that pass through it.
This orientation is a proposed scenario, not an already validated physical demo:
first test quarter/half turns, both directions, friction, recoil, and capture with
the actual rotated pack. Apply no artificial planar pose lock. Express shape
success in an explicitly declared assembly-relative frame if rigid whole-assembly
drift is allowed; never silently change an absolute-position goal.

Use the 2015 planar algorithm for move selection and make the boundary order,
chosen mobile cube, preserved connections, and growing line visible. The online
feedback executor is ModSim's addition to that geometric algorithm. Gravity,
floor friction, collisions, and flywheel dynamics remain active, with the
attachment approximation clearly identified in the demo description.

The Inspector should show live/target shapes, the next pivot and rejected
candidates, and measured landing success. Label any displayed connector force as
a constraint reaction; do not draw nonexistent magnetic attraction vectors.
Retain the vertical line/staircase physics demos as complementary views of
lifting and tumbling. Automatically planning bonded slab moves remains a later
extension.

A separate magnetic-model milestone can add connector-frame force/moment
telemetry, calibrated load-dependent detachment, and short-range attraction with
equal/opposite forces and torques. Validate it on pull-off, shear, peeling, and
capture experiments before combining it with the planning benchmark.

## Stage 1: Goals, observations, and legal moves

Add immutable lattice models under `src/modsim/planning/mblocks/`, independent of
MuJoCo and Studio. Represent a goal with cells, lattice pitch/frame, connection
requirements, and permitted motion planes. Initially modules are interchangeable
for shape matching, while execution retains their actual IDs and orientations.
Explicit ID-to-cell and final-orientation constraints can be added separately;
they must never be silently ignored.

Observe settled module poses and committed face connections from `WorldState`.
Keep all 24 orientations, quantization residuals, and topology/sample revisions.
Reject ambiguous occupancy and out-of-tolerance idle observations. Quantizing an
observation must never move a physical cube. A module in flight is expected to be
off lattice and is handled by the active controller, not discrete replanning.

Generate 90- and 180-degree pivot candidates with:

- a real supporting edge and compatible available hinge ports;
- clearance throughout the rotating cube's swept volume, including the floor;
- connectivity of the stationary structure after all proposed face releases;
- orientation-derived face mappings and all expected destination captures;
- physical capability and conservative support checks for the moving cube and
  remaining assembly, separately from geometric admissibility.

Preserve passive face capture: adjacent compatible faces may create loops.
Neither occupancy adjacency alone nor a requested dock proves a committed bond.
Conservative support screening is not a proof of dynamic stability.

Acceptance: deterministic legal-move fixtures covering both turn sizes, both
directions, rotated cubes, floor/sweep collisions, articulation points, loops,
unsupported axes, and invalid observations. No optional dependencies needed.

## Stage 2: Discrete route planning

Implement the planar part of the 2015 method first: boundary ordering, admissible
removal including its special three-module case, and traversal to a canonical
line. Construct target-shape routes through that intermediate configuration.
Validate inverse transitions explicitly: arbitrary 3D quarter-turns are not
automatically reversible. These choices follow the [2015 planning paper](https://doi.org/10.1109/ICRA.2015.7139451).

For ModSim, enforce a common lattice frame and compatible canonical-line placement.
Never translate a physical assembly or exchange its module identities just to
join two geometric plans. Recompute concrete connector actions using the actual
orientation reached at each step.

Keep a separate bounded state-search mode for small, physically constrained scenes
where the constructive route encounters the floor, unavailable axes, or an
unsupported pivot. Use deterministic costs/ties and explicit time, expansion,
and workspace limits. This is a ModSim extension; budget exhaustion is not proof
that a goal is impossible. Fixed anchors and physical restrictions remove any
general completeness claim inherited from the geometric reference.

First validate routes in a pure discrete simulator. A kinematic preview may be
added, clearly identified as such; the mock backend must not pretend to execute
hinge dynamics, which it explicitly does not support.

Acceptance: generated routes for several small start/goal pairs, independent
validation of every transition, reproducible results, and useful blocked/budget
diagnostics. Include cases where the selected moving module changes.

## Stage 3: Online execution in MuJoCo

Add `runtime/mblocks_online_planning.py`. Adapt the existing single-pivot and
multi-bond execution machinery to activate one generated action in the current
session without rebuilding the world. Exactly one owner advances physics.

The action lifecycle is:

```text
observe settled state -> plan/validate next action -> engage support hinge
-> spin flywheel -> release required faces -> brake/pivot
-> confirm measured face capture(s) -> release hinge -> settle -> observe again
```

Check the next action against current observations before committing it. Reuse a
valid remaining route; replan when the measured state or available actions differ.
Calibrate supported primitive profiles by direction, turn angle, and support
geometry. An upward pivot needs separate validation from its downward inverse.

On failure, bound recovery attempts. Replan only from a valid settled observation;
a cube stalled mid-pivot may need controller recovery or an explicit failed state.
Keep the resulting physical state available for inspection. Never teleport,
silently snap, reset the world, or claim a requested connection succeeded.

Run planning in cancellable, bounded chunks outside the Qt thread. Associate
results with observation revisions and reject stale actions. Keep the runtime
owner responsive to Stop and continue appropriate holding control during planning;
workers, if introduced, receive immutable data only.

Acceptance: generated actions under gravity with effort commands and normal
docking lifecycle only after initialization; multiple moving IDs; bounded handling
of missed captures, unexpected bonds, drift, and timeouts; measured stable success.

## Stage 4: Planner visualization

Extend the existing Inspector rather than introduce another renderer:

- **Runtime state:** live and target lattice views side by side, sharing cube
  graphics and projection controls. Distinguish goal cells from existing
  nearest-cell diagnostic ghosts. Offer linked cameras and module selection.
- **Planning:** selected moving cube, support edge/axis, pivot arc and swept
  region, future cells, and current action phase. Optional candidate/rejection
  overlays explain why alternatives would disconnect, collide, or lack actuation.
- **Diagnostics:** selected-action angle, flywheel speed, landing residuals,
  search effort, plan revision, retries, and remaining actions. Keep separate
  units explicit and preserve the existing time-only action-history navigation.
- **Event log:** retain its own tab and distinguish planner decisions from
  canonical docking/undocking events.
- **Result:** show target reached only after measured cells, required bonds,
  orientation constraints if supported, no temporary hinge, and a stable hold
  satisfy completion criteria. Stopped, blocked, failed, and timed out are distinct.

Use distinct categorical colors plus labels/line styles, hover/collapsible legends,
and layer toggles in every theme. Retain graphics objects and camera state; publish
bounded diagnostics rather than the entire search tree each frame. Add a tagged
planner snapshot variant with serialization tests and an explicit IPC version
transition if the wire contract changes. Preserve SMORES behavior.

Acceptance: inspect a successful run and a rejected move without reading console
logs; responsive pause/stop, stable camera/selection, and all-theme widget checks.

## Stage 5: Benchmarks, integration, and release gate

Expose scenario goals and planner settings through a headless example and named
Runtime Inspector demos. Goals belong to scenarios, not the Robot Pack schema.
Record enough settings, revisions, decisions, and final measurements to reproduce
and diagnose a run.

Use this progression:

1. Two-module primitive checks, including separately validated inverse motions.
2. A generated version of the existing twelve-module traversal, as an integration
   check of routing and execution; this alone is not general reconfiguration.
3. Small connected one-plane shape changes involving several moving modules,
   then a twelve-module case within the validated physical primitive set.
4. Injected capture refusal/drift, unsupported goals, search exhaustion, and
   interruption, each with an explicit terminal or recovery result.

Retain the authored line and staircase demos as regressions. The staircase moves
bonded two-cube slabs; automatically discovering those groups requires a later
planner extension with group swept volumes, support, and inertia checks.

Keep core tests runnable without NumPy, MuJoCo, or Qt. Run physical regressions in
the optional MuJoCo job and serialization/UI checks in their appropriate jobs.
Report move count, replans, planning cost, simulated duration, and observed success
over documented perturbations; do not turn a single passing run into a guarantee.

Release gate: a supplied supported target produces a generated plan, chooses
different moving modules, completes through measured physical captures, explains
its decisions in the Inspector, and stops or recovers clearly on failure.

## Subsequent milestones

1. Extend geometric planning to the 2015 3D slice algorithm, then the 2021
   non-convex-hole extension. Keep their admissibility checks and guarantees scoped
   to their respective models.
2. Add and validate the missing carrier/actuation planes and directed hinge
   catalog before advertising full 3D physical execution. Any simplified actuator
   alternative must be explicitly described as a surrogate.
3. Add group-move discovery and parallel scheduling only after sequential feedback
   is reliable. Shared supports, simultaneous connection cuts, and swept volumes
   all matter; independent-looking paths alone do not justify concurrency.
4. Implement distributed aggregation/line-formation policies as separate planner
   strategies, with local observations and communication assumptions made explicit.
