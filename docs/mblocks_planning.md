# Online planar M-Blocks planning

The first implementation generates single-cube lattice pivots, executes them
through MuJoCo, and verifies measured landings before continuing. The supported
physical demos form a line from either a four-block square or a larger six-block
rectangle. The anchor block remains in the shape,
but is not fixed to the ground. No new dependency or research-repository code
is used.

## Launch

From an editable checkout with the `studio` and `mujoco` extras installed:

```bash
.venv/bin/modsim runtime examples/robot_packs/mblocks_3d \
  --demo mblocks_online_lattice
```

For the longer six-cube run:

```bash
.venv/bin/modsim runtime examples/robot_packs/mblocks_3d \
  --demo mblocks_online_lattice_large
```

| Demo | Initial shape | Moving cubes | Generated pivots | Reference completion |
| --- | --- | --- | --- | --- |
| `mblocks_online_lattice` | 2 by 2, four cubes | 3 | 8 | 9.25 simulated seconds |
| `mblocks_online_lattice_large` | 2 by 3, six cubes | 5 | 21 | 27.59 simulated seconds |

The larger run uses the same physical controller, capture checks and 3 mm
settled-position tolerance. Its timestep is 0.0005 s and its time budget is
180 s. To watch it more slowly, append `--speed 0.5`: nominal playback lasts
about 55 seconds. Slower playback does not change the physics timestep.

This opens the Runtime Inspector and MuJoCo viewer. The CLI supplies gravity,
ground, 25 mm initial center height, a 0.0005 s step, the cubic-lattice view,
and a 120 s time budget for the small demo. Completion stops execution early and leaves the result
visible. `--no-viewer` keeps just the Inspector; `--speed 2` requests faster
playback without changing simulation time or solver steps.

Headless execution and diagnostic export:

```bash
.venv/bin/python examples/scenarios/mblocks_online_planning.py \
  --goal line --snapshot /tmp/mblocks-result.json
.venv/bin/python examples/scenarios/mblocks_online_planning.py \
  --preset large --snapshot /tmp/mblocks-large-result.json
```

Inspect a generated plan without MuJoCo or Qt:

```bash
.venv/bin/python examples/scenarios/mblocks_online_planning.py --plan-only
.venv/bin/python examples/scenarios/mblocks_online_planning.py --goal elbow --plan-only
```

The second runtime demo, `mblocks_online_lattice_elbow`, exercises the reverse
target construction. **Its physical execution is experimental:** the current
reference run completes the line and starts reversing it, but a later landing
can exhaust the flywheel speed ceiling. It reports failure and retains the last
measured state. It is available for controller debugging, not as a passing
physical benchmark. Other layouts, including the wider 3-by-2 six-block rectangle
and eight-block clusters, still expose capture/alignment or actuator-limit
failures. Physical completion is verified for the two presets above, not for
every geometrically valid input.

The headless script accepts `--preset small|large`, `--initial-file` (`LatticeState` JSON), `--goal-file`
(`LatticeGoal` JSON), `--quarter-rpm`, `--half-rpm`, `--dt`, and `--duration`.
Unknown goal fields are rejected. Example goal:

```json
{"id":"elbow","cells":[[1,1],[0,1],[0,0],[0,-1]],"pitch_m":0.05,"frame":"assembly_relative"}
```

For a larger stress fixture, create an initial input with
`rectangle_state(3, 2).model_dump_json()` from `modsim.runtime.mblocks_lattice`.
Pass it with `--initial-file`, which overrides the preset; the default line
target adapts to the initial state. The large preset uses `rectangle_state(2, 3)`.

## Geometric algorithm and scope

`modsim.planning.mblocks` is a simulator-independent, original implementation
based on Sung, Bern, Romanishin and Rus,
[Reconfiguration Planning for Pivoting Cube Modular Robots, ICRA 2015](https://doi.org/10.1109/ICRA.2015.7139451).
It implements the planar forbidden patterns, boundary traversal toward a common
line, the P3 removal queue, and reversal of the target-to-line construction.
Every emitted 90/180-degree pivot checks its swept cells and the connectivity
of the stationary remainder. A pivot stops at the first available face contact.
Module identities and quarter-turn orientations are preserved through the route.
The implementation has explicit move/search bounds and reports unsupported
constructions rather than asserting the paper's full completeness guarantee.

Inputs contain 2–32 cubes in one connected, admissible XY lattice. Goals specify
an **unlabelled shape** relative to the named extreme anchor. Initial and target
shapes share that extreme cell, except for the canonical line extending above
it. Arbitrary world-position goals, specified ID-to-cell assignments, final
orientation constraints, disconnected aggregation, simultaneous group moves,
and the full 3D/hole algorithms are not implemented. Runtime execution currently
requires the supplied 50 mm M-Blocks pack.

The discrete state supports the four orientations in the selected actuation
plane. The existing live cubic-lattice view still classifies all 24 cube
orientations and displays residuals. A tilted or otherwise unsupported settled
pose fails observation; it is never projected back into the simulation.

## Online execution and physical approximation

`OnlineLatticeScenario` owns the feedback loop. It observes `WorldState`,
compiles the next generated move into actual face/hinge connector IDs, and uses
the coordinated momentum controller to release all incident faces and capture
all landing faces. Every other face connection must remain unchanged. After
capture, it waits for settling, checks pose/orientation/occupancy and exact face
adjacency, then advances the discrete state. Success requires the measured goal
cells, the expected face topology, and no transient hinge.

A valid settled deviation can invoke a cooperative bounded BFS with the anchor
preserved: 20 expansions per batch, 4,000 total, at most three replans. Off-lattice
poses, missing captures, topology mismatches, timeouts, or actuator limits produce
an explicit failed state and clear joint commands. Search yields between batches
so the runtime can continue processing physics and cancellation.

The blocks start rotated so local +Y, the existing flywheel/hinge axis, points
along world +Z. Gravity, friction, collisions, recoil, flywheel inertia and
motor/brake effort remain active. The controller never writes root poses,
root velocities, or body wrenches after initialization. There is no planar lock.
The observation frame follows the measured anchor, so whole-assembly drift
does not silently change the goal's declared meaning.

The baseline uses 14,000 RPM quarter turns and 17,000 RPM half turns, the existing
0.03 N m motor/2.6 N m brake ceilings, and a 20,000 RPM speed ceiling. After
braking, a bounded motor-feedback controller helps settle yaw toward the lattice;
this is a **ModSim simulation control extension**, not a controller reproduced
from the geometric paper. Capture also requires yaw within 0.75 degrees of the
anchor frame, the normal connector acceptance checks, and a 1 mm distance gate.
Settled observation allows 3 mm position and 5 degrees orientation residual.
These are simulation calibration values, not hardware validation.

The demo sets `constraint_time_constant_s=0.002` for reserved MuJoCo welds and
hinges. Other demos retain their previous solver defaults. This improves the
stiffness of the ideal attachment approximation; it is not a magnetic strength.
Faces still latch at their measured pose. There is **no attractive magnetic
force field or enforced magnetic breakaway model**; weld/hinge transitions are
commanded through the existing lifecycle. See [Docking semantics](docking_semantics.md).

## Inspector

- **Runtime state:** measured cubes and target cubes use the same lattice
  renderer and projection controls. Target cubes are presentation-only intent.
- **Planning:** measured XY footprints, orange moving cube, cyan support, purple
  target cells, gold swept cells, magenta pivot point, and the proposed arc.
  The move table lists generated actions; eligibility is evaluated at the last
  settled lattice observation. Hover details and a collapsible legend explain
  the graphics. Overlays can be toggled and cameras retain their zoom.
- **Action history:** phases are horizontal time intervals. Zoom changes time
  only; rows scroll independently. Flywheel speed, pivot angle, landing effort,
  settled residual, completed moves and replans are also shown.
- **Event log:** canonical connector events and planner decisions remain separate.
  Completion, failure, user stop, and time-budget exhaustion have persistent,
  distinct status banners.

Protocol version 3 adds the optional immutable `lattice_planning` payload. It
must match the live view's time, sample sequence and topology revision. SMORES
continues using `planning`; frames cannot contain both. GUI and runtime child
must use the same installed version. Robot Pack persistence is unchanged.

## Validation and next work

Tests cover legal and inverse moves, independent sampled swept-volume collision
checks, connected remainders, multiple rectangle sizes, target reversal, bounded
recovery search, strict inputs, frame transport, retained GUI items/themes,
failure diagnostics, and both complete physical line-forming runs. The physical
tests reject any runtime root pose, twist or wrench command and require every
non-anchor cube to participate in the generated route.

Next work is physical calibration for larger clusters and reversed landings,
then richer goal authoring and calibrated magnetic forces/breakaway. The planner's
geometric success must remain distinct from physical execution success.
