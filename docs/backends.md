# Backends

ModSim owns modular-robot semantics; a backend owns physics. This document
covers how backends are selected, what an adapter must provide, and how the two
that ship today differ.

## Selecting a backend

Resolution order is explicit argument, then `MODSIM_BACKEND`, then `mock`.

```bash
modsim backends                                   # what is registered and installed
modsim dock path/to/pack --backend mujoco
MODSIM_BACKEND=mujoco modsim dock path/to/pack    # applies to the whole run
```

```python
from modsim import RuntimeSession, SceneSpec
from modsim.backends import create_backend

session = RuntimeSession.create(loaded_pack, scene, "mujoco", gravity=(0.0, 0.0, 0.0))
adapter = create_backend()  # honours MODSIM_BACKEND
```

`RuntimeSession.create` accepts an adapter instance, a registered backend name,
or `None`. Extra keyword arguments are forwarded to the adapter's constructor,
which is how engine-specific settings such as gravity and timestep stay out of
the shared interface.

Registration is open, so a new engine does not require changing ModSim:

```python
from modsim.backends import BackendEntry, register_backend

register_backend(
    BackendEntry(
        name="my_engine",
        summary="Experimental backend.",
        loader=lambda: MyAdapter,
        install_hint="Install with: pip install my-engine",
    )
)
```

Entries load lazily. Importing `modsim` never imports `mujoco`, which is what
keeps the core dependency-light and what lets a core-only environment run the
full test suite.

## The adapter contract

Seven methods. ModSim never asks a backend what a connector *means*; it asks for
observations and constraints, and the backend answers.

| Method | Responsibility |
|---|---|
| `capabilities()` | Declare what the backend can actually do |
| `load(pack, scene, *, root)` | Instantiate every placement, return the handle registry |
| `step(dt_s)` | Advance simulation |
| `snapshot()` | Report link poses, twists, optional connector frames and constraint forces |
| `create_physical_connection(request)` | Attempt the constraint, accept or refuse |
| `remove_physical_connection(handle)` | Release it |
| `shutdown()` | Free resources |

`root` is the Robot Pack directory. A backend that reads mechanical assets needs
it; the mock ignores it, which is why the mock can run from an in-memory pack.

**Capabilities are load-bearing, not documentation.** The docking engine checks
`supports_runtime_constraints` before attempting a commit. A backend that
reports `False` causes `DockFailed` rather than a silently faked connection.

**Measured connector frames are optional but preferred.** A backend that
materialises connector frames natively should report them in
`snapshot().connector_frames`; `WorldState` prefers them over recomposing the
parent link pose with the authored offset, which removes any chance of ModSim
and the engine disagreeing about where a connector is.

## The two backends today

| | `mock` | `mujoco` |
|---|---|---|
| Dependencies | none | `modsim-robotics[mujoco]` |
| Dynamics | none — kinematic | full rigid-body |
| Gravity, contacts, mass | no | yes |
| Articulated kinematics | no | yes |
| Joint state | omitted | position, velocity, actuator effort |
| Joint commands | no | declared scalar effort modes |
| Connector frames | composed from local pose | measured from sites |
| Runtime docking | fixed-style constraints; hinge explicitly refused | fixed welds and two-point hinges |
| Constraint forces | injectable, for tests | equality-force magnitudes for active welds and hinges |
| Contact exclusion on dock | not applicable | one reserved body-pair exclusion per active weld; none for hinges |

### mock

Deliberately kinematic. Bodies move at whatever twist they are given, nothing
falls, and nothing collides. What it models faithfully is the part ModSim
depends on: a physical connection makes two modules move as one rigid body, and
removing it lets them move independently.

Every link of a module reports the module pose, because the mock has no
articulated kinematics. Modules with internal joints therefore need a real
backend before their non-root connectors mean anything.

It stays in the project permanently as the CI backend and as the reference
semantics the conformance suite compares against.

The mock explicitly refuses hinge requests. Approximating a hinge as one of
its rigid welds would erase the required rotational degree of freedom and
could make a dynamics scenario appear to pass without hinge physics.

### mujoco

`load` composes one MuJoCo model from per-instance copies of each module's
mechanical asset:

- each placement is attached under its own name prefix, so `generic_cube_0`'s
  base link becomes the body `generic_cube_0/base_link`;
- every module's root link gets a free joint, because a URDF root attaches
  rigidly to the world and modules must be able to move;
- every connector becomes a site at its authored local pose, namespaced as
  `<module>/connector/<connector>`;
- pools of inactive weld and two-point hinge equality constraints are reserved;
- optionally a ground plane, off by default so it cannot intersect a module
  placed at the origin.

Options: `gravity`, `timestep_s`, `weld_pool_size`, `hinge_pool_size`, `ground`,
`ground_height_m`.
MuJoCo's integrator step is a model property, so `step(dt_s)` covers the
requested interval with whole solver steps and `snapshot().time_s` reports the
time actually reached.

#### Docking via reserved constraint pools

MuJoCo fixes model topology at compile time, so docking cannot *create* a
constraint — it claims one of the reserved welds, re-points it, and activates
it. `create_physical_connection` writes `eq_obj1id`, `eq_obj2id`, `eq_objtype`,
and `eq_data`, then sets `eq_active`. Release deactivates the slot and returns
it to the pool. An exhausted pool refuses the connection, which the two-phase
commit reports as `DockFailed` rather than faking a latch.

Each weld slot has a paired, precompiled contact-exclusion entry. Claiming a
weld publishes the constrained body-pair signature alongside any authored
static exclusions, keeps MuJoCo's signature array sorted, and returns both
entries on release. The exclusion covers that exact constrained body pair; it
does not recursively suppress contacts involving other articulated bodies in
either module or assembly.

A runtime hinge claims a separate slot containing two precompiled
`mjEQ_CONNECT` equalities. The adapter places their corresponding point anchors
symmetrically around the measured connector origins, separated by
`HingeConstraintSpec.anchor_separation_m` along the axis expressed in each
connector frame. Two constrained points preserve rotation about their common
line while removing the other five relative degrees of freedom. Hinge release
deactivates and returns both equalities together. Hinges deliberately retain
body collision and do not consume a contact-exclusion slot, which is required
for contact-driven edge pivots.

`weld_pool_size` counts welds; `hinge_pool_size` counts complete hinge slots,
not individual point constraints. Their automatic minima are eight slots each.
Exhausting either pool refuses the request through normal two-phase commit.

Optional `constraint_time_constant_s` sets the positive-format MuJoCo `solref`
time constant, with damping ratio 1, for reserved runtime welds and hinges only.
It must be finite, positive, and at least twice the compiled timestep. Omitting
it preserves previous defaults and authored equalities are never altered. The
[online M-Blocks baseline](mblocks_planning.md) uses 0.002 s. This numerical
stiffness setting is not a magnetic-force or breakaway parameter.

ModSim commits a relative pose between *connector frames*; a weld constrains
*bodies*. `modsim_backend_mujoco.welds.body_relative_transform` performs the
conversion:

```text
T_bodyA_bodyB = T_bodyA_connA . T_connA_connB . T_connB_bodyB
```

It lives in its own module so the geometry can be checked without a simulation
running, which is where a frame-convention error is cheapest to read.

`eq_data` layout is `[0:3]` anchor, `[3:6]` relpose position, `[6:10]` relpose
quaternion, `[10]` torque scale.

Snapshots expose a scalar force for every active runtime constraint. A weld's
value is the norm of its three translational equality rows. A hinge's value is
the norm of the vector sum of its two three-axis point-force rows. Rotational
rows and the hinge point-force difference represent moments and are not mixed
into a field expressed in newtons. These solver-reaction conventions enable
core break-force handling but do not yet constitute a complete connector
wrench.

Nominal snapping can target a connector on an articulated child body. The
adapter measures that body's transform relative to the module root, computes
the desired constrained-body pose, and converts it back to a root free-joint
pose before activating the weld. Treating the child body as the root injects a
large pose error and is covered by an articulated regression test.

The current adapter expects Robot Pack link/joint names to match the selected
URDF or MJCF asset. Full use of `asset_ref`, `link_map`, `joint_map`, and
`connector_frame_map` is the next backend-boundary task. A connector with only
a named `frame` is refused until that mapping is implemented; use a numeric
`local_pose` for the current MuJoCo slice.

#### The viewer

`modsim run --view` opens MuJoCo's passive viewer, paced to wall clock, and
holds the window open when the scenario ends so the final configuration can be
inspected. Runtime Inspector launches additionally accept `--speed FACTOR`
(`--real-time-factor` is an alias): the viewer targets simulated elapsed time
divided by that positive factor. This changes pacing only—solver steps, motor
limits, controller targets, simulated duration, and event timestamps remain
unchanged. A high requested factor is best-effort when physics or rendering
throughput cannot keep up.

For `modsim runtime`, MuJoCo's built-in **Run/Pause** item remains disabled by
design. ModSim uses the passive viewer and retains ownership of physics steps,
scenario progression, `WorldState`, and events; handing control to a
viewer-owned simulation loop would bypass that boundary. ModSim instead adds a
synchronized **Pause**/**Resume** button to the Runtime Inspector, a **Space**
shortcut in the native window, and a `RUNNING`/`PAUSED` viewer overlay. Both
inputs change one runtime-owned state at a completed-step boundary. While
paused, both displayed model states remain frozen but their windows and view
controls stay interactive. Resume resets wall-clock pacing so no catch-up burst
occurs, and Stop or window close remains available. The overlay is
feature-detected for compatibility with older supported MuJoCo releases; pause
control does not depend on it.

MuJoCo normally discards URDF `<visual>` geometry unless the URDF opts out.
The ModSim adapter retains it by default, while respecting an explicit
`discardvisual` setting authored in the URDF. When separate visual geometry is
available, imported URDF collision geoms are assigned to geom group 3. The
native viewer starts that debug group hidden, so detailed meshes in visual
group 1 are shown without opaque collision proxies covering them. Contacts
still use the hidden collision geoms; the viewer's **Group 3** toggle reveals
them when debugging. The ModSim ground belongs to visible environment group 2.
Its pale, near-white blue-gray surface keeps rendered modules and shadows
legible in screenshots; the color is visual only and does not change ground
contact or friction. Hand-authored MJCF geom groups are not rewritten.

**On macOS this must run under `mjpython`.** MuJoCo's passive viewer needs to own
the main thread, so `python` raises. The MuJoCo wheel installs `mjpython`
alongside `python` in the same environment:

```bash
mjpython -m modsim run path/to/pack --backend mujoco --view
```

The adapter translates MuJoCo's error into that instruction rather than letting
a raw traceback through.

`modsim runtime PACK` launches the complementary views together. Qt owns the
main process and renders ModSim's selected 2-D semantic model view and event
log. The built-in choices are a logical topology graph and a cubic-lattice
projection. A companion process owns the one authoritative MuJoCo session,
native viewer, physics stepping, and model-view generation. Immutable
inspector frames cross the process boundary, so the 3-D model, selected
semantic view, and events always describe the same simulation rather than two
approximately synchronized runs.

Playback requests cross that boundary in the opposite direction. The runtime
owner publishes the exact frame at a pause boundary and acknowledges the
resulting state, keeping the Inspector button and native-viewer overlay in
sync. The `--no-viewer` worker-thread arrangement honors the same Inspector
control and pause semantics without launching the companion process.

The public command is identical on macOS and Linux. On macOS ModSim
automatically locates the `mjpython` installed beside the active environment's
Python and uses it only for the viewer child. The native viewer is enabled by
default for MuJoCo; use `--no-viewer` to keep the existing headless-worker
arrangement when the native 3D window is not wanted:

```bash
modsim runtime path/to/pack --backend mujoco
modsim runtime path/to/pack --backend mujoco --no-viewer
```

The first command opens separate MuJoCo and Runtime Inspector windows. It does
not embed MuJoCo in Qt. Closing the Runtime Inspector shuts down its companion;
closing the native viewer first stops the runtime while leaving the final
semantic view and events available for inspection. The `--no-viewer` path still
opens the Qt semantic window and therefore requires a display or Xvfb; use
`modsim run` without `--view` for a completely non-GUI process.

#### Driving modules

`MuJoCoBackendAdapter` implements `SupportsModuleKinematics`, so a scenario can
place and drive modules directly. `set_module_twist` takes **world-frame**
velocities: a free joint stores angular velocity in the body frame, and the
adapter converts, so callers work in world coordinates everywhere in ModSim.
`apply_module_wrench` applies a persistent force that survives contact and
gravity, which is what driving against resistance needs.

These methods are scenario controls over a module's root free joint, not robot
actuator commands. Physical robot control uses the separate optional
`SupportsJointCommands` contract. A `JointCommand` carries a stable
`<module>/<joint>` ID, one declared Robot Pack control mode, and an SI target.
`RuntimeSession` validates a complete batch for existence, declaration,
backend support, finiteness, and limits before forwarding any command.

The current MuJoCo implementation supports `effort`. Scene composition creates
one unit-gear motor for every Robot Pack joint that declares effort control and
a finite matching effort limit. Commands persist in `data.ctrl` until replaced
or cleared. Snapshots report every semantic joint's position, velocity, and
actuator effort. Other modes and actuator/transmission catalogs remain later
work; the command boundary does not infer motors from arbitrary URDF tags.

`modsim run --fixed-connector ID --moving-connector ID` uses the measured root
and connector frames to arrange exactly two modules, then drives along the
fixed connector's docking axis. This is preferred over the legacy row layout
for robot packs whose connectors are not aligned with world X.

#### Named Runtime Inspector demonstrations

The M-Blocks traversal uses MuJoCo for detailed mesh rendering, backend state,
and endpoint face welds while a backend-neutral scenario writes the released
assembly along authored edge arcs:

```bash
modsim runtime examples/robot_packs/mblocks_3d \
  --backend mujoco \
  --demo mblocks_five_module_pivot
```

The M-Blocks pack defaults to `mblocks_lattice`, whose Runtime Inspector
renderer shows measured cubes, integer snap cells, axes, face-labelled
connections, and off-lattice/occupancy diagnostics in isometric or axis-plane
projections. Pass `--model-view mblocks_topology` for the connectivity-only
graph.

During an authored arc, `RuntimeSession.step(...,
process_connectors=False)` retains session-owned stepping, snapshot ingestion,
and overload handling while deferring passive connector capture until the exact
endpoint. Gravity and ground are disabled. This is intentionally kinematic and
does not claim a flywheel-, magnetic-hinge-, or contact-driven M-Block pivot.

The physical two-module M-Blocks path uses the new hinge pool, internal-joint
effort, ground contact, and gravity:

```bash
modsim runtime examples/robot_packs/mblocks_3d \
  --demo mblocks_momentum_pivot
```

The demo stages its initial face/edge contacts only at time zero. It then spins
the moving flywheel, releases the fixed face onto the retained +Y hinge,
applies a bounded brake pulse, captures the measured target face, and releases
the hinge. It never writes a module root pose, twist, or wrench after
initialization. This is a one-plane, deterministic face-to-hinge-to-face
approximation; continuous magnetic attraction, force-selected bond breakage,
and the published three-plane carrier remain deferred.

The larger kinematic visualization benchmark remains available as a fast
reference:

```bash
modsim runtime examples/robot_packs/mblocks_3d \
  --backend mujoco \
  --demo mblocks_twelve_module_line
```

Eleven cubes form a substrate while the twelfth performs ten authored quarter
traverses and one final half-turn to complete a 12-cell line. It is inspired by
the published 2019 line-formation experiments, not their exact unpublished
move trace, and it is not physical or autonomous.

The corresponding one-plane physics route has its own demo ID:

```bash
modsim runtime examples/robot_packs/mblocks_3d \
  --demo mblocks_physical_twelve_module_line \
  --speed 4
```

This path reserves 12 weld slots and two hinge slots, commits the complete
eleven-face initial structure at simulation time zero, and then composes eleven
`MomentumPivotScenario` primitives. Ten use 6,000 RPM targets for quarter-turn
surface traverses; the last uses a 9,000 RPM target for the half-turn into the
line. Every primitive pre-engages the appropriate directed edge hinge,
releases the old face, commands bounded flywheel effort, captures the measured
target face, and releases the hinge. The sequence never calls backend root
pose, twist, or wrench controls after initialization.

MuJoCo therefore remains responsible for gravity, contact, the two-point hinge,
flywheel reaction torque, and shell motion throughout the route. ModSim remains
responsible for action order, joint-command bounds, connector transitions,
canonical events, and the derived lattice view. The CLI defaults to the
validated 0.0005 s solver/controller step; its 12-second duration is a
simulated-time budget, and `--speed` only changes wall-clock pacing.

The route is a deterministic one-plane engineering approximation. It does not
add continuous magnetic attraction, force-selected bond changes, the physical
three-plane carrier, autonomous planning, or the exact unpublished 2019
hardware move trace.

The mat-to-staircase benchmark exercises a cyclic topology and coordinated
two-module motion through matched reference and physics entries:

```bash
modsim runtime examples/robot_packs/mblocks_3d \
  --backend mock \
  --demo mblocks_twelve_module_staircase \
  --no-viewer

modsim runtime examples/robot_packs/mblocks_3d \
  --demo mblocks_physical_twelve_module_staircase \
  --speed 4
```

`CoordinatedPivotPlan` declares complete tuples of take-off and landing faces
instead of assuming one edge replacement. Its reference executor writes the
detached slab along an analytical arc. Its physics executor sends the same
bounded effort law to both flywheels, retains one two-point hinge, and never
writes a module root after initialization. MuJoCo integrates gravity, contact,
constraints, reaction torque, and all root motion. The backend reserves 24
weld slots because the initial 2×6 mat has 16 fixed bonds and the final
staircase has 18; only one of its two hinge slots is active at once.

The maintained 0.5 ms MuJoCo regression completes eleven physical pivots near
9.5 simulated seconds. The exact route and controller targets are ModSim
engineering choices based on published M-Blocks primitives, not a reproduced
hardware trace or an autonomous planner. All rotations remain in one +Y plane
even though the final staircase occupies two Y rows and three Z layers.

The physical SMORES-EP cycle enables real gravity/contact and drives the wheel
joints rather than a module root:

```bash
modsim runtime examples/robot_packs/smores_ep \
  --demo smores_diff_drive_dock_undock
```

The pack selects a backend-specific MJCF for MuJoCo while retaining its URDF
as the Studio/imported mechanical source. Detailed STL meshes are visual-only.
Primitive tire cylinders and a rear skid carry ground contact; connector-face
proxies use a separate contact category so they can meet each other without
dragging on the floor. The scenario places an upright `pan`/TOP-to-`bottom`
pair once, settles under gravity, runs a bounded differential-drive effort
controller, commits the ordinary measured-frame fixed weld, waits 80 ms before
release, and reverses through the wheels. It never writes a root pose or twist
after initial placement. Because magnetic attraction is deferred, the physical
controller adds a 1 mm near-contact latch gate inside the pack's broader
acceptance region; the tuned model supports solver steps up to 0.005 s.

The tire/skid friction, effort gains and limits, damping, armature, and skid
shape are provisional simulation parameters. The 40 mm wheel radius and 67.2
mm track come from the Fusion-derived geometry; the 90°/s wheel cap comes from
published SMORES-EP descriptions. Magnetic attraction before latch is not yet
represented.

The Runtime Inspector exposes the two-module dock/release lifecycle as a named
demonstration:

```bash
modsim runtime examples/robot_packs/smores_ep \
  --backend mujoco \
  --demo dock_undock \
  --fixed-connector pan \
  --moving-connector pan \
  --duration 6.0 \
  --no-gravity
```

The SMORES-EP pack also supports a seven-module topology demonstration:

```bash
modsim runtime examples/robot_packs/smores_ep \
  --backend mujoco \
  --demo smores_driver_to_snake \
  --duration 14.0 \
  --no-gravity
```

The second run keeps the ground plane disabled, starts with seven nodes and six
connections, performs four `6 → 5 → 6` connection-count transitions,
and finishes as the chain `1–3–2–4–5–6–7`. Its topology and four connector
replacement pairs are based on Figure 16 and Table III of Liu, Whitzer, and
Yim's 2019
[*A Distributed Reconfiguration Planning Algorithm for Modular Robots*](https://www.modlabupenn.org/wp-content/uploads/2019/08/chao_smores_reconfiguration_2019.pdf)
([DOI `10.1109/LRA.2019.2930432`](https://doi.org/10.1109/LRA.2019.2930432)).

This is sequential kinematic staging through the ordinary backend connection
API, not autonomous planning, actuator control, collision-free locomotion, or
a reproduction of hardware dynamics. Gravity and ground are deliberately off.

The companion physics realization uses the same connector plan but drives the
left/right wheel joints of each moving component:

```bash
modsim runtime examples/robot_packs/smores_ep \
  --backend mujoco \
  --demo smores_physical_driver_to_snake \
  --speed 4
```

Its six initial connections are staged once at time zero. Thereafter all four
replacement actions use effort commands, gravity, ground/tire contact, measured
connector frames, runtime weld release/commit, and one preallocated contact
exclusion paired with each active weld. The final two actions command coherent
wheel targets across connected three-module components. The routes and control
tuning are ModSim-authored physics-demo inputs, not trajectories supplied by
the 2019 topology-planning paper and not autonomous reconfiguration planning.
The MuJoCo adapter remains the authoritative runtime owner, so the 3D viewer,
semantic view, event log, and metrics still describe one session.

## Cross-backend conformance

`tests/test_backend_conformance.py` runs the same pack and scene through every
installed backend and asserts they agree on the semantic picture: the same
modules, the same assemblies, the same connector world frames, the same docking
verdicts, and no drift when nothing is applied.

Comparisons run with gravity disabled, which isolates adapter correctness from
physics — a difference then means the adapter is wrong, not that the modules
fell. This is the test that catches a mismapped link, a connector reported in
the wrong frame, or a lost module.

It also compares the **committed event sequence**: both backends run the same
scripted dock-then-release scenario and must reach the same semantic
conclusions in the same order, along with the same connection identities and
mating orientation indices. Poses legitimately differ once dynamics act; the
decisions do not, because every one of them is made by backend-agnostic core
code from the state the backend reported.

The suite degrades gracefully: MuJoCo tests skip when the extra is not
installed, and conformance skips when fewer than two backends are available.

## Type checking

MuJoCo does not publish complete static declarations for its generated Python
API, so `src/modsim_backend_mujoco` is excluded from the strict core Pyright
configuration and checked separately by `pyright-mujoco.json`. That checker
uses standard mode and suppresses missing generated-attribute reports while
retaining ordinary Python type checking. This mirrors the separate handling of
the Qt-heavy Studio modules.

```bash
pyright                            # core, strict
pyright -p pyright-mujoco.json     # MuJoCo adapter, standard
pyright -p pyright-studio.json     # Studio, strict
```

## Experimental spatial mechanics

The optional backend-local `position_servos` mapping accepts compiled scalar
joint names and `PositionServo(kp, kv, effort_limit)` settings. It creates bounded
position actuators for the spatial benchmark, separately from the established
core effort-command API. The default scene adds no position servos.

`smores_spatial_handoff` uses a ModSim-owned controller and searches with
backend-provided `SpatialMotionServices`. Its in-memory benchmark pack selects
the CAD URDF, disables the pack's planar effort motors, and preserves the
benchmark's 10-degree capture tolerance and provisional 1.2 Nm servos. The
on-disk pack and planar locomotion settings remain unchanged. This preserves the
experimental model described in the manuscript; it does not claim those
actuator limits are hardware ratings. Scalar feedback comes from canonical
backend snapshots; missing joint observations clear previous measurements.
The benchmark sets `exclude_docked_contacts=False` so collision queries also
check faces that will separate during a handoff. Other demos retain the default
docked-body contact exclusions. See
[the algorithm](smores_3d_algorithm.md#modsim-architecture).
