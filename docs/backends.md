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
| Connector frames | composed from local pose | measured from sites |
| Runtime docking | yes | yes, via the weld pool |
| Constraint forces | injectable, for tests | **not yet reported** |
| Contact exclusion on dock | not applicable | **not yet** |

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

### mujoco

`load` composes one MuJoCo model from per-instance copies of each module's
mechanical asset:

- each placement is attached under its own name prefix, so `generic_cube_0`'s
  base link becomes the body `generic_cube_0/base_link`;
- every module's root link gets a free joint, because a URDF root attaches
  rigidly to the world and modules must be able to move;
- every connector becomes a site at its authored local pose, namespaced as
  `<module>/connector/<connector>`;
- a pool of inactive weld equality constraints is reserved;
- optionally a ground plane, off by default so it cannot intersect a module
  placed at the origin.

Options: `gravity`, `timestep_s`, `weld_pool_size`, `ground`, `ground_height_m`.
MuJoCo's integrator step is a model property, so `step(dt_s)` covers the
requested interval with whole solver steps and `snapshot().time_s` reports the
time actually reached.

#### Docking via the weld pool

MuJoCo fixes model topology at compile time, so docking cannot *create* a
constraint — it claims one of the reserved welds, re-points it, and activates
it. `create_physical_connection` writes `eq_obj1id`, `eq_obj2id`, `eq_objtype`,
and `eq_data`, then sets `eq_active`. Release deactivates the slot and returns
it to the pool. An exhausted pool refuses the connection, which the two-phase
commit reports as `DockFailed` rather than faking a latch.

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
inspected.

**On macOS this must run under `mjpython`.** MuJoCo's passive viewer needs to own
the main thread, so `python` raises. The MuJoCo wheel installs `mjpython`
alongside `python` in the same environment:

```bash
mjpython -m modsim run path/to/pack --backend mujoco --view
```

The adapter translates MuJoCo's error into that instruction rather than letting
a raw traceback through.

#### Driving modules

`MuJoCoBackendAdapter` implements `SupportsModuleKinematics`, so a scenario can
place and drive modules directly. `set_module_twist` takes **world-frame**
velocities: a free joint stores angular velocity in the body frame, and the
adapter converts, so callers work in world coordinates everywhere in ModSim.
`apply_module_wrench` applies a persistent force that survives contact and
gravity, which is what driving against resistance needs.

These methods are scenario controls over a module's root free joint, not robot
actuator commands. `supports_joint_commands` remains false until ModSim has a
joint command contract and the adapter maps it to real MuJoCo actuators.

`modsim run --fixed-connector ID --moving-connector ID` uses the measured root
and connector frames to arrange exactly two modules, then drives along the
fixed connector's docking axis. This is preferred over the legacy row layout
for robot packs whose connectors are not aligned with world X.

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
