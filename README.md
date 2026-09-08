# ModSim

ModSim is a Python-first, backend-agnostic framework for modular and multi-robot
systems. Robot Packs describe hardware and docking semantics; ModSim owns the
canonical world state and event log; backend adapters own physics.

The pre-alpha project currently supports these core workflows:

- import a local URDF into a self-contained Robot Pack;
- author and validate Robot Packs with strict split-YAML schemas;
- edit packs in the PySide6/PyVista Studio application;
- run docking and undocking through a backend-neutral runtime;
- simulate fixed and hinge connections, equality-force feedback, and bounded
  joint-effort commands with MuJoCo;
- run physical SMORES-EP differential-drive docking and seven-module
  reconfiguration on a ground plane;
- load the CAD-derived 3D M-Blocks pack, run one-plane momentum-driven
  two- or twelve-module physics demonstrations, and compare them with
  deterministic kinematic traversals, including a 2×6 mat-to-staircase route;
- generate immutable model-view snapshots; and
- inspect live topology or cubic-lattice state, events, status, and metrics in
  the Runtime Inspector.

Isaac Sim integration, actuator/transmission catalogs, autonomous
reconfiguration planning, continuous magnetic interaction, and MuJoCo
compliant/ball/custom connections are not yet implemented. The `simulation`
profile checks structural readiness; it does not launch a simulator.

## Install

ModSim requires Python 3.11 or newer. From a source checkout, install the full
development environment with `uv`:

```bash
uv sync --locked --extra dev --extra studio --extra mujoco --python 3.12
uv run --no-sync modsim --version
```

Or use a virtual environment and pip:

```bash
python3.12 -m venv .venv
.venv/bin/python -m pip install -e ".[studio,mujoco,dev]"
```

The extras are independent: `studio` adds the desktop applications, `mujoco`
adds physics and native-viewer support, and `dev` adds repository tooling. The
base install contains the Robot Pack, model-view, mock-backend, and CLI APIs.

## Robot Packs and CLI

Inspect the minimal example and verify that it is structurally ready to run:

```bash
modsim pack inspect examples/robot_packs/generic_cube
modsim pack validate examples/robot_packs/generic_cube --profile simulation
```

Create a new draft from a concrete URDF (Xacro must first be expanded):

```bash
modsim pack init --from-urdf path/to/module.urdf --out path/to/new_pack
```

Use repeated `--asset-root` options when mesh references cannot be resolved
relative to the URDF. The importer copies resolved local assets and rewrites
their paths so the resulting pack is self-contained.

Other useful command surfaces are:

```bash
modsim views examples/robot_packs/generic_cube
modsim dock examples/robot_packs/generic_cube --count 3
modsim run examples/robot_packs/generic_cube --backend mujoco
modsim backends
```

`dock` exercises connector semantics with the mock backend by default. `run`
executes a scripted lifecycle without a GUI unless `--view` is supplied.

## Studio

Install the `studio` extra, then open a pack in the Robot Pack Builder:

```bash
modsim studio examples/robot_packs/generic_cube
```

Studio supports URDF visualization, semantic property editing, validation,
atomic Save, non-overwriting Export As, and canonical YAML preview. URDF
remains the source of mechanical geometry and kinematics; Robot Pack YAML adds
modular-robot semantics.

See [Studio](docs/studio.md) for the authoring workflow and current UI limits.

## Runtime Inspector

Install both the `studio` and `mujoco` extras, then run the live two-module
docking demonstration:

```bash
modsim runtime examples/robot_packs/generic_cube \
  --fixed-connector front \
  --moving-connector front \
  --duration 4
```

The Runtime Inspector displays the selected immutable model view alongside
event snapshots while a single authoritative process owns the runtime. It can
render the generic module-topology graph or cubic-lattice diagnostics. With
MuJoCo, a separate native 3D viewer opens by default; `--no-viewer` suppresses
it while retaining the Qt inspector. Use `modsim run` for a fully non-GUI
execution.

The larger `examples/robot_packs/smores_ep` pack includes `dock`,
`dock_undock`, `smores_diff_drive_dock_undock`,
`smores_driver_to_snake`, and `smores_physical_driver_to_snake`. The physical
demonstrations use measured joint feedback and bounded MuJoCo effort actuators;
their platform-specific controller parameters and routes live under
`examples/scenarios/`.

See [Runtime Inspector](docs/runtime_inspector.md) for the execution boundary,
controls, and named demonstrations.

## Reproducible SMORES-EP examples

Run these commands from the repository root after installing the `studio` and
`mujoco` extras. If the virtual environment is not activated, replace `modsim`
with `uv run --no-sync modsim` or `.venv/bin/modsim`.

Inspect the pack, then apply both validation profiles:

```bash
modsim pack inspect examples/robot_packs/smores_ep
modsim pack validate examples/robot_packs/smores_ep
modsim pack validate \
  examples/robot_packs/smores_ep \
  --profile simulation
```

Run a two-module docking demonstration using the `pan` face on each module:

```bash
modsim runtime examples/robot_packs/smores_ep \
  --backend mujoco \
  --demo dock \
  --fixed-connector pan \
  --moving-connector pan \
  --model-view smores_topology \
  --connector-gap 0.02 \
  --approach 0.03 \
  --duration 4.0 \
  --dt 0.002 \
  --no-gravity
```

Run the physical two-module demonstration first when evaluating SMORES
locomotion and docking:

```bash
modsim runtime examples/robot_packs/smores_ep \
  --backend mujoco \
  --demo smores_diff_drive_dock_undock \
  --model-view smores_topology
```

That command supplies the physics defaults: MuJoCo, gravity, the ground plane,
a 5 cm initial root height, a 2 cm connector gap, a 2 ms solver step, and a
12-second run. The moving module drives its `pan`/TOP face into the target's
rear `bottom` face through the left/right wheel joints. The graph follows
`0 → 1 → 0` edges while the event log records candidate, dock, merge, undock,
and split events. After an 80 ms release delay, the moving module reverses away
under wheel effort. Both windows retain the final state until Stop or close.
The generic connector acceptance region remains 6 mm, but this physical
controller continues driving until the measured face-frame separation is at
most 1 mm before creating the ideal weld, avoiding a visibly floating latch.

The pack-local MJCF uses Fusion-derived 40 mm tire radius and 67.2 mm track
geometry. The published wheel-speed cap is 90°/s; the controller clips to that
limit. Tire friction, actuator effort limits and gains, joint damping/armature,
and the small rear support skid are explicitly provisional simulation
parameters, not measured hardware constants. Locomotion and contact are
dynamic, but EP-face magnetic attraction is not yet modeled: once measured
acceptance and the 1 mm near-contact gate succeed, the latch is represented by
an ideal fixed weld. Keep `--dt` at or below 0.005 s for this tuned model. See the
[SMORES-EP project](https://www.modlabupenn.org/smores-ep/),
the [assembly controller paper](https://www.modlabupenn.org/wp-content/uploads/2022/03/liu_smores_assembly_2020.pdf),
and the [EP-face characterization](https://www.modlabupenn.org/wp-content/uploads/tosun2016epface.pdf)
for the hardware/control basis.

This first dynamics slice starts the two modules in a deterministic, already
heading-aligned approach. It demonstrates differential-drive actuation and
contact, not yet robust navigation or recovery from arbitrary lateral/yaw
errors.

Run the older kinematic pair through docking, undocking, and visible
retraction:

```bash
modsim runtime examples/robot_packs/smores_ep \
  --backend mujoco \
  --demo dock_undock \
  --fixed-connector pan \
  --moving-connector pan \
  --model-view smores_topology \
  --connector-gap 0.02 \
  --approach 0.03 \
  --retract 0.03 \
  --duration 6.0 \
  --dt 0.002 \
  --no-gravity
```

Finally, run the seven-module Driver-to-Snake demonstration:

```bash
modsim runtime examples/robot_packs/smores_ep \
  --backend mujoco \
  --demo smores_driver_to_snake \
  --model-view smores_topology \
  --duration 14.0 \
  --dt 0.002 \
  --connector-gap 0.02 \
  --approach 0.03 \
  --no-gravity
```

Do not add `--ground` to Driver-to-Snake. It begins with seven nodes and six
connections, performs four `6 → 5 → 6` edge transitions, and ends in the chain
`module_1–module_3–module_2–module_4–module_5–module_6–module_7`. See the
[Runtime Inspector examples](docs/runtime_inspector.md#reproducible-smores-ep-workflows)
for expected events, metrics, connector substitutions, viewer behavior, and a
fully headless two-module alternative.

Run the corresponding wheel-driven physics realization with gravity and the
ground enabled by its demo defaults:

```bash
modsim runtime examples/robot_packs/smores_ep \
  --backend mujoco \
  --demo smores_physical_driver_to_snake \
  --model-view smores_topology \
  --speed 4
```

The initial Driver tree is placed upright once at simulation time zero. Each
published connector replacement then releases the existing weld, drives either
one module or its connected three-module component through an authored route,
and commits the replacement only after measured connector acceptance. No root
pose, root velocity, or external wrench is injected after initialization. The
viewer therefore shows wheel/contact dynamics throughout all four
`6 → 5 → 6` graph transitions.

The command reserves 210 simulated seconds. `--speed 4` requests four simulated
seconds per wall-clock second without changing the physics timestep, motor
limits, controller targets, or event timestamps. The tuned reference run still
completes at about 177 simulated seconds—nominally about 44 seconds of wall time
at 4×—and the full display budget is nominally 52.5 seconds. Omit `--speed` for
real-time pacing, or choose any positive factor such as `2`, `3`, or `0.5` for
slow motion. Pacing is best-effort when the CPU/GPU cannot keep up. Both windows
hold the final chain until Stop or close.

This is a ModSim physics realization of the topology plan in Liu, Whitzer, and
Yim's [2019 Driver-to-Snake example](https://www.modlabupenn.org/wp-content/uploads/2019/08/chao_smores_reconfiguration_2019.pdf),
not a reproduction of published hardware trajectories or an autonomous path
planner. The collision-clearance routes, contact parameters, control gains,
and ideal post-capture weld are provisional simulation choices.

## Reproducible 3D M-Blocks examples

Inspect the CAD-derived pack in Studio and run simulation-readiness validation:

```bash
modsim pack validate \
  examples/robot_packs/mblocks_3d \
  --profile simulation
modsim studio examples/robot_packs/mblocks_3d
```

Generate the generic cubic-lattice model view as JSON:

```bash
modsim views examples/robot_packs/mblocks_3d \
  --view mblocks_lattice \
  --count 5 \
  --spacing 0.05 \
  --output json
```

Run the five-module traversal with its default cubic-lattice view and the
MuJoCo mesh viewer in parallel:

```bash
modsim runtime examples/robot_packs/mblocks_3d \
  --backend mujoco \
  --demo mblocks_five_module_pivot
```

The four-block base retains three connections while `block_5` executes three
authored quarter-circle edge pivots, replacing one face connection after each
arc. The semantic window projects the measured cubes on an isometric grid by
default, with XY/XZ/YZ projection controls, a Z-layer filter, dashed nearest
snap cells, local and global axes, selectable modules/connections/warning
cells, and amber/red off-lattice and occupancy-conflict diagnostics. Use
`--model-view mblocks_topology` when the connectivity-only graph is preferred.

This five-module demonstration is intentionally kinematic: it uses normal ModSim
undock/dock events and physical MuJoCo welds at the endpoints, but it writes
the detached module's root pose along each arc. It does not yet claim
flywheel-, magnetic-hinge-, or contact-driven M-Block dynamics.

Run the two-module one-plane physics bootstrap:

```bash
modsim runtime examples/robot_packs/mblocks_3d \
  --demo mblocks_momentum_pivot
```

Its defaults select MuJoCo, gravity, the ground plane, the grounded physics
lattice view, a 0.00025 s solver step, and a five-second display run. The
moving cube spins its internal flywheel with at most 0.03 N m, transitions from
a fixed face to a +Y two-point hinge, brakes with at most 2.6 N m, captures the
measured target face, and releases the hinge. No root pose, velocity, or wrench
is written after initialization. The event log preserves `fixed` versus
`hinge`, while scenario detail reports flywheel speed, pivot angle, and brake
impulse. `--speed` changes display pacing only.

The controller and pack use the published 0.150 kg mass, `8.4e-6 kg m^2`
flywheel inertia, and 20,000 RPM speed ceiling. Regression runs at 0.5, 0.25,
and 0.1 ms complete around 0.8585–0.8596 simulated seconds, place the moving
root near x = 0.0502–0.0507 m, and integrate 0.00763–0.00767 N m s of braking
impulse. This is a deterministic face-to-hinge-to-face approximation, not a
continuous magnetic-field or full three-plane-carrier model.

Run the larger lattice benchmark:

```bash
modsim runtime examples/robot_packs/mblocks_3d \
  --backend mujoco \
  --demo mblocks_twelve_module_line
```

Eleven connected substrate blocks support `block_12` through ten authored
quarter traverses and one final half-turn, ending as a single 12-cell line near
21.30 simulated seconds inside the default 24-second display run. This larger
demo is intentionally kinematic and paper-inspired; it is neither the exact
2019 hardware move sequence nor a physical or autonomous execution.

Run the corresponding one-plane MuJoCo physics sequence:

```bash
modsim runtime examples/robot_packs/mblocks_3d \
  --demo mblocks_physical_twelve_module_line \
  --speed 4
```

This second command stages the same eleven-connection starting structure at
simulation time zero, then composes eleven momentum-pivot primitives: ten
90-degree surface traverses and one 180-degree roll. Each action uses the
moving block's flywheel effort, a temporary two-point +Y edge hinge, gravity,
contact, measured target-face capture, and ordinary ModSim dock/undock events.
No module root pose, velocity, or wrench is written after initialization. Its
demo-aware defaults select MuJoCo, gravity, ground, a 0.0005 s step, a
12-second simulated-time budget, and the grounded lattice view; `--speed 4`
only requests faster wall-clock playback.

The maintained real-MuJoCo full-route regression at the 0.0005 s demo default
completes all 11 pivots. It finishes with the exact 11 fixed connections, one
twelve-module assembly, cells `x = 0..11`, 33 dock events, 22 undock events,
and every module within the physics lattice's 3 mm / 3 degree tolerances. Root
pose, twist, and wrench controls are guarded against throughout the
post-initialization run; a separate 0.00025 s three-block regression covers the
first surface traversal.

The physics route is still a deterministic one-plane approximation. It does
not model continuous magnetic attraction, force-selected bond transitions,
the real three-plane actuator carrier, autonomous planning, or the unpublished
2019 hardware move trace. See the
[3D M-Blocks integration notes](docs/mblocks_3d.md) for asset provenance,
model-view semantics, the two execution paths, and the remaining fidelity
gates.

### Twelve-module mat-to-staircase demonstration

This demonstration has two complementary execution phases. The reference
executor proves the authored route and exact lattice/topology result; the
physics executor realizes the same eleven actions through MuJoCo dynamics.

| Execution | Motion source | Intended use |
| --- | --- | --- |
| Mock reference | Analytical quarter-circle root poses | Fast route, topology, event-log, and lattice validation |
| MuJoCo reference | The same analytical route with physical endpoint constraints | Inspect the detailed meshes while debugging the authored sequence |
| MuJoCo physics | Bounded flywheel effort, hinge constraints, gravity, and contact | Evaluate the dynamic realization without post-start pose writes |

Run the fast reference executor first:

```bash
modsim runtime examples/robot_packs/mblocks_3d \
  --backend mock \
  --demo mblocks_twelve_module_staircase \
  --no-viewer
```

`--no-viewer` suppresses only the native 3D window. The Runtime Inspector still
opens and shows the changing lattice, connection topology, metrics, status,
and ordered lifecycle events.

Use MuJoCo when a detailed mesh window is useful while retaining the same
analytical reference motion:

```bash
modsim runtime examples/robot_packs/mblocks_3d \
  --backend mujoco \
  --demo mblocks_twelve_module_staircase
```

Then run the full dynamics realization:

```bash
modsim runtime examples/robot_packs/mblocks_3d \
  --backend mujoco \
  --demo mblocks_physical_twelve_module_staircase \
  --speed 4
```

Twelve cubes begin as a face-connected 2×6 mat at cells
`x=0..5, y=0..1, z=0`. Pairs across Y remain rigid slabs. Eleven positive-Y
edge pivots move the three leftmost slabs and finish with a two-block-wide,
three-step structure whose column heights are 1, 2, and 3. Both executors use
the same cyclic-capable topology plan, including simultaneous face releases
and landings when a slab touches two supports.

```text
initial (each cell is duplicated at y=0 and y=1)
z=0:  A B C D E F

final
z=2:          C
z=1:        B A
z=0:      D E F
```

The physical executor commands both flywheels in the moving slab, retains one
temporary two-point edge hinge, and lets MuJoCo integrate every root pose from
gravity, contact, constraints, and reaction torque. It uses 6,000 RPM quarter
turns, 15,000 RPM ground-level half turns, and a 19,500 RPM elevated half turn,
all below the pack's 20,000 RPM reference ceiling. The maintained real-MuJoCo
regression completes all 11 actions in about 9.5 simulated seconds, with 18
final fixed connections, one assembly, 53 dock commits, and 35 undock commits.
The named demo reserves 12 simulated seconds at a 0.0005 s physics step;
`--speed 4` changes wall-clock pacing only and does not alter controller gains,
effort limits, physics, or event timestamps. Both windows retain the terminal
state for inspection after the route completes.

This staircase is a ModSim-designed, paper-inspired demonstration—not a
reproduction of a published M-Blocks hardware trace or an autonomous plan. Its
two-block depth makes the result three-dimensional, but every active pivot is
still in the pack's current +Y plane. Continuous magnet forces and the real
three-plane carrier remain future work. The fixed-world lattice view can mark
some final physical cubes amber because accumulated solver/whole-assembly
drift exceeds its strict 3 mm / 3 degree diagnostic tolerance even when their
nearest integer cells and final connector topology are correct.

## Python API

```python
from pathlib import Path

from modsim import MockBackendAdapter, RobotPackLoader, RuntimeSession, SceneSpec
from modsim.robot_packs import RobotPackValidator, ValidationProfile

loaded = RobotPackLoader().load(Path("examples/robot_packs/generic_cube"))
report = RobotPackValidator().validate(
    loaded,
    profile=ValidationProfile.SIMULATION,
)
report.raise_for_errors()

scene = SceneSpec.grid("generic_cube", count=2, spacing_m=0.1)
session = RuntimeSession.create(loaded.pack, scene, MockBackendAdapter())
session.step(0.01)
```

The backend contract ensures that logical connections are committed only after
the backend confirms their physical constraint. Generated model views remain
derived snapshots rather than canonical state.

## Documentation

- [Long-form technical report](docs/technical_report/modsim_technical_report.tex) —
  LaTeX paper covering motivation, architecture, subsystems, SMORES-EP experiments,
  performance analysis, limitations, and future work
- [Robot Pack format](docs/robot_pack_spec.md) — normative split-YAML schema
- [Docking semantics](docs/docking_semantics.md) — acceptance and lifecycle
- [Backend adapters](docs/backends.md) — runtime/backend responsibilities
- [Model views](docs/model_views.md) — recipes and immutable generated views
- [Runtime Inspector](docs/runtime_inspector.md) — live visualization contract
- [3D M-Blocks integration](docs/mblocks_3d.md) — CAD-derived pack, lattice view,
  physical momentum pivots, kinematic/physics benchmarks, and fidelity gates
- [Studio](docs/studio.md) — desktop authoring workflow
- [Archived design history](docs/archive/) — dated roadmaps and handoff records

## Development checks

```bash
uv lock --check
uv run --no-sync ruff check .
uv run --no-sync ruff format --check .
uv run --no-sync pyright
uv run --no-sync pytest
git diff --check
```

GUI tests require a display or Xvfb. MuJoCo tests require the `mujoco` extra.
