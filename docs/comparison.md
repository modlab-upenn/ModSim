# ModSim vs. Existing Modular-Robot Simulation Tools

*Comparison compiled August 2026. ModSim details are drawn from the repository's
own `README.md`, `docs/backends.md`, and `docs/docking_semantics.md`; every other
tool is sourced from its paper or repository (links at the end).*

## How to read this comparison

The tools below are not all trying to do the same thing, and that is the most
important thing to understand before reading the table. They fall into three
families, and ModSim deliberately sits between them:

- **Dedicated modular-robot simulators** (VisibleSim, ReMod3D, ARGoS) bundle a
  world model *and* an execution model together. You get modules, connections,
  and reconfiguration, but the physics fidelity and the module vocabulary are
  fixed by the tool.
- **General physics engines** (MuJoCo, Isaac Sim, Gazebo, PyBullet, Webots)
  give you excellent dynamics but no concept of a "module," a "connector," or a
  "dock." You build all of that yourself out of welds, fixed joints, or magnets.
- **Modular-robotics Python libraries** (revolve2, scikit-robot) give you
  programmable robot models in Python, but are oriented toward kinematics,
  visualization, or evolutionary morphology rather than runtime docking and
  reconfiguration.

**ModSim's distinctive position is that it is a semantic layer, not a physics
engine.** It owns modular-robot meaning — connector compatibility, docking
acceptance, the lifecycle state machine, logical connections, assembly
membership, the event log — and delegates physics to a swappable backend
adapter (mock or MuJoCo today, Isaac Sim planned). Almost none of the other
tools separate these two concerns, which is exactly why a feature-by-feature
table can look lopsided: some cells are "N/A" not because a tool is weak but
because it lives in a different layer of the stack.

## Feature matrix

| | **ModSim** | **VisibleSim** | **ReMod3D** | **ARGoS** | **revolve2** | **scikit-robot** | **General engines** (MuJoCo / Isaac / Gazebo / PyBullet) |
|---|---|---|---|---|---|---|---|
| **Primary purpose** | Backend-agnostic semantic layer for modular / multi-robot systems | Behavioral simulation of lattice modular robots | Physics-based sim for self-reconfigurable robots | Multi-engine swarm-robotics simulator | Evolutionary modular-robot research framework | Kinematics, planning & visualization library | General rigid-body physics simulation |
| **Language / API** | Python-first | C++ (BlockCode programs) | C++ / Unreal Engine | C++ (with Lua/Python bindings) | Python | Pure Python | C/C++ core, Python bindings |
| **Physics model** | None of its own — delegated to backend adapter | None — discrete-event / behavioral | Full rigid-body (Unreal / PhysX) | Pluggable multi-engine (2D/3D dynamics or kinematics) | Full rigid-body via MuJoCo | None (kinematics only; optional PyBullet) | Full rigid-body, contacts, gravity |
| **Module / connector model** | First-class: Robot Packs describe hardware + typed connectors + docking semantics | First-class: lattice cells, neighbor connections | First-class: reconfigurable modules | Generic robot entities; modularity is in the software architecture, not the robot model | Modular bodies (body + brain), tree morphology | Generic URDF robot; no module/connector concept | None — you model modules as bodies and roll your own joints |
| **Runtime docking / undocking** | Yes — two-phase commit; connection committed only after backend confirms the constraint | Yes — lattice attach/detach via motion rules | Yes — docking between modules | Not a built-in primitive | No runtime docking (fixed morphology per evaluation) | No | Only if you build it (welds / fixed joints / magnets) |
| **Reconfiguration / topology tracking** | Logical connections, assembly membership, immutable model-view snapshots, event log | Strong — designed for self-reconfiguration algorithms at scale | Yes — reconfiguration is the point | Via user code | Morphology is evolved, not reconfigured at runtime | No | No |
| **Backend / engine swappability** | Core design goal — open backend registry, cross-backend conformance tests | No (single behavioral engine) | No (tied to Unreal) | Yes — swap physics engines per simulation | Tied to MuJoCo | N/A | N/A (each *is* the engine) |
| **Physics fidelity** | As high as the chosen backend (full rigid-body under MuJoCo) | Low by design (behavior, not dynamics) | High | Medium–high, engine-dependent | High (MuJoCo) | N/A | High |
| **Scale (module count)** | Small assemblies (demos in the 2–7 module range) | Very high — up to millions of modules | Moderate | High (swarms of many robots) | Small–moderate | N/A | Low–moderate (contact cost grows) |
| **GUI / visualization** | PySide6/PyVista Studio editor + live Runtime Inspector + MuJoCo native viewer | Built-in 3D OpenGL viewer | Unreal-based 3D | Qt-OpenGL 3D viewer | MuJoCo viewer | Trimesh / PyRender / Viser, browser IK | Native viewers per engine |
| **Actuator / joint command control** | Not yet — scenario-level module driving only; no joint-command contract yet | Motion rules, not torque control | Yes | Yes | Yes (CPG brains drive joints) | Commands via PyBullet/ROS interfaces | Yes |
| **Autonomous reconfiguration planning** | Not yet — sequential kinematic staging, no planner | Yes — a primary research use | Yes | Via user code | N/A | No | No |
| **Import format** | URDF → self-contained Robot Pack (split-YAML) | Custom module definitions | Custom | Custom XML | Custom morphology spec | URDF | URDF / MJCF / SDF |
| **License / openness** | Project repo (pre-alpha) | Open source (AGPLv3) | Open source (research, dormant) | Open source (MIT) | Open source | Open source (MIT) | Open source (varies) |
| **Maintenance status** | Active, pre-alpha | Actively maintained (FEMTO-ST) | Largely dormant (~10 commits) | Mature, maintained | Active (CI-Group, VU Amsterdam) | Active | All actively maintained |

## Tool-by-tool notes

**VisibleSim** (FEMTO-ST / programmable-matter project). The closest thing to a
purpose-built modular-robot simulator in the academic world. It is a *behavioral*
framework using discrete-event simulation: modules run identical distributed
C++ "BlockCode" programs and communicate by message passing, and the scheduler
executes actions rather than integrating physics. Its headline strength is
scale — it can simulate up to millions of lattice modules (Blinky Blocks, 2D/3D
Catoms) on a single machine — which makes it the reference tool for
self-reconfiguration and distributed-algorithm research. Its headline limitation
is the flip side: there is no rigid-body dynamics, no gravity, no contact
forces, so it cannot tell you whether a reconfiguration is physically stable,
only whether the algorithm is correct. It is also lattice-centric, so
free-form or chain robots are awkward. *ModSim contrast:* ModSim trades
VisibleSim's massive scale for real physics fidelity and backend
independence — it can put a two-module dock under a full MuJoCo weld and check
the actual constrained pose, which VisibleSim structurally cannot.

**ReMod3D** (USC Polymorphic Robotics Lab). A physics-based simulator built for
autonomous self-reconfigurable robots, layered on Unreal Engine for
high-performance rendering and dynamics. Conceptually it is the family member
closest to what ModSim does under MuJoCo: real physics plus docking and
reconfiguration. In practice it is a research artifact — the public repository
shows only a handful of commits and no recent activity, it is tied to Unreal
(a heavy, non-Python toolchain), and it is not packaged for general reuse.
*ModSim contrast:* ModSim aims for the same physics-plus-semantics goal but
through a Python-first, pip-installable, backend-swappable design with an
explicit conformance suite, rather than a single monolithic engine binding.

**ARGoS** (multi-engine swarm simulator, Pincy et al.). Not a modular-robot
simulator per se, but frequently cited alongside them because its architecture
is genuinely modular *in software*: you can assign different physics engines to
different regions of the same simulation and swap sensors/actuators as plugins.
It excels at large heterogeneous swarms in real time. It has no built-in
concept of physical docking or a self-reconfiguration lifecycle — those live in
user code. *ModSim contrast:* ARGoS makes the *simulator* modular; ModSim makes
the *robot semantics* modular. ARGoS's pluggable-engine idea is philosophically
similar to ModSim's backend registry, but ARGoS still expects you to author
connection logic yourself, whereas that logic is ModSim's core product.

**revolve2** (CI-Group, VU Amsterdam). A Python framework for evolutionary
robotics and modular-robot research, using MuJoCo as its physics backend. It
represents robots as modular bodies (a tree of modules) paired with "brains"
(e.g. CPG controllers) and evolves morphologies and controllers. It shares
ModSim's Python-first, MuJoCo-backed footing, but its axis of interest is
*evolving and evaluating fixed morphologies*, not runtime docking, undocking,
or topology change. There is no docking lifecycle or connector-acceptance
model. *ModSim contrast:* revolve2 answers "what body/brain performs best";
ModSim answers "did these two connectors legally mate, and is the assembly's
topology what I think it is." They could even be complementary layers.

**scikit-robot** (iory) — the tool you named. A lightweight pure-Python library
for robot kinematics, motion planning, collision-free trajectory optimization,
and visualization (Trimesh/PyRender/Viser, interactive browser IK), loading
robots from URDF and offering PyBullet/ROS command interfaces. It is
well-maintained and pleasant, but it is a *single-robot kinematics and viz*
library: no modular/self-reconfigurable robot concept, no connectors, no
docking, and it is not itself a physics engine. *ModSim contrast:* the overlap
is mostly "Python + URDF + visualization." Everything ModSim exists to do —
typed connectors, docking acceptance, multi-module assemblies, reconfiguration
events — is outside scikit-robot's scope. It is a useful baseline precisely
because it shows what a strong general Python robotics library looks like
*without* modular semantics.

**General engines — MuJoCo, Isaac Sim, Gazebo, PyBullet, Webots, CoppeliaSim.**
These provide the physics ModSim (and revolve2, and ReMod3D) depend on, and
you *can* simulate a modular robot in any of them. But none has a native notion
of a module boundary, a typed connector, a dock/undock lifecycle, or logical
topology. You represent modules as bodies, attach them with fixed joints,
equality/weld constraints, or simulated magnets, and you write your own code to
decide when a connection forms, whether it is valid, and what the resulting
assembly is. That is a large amount of bespoke, error-prone glue — which is the
gap ModSim is built to fill. *ModSim contrast:* ModSim is not a competitor to
these engines; it is a consumer of them. Its MuJoCo adapter is exactly this
kind of glue, written once, generically, with the connector-to-weld geometry
factored out and tested — so that the modular semantics stay identical no
matter which engine sits underneath.

## Where ModSim leads, and where it is still behind

**ModSim's differentiators**

- *Clean separation of semantics and physics.* Alone in this set, ModSim treats
  "what a dock means" and "what the physics does" as separately testable layers,
  with a cross-backend conformance suite that asserts a mock kinematic backend
  and a full MuJoCo backend reach the *same* semantic conclusions.
- *Backend-agnostic by construction.* An open backend registry means a new
  engine (Isaac Sim is the stated next target) can be added without touching
  modular logic — a portability story none of the dedicated simulators offer.
- *Two-phase docking correctness.* A logical connection is committed only after
  the backend confirms the physical constraint, so ModSim cannot silently
  "fake" a latch the physics never made — a subtle guarantee that hand-rolled
  glue on a general engine usually lacks.
- *Python-first authoring with real tooling.* URDF import into self-contained
  Robot Packs, a Studio GUI editor, and a live Runtime Inspector give it an
  authoring/inspection workflow closer to a product than a research script.

**ModSim's current gaps (per its own README)**

- *Scale.* Demonstrations are in the 2–7 module range; VisibleSim operates at
  millions and ARGoS at large swarm counts. ModSim is not (yet) a
  many-thousand-module tool.
- *No actuator/joint-command control yet.* Modules can be driven at the scenario
  level, but there is no joint-command contract mapping to real MuJoCo
  actuators — ReMod3D, revolve2, and the general engines all do torque/joint
  control today.
- *No autonomous reconfiguration planning.* Reconfiguration is currently
  sequential kinematic staging through the connection API, not a planner;
  VisibleSim and ReMod3D target autonomous reconfiguration directly.
- *Physics coverage still partial under MuJoCo.* Constraint-force reporting and
  contact exclusion on dock are not yet implemented, and non-fixed connections
  are unfinished. Isaac Sim support is planned, not present.
- *Pre-alpha maturity.* The established tools have years of papers and users
  behind them; ModSim is early.

**One-line summary:** ModSim is best understood not as "another modular-robot
simulator" but as the *missing semantic middle layer* — the piece that
VisibleSim implements only behaviorally, that ReMod3D welds to one engine, and
that general engines like MuJoCo leave entirely to the user. Its bet is that
keeping modular meaning separate from, and portable across, physics backends is
worth more than bundling them — at the cost, for now, of scale, actuator
control, and autonomous planning that more mature tools already have.

## Sources

- ModSim — repository `README.md`, `docs/backends.md`, `docs/docking_semantics.md` (local project)
- VisibleSim — [Behavioral simulation framework for lattice modular robots (ScienceDirect)](https://www.sciencedirect.com/science/article/abs/pii/S0921889021001986); [FEMTO-ST project page](https://projects.femto-st.fr/programmable-matter/visiblesim-behavioral-modular-robotic-simulation-framework)
- ReMod3D — [IEEE paper](https://ieeexplore.ieee.org/document/6696970/); [USC-PRL GitHub](https://github.com/USC-PRL/ReMod3D); [project page](https://robots.isi.edu/remod3d/index.html)
- ARGoS — [ilpincy/argos3 GitHub](https://github.com/ilpincy/argos3); [ARGoS: a modular, parallel, multi-engine simulator (Swarm Intelligence)](https://link.springer.com/article/10.1007/s11721-012-0072-5)
- revolve2 — [ci-group/revolve2 GitHub](https://github.com/ci-group/revolve2)
- scikit-robot — [iory/scikit-robot GitHub](https://github.com/iory/scikit-robot); [documentation](https://scikit-robot.readthedocs.io/)
- General-engine modular evaluation — [Evaluating Simulation Platforms for Modular Mobile Robotic Systems (Machines, MDPI)](https://www.mdpi.com/2075-1702/14/6/666)
- Related survey — [A survey of autonomous self-reconfiguration methods for robot-based programmable matter (ScienceDirect)](https://www.sciencedirect.com/science/article/abs/pii/S0921889019301459)
