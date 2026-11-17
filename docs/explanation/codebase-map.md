---
title: Source map and reading guide
sidebar_label: Source map
---

Use this map to locate a responsibility after reading the
[architecture overview](index.md). It inventories **every Python file under
`src/` in this documentation revision**: 115 files across the core package
(79), MuJoCo adapter (7), and Studio (29), grouped by their owning subsystem. The
architecture chapters explain how those files cooperate; the
[API reference](../reference/python-api.mdx) describes individual public objects
in the currently published API slice.

## Start with the behavior you are tracing

| Question | Read these entry points first |
| --- | --- |
| How does a library name become a RobotPack? | `user_data.resolve_robot_pack`, then `RobotPackLoader.load`. |
| Where is a Studio edit held before Save? | `StudioProject` and its replacement methods, then `MainWindow._apply_project_change`. |
| How does importing URDF create semantic files? | `URDFImporter.load`, `DraftPackBuilder.build`, and `RobotPackWriter`. |
| Where does the simulation loop live? | `RuntimeSession.create` and `step`, then the selected scenario's `step`. |
| Why was a connector pair accepted or refused? | `DockingManager.run`, compatibility, acceptance, and guards. |
| Where is a connection actually created in MuJoCo? | `MuJoCoBackendAdapter.create_physical_connection` and the weld/hinge pools. |
| Who owns simulation when the Inspector is open? | `RuntimeInspectorRunner`, then the worker or process controller. |
| Why is the displayed graph stale or different from a target? | `ModelViewFactory`, source revisions, Inspector frame construction, and the presenter. |
| Where is a route or pivot generated? | `planning/`, then the physical or kinematic executor selected by the runner. |

The stable path through one experiment is **resolve → load → validate → place →
create session → step and commit → derive views → present → shutdown**. Start
with that path before reading rendering widgets or large calibrated scenarios.

## Understand module boundaries

All paths in the inventory are relative to `src/`. `__init__.py` files define
package exports and `__main__.py` files define module launch behavior. They are
included so the inventory has no hidden omissions. The entry-point column names
representative public types or functions; it is a reading aid, not a complete
symbol index. Private helpers are explained in their subsystem chapter when
they determine important behavior.

## Application entry points

`src/modsim_robotics/`

Entry points translate user selections into shared services. They do not implement a separate robot model. See [application entry points architecture](index.md).

| File | Responsibility | Useful entry points |
| --- | --- | --- |
| `__init__.py` | ModSim public package. | Package exports |
| `__main__.py` | Run ``modsim-robotics`` with ``python -m modsim_robotics``. | Module launcher |
| `cli.py` | ModSim command-line interface for ``modsim-robotics``. | `main`<br/>`setup_command`<br/>`studio_command` |
| `cli_options.py` | Shared CLI option definitions and scenario argument validation. | `OutputFormat`<br/>`ScenarioOptions` |
| `cli_render.py` | Rich/JSON rendering helpers for the ModSim CLI. | `render_model_view_catalog`<br/>`render_generated_model_view`<br/>`render_docking_session` |
| `user_data.py` | Explicit initialization of per-user ModSim data and the bundled starter pack. | `UserDataError`<br/>`InstalledRobotPack`<br/>`SetupResult` |

## RobotPack schema, validation, and persistence

`src/modsim_robotics/robot_packs/`

This boundary owns durable semantic input and safe file operations. See [robotpack schema, validation, and persistence architecture](robot-pack-pipeline.md).

| File | Responsibility | Useful entry points |
| --- | --- | --- |
| `__init__.py` | Robot Pack schemas, persistence, and validation. | Package exports |
| `errors.py` | Robot Pack loading and writing exceptions. | `RobotPackError`<br/>`RobotPackLoadError`<br/>`RobotPackWriteError` |
| `issues.py` | Structured validation issues shared by the CLI and Studio. | `Severity`<br/>`ValidationIssue`<br/>`ValidationReport` |
| `loader.py` | Robot Pack manifest and split-document loader. | `RobotPackLoader` |
| `schema.py` | Typed Robot Pack configuration models. | `RobotPack`<br/>`LoadedRobotPack`<br/>`ModuleType`<br/>`ConnectorTypeSpec`<br/>`ModelViewSpec` |
| `validator.py` | Cross-document and local-filesystem Robot Pack validation. | `ValidationProfile`<br/>`RobotPackValidator` |
| `writer.py` | Deterministic, non-destructive Robot Pack writer. | `RobotPackWriter` |

## Mechanical import

`src/modsim_robotics/importers/`

Mechanical parsing produces typed assets before a draft pack is authored. See [mechanical import architecture](robot-pack-pipeline.md).

| File | Responsibility | Useful entry points |
| --- | --- | --- |
| `__init__.py` | Mechanical-asset import and Robot Pack draft creation. | Package exports |
| `draft_builder.py` | Create a strict draft Robot Pack from an imported URDF. | `DraftBuildResult`<br/>`DraftPackBuilder` |
| `urdf.py` | Typed, dependency-light URDF import for Robot Pack authoring. | `URDFImporter`<br/>`ImportedRobotAsset`<br/>`ImportedLink`<br/>`ImportedJoint` |

## Canonical runtime data

`src/modsim_robotics/core/`

State, stable IDs, scenes, observations, and events are shared across all clients and adapters. See [canonical runtime data architecture](runtime-model.mdx).

| File | Responsibility | Useful entry points |
| --- | --- | --- |
| `__init__.py` | Canonical runtime state for modular robot worlds. | Package exports |
| `assemblies.py` | Derived index of physically connected components. | `MergeResult`<br/>`SplitResult`<br/>`AssemblyIndex` |
| `entities.py` | Mutable runtime entities tracked by a ModSim world. | `ModuleInstance`<br/>`ConnectorInstance`<br/>`ConnectionRuntime`<br/>`JointCommand` |
| `events.py` | Runtime events and the append-only event log. | `Event`<br/>`DockCommitted`<br/>`UndockCommitted`<br/>`EventLog` |
| `ids.py` | Stable runtime identifiers. | `connector_instance_id`<br/>`split_connector_instance_id`<br/>`joint_instance_id` |
| `scene.py` | Multi-module scene composition. | `SceneError`<br/>`ModulePlacement`<br/>`SceneSpec` |
| `snapshot.py` | The read-only state contract a backend returns to ModSim each step. | `BodyState`<br/>`JointState`<br/>`BackendStateSnapshot` |
| `state.py` | Canonical modular-robot world state. | `WorldState`<br/>`WorldStateRevision` |
| `transforms.py` | Rigid-body transforms for the ModSim core. | `Transform`<br/>`quat_from_rpy`<br/>`quat_rotate` |
| `validation.py` | Small reusable validators for finite numeric configuration values. | `require_finite`<br/>`require_finite_nonnegative`<br/>`require_finite_positive` |

## Connector semantics

`src/modsim_robotics/connectors/`

The connector pipeline decides eligibility and lifecycle; the backend confirms physical operations. See [connector semantics architecture](docking-lifecycle.md).

| File | Responsibility | Useful entry points |
| --- | --- | --- |
| `__init__.py` | Connector semantics: compatibility, acceptance, guards, and docking execution. | Package exports |
| `acceptance.py` | Acceptance-region evaluation for a candidate connector pair. | `AcceptanceCriterion`<br/>`AcceptanceResult` |
| `broadphase.py` | Broad-phase candidate detection over connector world positions. | `SpatialHash` |
| `compatibility.py` | Connector-type compatibility. | `CompatibilityResult` |
| `docking.py` | Docking and undocking execution. | `DockProposal`<br/>`DockingManager` |
| `guards.py` | Guards evaluated before a dock or undock is allowed to proceed. | `GuardResult` |

## Backend contracts and mock execution

`src/modsim_robotics/backends/`

The registry selects lazily loaded adapters behind a backend-neutral protocol. See [backend contracts and mock execution architecture](backends.md).

| File | Responsibility | Useful entry points |
| --- | --- | --- |
| `__init__.py` | Backend adapter contract and the built-in mock backend. | Package exports |
| `base.py` | The contract between ModSim semantics and a physics backend. | `BackendAdapter`<br/>`BackendCapabilities`<br/>`ConnectionRequest`<br/>`ConnectionOutcome` |
| `mock.py` | A dependency-free kinematic backend. | `MockBackendAdapter` |
| `registry.py` | Named backend selection. | `BackendEntry`<br/>`create_backend`<br/>`register_backend` |

## Runtime coordination and scenario execution

`src/modsim_robotics/runtime/`

Sessions coordinate world state; scenarios add control, staging, measured completion, and diagnostics. See [runtime coordination and scenario execution architecture](planning-and-scenarios.md).

| File | Responsibility | Useful entry points |
| --- | --- | --- |
| `__init__.py` | Public runtime API. | Package exports |
| `coordinated_pivot.py` | Coordinated rigid-slab pivots for lattice modular robots. | `CoordinatedPivotAction`<br/>`CoordinatedPivotPlan`<br/>`CoordinatedMomentumTelemetry` |
| `demos.py` | Stable names for Runtime Inspector demonstration entry points. | `RuntimeDemo` |
| `differential_drive.py` | Backend-neutral differential-drive kinematics and effort controllers. | `DifferentialDriveGeometry`<br/>`RigidAssemblyModuleTarget`<br/>`RigidAssemblyWheelAllocation` |
| `inspection.py` | Immutable transport objects for runtime-inspector clients. | `RuntimeInspectionError`<br/>`RuntimeEventRow`<br/>`RuntimeInspectorFrame` |
| `inspection_protocol.py` | Versioned newline JSON protocol for out-of-process runtime inspection. | `RuntimeInitialize`<br/>`RuntimeFrame`<br/>`RuntimeSetPaused`<br/>`RuntimeFinished` |
| `inspector_runner.py` | Qt-free execution boundary for one Runtime Inspector demonstration. | `RuntimeInspectorConfig`<br/>`RuntimeInspectorRunner`<br/>`RuntimeScenario` |
| `kinematic_pivot.py` | Backend-neutral kinematic pivot execution for authored reconfiguration routes. | `KinematicPivotRoute`<br/>`KinematicPivotConfig`<br/>`KinematicPivotScenario` |
| `mblocks_lattice.py` | Measured tabletop lattice observations and generated connector actions. | `rectangle_state`<br/>`line_goal`<br/>`starter_state` |
| `mblocks_online_planning.py` | Feedback execution of generated planar M-Blocks pivots. | `OnlineLatticeScenario` |
| `metrics.py` | Modular-robot metrics derived from world state and the event log. | `DockingMetrics` |
| `momentum_pivot.py` | Backend-neutral momentum-driven edge-pivot orchestration. | `MomentumPivotConfig`<br/>`MomentumPivotTelemetry`<br/>`MomentumPivotScenario` |
| `momentum_sequence.py` | Sequential orchestration for physics-driven M-Blocks edge pivots. | `MomentumPivotSequencePlan`<br/>`MomentumPivotSequenceScenario` |
| `online_planning.py` | Single-owner, feedback-driven execution of planar assembly plans. | `OnlineAssemblyScenario` |
| `pacing.py` | Dependency-free wall-clock pacing helpers for runtime presentations. | `wall_clock_deadline_s` |
| `physical_reconfiguration.py` | Wheel-driven execution of an authored modular-robot reconfiguration plan. | `DifferentialDriveReconfigurationScenario` |
| `physics_docking.py` | Physical two-module docking driven through differential-drive wheel joints. | `DifferentialDriveDockingScenario` |
| `reconfiguration.py` | Backend-neutral orchestration for scripted modular-robot reconfiguration. | `ReconfigurationPlan`<br/>`ReconfigurationStatus`<br/>`ScriptedReconfigurationScenario` |
| `session.py` | The runtime loop that binds a Robot Pack, a world, and a backend together. | `JointCommandError`<br/>`RuntimeSession` |

## Generated model views

`src/modsim_robotics/model_views/`

Builders derive immutable projections; the factory owns recipe resolution and bounded caching. See [generated model views architecture](model-views.md).

| File | Responsibility | Useful entry points |
| --- | --- | --- |
| `__init__.py` | Backend-neutral mathematical model-view generation. | Package exports |
| `base.py` | Contracts for backend-neutral model-view generation. | `ModelViewBuilder`<br/>`ModelViewContext`<br/>`ModelViewDescriptor` |
| `factory.py` | Registration, selection, and bounded caching for model-view builders. | `ModelViewFactory` |
| `lattice.py` | Built-in cubic-lattice model-view generation. | `CubicLatticeBuilder` |
| `models.py` | Immutable, backend-neutral model-view result objects. | `ModelViewSourceStamp`<br/>`ModuleTopologyGraphView`<br/>`CubicLatticeView` |
| `topology.py` | Built-in module-topology graph generation. | `ModuleTopologyGraphBuilder` |

## Assembly planning and routing

`src/modsim_robotics/planning/`

Planning models, assignment, and routing express intent separately from physics. See [assembly planning and routing architecture](planning-and-scenarios.md).

| File | Responsibility | Useful entry points |
| --- | --- | --- |
| `__init__.py` | Backend-independent planar assembly planning. | Package exports |
| `assembly.py` | Liu et al. parallel planar assembly, with explicit fixed-mapping reconfiguration. | `minimum_assignment`<br/>`tree_root`<br/>`plan_assembly` |
| `models.py` | Immutable planning inputs and observations; independent of physics and GUI libraries. | `AssemblyGoal`<br/>`AssemblyPlan`<br/>`AssemblyAction`<br/>`PlanningSnapshot` |
| `routing.py` | Bounded space-time A* with oriented assembly footprints and swept checks. | `Footprint`<br/>`Obstacle`<br/>`Reservation` |
| `smores.py` | SMORES-EP reference goals, separate from Robot Pack hardware semantics. | `mobile_manipulator_goal`<br/>`paper_initial_poses`<br/>`driver_to_snake_goal` |

## M-Blocks geometric planning

`src/modsim_robotics/planning/mblocks/`

Planar geometry and planning are separate from flywheel/hinge execution. See [m-blocks geometric planning architecture](planning-and-scenarios.md).

| File | Responsibility | Useful entry points |
| --- | --- | --- |
| `__init__.py` | Dependency-light planar M-Blocks planning. | Package exports |
| `geometry.py` | Exact grid masks for the swept volume of 90/180 degree cube pivots. | `add`<br/>`sub`<br/>`neighbors` |
| `models.py` | Immutable inputs and diagnostics for single-plane pivoting cubes. | `LatticeBlock`<br/>`LatticeState`<br/>`LatticeGoal` |
| `planner.py` | Original implementation of Sung et al.'s 2015 planar line construction. | `LatticePlanningError` |

## Packaged demonstration configurations

`src/modsim_robotics/demo_configs/`

These Python configurations construct maintained examples; RobotPacks remain data and mechanical assets. See [packaged demonstration configurations architecture](planning-and-scenarios.md).

| File | Responsibility | Useful entry points |
| --- | --- | --- |
| `__init__.py` | Packaged, backend-neutral configurations for the built-in demonstrations. | Package exports |
| `mblocks_five_module_pivot.py` | Authored five-module M-Blocks kinematic traversal example. | `build_plan`<br/>`build_routes` |
| `mblocks_twelve_module_line.py` | Twelve-module M-Blocks structure-to-line reference plan. | `build_plan`<br/>`build_routes` |
| `mblocks_twelve_module_physics.py` | Twelve-module one-plane M-Blocks physics demonstration. | `build_scene`<br/>`build_reconfiguration_plan`<br/>`build_plan` |
| `mblocks_twelve_module_staircase.py` | Twelve M-Blocks reconfigure from a 2 x 6 mat into a 3-step staircase. | `build_scene`<br/>`build_plan`<br/>`build_kinematic_scenario` |
| `mblocks_two_module_momentum_pivot.py` | Two-module M-Blocks momentum-pivot physics bootstrap. | `build_scene`<br/>`build_config`<br/>`build_scenario` |
| `smores_driver_to_snake.py` | SMORES-EP Driver-to-Snake example plan from Liu, Whitzer, and Yim (2019). | `build_plan` |
| `smores_ep_diff_drive_dock_undock.py` | SMORES-EP parameters for the physical two-module docking demonstration. | `build_config` |
| `smores_ep_physical_driver_to_snake.py` | Physical SMORES-EP execution parameters for Driver-to-Snake. | `build_config` |

## MuJoCo implementation

`src/modsim_robotics_backend_mujoco/`

The optional adapter owns compiled mechanics, native handles, actuators, constraints, and native viewer execution. See [mujoco implementation architecture](backends.md).

| File | Responsibility | Useful entry points |
| --- | --- | --- |
| `__init__.py` | MuJoCo backend adapter for ModSim. | Package exports |
| `adapter.py` | MuJoCo backend adapter. | `MuJoCoBackendAdapter` |
| `hinges.py` | Runtime hinge management over pre-allocated MuJoCo connect constraints. | `HingePoolExhaustedError`<br/>`HingeAnchorPair`<br/>`HingeSlot` |
| `runtime_inspector_process.py` | No-Qt MuJoCo runtime host for the dual-window Runtime Inspector. | `run_runtime_inspector_process`<br/>`main` |
| `scene.py` | Compile a ModSim scene into a single MuJoCo model. | `MuJoCoSceneError`<br/>`CompiledScene` |
| `viewer.py` | Real-time visualisation of a running session. | `run_with_viewer` |
| `welds.py` | Runtime weld management over a pre-allocated equality-constraint pool. | `WeldPoolExhaustedError`<br/>`ContactExclusionPoolExhaustedError`<br/>`WeldSlot` |

## Studio applications

`src/modsim_robotics_studio/`

Studio consumes core APIs and immutable runtime frames. It owns widgets, cameras, themes, and user interaction. See [studio applications architecture](studio-and-inspector.md).

Both application entry points create the Qt application, configure the shared
desktop identity, choose a theme-matched application icon, and paint the startup
splash. Its animated intro precedes the heavier main-window imports and
construction; the prepared window is presented after the splash's three-second
minimum, with initialization time included. Linux desktop
registration runs before any native window is created. The shared loading
helpers also keep Qt-free document work
off the GUI thread during Builder open, import, Save, and Export As operations;
widget and viewport updates remain on the GUI thread. Inspector runtime ownership
still belongs to its worker or process controller, independently of loading UI.

| File | Responsibility | Useful entry points |
| --- | --- | --- |
| `__init__.py` | ModSim Studio desktop application. | Package exports |
| `__main__.py` | Run ``modsim-robotics-studio`` with ``python -m modsim_robotics_studio``. | Module launcher |
| `_branding.py` | Private package-relative production assets, authoritative theme palettes, proportional SVG rendering, application icons, and Qt application identity. | `asset_path`<br/>`theme_palette`<br/>`vector_pixmap`<br/>`application_icon`<br/>`configure_application_identity` |
| `_desktop.py` | Qt-free Windows process identity before Qt startup and Linux per-user launcher/icon registration using the current Python environment. | `APPLICATION_ID`<br/>`configure_native_process_identity`<br/>`register_desktop_entry` |
| `_loading.py` | Private shared wordmark, startup splash, live loading panels, and guarded background document operations. | `BrandLogo`<br/>`StartupSplash`<br/>`LoadingPanel`<br/>`BusyDialog` |
| `_splash_animation.py` | Private SVG-layer animation that assembles the supplied symbol, reveals the outlined wordmark, and pulses the completed startup logo. | `AnimatedBrandLogo` |
| `_window_sizing.py` | Shared logical-pixel window sizing, monitor bounds, font-aware preferences, normal geometry restoration, and main-window dock persistence. | `WindowSizingPolicy`<br/>`manage_window` |
| `app.py` | ModSim Studio process entry point. | `main`<br/>`cli` |
| `appearance.py` | Application-wide Studio colors and local user appearance preferences. | `StudioTheme`<br/>`ThemeManager` |
| `chrome.py` | Shared native Studio header, theme picker, and compact property widgets. | `ThemePicker`<br/>`StudioHeader`<br/>`VectorEdit` |
| `icons.py` | Theme-matched packaged toolbar glyphs, in-memory color selection, and small fallback glyphs for search, joints, and files. | `studio_icon` |
| `launcher.py` | Dependency-friendly launcher for the optional ``modsim-robotics-studio`` command. | `cli` |
| `main_window.py` | Main ModSim Studio Robot Pack Builder window. | `MainWindow` |
| `project.py` | GUI-independent document model for ModSim Studio. | `StudioProject` |
| `runtime_app.py` | Process entry point for the standalone ModSim Runtime Inspector. | `main` |
| `runtime_events.py` | Qt event-table widgets for immutable runtime event rows. | `RuntimeEventTableModel`<br/>`RuntimeEventLogWidget` |
| `runtime_graph.py` | PyQtGraph renderer for immutable runtime graph presentations. | `TopologyGraphWidget` |
| `runtime_lattice.py` | PyQtGraph renderer for immutable cubic-lattice presentations. | `LatticeViewBox`<br/>`CubicLatticeWidget` |
| `runtime_lattice_presenter.py` | Qt-free projection geometry for cubic-lattice Runtime Inspector views. | `LatticeProjection`<br/>`LatticeCamera`<br/>`PresentedGridLine` |
| `runtime_mblocks_planning.py` | Retained planner graphics for the one-plane M-Blocks demonstration. | `LatticePlanningWorkspace` |
| `runtime_planning.py` | Planar workspace, intent overlays, and execution history from immutable frames. | `ActionHistory`<br/>`PlanningWorkspace` |
| `runtime_presenter.py` | Qt-free presentation state for the Studio Runtime Inspector. | `RuntimePresentationError`<br/>`RuntimeEventSequenceError`<br/>`GraphSelection` |
| `runtime_process.py` | Execution controllers for the standalone Runtime Inspector. | `RuntimeViewerLaunchError`<br/>`RuntimeInspectorThreadController`<br/>`RuntimeInspectorProcessController` |
| `runtime_window.py` | Standalone Qt window for immutable ModSim runtime inspection. | `RuntimeInspectorWindow` |
| `runtime_worker.py` | Background execution for the standalone Studio Runtime Inspector. | `RuntimeInspectorWorker` |
| `session_logging.py` | Per-launch file logging for ModSim Studio. | `resolve_session_log_path`<br/>`configure_session_logging`<br/>`shutdown_session_logging` |
| `user_errors.py` | Readable user and diagnostic formatting for Studio input failures. | `IdentifierInputError` |
| `viewport.py` | Embedded PyVistaQt viewport for imported URDF and Robot Pack overlays. | `RobotViewport` |
| `visual_colors.py` | Categorical visualization colors, independent of Qt and application accents. | `CategoryColors` |

## Repository integration and supporting code

The application source is only one part of the development workflow. These
surfaces determine how code is installed, exercised, documented, and released.

| Location | Responsibility and relationship to the architecture |
| --- | --- |
| `examples/robot_packs/` | Robot descriptions and their mechanical assets. Generic Cube is bundled; larger example packs are not distributed as the starter. |
| `examples/scenarios/` | Checkout-oriented scenario entry points that use the packaged runtime and demo configurations. |
| `tests/` | Domain semantics, persistence, adapter conformance, physics regressions, scenario integration, and optional GUI behavior. Test results are evidence for a particular fixture and environment, not arbitrary-layout guarantees. |
| `pyproject.toml`, `uv.lock` | Distribution metadata, package namespaces, extras, the documentation dependency group, Python commands, and locked resolution. |
| `.github/workflows/ci.yml` | Core, MuJoCo, Studio, documentation, and distribution checks with separate dependency requirements. |
| `.github/workflows/release.yml` | Stable-tag validation and promotion of checked artifacts to PyPI and GitHub Releases. It does not deploy the documentation website. |
| `tools/release/` | Version/tag validation, archive-content checks, and installed-artifact smoke tests. |
| `tools/docs/` | Static public-API documentation policy and scoped Pyreverse generation. |
| `website/api/` | The source-static AutoAPI scope, docstring policy baseline, templates, and readable Markdown renderer. |
| `website/scripts/` | API/UML generation orchestration and the local production preview server. |
| `website/src/` | Docusaurus presentation, shared syntax styling, responsive table transformation, and the theme-aware Mermaid renderer. |
| `website/static/theme/` | Shared color, typography, and surface tokens used by the site and diagrams. |
| `docs/` | Authored user tasks, architecture explanation, API guidance, and current contracts. Generated API Markdown is ignored build output. |
| `docs/archive/`, `docs/technical_report/` | Historical plans and reports. They are not the current architecture inventory. |

## Use tests to answer a source-reading question

For persistence, start with `test_loader.py`, `test_writer.py`,
`test_validator.py`, and `test_urdf_importer.py`. For state and connectors, use
`test_world_state_revisions.py`, `test_docking.py`, `test_acceptance.py`, and
`test_assemblies.py`. For the simulator boundary, compare
`test_backend_conformance.py` with the focused MuJoCo constraint/contact tests.
For transport and presentation, use `test_runtime_inspection_protocol.py`,
`test_runtime_runner.py`, `test_runtime_presenter.py`, and the optional widget
checks. Planner tests distinguish geometric plans, executor behavior, and
physical reference runs.

Read the setup and assertions around one relevant test instead of treating the
suite as prose documentation. Tests often make an important invariant concrete:
a rejected backend request must not create a logical edge; a generated view
must include disconnected modules; or a physical controller must not overwrite
root motion after initialization.

## Continue into the architecture

Follow the [RobotPack pipeline](robot-pack-pipeline.md) to understand durable
input, then [Runtime and state](runtime-model.mdx) and
[Docking lifecycle](docking-lifecycle.md) to understand execution. UI work starts
with [Studio and Inspector](studio-and-inspector.md). New planner or view work
starts with [Planning and scenarios](planning-and-scenarios.md) or
[Model views](model-views.md), respectively.
