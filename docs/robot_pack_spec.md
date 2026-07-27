# Robot Pack format 0.1

This document is the implemented Phase 1 contract for Robot Packs. The broader
architecture in `HANDOFF.md` is a roadmap; when examples differ, this document
and the typed models in `src/modsim/robot_packs/schema.py` describe the code that
currently runs.

Format 0.1 covers structural authoring, persistence, and validation. The
separate URDF importer can generate a draft format-0.1 pack and source-name
mapping, but format validation still does not launch a simulator or prove that
a backend can execute a pack.

## Directory layout

A Robot Pack is a self-contained directory with a fixed root manifest name:

```text
my_robot_pack/
  robot_pack.yaml
  assets/
    urdf/
    meshes/
    mujoco/
    isaac/
  specs/
    module_types.yaml
    connector_types.yaml
    capabilities.yaml
  mappings/
    urdf_mapping.yaml
```

All YAML documents require `schema_version: "0.1"`. Referenced paths:

- are relative to the pack root;
- use `/`, including on Windows;
- cannot be absolute, home-relative, or contain `..`;
- cannot contain control characters;
- must resolve inside the pack;
- cannot give the same path both an asset and document role.

Asset trees must contain regular files and directories only. Symlinks, FIFOs,
sockets, and device entries are rejected so that a pack remains self-contained.
The pack root, root manifest, and referenced YAML documents also cannot be
symlinks.

Identifiers use lower snake case, begin with a letter, and are at most 64
characters. Pack versions use semantic version syntax such as `0.1.0`.

## Units and directions

All physical values use SI units. Units are carried in field names:

- position: metres (`_m`) or radians (`_rad`);
- linear velocity: metres per second (`_m_per_s` or `_m_s`);
- angular velocity: radians per second (`_rad_per_s`);
- force: newtons (`_n`);
- torque or moment: newton-metres (`_nm`);
- stiffness: newtons per metre (`_n_per_m`) or newton-metres per radian
  (`_nm_per_rad`).

`xyz_m` and `rpy_rad` are three-element vectors. Joint, docking, and approach
axes are three-element unit vectors. Joint axes follow URDF semantics and are
expressed in the source joint frame. Connector docking and approach axes are
expressed in the connector's parent-link frame.

Physical numeric fields accept YAML numbers, including integer literals, but
reject booleans and quoted numeric strings.

An explicit YAML `null` means “known to be unknown.” Missing optional authoring
data and explicit unknown limits produce warnings in the `authoring` profile
and errors in the `simulation` profile when required by a declared control
mode.

## Root manifest

```yaml
schema_version: "0.1"
id: my_robot
name: My Modular Robot
version: 0.1.0
description: Optional description.

assets:
  urdf:
    base_module: assets/urdf/base_module.urdf
    tool_module: assets/urdf/tool_module.urdf
  mesh_directories:
    robot_meshes: assets/meshes
  mujoco: {}
  isaac_usd: {}
  thumbnails: {}

specs:
  modules: specs/module_types.yaml
  connectors: specs/connector_types.yaml
  capabilities: specs/capabilities.yaml

mappings:
  urdf: mappings/urdf_mapping.yaml
```

Asset catalogs are keyed, so a pack can contain multiple module types and
multiple mechanical assets:

- `urdf` is required and contains at least one file.
- `mesh_directories` contains directories copied as part of the pack.
- `mujoco`, `isaac_usd`, and `thumbnails` contain optional files.

Catalog keys are stable asset IDs. Paths must be unique across all catalogs.
A `ModuleType.asset_ref` always names an entry in `assets.urdf`. A backend
mapping declares which mechanical catalog its asset references use.

## Module types

Catalog IDs are YAML mapping keys and are not repeated inside each item:

```yaml
schema_version: "0.1"

module_types:
  base_module:
    name: Base Module
    asset_ref: base_module
    root_link: base_link
    mass_kg: 1.25
    joints:
      - id: wheel
        source_joint_name: wheel_joint
        type: continuous
        parent_link: base_link
        child_link: wheel_link
        axis: [0.0, 1.0, 0.0]
        control_modes: [velocity]
        limits:
          lower_position_rad: null
          upper_position_rad: null
          lower_position_m: null
          upper_position_m: null
          max_velocity_rad_per_s: 12.0
          max_velocity_m_per_s: null
          max_effort_nm: 0.8
          max_effort_n: null
    connectors:
      - id: front
        connector_type: magnetic_face
        parent_link: base_link
        frame: null
        local_pose:
          xyz_m: [0.05, 0.0, 0.0]
          rpy_rad: [0.0, 0.0, 0.0]
        docking_axis: [1.0, 0.0, 0.0]
        approach_axis: [1.0, 0.0, 0.0]
    capabilities: [dock, undock]
```

Module rules:

- At least one module type is required.
- Joint and connector IDs are unique within their module type.
- `asset_ref`, connector types, and capabilities must resolve to their catalogs.
- A capability's required connector types must be present on the advertising
  module.
- A connector needs `frame`, `local_pose`, or both.

Joint types are `fixed`, `revolute`, `continuous`, `prismatic`, `floating`, and
`planar`. Revolute, continuous, prismatic, and planar joints require a unit
axis. Fixed and floating joints forbid an axis. Only revolute, continuous, and
prismatic joints support scalar control modes and limits in format 0.1.

Angular joints use the `_rad`, `_rad_per_s`, and `_nm` limit fields. Prismatic
joints use `_m`, `_m_per_s`, and `_n`. Continuous joints cannot define angular
position bounds. Control modes are `position`, `velocity`, and `effort`.
Actuator and transmission catalogs are intentionally deferred until the URDF
import milestone rather than represented by an unresolvable string reference.

## Connector types

```yaml
schema_version: "0.1"

connector_types:
  magnetic_face:
    name: Magnetic Face
    active: true
    gender: hermaphroditic
    compatible_with: [magnetic_face]
    allowed_orientations:
      mode: discrete
      values_rad: [0.0, 1.5707963267948966, 3.141592653589793]
    acceptance_region:
      shape: box
      position_tolerance_m: 0.006
      orientation_tolerance_rad: 0.13962634015954636
      max_relative_velocity_m_s: 0.05
    physical_connection:
      constraint: fixed
      compliance: null
    limits:
      max_normal_force_n: 90.0
      max_shear_force_n: 40.0
      max_bending_moment_nm: 1.8
    supports_undocking: true
```

Connector genders are `male`, `female`, `hermaphroditic`, and `genderless`.
Orientation mode is either:

- `discrete`, with one or more unique `values_rad`; or
- `continuous`, with an empty `values_rad` list.

Acceptance shapes are `box`, `sphere`, and `cylinder`. Format 0.1 fully models
`fixed` and `compliant` connection intent. A compliant connection requires at
least one positive stiffness:

```yaml
physical_connection:
  constraint: compliant
  compliance:
    translational_stiffness_n_per_m: 10000.0
    rotational_stiffness_nm_per_rad: 1000.0
```

`hinge`, `ball`, and `custom` values may be recorded during authoring, but the
simulation profile reports them as unsupported because their required
parameters are not yet modeled.

## Capabilities

```yaml
schema_version: "0.1"

capabilities:
  dock:
    name: Dock
    kind: primitive_action
    description: Form a compatible connector connection.
    required_connector_types: [magnetic_face]
```

Capability kinds are `primitive_action` and `behavior`. Capability definitions
advertise semantic support; Phase 1 does not execute them.

## Backend mappings

Mappings translate stable ModSim IDs and source names into names understood by a
backend or source format:

```yaml
schema_version: "0.1"
backend: urdf
asset_catalog: urdf

module_types:
  base_module:
    asset_ref: base_module
    link_map:
      base_link: base_link
      wheel_link: wheel_link
    joint_map:
      wheel: wheel_joint
    connector_frame_map: {}
    constraint_map: {}
```

- `asset_catalog` is `urdf`, `mujoco`, or `isaac_usd`.
- `asset_ref` names an entry in that catalog. It may be omitted for an URDF
  mapping, where the module type's URDF asset is the default.
- `link_map` maps source link names to backend link names.
- `joint_map` maps ModSim joint IDs to backend joint names.
- `connector_frame_map` maps ModSim connector IDs to backend frame names.
- `constraint_map` is reserved for backend constraint names.

Every manifest mapping entry must have exactly one corresponding loaded mapping
document, and its `backend` must match the manifest key. Simulation-profile
validation requires mapping coverage for every module type, root link, joint,
and connector that relies only on a named frame.

## Validation profiles

```bash
modsim pack inspect path/to/pack
modsim pack validate path/to/pack
modsim pack validate path/to/pack --profile simulation
modsim pack validate path/to/pack --output json
```

`authoring` is the default. It reports incomplete docking, joint, and mapping
metadata as warnings where continued editing is reasonable. Cross-reference,
schema, path, and asset failures are always errors.

`simulation` promotes completeness warnings to errors. It means “structurally
ready for a future simulator adapter”; it does not run physics, parse the URDF,
or guarantee backend support.

Issues include a stable code, severity, document, JSON-pointer-style path,
optional entity reference, and optional suggested fix. Invalid packs produce
CLI exit status 1; valid packs produce 0.

## Python loading and canonical export

```python
from pathlib import Path

from modsim.robot_packs import (
    RobotPackLoader,
    RobotPackValidator,
    RobotPackWriter,
    ValidationProfile,
)

loaded = RobotPackLoader().load(Path("path/to/my_robot_pack"))
report = RobotPackValidator().validate(
    loaded,
    profile=ValidationProfile.AUTHORING,
)
report.raise_for_errors()

exported = RobotPackWriter().write(
    loaded,
    Path("path/to/new_export_directory"),
)
```

The Phase 1 writer is a canonical exporter:

- the destination must not already exist;
- the supplied destination cannot be a symbolic link;
- a complete export is staged, reloaded, compared with the source model, and
  then atomically published;
- declared assets are copied without following symlinks;
- source files are never modified;
- YAML ordering and output are deterministic for a given model;
- comments, anchors, quote choices, and hand formatting are not preserved.

Model attributes and sequence fields are frozen, but keyed catalogs are
currently shallow-frozen Python dictionaries. Treat loaded models as read-only.
For programmatic edits, rebuild models through Pydantic validation and run
`RobotPackValidator` before export; the validator and writer both revalidate
their in-memory snapshots.

Transactional editing of an existing pack is deferred to the Studio document
model milestone.

## Starting a SMORES-EP pack

Until the URDF importer exists, use the generic pack as a manual template:

1. Copy `examples/robot_packs/generic_cube` to a new directory.
2. Change the root ID, name, version, asset catalogs, and mapping references.
3. Put the SMORES-EP URDF and any redistributable mesh directory under
   `assets/`.
4. Define each module type, source joint, unit-bearing limit, connector
   instance, connector frame or pose, and docking/approach axis.
5. Define connector compatibility, orientation states, docking tolerances,
   physical connection intent, and known load limits.
6. Define advertised capabilities and their required connector types.
7. Update the URDF mapping with exact source link and joint names.
8. Run inspect, authoring validation, and then simulation-profile validation.

Do not add proprietary or redistribution-restricted CAD, URDF, or mesh assets
to the repository until their distribution terms are confirmed.
