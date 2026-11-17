---
title: Prepare a module URDF
sidebar_label: Prepare a URDF
---

Start by creating a URDF file for a single reusable module. It supplies links,
geometry, joints, and inertial properties. Studio will add the connector
semantics after import.

This guide provides an example model of a cube with a
known size, mass, and coordinate frame. This does not require making a CAD model or any additional installations as the motivation is to understand the workflow and the process. 

Generally you would use the CAD model of the robot and export the URDF from the CAD tool.

## Create the example model

In a text editor, save the following as **`workshop_cube.urdf`** in an ordinary
working folder, such as `Documents/modsim_models`. Check to make sure that the file is not saved with a .txt extension and is a URDF extension.

```xml
<?xml version="1.0"?>
<robot name="workshop_cube">
  <link name="base_link">
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.001666667" ixy="0" ixz="0"
               iyy="0.001666667" iyz="0" izz="0.001666667"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
      <material name="workshop_blue">
        <color rgba="0.15 0.45 0.85 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </collision>
  </link>
</robot>
```

The cube is centered at `base_link`. Its front face is at X = +0.05 m and its
rear face at X = −0.05 m. Those positions will become connector locations.
All dimensions use metres, rotations use radians, and mass uses kilograms.

For a uniform solid cube, each diagonal inertia is
`mass × (side² + side²) / 12`, giving approximately
`0.001666667 kg m²` for this model. The center of mass and link origin coincide,
so the off-diagonal inertia terms are zero. These values describe the example
geometry rather than an assumed physical robot.

The model intentionally has no joints and no external assets. The matching
visual and collision boxes make its appearance and contact geometry easy to
compare. The URDF robot name makes the imported pack and module IDs
`workshop_cube`, distinct from the installed `generic_cube` starter.

## Using your own URDF models

For an existing robot, check these parts of your URDF before importing:

| Part | What to check |
| --- | --- |
| Robot and link names | Use a stable robot name and unique link/joint names. Imported semantic IDs are normalized to lowercase snake case. |
| Link tree | Joints reference existing parent/child links. A child has one parent. Prefer one connected tree with one root link. |
| Geometry | Visual and collision shapes have appropriate scale and origins. A visible mesh alone does not establish useful contact geometry. |
| Mass and inertia | Provide physically consistent positive mass and inertia for the bodies the backend will simulate. Structural pack validation does not prove the inertia tensor is correct. |
| Joint data | Provide the intended joint type, axis, and available limits. Control modes are added later in Studio. |
| Mesh references | Refer to local files that you can access and include with the pack. Keep the mesh files available during import. |

Studio accepts a concrete `.urdf` or URDF XML file. Expand Xacro before importing;
the importer does not execute it. It resolves relative paths, local `file://`
paths, and locally available `package://` meshes. It does not download HTTP or
HTTPS assets.

Boxes, spheres, and cylinders avoid mesh dependencies. For mesh-based models,
keep the URDF and meshes together or identify the package/mesh root when the
import dialog asks. Texture images and indirect OBJ/DAE material dependencies
are not bundled or rendered automatically; review those assets separately.

## Before continuing

Confirm that you know where the source file is and where your ModSim home is.
The import will copy the model into a new pack; later editing the original
source file will not automatically update that copy. Changes to shape or
mechanics belong in the URDF and require a deliberate import/update workflow.

Next: [Import the URDF in Studio](import-urdf.md).
Return to the [workflow overview](index.md).
