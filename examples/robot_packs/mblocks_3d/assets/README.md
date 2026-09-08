# M-Blocks asset provenance

These three binary STL files were copied from the user-provided Fusion 360
ROS export and are consumed, at a `0.001` scale, by both the expanded URDF and
the pack-local MuJoCo model:

| File | SHA-256 |
|---|---|
| `meshes/actuator_carrier_link_1.stl` | `5585e39ddf1f95a6dfa5e53715ac0a276fc9115aa29f57c70b448b501e8156a7` |
| `meshes/base_link.stl` | `cc72540741edcf58960a3ef19954ca639006643f97098ac6821486aed71be797` |
| `meshes/flywheel_link_1.stl` | `5ec35f21ad5342e65c732cd8ceeddc279cee30c34b6f8a3676f3c63f7f1370e5` |

`urdf/mblocks_3d.urdf` is a self-contained adaptation of the supplied Xacro.
ROS package substitutions and Gazebo/ros2_control includes were removed;
source link names, visual meshes, and joint topology were retained. For the
one-plane physics bootstrap, the inertials were normalized to a 0.150 kg total,
the flywheel axial inertia was set to `8.4e-6 kg m^2`, and its axis was aligned
with +Y. The detailed base collision was replaced with a nominal 50 mm box so
contact remains stable and inexpensive. The source export's original mass and
diagonal-axis details remain recorded as provenance in the pack metadata and
integration guide, not as active URDF physics values.

The source package did not include a usable redistribution license. These
assets were added to this working tree at the user's direction. Confirm the
owner's license or permission before publishing the binary meshes outside the
authorized project repository.
