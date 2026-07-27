"""URDF importer and draft builder tests."""

from __future__ import annotations

from pathlib import Path

import pytest

from modsim.importers import DraftPackBuilder, URDFImporter, URDFImportError
from modsim.robot_packs import RobotPackValidator, ValidationProfile


def _write_urdf(root: Path, *, with_mesh: bool = False) -> Path:
    root.mkdir(parents=True, exist_ok=True)
    geometry = (
        '<mesh filename="meshes/body.stl" scale="1 2 3"/>'
        if with_mesh
        else '<box size="0.1 0.2 0.3"/>'
    )
    path = root / "sample.urdf"
    path.write_text(
        f"""<?xml version="1.0"?>
<robot name="sample_bot">
  <link name="base_link">
    <inertial><mass value="1.5"/></inertial>
    <visual name="body"><origin xyz="0 0 0.1"/><geometry>{geometry}</geometry></visual>
  </link>
  <link name="arm_link">
    <visual><geometry><cylinder radius="0.01" length="0.2"/></geometry></visual>
  </link>
  <joint name="arm_joint" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
    <axis xyz="0 0 2"/>
    <limit lower="-1" upper="1" effort="2" velocity="3"/>
  </joint>
</robot>
""",
        encoding="utf-8",
    )
    return path


def test_importer_extracts_links_joints_primitives_and_limits(tmp_path: Path) -> None:
    asset = URDFImporter().load(_write_urdf(tmp_path / "source"))

    assert asset.name == "sample_bot"
    assert asset.root_links == ("base_link",)
    assert [link.name for link in asset.links] == ["base_link", "arm_link"]
    assert asset.link("base_link").mass_kg == 1.5
    assert asset.joints[0].axis == (0.0, 0.0, 1.0)
    assert asset.joints[0].limits is not None
    assert asset.joints[0].limits.upper == 1.0


def test_importer_resolves_relative_mesh(tmp_path: Path) -> None:
    source = tmp_path / "source"
    mesh = source / "meshes" / "body.stl"
    mesh.parent.mkdir(parents=True)
    mesh.write_text("solid body\nendsolid body\n", encoding="utf-8")

    asset = URDFImporter().load(_write_urdf(source, with_mesh=True))

    geometry = asset.links[0].visuals[0].geometry
    assert geometry.resolved_mesh_path == mesh.resolve()
    assert geometry.scale == (1.0, 2.0, 3.0)
    assert not asset.warnings


def test_importer_rejects_document_type(tmp_path: Path) -> None:
    path = tmp_path / "unsafe.urdf"
    path.write_text(
        '<!DOCTYPE robot [<!ENTITY x "bad">]><robot name="x"><link name="a"/></robot>',
        encoding="utf-8",
    )

    with pytest.raises(URDFImportError, match="entity declarations"):
        URDFImporter().load(path)


def test_draft_builder_creates_self_contained_valid_pack(tmp_path: Path) -> None:
    source = tmp_path / "source"
    mesh = source / "meshes" / "body.stl"
    mesh.parent.mkdir(parents=True)
    mesh.write_text("solid body\nendsolid body\n", encoding="utf-8")
    urdf = _write_urdf(source, with_mesh=True)
    destination = tmp_path / "pack"

    result = DraftPackBuilder().build(urdf, destination)

    assert result.loaded.root == destination
    assert (destination / "assets/urdf/sample_bot.urdf").is_file()
    assert (destination / "assets/meshes/meshes/body.stl").is_file()
    copied_urdf = (destination / "assets/urdf/sample_bot.urdf").read_text(encoding="utf-8")
    assert "../meshes/meshes/body.stl" in copied_urdf
    module = result.loaded.pack.hardware_catalog.module_types["sample_bot"]
    assert module.root_link == "base_link"
    assert module.mass_kg == 1.5
    assert module.joints[0].source_joint_name == "arm_joint"
    report = RobotPackValidator().validate(
        result.loaded,
        profile=ValidationProfile.AUTHORING,
    )
    assert report.valid


def test_draft_builder_refuses_unresolved_mesh(tmp_path: Path) -> None:
    urdf = _write_urdf(tmp_path / "source", with_mesh=True)

    with pytest.raises(ValueError, match="unresolved mesh"):
        DraftPackBuilder().build(urdf, tmp_path / "pack")
