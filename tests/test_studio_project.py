"""GUI-independent ModSim Studio document-model tests."""

from __future__ import annotations

from pathlib import Path

import pytest
from pydantic import ValidationError

from modsim.robot_packs import (
    AcceptanceRegion,
    AcceptanceShape,
    AllowedOrientations,
    ConnectorGender,
    ConnectorLimits,
    ConnectorSpec,
    ConnectorTypeSpec,
    OrientationMode,
    PhysicalConnectionSpec,
    PhysicalConstraintType,
    PoseSpec,
    RobotPackWriteError,
    ValidationProfile,
)
from modsim_studio import StudioProject


def test_studio_project_opens_assets_and_renders_yaml(example_pack_dir: Path) -> None:
    project = StudioProject.open(example_pack_dir)

    assert project.pack.id == "generic_cube"
    assert project.imported_assets["generic_cube"].root_links == ("base_link",)
    preview = project.yaml_preview()
    assert "# robot_pack.yaml" in preview
    assert "# specs/module_types.yaml" in preview
    assert "generic_cube:" in preview


def test_studio_project_edits_connector_and_exports(
    tmp_path: Path,
    example_pack_dir: Path,
) -> None:
    project = StudioProject.open(example_pack_dir)
    connector = ConnectorSpec(
        id="top",
        connector_type="debug_port",
        parent_link="base_link",
        local_pose=PoseSpec(xyz_m=(0.0, 0.0, 0.05)),
        docking_axis=(0.0, 0.0, 1.0),
        approach_axis=(0.0, 0.0, 1.0),
    )

    edited = project.add_connector("generic_cube", connector)

    assert edited.dirty
    assert "debug_port" in edited.pack.hardware_catalog.connector_types
    assert len(edited.pack.hardware_catalog.module_types["generic_cube"].connectors) == 3
    assert edited.validate(ValidationProfile.AUTHORING).valid

    exported = edited.export(tmp_path / "edited")
    reopened = StudioProject.open(exported.loaded.root)
    connector_ids = {
        item.id for item in reopened.pack.hardware_catalog.module_types["generic_cube"].connectors
    }
    assert connector_ids == {"front", "rear", "top"}


def test_studio_project_updates_metadata(example_pack_dir: Path) -> None:
    project = StudioProject.open(example_pack_dir)

    edited = project.update_manifest(
        name="Edited Generic Cube",
        version="0.2.0",
        description="Edited in Studio.",
    )

    assert edited.pack.manifest.name == "Edited Generic Cube"
    assert edited.pack.version == "0.2.0"
    assert "Edited in Studio" in edited.yaml_preview()


def test_studio_project_updates_module_metadata_immutably(example_pack_dir: Path) -> None:
    project = StudioProject.open(example_pack_dir)
    original = project.pack.hardware_catalog.module_types["generic_cube"]

    edited = project.update_module_metadata(
        "generic_cube",
        "Edited Generic Cube",
        "base_link",
        1.25,
    )

    updated = edited.pack.hardware_catalog.module_types["generic_cube"]
    assert edited.dirty
    assert not project.dirty
    assert updated.name == "Edited Generic Cube"
    assert updated.root_link == "base_link"
    assert updated.mass_kg == 1.25
    assert updated.id == original.id
    assert updated.asset_ref == original.asset_ref
    assert updated.joints == original.joints
    assert updated.connectors == original.connectors
    assert updated.capabilities == original.capabilities
    assert project.pack.hardware_catalog.module_types["generic_cube"] == original


def test_studio_project_rejects_module_root_link_missing_from_urdf(
    example_pack_dir: Path,
) -> None:
    project = StudioProject.open(example_pack_dir)

    with pytest.raises(ValueError, match="not present in imported URDF asset"):
        project.update_module_metadata(
            "generic_cube",
            "Generic Cube",
            "missing_link",
            1.0,
        )

    assert not project.dirty


@pytest.mark.parametrize("mass_kg", [0.0, -1.0, True, "1.0"])
def test_studio_project_module_mass_uses_strict_schema(
    example_pack_dir: Path,
    mass_kg: object,
) -> None:
    project = StudioProject.open(example_pack_dir)

    with pytest.raises(ValidationError):
        project.update_module_metadata(
            "generic_cube",
            "Generic Cube",
            "base_link",
            mass_kg,  # type: ignore[arg-type]
        )


def test_studio_project_saves_existing_pack_atomically(copied_pack: Path) -> None:
    project = StudioProject.open(copied_pack).update_manifest(
        name="Saved Generic Cube",
        version="0.1.1",
        description="Saved in place.",
    )

    saved = project.save()

    assert not saved.dirty
    reopened = StudioProject.open(copied_pack)
    assert reopened.pack.manifest.name == "Saved Generic Cube"
    assert reopened.pack.version == "0.1.1"
    assert not list(copied_pack.parent.glob(f".{copied_pack.name}.update.*"))


def test_studio_project_updates_connector_type(example_pack_dir: Path) -> None:
    project = StudioProject.open(example_pack_dir)
    updated_type = ConnectorTypeSpec(
        id="fixed_face",
        name="Edited Fixed Face",
        active=True,
        gender=ConnectorGender.HERMAPHRODITIC,
        compatible_with=("fixed_face",),
        allowed_orientations=AllowedOrientations(mode=OrientationMode.CONTINUOUS),
        acceptance_region=AcceptanceRegion(
            shape=AcceptanceShape.BOX,
            position_tolerance_m=0.01,
            orientation_tolerance_rad=0.1,
            max_relative_velocity_m_s=0.1,
        ),
        physical_connection=PhysicalConnectionSpec(constraint=PhysicalConstraintType.FIXED),
        limits=ConnectorLimits(
            max_normal_force_n=10.0,
            max_shear_force_n=5.0,
            max_bending_moment_nm=1.0,
        ),
        supports_undocking=True,
    )

    edited = project.update_connector_type(updated_type)

    assert edited.pack.hardware_catalog.connector_types["fixed_face"].name == "Edited Fixed Face"
    assert edited.validate(ValidationProfile.SIMULATION).valid


def test_studio_save_rolls_back_failed_directory_swap(
    copied_pack: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    import modsim.robot_packs.writer as writer_module

    project = StudioProject.open(copied_pack).update_manifest(
        name="Must Roll Back",
        version="0.1.1",
        description=None,
    )
    real_replace = writer_module.os.replace

    def fail_publish(source: str | Path, destination: str | Path) -> None:
        if Path(source).name == "staged" and Path(destination) == copied_pack:
            raise OSError("injected publish failure")
        real_replace(source, destination)

    monkeypatch.setattr(writer_module.os, "replace", fail_publish)

    with pytest.raises(RobotPackWriteError, match="injected publish failure"):
        project.save()

    reopened = StudioProject.open(copied_pack)
    assert reopened.pack.manifest.name == "Generic Cube Module"
    assert not list(copied_pack.parent.glob(f".{copied_pack.name}.update.*"))
