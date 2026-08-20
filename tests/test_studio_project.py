"""GUI-independent ModSim Studio document-model tests."""

from __future__ import annotations

from pathlib import Path

import pytest
from pydantic import ValidationError

from modsim.robot_packs import (
    AcceptanceRegion,
    AcceptanceShape,
    AlignmentMode,
    AllowedOrientations,
    ConnectorGender,
    ConnectorLimits,
    ConnectorSpec,
    ConnectorTypeSpec,
    DockingPolicySpec,
    ModelViewMode,
    ModelViewSpec,
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
        metadata={"face_index": 3},
    )

    edited = project.add_connector_type(
        ConnectorTypeSpec(
            id="debug_port",
            name="Debug Port",
            compatible_with=("debug_port",),
        )
    ).add_connector("generic_cube", connector)

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
    top = next(
        item
        for item in reopened.pack.hardware_catalog.module_types["generic_cube"].connectors
        if item.id == "top"
    )
    assert top.metadata == {"face_index": 3}


def test_studio_project_updates_metadata(copied_pack: Path) -> None:
    project = StudioProject.open(copied_pack)

    edited = project.update_manifest(
        name="Edited Generic Cube",
        version="0.2.0",
        description="Edited in Studio.",
        metadata={"hardware_revision": "test", "tags": ["example", "edited"]},
    )

    assert edited.pack.manifest.name == "Edited Generic Cube"
    assert edited.pack.version == "0.2.0"
    assert "Edited in Studio" in edited.yaml_preview()
    assert edited.pack.manifest.metadata["hardware_revision"] == "test"
    saved = edited.save()
    reopened = StudioProject.open(saved.loaded.root)
    assert reopened.pack.manifest.metadata == {
        "hardware_revision": "test",
        "tags": ["example", "edited"],
    }
    root_yaml = (copied_pack / "robot_pack.yaml").read_text(encoding="utf-8")
    assert "hardware_revision: test" in root_yaml


def test_studio_project_model_view_crud_persists_across_save(copied_pack: Path) -> None:
    project = StudioProject.open(copied_pack)
    original_model_views = project.pack.manifest.model_views
    recipe = ModelViewSpec(
        id="smores_topology",
        name="SMORES Topology",
        builder="module_topology_graph",
    )

    added = project.add_model_view(recipe)
    assert project.pack.manifest.model_views == original_model_views
    assert added.pack.manifest.model_views == (*original_model_views, recipe)
    with pytest.raises(ValueError, match="already exists"):
        added.add_model_view(recipe)

    updated_recipe = ModelViewSpec(
        id="smores_topology",
        name="Live SMORES Topology",
        builder="platform_graph",
        modes=(ModelViewMode.AUTHORING, ModelViewMode.RUNTIME),
        default=True,
        configuration={"layout": {"algorithm": "spring", "seed": 7}},
    )
    saved = added.update_model_view(updated_recipe).save()
    reopened = StudioProject.open(saved.loaded.root)

    persisted_recipe = next(
        item for item in reopened.pack.manifest.model_views if item.id == "smores_topology"
    )
    assert persisted_recipe == updated_recipe
    assert persisted_recipe.configuration == {"layout": {"algorithm": "spring", "seed": 7}}
    root_yaml = (copied_pack / "robot_pack.yaml").read_text(encoding="utf-8")
    assert "model_views:" in root_yaml
    assert "builder: platform_graph" in root_yaml

    removed = reopened.remove_model_view("smores_topology").save()
    assert removed.pack.manifest.model_views == original_model_views
    assert StudioProject.open(copied_pack).pack.manifest.model_views == original_model_views


def test_studio_project_requires_explicit_connector_type(example_pack_dir: Path) -> None:
    project = StudioProject.open(example_pack_dir)
    connector = ConnectorSpec(
        id="top",
        connector_type="missing_type",
        parent_link="base_link",
        local_pose=PoseSpec(),
    )

    with pytest.raises(ValueError, match="create the type first"):
        project.add_connector("generic_cube", connector)


def test_studio_project_adds_and_removes_connector_types(copied_pack: Path) -> None:
    project = StudioProject.open(copied_pack)
    added = project.add_connector_type(
        ConnectorTypeSpec(
            id="service_port",
            name="Service Port",
            compatible_with=("service_port",),
            metadata={"protocol": "debug"},
        )
    )

    assert added.pack.hardware_catalog.connector_types["service_port"].metadata == {
        "protocol": "debug"
    }
    saved = added.save()
    connector_yaml_path = copied_pack / "specs" / "connector_types.yaml"
    assert "service_port:" in connector_yaml_path.read_text(encoding="utf-8")
    assert StudioProject.open(copied_pack).pack.hardware_catalog.connector_types[
        "service_port"
    ].metadata == {"protocol": "debug"}

    removed = saved.remove_connector_type("service_port").save()
    assert "service_port" not in removed.pack.hardware_catalog.connector_types
    assert "service_port:" not in connector_yaml_path.read_text(encoding="utf-8")


def test_studio_project_blocks_removing_type_used_by_connector(example_pack_dir: Path) -> None:
    project = StudioProject.open(example_pack_dir)

    with pytest.raises(ValueError, match="still referenced"):
        project.remove_connector_type("fixed_face")


def test_studio_project_removes_type_and_cleans_semantic_references(
    example_pack_dir: Path,
) -> None:
    project = StudioProject.open(example_pack_dir)
    without_front = project.remove_connector("generic_cube", "front")
    without_connectors = without_front.remove_connector("generic_cube", "rear")
    removed = without_connectors.remove_connector_type("fixed_face")

    assert "fixed_face" not in removed.pack.hardware_catalog.connector_types
    assert all(
        "fixed_face" not in capability.required_connector_types
        for capability in removed.pack.capability_catalog.capabilities.values()
    )


def test_studio_project_removes_connector_backend_frame_mapping(copied_pack: Path) -> None:
    mapping_path = copied_pack / "mappings" / "urdf_mapping.yaml"
    mapping_path.write_text(
        mapping_path.read_text(encoding="utf-8").replace(
            "connector_frame_map: {}",
            "connector_frame_map:\n      front: front_docking_frame",
        ),
        encoding="utf-8",
    )
    project = StudioProject.open(copied_pack)

    edited = project.remove_connector("generic_cube", "front")

    module_mapping = edited.pack.backend_mappings["urdf"].module_types["generic_cube"]
    assert "front" not in module_mapping.connector_frame_map


def test_studio_project_reassociates_connector_with_imported_urdf_body(
    copied_pack: Path,
) -> None:
    urdf = copied_pack / "assets" / "urdf" / "generic_cube.urdf"
    urdf.write_text(
        urdf.read_text(encoding="utf-8").replace(
            "</robot>",
            '  <link name="tool_link"/>\n</robot>',
        ),
        encoding="utf-8",
    )
    project = StudioProject.open(copied_pack)
    connector = project.pack.hardware_catalog.module_types["generic_cube"].connectors[0]
    reassociated = ConnectorSpec.model_validate(
        {
            **connector.model_dump(mode="python"),
            "parent_link": "tool_link",
            "metadata": {"body_role": "moving_face"},
        }
    )

    edited = project.update_connector("generic_cube", reassociated).save()
    reopened = StudioProject.open(edited.loaded.root)
    saved_connector = reopened.pack.hardware_catalog.module_types["generic_cube"].connectors[0]
    assert saved_connector.parent_link == "tool_link"
    assert saved_connector.metadata == {"body_role": "moving_face"}
    module_yaml = (copied_pack / "specs" / "module_types.yaml").read_text(encoding="utf-8")
    assert "parent_link: tool_link" in module_yaml
    assert "body_role: moving_face" in module_yaml


def test_studio_project_rejects_connector_body_missing_from_urdf(
    example_pack_dir: Path,
) -> None:
    project = StudioProject.open(example_pack_dir)
    connector = project.pack.hardware_catalog.module_types["generic_cube"].connectors[0]
    invalid = ConnectorSpec.model_validate(
        {**connector.model_dump(mode="python"), "parent_link": "missing_link"}
    )

    with pytest.raises(ValueError, match="not present in imported URDF"):
        project.update_connector("generic_cube", invalid)


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


def test_studio_project_updates_connector_type(copied_pack: Path) -> None:
    project = StudioProject.open(copied_pack)
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
        docking_policy=DockingPolicySpec(
            auto_latch=True,
            alignment=AlignmentMode.NOMINAL,
            redock_cooldown_s=0.5,
            break_force_n=25.0,
        ),
        metadata={"interface.standard": "smores_ep"},
    )

    edited = project.update_connector_type(updated_type).save()
    reopened = StudioProject.open(edited.loaded.root)
    saved_type = reopened.pack.hardware_catalog.connector_types["fixed_face"]

    assert saved_type.name == "Edited Fixed Face"
    assert saved_type.metadata == {"interface.standard": "smores_ep"}
    assert saved_type.docking_policy is not None
    assert saved_type.docking_policy.auto_latch
    assert saved_type.docking_policy.alignment is AlignmentMode.NOMINAL
    assert saved_type.docking_policy.redock_cooldown_s == 0.5
    assert saved_type.docking_policy.break_force_n == 25.0
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
