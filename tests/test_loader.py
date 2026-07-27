"""Tests for split-document Robot Pack loading."""

from __future__ import annotations

from pathlib import Path

import pytest

from modsim.robot_packs import RobotPackLoader, RobotPackLoadError


def test_generic_pack_loads(example_pack_dir: Path) -> None:
    loaded = RobotPackLoader().load(example_pack_dir)

    assert loaded.root == example_pack_dir.resolve()
    assert loaded.pack.id == "generic_cube"
    assert loaded.pack.version == "0.1.0"
    assert tuple(loaded.pack.hardware_catalog.module_types) == ("generic_cube",)
    assert tuple(loaded.pack.hardware_catalog.connector_types) == ("fixed_face",)
    assert tuple(loaded.pack.capability_catalog.capabilities) == ("dock", "undock")
    assert tuple(loaded.pack.backend_mappings) == ("urdf",)


def test_loader_accepts_manifest_path(example_pack_dir: Path) -> None:
    loaded = RobotPackLoader().load(example_pack_dir / "robot_pack.yaml")
    assert loaded.pack.id == "generic_cube"


def test_loader_rejects_symlinked_root_manifest(example_pack_dir: Path, tmp_path: Path) -> None:
    selected_pack = tmp_path / "selected"
    selected_pack.mkdir()
    try:
        (selected_pack / "robot_pack.yaml").symlink_to(example_pack_dir / "robot_pack.yaml")
    except OSError:
        pytest.skip("symlinks are not available on this platform")

    with pytest.raises(RobotPackLoadError) as error:
        RobotPackLoader().load(selected_pack)

    assert error.value.issue.code == "document.symlink_not_allowed"
    assert error.value.source == selected_pack / "robot_pack.yaml"


def test_loader_rejects_alternate_manifest_filename(tmp_path: Path) -> None:
    alternate_manifest = tmp_path / "pack.yaml"

    with pytest.raises(RobotPackLoadError) as error:
        RobotPackLoader().load(alternate_manifest)

    assert error.value.issue.code == "pack.manifest_name_invalid"
    assert error.value.source == alternate_manifest


def test_missing_manifest_has_stable_error(tmp_path: Path) -> None:
    missing_pack = tmp_path / "missing"
    missing_pack.mkdir()

    with pytest.raises(RobotPackLoadError) as error:
        RobotPackLoader().load(missing_pack)

    assert error.value.issue.code == "pack.manifest_missing"
    assert error.value.issue.document == "robot_pack.yaml"


def test_nonexistent_directory_path_looks_for_root_manifest(tmp_path: Path) -> None:
    missing_pack = tmp_path / "not_created"

    with pytest.raises(RobotPackLoadError) as error:
        RobotPackLoader().load(missing_pack)

    assert error.value.source == missing_pack / "robot_pack.yaml"


def test_missing_referenced_document_has_stable_error(copied_pack: Path) -> None:
    (copied_pack / "specs" / "capabilities.yaml").unlink()

    with pytest.raises(RobotPackLoadError) as error:
        RobotPackLoader().load(copied_pack)

    assert error.value.issue.code == "document.missing"
    assert error.value.issue.document == "specs/capabilities.yaml"


def test_loader_rejects_symlinked_referenced_document(copied_pack: Path) -> None:
    capabilities = copied_pack / "specs" / "capabilities.yaml"
    capabilities.unlink()
    try:
        capabilities.symlink_to("capabilities.yaml")
    except OSError:
        pytest.skip("symlinks are not available on this platform")

    with pytest.raises(RobotPackLoadError) as error:
        RobotPackLoader().load(copied_pack)

    assert error.value.issue.code == "document.symlink_not_allowed"


def test_duplicate_yaml_key_is_rejected(copied_pack: Path) -> None:
    manifest_path = copied_pack / "robot_pack.yaml"
    manifest_path.write_text(
        manifest_path.read_text(encoding="utf-8") + "\nid: duplicate\n",
        encoding="utf-8",
    )

    with pytest.raises(RobotPackLoadError) as error:
        RobotPackLoader().load(copied_pack)

    assert error.value.issue.code == "yaml.duplicate_key"


def test_schema_error_retains_document_and_pointer(copied_pack: Path) -> None:
    module_path = copied_pack / "specs" / "module_types.yaml"
    module_path.write_text(
        module_path.read_text(encoding="utf-8") + "\nunexpected: true\n",
        encoding="utf-8",
    )

    with pytest.raises(RobotPackLoadError) as error:
        RobotPackLoader().load(copied_pack)

    issue = error.value.issue
    assert issue.code == "schema.invalid"
    assert issue.document == "specs/module_types.yaml"
    assert issue.json_pointer == "/unexpected"


def test_schema_error_reports_all_structural_issues(copied_pack: Path) -> None:
    module_path = copied_pack / "specs" / "module_types.yaml"
    module_path.write_text(
        module_path.read_text(encoding="utf-8")
        + "\nfirst_unexpected: true\nsecond_unexpected: true\n",
        encoding="utf-8",
    )

    with pytest.raises(RobotPackLoadError) as error:
        RobotPackLoader().load(copied_pack)

    assert [issue.json_pointer for issue in error.value.issues] == [
        "/first_unexpected",
        "/second_unexpected",
    ]


def test_unsupported_schema_version_is_rejected(copied_pack: Path) -> None:
    manifest_path = copied_pack / "robot_pack.yaml"
    manifest_path.write_text(
        manifest_path.read_text(encoding="utf-8").replace(
            'schema_version: "0.1"',
            'schema_version: "9.9"',
            1,
        ),
        encoding="utf-8",
    )

    with pytest.raises(RobotPackLoadError) as error:
        RobotPackLoader().load(copied_pack)

    assert error.value.issue.code == "schema.version_unsupported"
    assert error.value.issue.path == ("schema_version",)


def test_manifest_rejects_document_asset_role_collision(copied_pack: Path) -> None:
    manifest_path = copied_pack / "robot_pack.yaml"
    manifest_path.write_text(
        manifest_path.read_text(encoding="utf-8").replace(
            "generic_cube: assets/urdf/generic_cube.urdf",
            "generic_cube: specs/module_types.yaml",
        ),
        encoding="utf-8",
    )

    with pytest.raises(RobotPackLoadError) as error:
        RobotPackLoader().load(copied_pack)

    assert error.value.issue.code == "schema.invalid"
    assert "document and asset paths overlap" in error.value.issue.message


def test_explicit_catalog_id_must_match_key(copied_pack: Path) -> None:
    module_path = copied_pack / "specs" / "module_types.yaml"
    module_path.write_text(
        module_path.read_text(encoding="utf-8").replace(
            "  generic_cube:\n",
            "  generic_cube:\n    id: another_cube\n",
            1,
        ),
        encoding="utf-8",
    )

    with pytest.raises(RobotPackLoadError) as error:
        RobotPackLoader().load(copied_pack)

    assert error.value.issue.code == "schema.invalid"
    assert error.value.issue.json_pointer == "/module_types/generic_cube/id"


def test_explicit_null_catalog_id_is_not_silently_repaired(copied_pack: Path) -> None:
    module_path = copied_pack / "specs" / "module_types.yaml"
    module_path.write_text(
        module_path.read_text(encoding="utf-8").replace(
            "  generic_cube:\n",
            "  generic_cube:\n    id: null\n",
            1,
        ),
        encoding="utf-8",
    )

    with pytest.raises(RobotPackLoadError) as error:
        RobotPackLoader().load(copied_pack)

    assert error.value.issue.code == "schema.invalid"
    assert error.value.issue.json_pointer == "/module_types/generic_cube/id"
