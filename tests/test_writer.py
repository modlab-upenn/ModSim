"""Tests for deterministic Robot Pack persistence."""

from __future__ import annotations

import os
from collections.abc import Mapping
from pathlib import Path

import pytest

from modsim.robot_packs import (
    AssetCatalogKind,
    BackendMapping,
    ModuleBackendMapping,
    RobotPackLoader,
    RobotPackLoadError,
    RobotPackValidator,
    RobotPackWriteError,
    RobotPackWriter,
)


def test_split_yaml_semantic_round_trip(example_pack_dir: Path, tmp_path: Path) -> None:
    loader = RobotPackLoader()
    original = loader.load(example_pack_dir)
    destination = tmp_path / "round_trip"

    written = RobotPackWriter().write(original, destination)
    reloaded = loader.load(written.root)

    assert reloaded.pack == original.pack
    assert (destination / "assets" / "urdf" / "generic_cube.urdf").is_file()
    assert "mujoco: {}" in (destination / "robot_pack.yaml").read_text(encoding="utf-8")
    module_yaml = (destination / "specs" / "module_types.yaml").read_text(encoding="utf-8")
    assert "\n    id: generic_cube\n" not in module_yaml
    assert module_yaml.index("- id: front") < module_yaml.index("- id: rear")


def test_writer_output_is_deterministic(example_pack_dir: Path, tmp_path: Path) -> None:
    loaded = RobotPackLoader().load(example_pack_dir)
    first = tmp_path / "first"
    second = tmp_path / "second"

    RobotPackWriter().write(loaded, first)
    RobotPackWriter().write(loaded, second)

    relative_documents = (
        "robot_pack.yaml",
        "specs/module_types.yaml",
        "specs/connector_types.yaml",
        "specs/capabilities.yaml",
        "mappings/urdf_mapping.yaml",
    )
    for relative_path in relative_documents:
        assert (first / relative_path).read_bytes() == (second / relative_path).read_bytes()


def test_writer_refuses_to_overwrite_by_default(example_pack_dir: Path, tmp_path: Path) -> None:
    loaded = RobotPackLoader().load(example_pack_dir)
    destination = tmp_path / "existing"
    writer = RobotPackWriter()
    writer.write(loaded, destination)

    with pytest.raises(RobotPackWriteError, match="refusing existing destination"):
        writer.write(loaded, destination)


def test_writer_does_not_mutate_source_pack(example_pack_dir: Path, tmp_path: Path) -> None:
    source_documents = {
        path.relative_to(example_pack_dir): path.read_bytes()
        for path in example_pack_dir.rglob("*")
        if path.is_file()
    }
    loaded = RobotPackLoader().load(example_pack_dir)

    RobotPackWriter().write(loaded, tmp_path / "copy")

    assert source_documents == {
        path.relative_to(example_pack_dir): path.read_bytes()
        for path in example_pack_dir.rglob("*")
        if path.is_file()
    }


def test_writer_with_missing_aggregate_mapping_fails_before_writing(
    example_pack_dir: Path, tmp_path: Path
) -> None:
    loaded = RobotPackLoader().load(example_pack_dir)
    invalid_pack = loaded.pack.model_copy(update={"backend_mappings": {}})
    destination = tmp_path / "invalid"

    with pytest.raises(RobotPackWriteError, match="missing mappings"):
        RobotPackWriter().write(loaded.with_pack(invalid_pack), destination)

    assert not destination.exists()


def test_writer_refuses_undeclared_aggregate_mapping(
    example_pack_dir: Path, tmp_path: Path
) -> None:
    loaded = RobotPackLoader().load(example_pack_dir)
    extra_mapping = BackendMapping(
        backend="mock",
        asset_catalog=AssetCatalogKind.URDF,
        module_types={"generic_cube": ModuleBackendMapping()},
    )
    invalid_pack = loaded.pack.model_copy(
        update={
            "backend_mappings": {
                **loaded.pack.backend_mappings,
                "mock": extra_mapping,
            }
        }
    )

    with pytest.raises(RobotPackWriteError, match="undeclared mappings"):
        RobotPackWriter().write(
            loaded.with_pack(invalid_pack),
            tmp_path / "invalid",
        )


def test_writer_revalidates_mutated_nested_mappings(example_pack_dir: Path, tmp_path: Path) -> None:
    loaded = RobotPackLoader().load(example_pack_dir)
    module = loaded.pack.hardware_catalog.module_types["generic_cube"]
    loaded.pack.hardware_catalog.module_types["wrong_key"] = module
    destination = tmp_path / "invalid"

    with pytest.raises(RobotPackWriteError, match="does not match id"):
        RobotPackWriter().write(loaded, destination)

    assert not destination.exists()


def test_writer_refuses_symlink_inside_asset_directory(copied_pack: Path, tmp_path: Path) -> None:
    mesh_directory = copied_pack / "assets" / "meshes"
    mesh_directory.mkdir()
    external_mesh = tmp_path / "external.stl"
    external_mesh.write_text("solid external\nendsolid external\n", encoding="utf-8")
    try:
        (mesh_directory / "external.stl").symlink_to(external_mesh)
    except OSError:
        pytest.skip("symlinks are not available on this platform")

    manifest_path = copied_pack / "robot_pack.yaml"
    manifest_path.write_text(
        manifest_path.read_text(encoding="utf-8").replace(
            "  mesh_directories: {}",
            "  mesh_directories:\n    meshes: assets/meshes",
        ),
        encoding="utf-8",
    )
    loaded = RobotPackLoader().load(copied_pack)
    report = RobotPackValidator().validate(loaded)

    assert any(issue.code == "asset.symlink_not_allowed" for issue in report.errors)
    with pytest.raises(RobotPackWriteError, match="symbolic link inside"):
        RobotPackWriter().write(loaded, tmp_path / "copy")


def test_writer_rejects_destination_inside_source_asset_directory(
    copied_pack: Path,
) -> None:
    mesh_directory = copied_pack / "assets" / "meshes"
    mesh_directory.mkdir()
    (mesh_directory / "cube.stl").write_text(
        "solid cube\nendsolid cube\n",
        encoding="utf-8",
    )
    manifest_path = copied_pack / "robot_pack.yaml"
    manifest_path.write_text(
        manifest_path.read_text(encoding="utf-8").replace(
            "  mesh_directories: {}",
            "  mesh_directories:\n    meshes: assets/meshes",
        ),
        encoding="utf-8",
    )
    loaded = RobotPackLoader().load(copied_pack)
    destination = mesh_directory / "export"

    with pytest.raises(RobotPackWriteError, match="inside source asset directory"):
        RobotPackWriter().write(loaded, destination)

    assert not destination.exists()


def test_writer_copies_nested_regular_asset_directory(copied_pack: Path, tmp_path: Path) -> None:
    mesh_directory = copied_pack / "assets" / "meshes"
    nested_directory = mesh_directory / "visual"
    nested_directory.mkdir(parents=True)
    mesh = nested_directory / "cube.stl"
    mesh.write_text("solid cube\nendsolid cube\n", encoding="utf-8")
    manifest_path = copied_pack / "robot_pack.yaml"
    manifest_path.write_text(
        manifest_path.read_text(encoding="utf-8").replace(
            "  mesh_directories: {}",
            "  mesh_directories:\n    meshes: assets/meshes",
        ),
        encoding="utf-8",
    )
    loaded = RobotPackLoader().load(copied_pack)
    destination = tmp_path / "copy"

    exported = RobotPackWriter().write(loaded, destination)

    assert (
        destination / "assets" / "meshes" / "visual" / "cube.stl"
    ).read_bytes() == mesh.read_bytes()
    assert exported.pack == loaded.pack


def test_writer_rejects_dangling_symbolic_link_destination(
    example_pack_dir: Path, tmp_path: Path
) -> None:
    loaded = RobotPackLoader().load(example_pack_dir)
    destination = tmp_path / "export_link"
    redirected_target = tmp_path / "redirected" / "export"
    try:
        destination.symlink_to(redirected_target, target_is_directory=True)
    except OSError:
        pytest.skip("symlinks are not available on this platform")

    with pytest.raises(RobotPackWriteError, match="symbolic-link export destination"):
        RobotPackWriter().write(loaded, destination)

    assert destination.is_symlink()
    assert not redirected_target.exists()


def test_writer_cleans_staging_directory_after_write_failure(
    example_pack_dir: Path,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    loaded = RobotPackLoader().load(example_pack_dir)
    destination = tmp_path / "atomic"
    writer = RobotPackWriter()
    atomic_dump = writer._atomic_dump  # pyright: ignore[reportPrivateUsage]
    call_count = 0

    def fail_on_second_document(path: Path, data: Mapping[str, object]) -> None:
        nonlocal call_count
        call_count += 1
        if call_count == 2:
            raise RobotPackWriteError("injected write failure")
        atomic_dump(path, data)

    monkeypatch.setattr(writer, "_atomic_dump", fail_on_second_document)
    staging_before = set(tmp_path.glob(".atomic.*.tmp"))

    with pytest.raises(RobotPackWriteError, match="injected write failure"):
        writer.write(loaded, destination)

    assert not destination.exists()
    assert set(tmp_path.glob(".atomic.*.tmp")) == staging_before


def test_writer_rejects_special_file_inside_asset_directory(
    copied_pack: Path, tmp_path: Path
) -> None:
    mesh_directory = copied_pack / "assets" / "meshes"
    mesh_directory.mkdir()
    fifo_path = mesh_directory / "unsafe.fifo"
    try:
        os.mkfifo(fifo_path)
    except (AttributeError, OSError):
        pytest.skip("FIFO creation is not available on this platform")
    manifest_path = copied_pack / "robot_pack.yaml"
    manifest_path.write_text(
        manifest_path.read_text(encoding="utf-8").replace(
            "  mesh_directories: {}",
            "  mesh_directories:\n    meshes: assets/meshes",
        ),
        encoding="utf-8",
    )
    loaded = RobotPackLoader().load(copied_pack)
    report = RobotPackValidator().validate(loaded)
    destination = tmp_path / "copy"

    assert any(issue.code == "asset.special_file" for issue in report.errors)
    with pytest.raises(RobotPackWriteError, match="non-regular entry"):
        RobotPackWriter().write(loaded, destination)

    assert not destination.exists()


def test_loader_rejects_writer_target_document_that_is_directory(
    copied_pack: Path,
) -> None:
    capabilities = copied_pack / "specs" / "capabilities.yaml"
    capabilities.unlink()
    capabilities.mkdir()

    with pytest.raises(RobotPackLoadError) as error:
        RobotPackLoader().load(copied_pack)

    assert error.value.issue.code == "document.expected_file"
