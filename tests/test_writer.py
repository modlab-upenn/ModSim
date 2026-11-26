"""Tests for deterministic Robot Pack persistence."""

from __future__ import annotations

import io
import os
import shutil
import subprocess
import sys
import tarfile
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


# ----------------------------------------------------------------------
# compatibility with builds from before continuous docking (review B1)
# ----------------------------------------------------------------------

_REPO = Path(__file__).resolve().parents[1]
_PRE_SURFACE_COMMIT = "654b35c"
"""The last commit before continuous docking added fields to schema 0.1."""
_GOLDEN = Path(__file__).resolve().parent / "data" / f"writer_{_PRE_SURFACE_COMMIT}"
_POINT_ONLY_PACKS = ("generic_cube", "smores_ep", "mblocks_3d")


def _resave(name: str, destination: Path) -> Path:
    loaded = RobotPackLoader().load(_REPO / "examples" / "robot_packs" / name)
    return RobotPackWriter().write(loaded, destination / name).root


@pytest.mark.parametrize("name", _POINT_ONLY_PACKS)
def test_resaving_a_point_only_pack_reproduces_the_pre_surface_writer(
    name: str, tmp_path: Path
) -> None:
    """New fields at their defaults are left out, so the output is unchanged.

    The golden files are what the writer at the pre-surface commit produced
    for the same example pack.
    """
    written = _resave(name, tmp_path)
    golden = sorted(path.relative_to(_GOLDEN / name) for path in (_GOLDEN / name).rglob("*.yaml"))
    produced = sorted(
        path.relative_to(written)
        for path in written.rglob("*.yaml")
        if "assets" not in path.relative_to(written).parts
    )

    assert produced == golden
    for relative in golden:
        assert (written / relative).read_text(encoding="utf-8") == (
            _GOLDEN / name / relative
        ).read_text(encoding="utf-8"), relative


def test_resaved_point_only_packs_load_under_the_pre_surface_schema(tmp_path: Path) -> None:
    """Load the re-saved packs with the source tree of the pre-surface commit.

    Skipped when that commit is not available, as in a shallow clone; the
    golden comparison above still pins the output.
    """
    if shutil.which("git") is None:
        pytest.skip("git is not available")
    probe = subprocess.run(
        ["git", "-C", str(_REPO), "cat-file", "-e", f"{_PRE_SURFACE_COMMIT}^{{commit}}"],
        capture_output=True,
        check=False,
    )
    if probe.returncode != 0:
        pytest.skip(f"commit {_PRE_SURFACE_COMMIT} is not in this checkout")
    old_tree = tmp_path / "old"
    old_tree.mkdir()
    archive = subprocess.run(
        ["git", "-C", str(_REPO), "archive", _PRE_SURFACE_COMMIT, "src/modsim"],
        capture_output=True,
        check=True,
    )
    with tarfile.open(fileobj=io.BytesIO(archive.stdout)) as tar:
        tar.extractall(old_tree, filter="data")
    packs = [str(_resave(name, tmp_path / "saved")) for name in _POINT_ONLY_PACKS]

    script = (
        "import sys\n"
        "sys.path.insert(0, sys.argv[1])\n"
        "import modsim\n"
        "from modsim.robot_packs import RobotPackLoader\n"
        "assert modsim.__file__.startswith(sys.argv[1]), modsim.__file__\n"
        "for root in sys.argv[2:]:\n"
        "    RobotPackLoader().load(root)\n"
    )
    result = subprocess.run(
        [sys.executable, "-I", "-c", script, str(old_tree / "src"), *packs],
        capture_output=True,
        text=True,
        check=False,
        cwd=tmp_path,
    )
    assert result.returncode == 0, result.stderr
