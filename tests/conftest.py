"""Shared Robot Pack test fixtures."""

from __future__ import annotations

import shutil
from pathlib import Path
from typing import Any

import pytest

from modsim.robot_packs import LoadedRobotPack, RobotPack, RobotPackLoader


@pytest.fixture
def example_pack_dir() -> Path:
    """Return the immutable generic example pack."""
    return Path(__file__).resolve().parents[1] / "examples" / "robot_packs" / "generic_cube"


@pytest.fixture
def smores_pack_dir() -> Path:
    """Return the immutable SMORES-EP example pack."""
    return Path(__file__).resolve().parents[1] / "examples" / "robot_packs" / "smores_ep"


@pytest.fixture
def smores_loaded_pack(smores_pack_dir: Path) -> LoadedRobotPack:
    """Load the committed SMORES-EP example pack and its filesystem context."""
    return RobotPackLoader().load(smores_pack_dir)


@pytest.fixture
def copied_pack(tmp_path: Path, example_pack_dir: Path) -> Path:
    """Copy the generic pack into repository-local pytest scratch space."""
    destination = tmp_path / "generic_cube"
    shutil.copytree(example_pack_dir, destination)
    return destination


@pytest.fixture
def example_pack(example_pack_dir: Path) -> RobotPack:
    """Return the loaded generic example pack."""
    return RobotPackLoader().load(example_pack_dir).pack


def with_connector_policy(
    pack: RobotPack,
    connector_type_id: str,
    **policy: Any,
) -> RobotPack:
    """Return a copy of ``pack`` with a docking policy set on one connector type.

    Robot Pack models are frozen, so edits go back through Pydantic validation
    rather than mutating nested dictionaries in place.
    """
    data: dict[str, Any] = pack.model_dump(mode="python")
    data["hardware_catalog"]["connector_types"][connector_type_id]["docking_policy"] = policy
    return RobotPack.model_validate(data)


def with_connector_field(
    pack: RobotPack,
    connector_type_id: str,
    **fields: Any,
) -> RobotPack:
    """Return a copy of ``pack`` with top-level connector-type fields replaced."""
    data: dict[str, Any] = pack.model_dump(mode="python")
    data["hardware_catalog"]["connector_types"][connector_type_id].update(fields)
    return RobotPack.model_validate(data)
