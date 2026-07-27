"""Shared Robot Pack test fixtures."""

from __future__ import annotations

import shutil
from pathlib import Path

import pytest


@pytest.fixture
def example_pack_dir() -> Path:
    """Return the immutable generic example pack."""
    return Path(__file__).resolve().parents[1] / "examples" / "robot_packs" / "generic_cube"


@pytest.fixture
def copied_pack(tmp_path: Path, example_pack_dir: Path) -> Path:
    """Copy the generic pack into repository-local pytest scratch space."""
    destination = tmp_path / "generic_cube"
    shutil.copytree(example_pack_dir, destination)
    return destination
