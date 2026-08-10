"""MuJoCo backend adapter for ModSim.

This package is the only place in the project that imports ``mujoco``. It is
loaded lazily by :mod:`modsim.backends.registry`, so ``import modsim`` never
pulls in a physics engine.
"""

from modsim_backend_mujoco.adapter import MUJOCO_BACKEND_NAME, MuJoCoBackendAdapter
from modsim_backend_mujoco.scene import (
    CompiledScene,
    MuJoCoSceneError,
    body_name,
    build_scene,
    site_name,
)
from modsim_backend_mujoco.welds import (
    WeldPool,
    WeldPoolExhaustedError,
    WeldSlot,
    body_relative_transform,
)

__all__ = [
    "MUJOCO_BACKEND_NAME",
    "CompiledScene",
    "MuJoCoBackendAdapter",
    "MuJoCoSceneError",
    "WeldPool",
    "WeldPoolExhaustedError",
    "WeldSlot",
    "body_name",
    "body_relative_transform",
    "build_scene",
    "site_name",
]
