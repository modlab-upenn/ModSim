"""MuJoCo backend adapter for ModSim.

This package is the only place in the project that imports ``mujoco``. It is
loaded lazily by :mod:`modsim_robotics.backends.registry`, so ``import modsim_robotics`` never
pulls in a physics engine.
"""

from modsim_robotics_backend_mujoco.adapter import MUJOCO_BACKEND_NAME, MuJoCoBackendAdapter
from modsim_robotics_backend_mujoco.hinges import (
    HingePool,
    HingePoolExhaustedError,
    HingeSlot,
    hinge_anchor_pairs,
)
from modsim_robotics_backend_mujoco.scene import (
    CompiledScene,
    MuJoCoSceneError,
    body_name,
    build_scene,
    site_name,
)
from modsim_robotics_backend_mujoco.welds import (
    WeldPool,
    WeldPoolExhaustedError,
    WeldSlot,
    body_relative_transform,
)

__all__ = [
    "MUJOCO_BACKEND_NAME",
    "CompiledScene",
    "HingePool",
    "HingePoolExhaustedError",
    "HingeSlot",
    "MuJoCoBackendAdapter",
    "MuJoCoSceneError",
    "WeldPool",
    "WeldPoolExhaustedError",
    "WeldSlot",
    "body_name",
    "body_relative_transform",
    "build_scene",
    "hinge_anchor_pairs",
    "site_name",
]
