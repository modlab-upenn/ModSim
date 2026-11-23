"""O(log P) pool allocation must choose exactly what the former linear scans chose."""

from __future__ import annotations

import random
from pathlib import Path

import pytest

mujoco = pytest.importorskip("mujoco")
np = pytest.importorskip("numpy")

from modsim.core.ids import ConstraintHandle, ModuleInstanceId  # noqa: E402
from modsim.core.scene import ModulePlacement, SceneSpec  # noqa: E402
from modsim.core.transforms import Transform  # noqa: E402
from modsim.robot_packs import RobotPackLoader  # noqa: E402
from modsim.runtime.session import RuntimeSession  # noqa: E402
from modsim_backend_mujoco.adapter import (  # noqa: E402
    _insert_exclusion,  # pyright: ignore[reportPrivateUsage]
    _remove_exclusion,  # pyright: ignore[reportPrivateUsage]
)
from modsim_backend_mujoco.hinges import HingePool  # noqa: E402
from modsim_backend_mujoco.welds import ContactExclusionPool, WeldPool  # noqa: E402

PACK = Path(__file__).parents[1] / "examples/robot_packs/generic_cube"


def _lowest_free(slots: tuple[object, ...]) -> int:
    return next(index for index, slot in enumerate(slots) if slot.free)


@pytest.mark.parametrize("kind", ["weld", "hinge", "exclusion"])
def test_pools_claim_the_lowest_free_slot_like_a_linear_scan(kind: str) -> None:
    size = 24
    if kind == "weld":
        pool: object = WeldPool.over(tuple(range(100, 100 + size)))
    elif kind == "hinge":
        pool = HingePool.over(tuple((2 * i, 2 * i + 1) for i in range(size)))
    else:
        pool = ContactExclusionPool.over(tuple(range(size)))
    rng = random.Random(3)
    held: list[ConstraintHandle] = []
    for step in range(400):
        slots = pool.slots  # type: ignore[attr-defined]
        if held and (len(held) == size or rng.random() < 0.45):
            pool.release(held.pop(rng.randrange(len(held))))  # type: ignore[attr-defined]
            continue
        expected = _lowest_free(slots)
        handle = ConstraintHandle(f"c{step}")
        if kind == "exclusion":
            slot = pool.claim(handle, 1 + step % 50, 60 + step % 7)  # type: ignore[attr-defined]
        else:
            slot = pool.claim(handle)  # type: ignore[attr-defined]
        assert slot is slots[expected]
        held.append(handle)
    pool.clear()  # type: ignore[attr-defined]
    assert all(slot.free for slot in pool.slots)  # type: ignore[attr-defined]


def test_incremental_exclusions_equal_a_full_resort() -> None:
    scene = SceneSpec.of(
        ModulePlacement(
            instance_id=ModuleInstanceId(f"c{i}"),
            module_type_id="generic_cube",
            pose=Transform.from_translation((0.5 * i, 0.0, 1.0)),
        )
        for i in range(12)
    )
    session = RuntimeSession.create(
        RobotPackLoader().load(PACK), scene, "mujoco", weld_pool_size=10
    )
    try:
        model = session.adapter._compiled.model  # type: ignore[attr-defined]  # pyright: ignore[reportPrivateUsage]
        initial = np.array(model.exclude_signature)
        assert (initial == 0).sum() >= 10
        reference = list(initial)
        rng = random.Random(11)
        active: list[int] = []
        bodies = list(range(1, int(model.nbody)))
        for _ in range(300):
            if active and (reference.count(0) == 0 or rng.random() < 0.45):
                signature = active.pop(rng.randrange(len(active)))
                _remove_exclusion(model, signature)
                reference.remove(signature)
                reference.append(0)
            else:
                a, b = sorted(rng.sample(bodies, 2))
                signature = (a << 16) | b
                _insert_exclusion(model, signature)
                reference.remove(0)
                reference.append(signature)
                active.append(signature)
            assert list(model.exclude_signature) == sorted(reference)
    finally:
        session.shutdown()
