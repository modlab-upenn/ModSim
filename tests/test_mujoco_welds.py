"""Weld slot allocation and the connector-to-body transform conversion.

Layer 1 of the weld verification: pure geometry and bookkeeping, with no
simulation running. A failure here is an arithmetic or convention error, which
is far cheaper to read than the same bug surfacing as a wobbling assembly.
"""

from __future__ import annotations

import math

import pytest

pytest.importorskip("mujoco", reason="the MuJoCo backend extra is not installed")

from modsim.core.ids import ConstraintHandle
from modsim.core.transforms import Transform, angle_between, quat_from_axis_angle, quat_rotate
from modsim_backend_mujoco.welds import (
    EQ_DATA_WIDTH,
    WeldPool,
    WeldPoolExhaustedError,
    body_relative_transform,
)

pytestmark = pytest.mark.mujoco

X_AXIS = (1.0, 0.0, 0.0)


def handle(name: str) -> ConstraintHandle:
    return ConstraintHandle(name)


# ----------------------------------------------------------------------
# transform conversion
# ----------------------------------------------------------------------


def test_eq_data_slices_cover_the_mujoco_field_width() -> None:
    assert EQ_DATA_WIDTH == 11


def test_coincident_connectors_at_the_body_origin_give_the_identity() -> None:
    result = body_relative_transform(
        Transform.identity(), Transform.identity(), Transform.identity()
    )

    assert result.is_close(Transform.identity())


def test_offset_connectors_sum_along_the_mating_axis() -> None:
    """Two cubes meeting face to face sit two half-widths apart.

    Connector A is 0.05 m along body A's +x; connector B is 0.05 m along body
    B's -x. With the connector frames coincident, body B must therefore sit
    0.1 m from body A.
    """
    result = body_relative_transform(
        connector_a_local=Transform.from_translation((0.05, 0.0, 0.0)),
        connector_b_local=Transform.from_translation((-0.05, 0.0, 0.0)),
        connector_relative=Transform.identity(),
    )

    assert result.translation == pytest.approx((0.1, 0.0, 0.0), abs=1e-12)


def test_the_mate_rotation_carries_through_to_the_bodies() -> None:
    quarter = quat_from_axis_angle(X_AXIS, math.pi / 2)
    result = body_relative_transform(
        connector_a_local=Transform.from_translation((0.05, 0.0, 0.0)),
        connector_b_local=Transform.from_translation((-0.05, 0.0, 0.0)),
        connector_relative=Transform(rotation=quarter),
    )

    # Body B is rotated a quarter turn about x relative to body A, and its
    # origin swings to the far side of the rotated connector offset.
    assert angle_between(quat_rotate(result.rotation, (0.0, 1.0, 0.0)), (0.0, 0.0, 1.0)) == (
        pytest.approx(0.0, abs=1e-9)
    )


def test_a_rotated_connector_mount_is_undone_correctly() -> None:
    """A connector mounted rotated on its own body must not leak that rotation.

    Connector B is mounted with a half turn about z, which is exactly how the
    example pack authors its rear connector. Composing across the mate and back
    out to body B has to cancel it.
    """
    half_turn = quat_from_axis_angle((0.0, 0.0, 1.0), math.pi)
    result = body_relative_transform(
        connector_a_local=Transform.from_translation((0.05, 0.0, 0.0)),
        connector_b_local=Transform(translation=(-0.05, 0.0, 0.0), rotation=half_turn),
        connector_relative=Transform(rotation=half_turn),
    )

    assert result.translation == pytest.approx((0.1, 0.0, 0.0), abs=1e-12)
    assert result.is_close(Transform.from_translation((0.1, 0.0, 0.0)))


def test_the_conversion_is_self_consistent_with_forward_composition() -> None:
    """Round-trip check against the definition the docstring states."""
    connector_a = Transform(
        translation=(0.03, -0.01, 0.02),
        rotation=quat_from_axis_angle((0.0, 1.0, 0.0), 0.4),
    )
    connector_b = Transform(
        translation=(-0.02, 0.04, 0.01),
        rotation=quat_from_axis_angle((1.0, 0.0, 1.0), 1.1),
    )
    mate = Transform(
        translation=(0.001, 0.0, 0.0),
        rotation=quat_from_axis_angle((0.0, 0.0, 1.0), 2.3),
    )

    body_relative = body_relative_transform(connector_a, connector_b, mate)

    # Placing body B at the computed pose must put connector B exactly where
    # the mate says it should be relative to connector A.
    recovered = connector_a.inverse().compose(body_relative).compose(connector_b)
    assert recovered.is_close(mate)


# ----------------------------------------------------------------------
# slot allocation
# ----------------------------------------------------------------------


def test_a_pool_claims_slots_in_order() -> None:
    pool = WeldPool.over((3, 4, 5))
    assert pool.capacity == 3
    assert pool.available == 3

    first = pool.claim(handle("a"))
    second = pool.claim(handle("b"))

    assert (first.equality_id, second.equality_id) == (3, 4)
    assert pool.in_use == 2
    assert pool.available == 1


def test_releasing_a_slot_returns_it_to_the_pool() -> None:
    pool = WeldPool.over((0, 1))
    claimed = pool.claim(handle("a"))
    assert pool.release(handle("a")) is claimed
    assert pool.available == 2

    # The freed slot is reused rather than leaked.
    assert pool.claim(handle("b")).equality_id == 0


def test_releasing_an_unknown_handle_reports_nothing() -> None:
    pool = WeldPool.over((0,))
    assert pool.release(handle("never_claimed")) is None


def test_exhausting_the_pool_explains_how_to_fix_it() -> None:
    pool = WeldPool.over((0,))
    pool.claim(handle("a"))

    with pytest.raises(WeldPoolExhaustedError, match="raise weld_pool_size"):
        pool.claim(handle("b"))


def test_claiming_the_same_handle_twice_is_refused() -> None:
    pool = WeldPool.over((0, 1))
    pool.claim(handle("a"))

    with pytest.raises(WeldPoolExhaustedError, match="already holds a weld slot"):
        pool.claim(handle("a"))


def test_clearing_frees_every_slot() -> None:
    pool = WeldPool.over((0, 1, 2))
    pool.claim(handle("a"))
    pool.claim(handle("b"))
    pool.clear()

    assert pool.available == 3
    assert pool.slot_for(handle("a")) is None
