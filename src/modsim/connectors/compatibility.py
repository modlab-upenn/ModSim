"""Connector-type compatibility.

Compatibility is a *type-level* question answered before any geometry is
considered, because it is far cheaper than the acceptance-region math and
rejects most candidate pairs outright.
"""

from __future__ import annotations

from dataclasses import dataclass

from modsim.robot_packs.schema import ConnectorGender, ConnectorTypeSpec

_GENDER_PAIRS: frozenset[tuple[ConnectorGender, ConnectorGender]] = frozenset(
    {
        (ConnectorGender.MALE, ConnectorGender.FEMALE),
        (ConnectorGender.FEMALE, ConnectorGender.MALE),
        (ConnectorGender.HERMAPHRODITIC, ConnectorGender.HERMAPHRODITIC),
        (ConnectorGender.GENDERLESS, ConnectorGender.GENDERLESS),
    }
)


@dataclass(frozen=True, slots=True)
class CompatibilityResult:
    """Whether two connector types may ever mate, and why not."""

    compatible: bool
    reason: str | None = None

    def __bool__(self) -> bool:
        return self.compatible


def genders_match(left: ConnectorGender, right: ConnectorGender) -> bool:
    """Return whether two connector genders can mate."""
    return (left, right) in _GENDER_PAIRS


def evaluate_compatibility(
    type_a: ConnectorTypeSpec,
    type_b: ConnectorTypeSpec,
) -> CompatibilityResult:
    """Return whether two connector types are declared mutually compatible.

    Compatibility must be declared in both directions. A one-sided declaration
    is treated as incompatible rather than silently assumed symmetric, because
    in a heterogeneous pack the omission is far more likely to be an authoring
    mistake than a deliberate asymmetry.
    """
    if type_a.id not in type_b.compatible_with or type_b.id not in type_a.compatible_with:
        return CompatibilityResult(
            compatible=False,
            reason=(
                f"connector types '{type_a.id}' and '{type_b.id}' are not "
                "declared mutually compatible"
            ),
        )
    if not genders_match(type_a.gender, type_b.gender):
        return CompatibilityResult(
            compatible=False,
            reason=(
                f"connector genders '{type_a.gender.value}' and '{type_b.gender.value}' cannot mate"
            ),
        )
    return CompatibilityResult(compatible=True)
