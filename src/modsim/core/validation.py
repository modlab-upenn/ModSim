"""Small reusable validators for finite numeric configuration values."""

from __future__ import annotations

import math


def require_finite(value: float, name: str) -> None:
    """Raise ``ValueError`` when ``value`` is NaN or infinite."""
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")


def require_finite_nonnegative(value: float, name: str) -> None:
    """Require a finite value greater than or equal to zero."""
    require_finite(value, name)
    if value < 0.0:
        raise ValueError(f"{name} must not be negative")


def require_finite_positive(value: float, name: str) -> None:
    """Require a finite value strictly greater than zero."""
    require_finite(value, name)
    if value <= 0.0:
        raise ValueError(f"{name} must be greater than zero")


__all__ = ["require_finite", "require_finite_nonnegative", "require_finite_positive"]
