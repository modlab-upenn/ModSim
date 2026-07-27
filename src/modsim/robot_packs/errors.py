"""Robot Pack loading and writing exceptions."""

from __future__ import annotations

from pathlib import Path

from modsim.robot_packs.issues import ValidationIssue


class RobotPackError(Exception):
    """Base class for Robot Pack failures."""


class RobotPackLoadError(RobotPackError):
    """A Robot Pack could not be parsed into an aggregate model."""

    def __init__(
        self,
        issues: ValidationIssue | tuple[ValidationIssue, ...],
        *,
        source: Path | None = None,
    ) -> None:
        normalized = (issues,) if isinstance(issues, ValidationIssue) else issues
        if not normalized:
            raise ValueError("RobotPackLoadError requires at least one issue")
        self.issues = normalized
        self.issue = normalized[0]
        self.source = source
        if len(normalized) == 1:
            message = f"{self.issue.code} at {self.issue.location}: {self.issue.message}"
        else:
            message = (
                f"{len(normalized)} Robot Pack load errors; first: "
                f"{self.issue.code} at {self.issue.location}: {self.issue.message}"
            )
        super().__init__(message)


class RobotPackWriteError(RobotPackError):
    """A Robot Pack could not be written safely."""
