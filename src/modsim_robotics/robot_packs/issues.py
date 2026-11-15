"""Structured validation issues shared by the CLI and future Studio UI."""

from __future__ import annotations

from enum import StrEnum
from typing import Self

from pydantic import BaseModel, ConfigDict, Field


class Severity(StrEnum):
    """Validation issue severity."""

    ERROR = "error"
    WARNING = "warning"
    INFO = "info"


class ValidationIssue(BaseModel):
    """One stable, actionable Robot Pack issue."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    severity: Severity
    code: str = Field(min_length=1)
    message: str = Field(min_length=1)
    document: str | None = None
    path: tuple[str | int, ...] = ()
    entity_ref: str | None = None
    suggested_fix: str | None = None

    @property
    def json_pointer(self) -> str:
        """Return the issue path as an RFC 6901-style JSON pointer."""
        if not self.path:
            return ""
        parts = (str(part).replace("~", "~0").replace("/", "~1") for part in self.path)
        return "/" + "/".join(parts)

    @property
    def location(self) -> str:
        """Return a compact document and field location."""
        document = self.document or "<robot-pack>"
        return f"{document}:{self.json_pointer}" if self.json_pointer else document


class ValidationReport(BaseModel):
    """Deterministically ordered collection of validation issues."""

    model_config = ConfigDict(extra="forbid", frozen=True)

    issues: tuple[ValidationIssue, ...] = ()

    @classmethod
    def from_issues(cls, issues: list[ValidationIssue]) -> Self:
        """Build a report with a stable presentation order."""
        severity_order = {
            Severity.ERROR: 0,
            Severity.WARNING: 1,
            Severity.INFO: 2,
        }
        ordered = sorted(
            issues,
            key=lambda issue: (
                severity_order[issue.severity],
                issue.document or "",
                tuple(str(part) for part in issue.path),
                issue.code,
                issue.message,
            ),
        )
        return cls(issues=tuple(ordered))

    @property
    def valid(self) -> bool:
        """Whether the report contains no errors."""
        return not any(issue.severity is Severity.ERROR for issue in self.issues)

    @property
    def errors(self) -> tuple[ValidationIssue, ...]:
        """Return all error issues."""
        return tuple(issue for issue in self.issues if issue.severity is Severity.ERROR)

    @property
    def warnings(self) -> tuple[ValidationIssue, ...]:
        """Return all warning issues."""
        return tuple(issue for issue in self.issues if issue.severity is Severity.WARNING)

    @property
    def infos(self) -> tuple[ValidationIssue, ...]:
        """Return all informational issues."""
        return tuple(issue for issue in self.issues if issue.severity is Severity.INFO)

    def raise_for_errors(self) -> None:
        """Raise when the report contains validation errors."""
        if not self.valid:
            raise RobotPackValidationError(self)


class RobotPackValidationError(Exception):
    """Raised when a caller requires a valid Robot Pack."""

    def __init__(self, report: ValidationReport) -> None:
        self.report = report
        super().__init__(f"Robot Pack validation failed with {len(report.errors)} error(s)")
