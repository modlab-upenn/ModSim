"""Readable user and diagnostic formatting for Studio input failures."""

from __future__ import annotations

from collections.abc import Iterable
from typing import Any, cast

from pydantic import TypeAdapter, ValidationError
from pydantic_core import ErrorDetails

from modsim.robot_packs.schema import Identifier

_IDENTIFIER_PATTERN = r"^[a-z][a-z0-9]*(?:_[a-z0-9]+)*$"
_IDENTIFIER_ADAPTER: TypeAdapter[str] = TypeAdapter(Identifier)


class IdentifierInputError(ValueError):
    """Readable identifier failure with structured local-log context."""

    def __init__(self, message: str, *, field_name: str, rejected_value: str) -> None:
        super().__init__(message)
        self.field_name = field_name
        self.rejected_value = rejected_value


def validate_identifier(raw_value: str, *, field_name: str) -> str:
    """Validate one schema identifier and explain the convention in UI terms."""
    candidate = raw_value.strip()
    try:
        return _IDENTIFIER_ADAPTER.validate_python(candidate)
    except ValidationError as error:
        suggestion = _lowercase_suggestion(candidate)
        suggestion_text = f' Try "{suggestion}".' if suggestion is not None else ""
        raise IdentifierInputError(
            (
                f"{field_name} must use lowercase snake_case.{suggestion_text}\n\n"
                "It must start with a lowercase letter and contain only lowercase letters, "
                "numbers, and single underscores. Examples: ep, smores_ep, magnetic_face_1.\n\n"
                "Use the separate Name field for display capitalization such as EP."
            ),
            field_name=field_name,
            rejected_value=candidate,
        ) from error


def user_error_message(error: Exception) -> str:
    """Convert an exception into concise, field-oriented dialog text."""
    if not isinstance(error, ValidationError):
        message = str(error).strip()
        return message or type(error).__name__

    issues = error.errors(include_url=False)
    heading = "Please correct this field:" if len(issues) == 1 else "Please correct these fields:"
    lines = [heading]
    for issue in issues:
        location = _format_location(issue.get("loc", ()))
        message = _format_issue_message(issue)
        lines.append(f"• {location}: {message}")
    return "\n".join(lines)


def error_log_details(error: Exception) -> str:
    """Return stable technical details suitable for the local session log."""
    if isinstance(error, IdentifierInputError):
        return (
            f"error_type=IdentifierInputError; field={error.field_name!r}; "
            f"input={_bounded_repr(error.rejected_value)}; rule=Robot Pack Identifier"
        )
    if not isinstance(error, ValidationError):
        return f"error_type={type(error).__name__}; message={error}"

    issues = error.errors(include_url=False)
    lines = [f"error_type=ValidationError; model={error.title}; issue_count={len(issues)}"]
    for issue in issues:
        location = ".".join(str(part) for part in issue.get("loc", ())) or "<root>"
        issue_type = str(issue.get("type", "unknown"))
        message = str(issue.get("msg", "validation failed"))
        input_value = _bounded_repr(issue.get("input"))
        lines.append(
            f"location={location}; type={issue_type}; input={input_value}; message={message}"
        )
    return "\n".join(lines)


def _lowercase_suggestion(candidate: str) -> str | None:
    lowered = candidate.lower()
    if lowered == candidate:
        return None
    try:
        return _IDENTIFIER_ADAPTER.validate_python(lowered)
    except ValidationError:
        return None


def _format_location(raw_location: Iterable[str | int]) -> str:
    parts: list[str] = []
    for part in raw_location:
        if isinstance(part, int):
            parts.append(f"item {part + 1}")
        else:
            parts.append(str(part).replace("_", " "))
    if not parts:
        return "Input"
    return " → ".join(parts).capitalize()


def _format_issue_message(issue: ErrorDetails) -> str:
    issue_type = str(issue.get("type", ""))
    context = cast(dict[str, object], issue.get("ctx") or {})
    if issue_type == "string_pattern_mismatch" and context.get("pattern") == _IDENTIFIER_PATTERN:
        return (
            "Use lowercase snake_case: start with a lowercase letter, then use only "
            "lowercase letters, numbers, and single underscores (for example, smores_ep)."
        )
    if issue_type in {"float_parsing", "float_type"}:
        return "Enter a number."
    if issue_type == "missing":
        return "This field is required."
    message = str(issue.get("msg", "Validation failed."))
    if message.startswith("Value error, "):
        message = message.removeprefix("Value error, ")
    return message.rstrip(".") + "."


def _bounded_repr(value: Any, limit: int = 240) -> str:
    rendered = repr(value)
    if len(rendered) <= limit:
        return rendered
    return rendered[: limit - 3] + "..."
