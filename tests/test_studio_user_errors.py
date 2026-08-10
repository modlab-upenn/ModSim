"""Tests for readable Studio input and validation errors."""

from __future__ import annotations

import pytest
from pydantic import ValidationError

from modsim.robot_packs import ConnectorTypeSpec
from modsim_studio.user_errors import (
    error_log_details,
    user_error_message,
    validate_identifier,
)


def test_identifier_validation_explains_lowercase_display_name_split() -> None:
    with pytest.raises(ValueError) as captured:
        validate_identifier("EP", field_name="Connector type ID")

    message = str(captured.value)
    assert "lowercase snake_case" in message
    assert 'Try "ep"' in message
    assert "separate Name field" in message
    assert "pydantic.dev" not in message

    details = error_log_details(captured.value)
    assert "error_type=IdentifierInputError" in details
    assert "field='Connector type ID'" in details
    assert "input='EP'" in details
    assert "rule=Robot Pack Identifier" in details


def test_identifier_validation_strips_valid_input() -> None:
    assert validate_identifier("  smores_ep  ", field_name="Connector type ID") == "smores_ep"


def test_pydantic_errors_are_field_oriented_without_documentation_urls() -> None:
    with pytest.raises(ValidationError) as captured:
        ConnectorTypeSpec(id="EP", compatible_with=("EP",))

    message = user_error_message(captured.value)
    assert "Please correct these fields" in message
    assert "Id:" in message
    assert "Compatible with → item 1:" in message
    assert "lowercase snake_case" in message
    assert "string_pattern_mismatch" not in message
    assert "pydantic.dev" not in message


def test_validation_log_details_preserve_machine_readable_context() -> None:
    with pytest.raises(ValidationError) as captured:
        ConnectorTypeSpec(id="EP", compatible_with=("EP",))

    details = error_log_details(captured.value)
    assert "model=ConnectorTypeSpec" in details
    assert "issue_count=2" in details
    assert "location=id" in details
    assert "location=compatible_with.0" in details
    assert "type=string_pattern_mismatch" in details
    assert "input='EP'" in details
