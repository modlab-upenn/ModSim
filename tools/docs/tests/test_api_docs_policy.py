"""Regression cases for static API contracts and the legacy-debt ratchet."""

from __future__ import annotations

import tempfile
import textwrap
import unittest
from pathlib import Path

from tools.docs.check_api_docs import Symbol, evaluate, inspect_module, read_scope


def inspect(source: str) -> list[Symbol]:
    return inspect_module(textwrap.dedent(source), "example.api")


def baseline(symbols: list[Symbol]) -> dict:
    return {
        "schema_version": 1,
        "symbols": {
            symbol.name: {"signature": symbol.signature, "missing": list(symbol.missing)}
            for symbol in symbols
            if symbol.missing
        },
    }


class ApiDocumentationPolicyTests(unittest.TestCase):
    def test_new_api_requires_summaries_annotations_and_contract(self) -> None:
        symbols = inspect("""
            class Example:
                def compute(self, value):
                    return value
        """)
        issues = {symbol.name: set(symbol.missing) for symbol in symbols}
        self.assertEqual(issues["example.api"], {"doc:summary"})
        self.assertEqual(issues["example.api.Example"], {"doc:summary"})
        self.assertEqual(
            issues["example.api.Example.compute"],
            {
                "doc:summary",
                "annotation:param:value",
                "annotation:return",
                "doc:param:value",
            },
        )
        self.assertTrue(evaluate(symbols, {"schema_version": 1, "symbols": {}})[0])

    def test_google_contract_covers_positional_keyword_and_variadic_arguments(self) -> None:
        symbols = inspect('''
            """Example API."""
            def compute(
                value: int, /, *items: int, flag: bool = True, **options: str
            ) -> int | None:
                """Compute a result.

                Args:
                    value: Initial value.
                    *items: Other values.
                    flag: Whether computation is enabled.
                    **options: Named settings.

                Returns:
                    A result or None when disabled.

                Raises:
                    ValueError: If value is negative.
                """
                if value < 0:
                    raise ValueError("negative")
                return value if flag else None
        ''')
        self.assertTrue(all(not symbol.missing for symbol in symbols))

    def test_async_static_class_and_property_surfaces(self) -> None:
        symbols = inspect('''
            """Example API."""
            class Example:
                """An example service."""
                @staticmethod
                def parse(self: str) -> None:
                    """Read text.

                    Args:
                        self: Text rather than an instance receiver.
                    """
                @classmethod
                async def create(cls, value: int) -> "Example":
                    """Create a service.

                    Args:
                        value: Initial value.

                    Returns:
                        The created service.
                    """
                @property
                def value(self) -> int:
                    """Read the current value.

                    Returns:
                        The current value.
                    """
                    return 1
                @value.setter
                def value(self, value):
                    pass
        ''')
        self.assertTrue(all(not symbol.missing for symbol in symbols), symbols)
        self.assertEqual(sum(symbol.name.endswith(".value") for symbol in symbols), 1)

    def test_rst_fields_and_multiline_descriptions(self) -> None:
        symbols = inspect('''
            """Example API."""
            def lookup(value: int, **options: str) -> str:
                """Look up an item.

                :param int value:
                    The item identifier.
                :param **options: Named settings.
                :returns: The matching item.
                :raises KeyError: If the item is absent.
                """
                raise KeyError(value)
        ''')
        self.assertTrue(all(not symbol.missing for symbol in symbols), symbols)

    def test_stale_or_empty_parameter_descriptions_fail(self) -> None:
        symbols = inspect('''
            """Example API."""
            def compute(value: int, flag: bool) -> None:
                """Compute a value.

                Args:
                    old_name: A parameter that no longer exists.
                    value:
                    flag: Enables computation.
                """
        ''')
        self.assertEqual(set(symbols[-1].missing), {"doc:param:value", "doc:stale-param:old_name"})

    def test_generators_accept_yields_and_ignore_nested_raise(self) -> None:
        symbols = inspect('''
            """Example API."""
            def iterate() -> Iterator[int]:
                """Iterate over values.

                Yields:
                    Each value in order.
                """
                def helper():
                    raise ValueError("outside this callable's contract")
                yield 1
        ''')
        self.assertTrue(all(not symbol.missing for symbol in symbols))

    def test_direct_exception_types_require_documentation(self) -> None:
        symbols = inspect('''
            """Example API."""
            def fail() -> None:
                """Reject the request."""
                raise errors.CustomError("failure")
        ''')
        self.assertEqual(symbols[-1].missing, ("doc:raises:CustomError",))

    def test_none_and_never_return_annotations_need_no_returns_section(self) -> None:
        symbols = inspect('''
            """Example API."""
            def finish() -> "None":
                """Finish the operation."""
            def abort() -> Never:
                """Abort the operation.

                Raises:
                    RuntimeError: Always; this operation cannot finish.
                """
                raise RuntimeError()
        ''')
        self.assertTrue(all(not symbol.missing for symbol in symbols))

    def test_changed_signature_cannot_reuse_legacy_exemption(self) -> None:
        old = inspect('''
            """Example API."""
            def compute(value: int) -> int:
                """Compute a value."""
                return value
        ''')
        changed = inspect('''
            """Example API."""
            def compute(value: int = 1) -> int:
                """Compute a value."""
                return value
        ''')
        self.assertFalse(evaluate(old, baseline(old))[0])
        self.assertTrue(evaluate(changed, baseline(old))[0])

    def test_doc_clarification_may_preserve_existing_debt(self) -> None:
        source = '''
            """Example API."""
            def compute(value: int) -> int:
                """Compute a value."""
                return value
        '''
        old = inspect(source)
        clarified = inspect(source.replace("Compute a value.", "Compute a deterministic value."))
        self.assertFalse(evaluate(clarified, baseline(old))[0])

    def test_body_only_change_cannot_add_undocumented_exception_debt(self) -> None:
        source = '''
            """Example API."""
            def compute(value: int) -> int:
                """Compute a value."""
                return value
        '''
        old = inspect(source)
        changed = inspect(source.replace("return value", "raise ValueError(value)"))
        self.assertEqual(old[-1].signature, changed[-1].signature)
        self.assertTrue(evaluate(changed, baseline(old))[0])

    def test_baseline_only_shrinks_and_never_adds_new_debt(self) -> None:
        source = '''
            """Example API."""
            def compute(value: int) -> int:
                """Compute a value."""
                return value
        '''
        old = inspect(source)
        improved = inspect(
            source.replace(
                '"""Compute a value."""',
                '''"""Compute a value.

                Args:
                    value: The initial value.
                """''',
            )
        )
        errors, smaller = evaluate(improved, baseline(old))
        self.assertFalse(errors)
        self.assertEqual(smaller["symbols"]["example.api.compute"]["missing"], ["doc:returns"])
        self.assertTrue(evaluate(old, smaller)[0])
        self.assertEqual(evaluate(improved, smaller)[1], smaller)
        complete = inspect('''
            """Example API."""
            def compute(value: int) -> int:
                """Compute a value.

                Args:
                    value: The initial value.

                Returns:
                    The computed value.
                """
                return value
        ''')
        self.assertEqual(evaluate(complete, smaller), ([], {"schema_version": 1, "symbols": {}}))

    def test_registry_is_read_without_executing_its_code(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "scope.py"
            path.write_text(
                'raise RuntimeError("must not execute")\nAPI_MODULES = {"core/state.py"}\n'
            )
            self.assertEqual(read_scope(path), ("core/state.py",))
            path.write_text('API_MODULES = {"../outside.py"}\n')
            with self.assertRaises(ValueError):
                read_scope(path)


if __name__ == "__main__":
    unittest.main()
