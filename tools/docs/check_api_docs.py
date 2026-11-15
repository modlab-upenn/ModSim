"""Check source-level contracts for the published Python API without imports.

Existing documentation debt is recorded once in the checked-in baseline. New
symbols and changed signatures must satisfy every rule; unchanged signatures
may retain or reduce their recorded debt. The updater never adds exemptions.
"""

from __future__ import annotations

import argparse
import ast
import hashlib
import json
import re
from dataclasses import dataclass
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SCOPE = ROOT / "website" / "api" / "scope.py"
BASELINE = ROOT / "website" / "api" / "docstring-baseline.json"
SOURCE = ROOT / "src" / "modsim_robotics"
_SECTION = re.compile(
    r"^(Args|Arguments|Parameters|Keyword Args|Keyword Arguments|Returns|Yields|Raises):$"
)
_ENTRY = re.compile(r"^([*\w., ]+?)(?:\s+\([^)]*\))?:\s*(.*)$")
_FIELD = re.compile(r"^:(\w+)(?:\s+([^:]+))?:\s*(.*)$")


@dataclass(frozen=True)
class Symbol:
    """One published source object and its mechanically detectable issues."""

    name: str
    path: str
    line: int
    signature: str
    missing: tuple[str, ...]


def read_scope(path: Path) -> tuple[str, ...]:
    """Read the literal API_MODULES registry without executing configuration."""
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(
            isinstance(target, ast.Name) and target.id == "API_MODULES" for target in node.targets
        ):
            modules = ast.literal_eval(node.value)
            if not isinstance(modules, set | list | tuple) or not modules:
                raise ValueError("API_MODULES must be a nonempty literal collection")
            if not all(
                isinstance(item, str)
                and item.endswith(".py")
                and not Path(item).is_absolute()
                and ".." not in Path(item).parts
                for item in modules
            ):
                raise ValueError("API_MODULES must contain relative Python source paths")
            return tuple(sorted(modules))
    raise ValueError("scope registry has no literal API_MODULES assignment")


def _descriptions(lines: list[str]) -> dict[str, str]:
    """Read entries at a Google section's first indentation level."""
    nonempty = [line for line in lines if line.strip()]
    if not nonempty:
        return {}
    indent = min(len(line) - len(line.lstrip()) for line in nonempty)
    entries: dict[str, str] = {}
    names: list[str] = []
    for line in lines:
        text = line.strip()
        depth = len(line) - len(line.lstrip())
        match = _ENTRY.match(text) if depth == indent else None
        if match:
            names = [name.strip().lstrip("*") for name in match[1].split(",")]
            for name in names:
                entries[name] = match[2].strip()
        elif names and text and depth > indent:
            for name in names:
                entries[name] += " " + text
    return entries


def parse_docstring(docstring: str) -> tuple[set[str], bool, bool, set[str]]:
    """Read described arguments, returns/yields, and raises in Google or RST."""
    lines = docstring.splitlines()
    params: set[str] = set()
    raises: set[str] = set()
    returns = yields = False
    for index, line in enumerate(lines):
        # RST fields use an optional type before the parameter's final name.
        field = _FIELD.match(line.strip())
        if field:
            body = field[3].strip()
            for continuation in lines[index + 1 :]:
                if continuation.strip() and not continuation[:1].isspace():
                    break
                body += " " + continuation.strip()
            if body.strip():
                kind, target = field[1], field[2] or ""
                if kind in {"param", "parameter", "arg", "keyword"} and target:
                    params.add(target.split()[-1].lstrip("*"))
                elif kind in {"return", "returns"}:
                    returns = True
                elif kind in {"yield", "yields"}:
                    yields = True
                elif kind in {"raise", "raises", "except", "exception"} and target:
                    raises.add(target.rsplit(".", 1)[-1])
        section = _SECTION.fullmatch(line)
        if section is None:
            continue
        body_lines: list[str] = []
        for candidate in lines[index + 1 :]:
            if candidate.strip() and not candidate[:1].isspace():
                break
            body_lines.append(candidate)
        title = section[1]
        entries = _descriptions(body_lines)
        if title in {"Args", "Arguments", "Parameters", "Keyword Args", "Keyword Arguments"}:
            params.update(name for name, description in entries.items() if description.strip())
        elif title == "Raises":
            raises.update(
                name.rsplit(".", 1)[-1]
                for name, description in entries.items()
                if description.strip()
            )
        elif any(text.strip() for text in body_lines):
            returns |= title == "Returns"
            yields |= title == "Yields"
    return params, returns, yields, raises


class _BodyFacts(ast.NodeVisitor):
    """Inspect a callable body, excluding nested functions/classes."""

    def __init__(self) -> None:
        self.raises: set[str] = set()
        self.generator = False

    def visit_Raise(self, node: ast.Raise) -> None:
        target = node.exc.func if isinstance(node.exc, ast.Call) else node.exc
        name = target.id if isinstance(target, ast.Name) else ""
        if isinstance(target, ast.Attribute):
            name = target.attr
        if name and name[:1].isupper():
            self.raises.add(name)

    def visit_Yield(self, node: ast.Yield) -> None:
        self.generator = True

    def visit_YieldFrom(self, node: ast.YieldFrom) -> None:
        self.generator = True

    def visit_FunctionDef(self, node: ast.FunctionDef) -> None:
        pass

    def visit_AsyncFunctionDef(self, node: ast.AsyncFunctionDef) -> None:
        pass

    def visit_ClassDef(self, node: ast.ClassDef) -> None:
        pass

    def visit_Lambda(self, node: ast.Lambda) -> None:
        pass


def _fingerprint(parts: list[object]) -> str:
    return hashlib.sha256(json.dumps(parts, sort_keys=True).encode()).hexdigest()


def _callable_issues(node: ast.FunctionDef | ast.AsyncFunctionDef, method: bool) -> set[str]:
    docstring = ast.get_docstring(node) or ""
    documented, returns, yields, raises = parse_docstring(docstring)
    args = [*node.args.posonlyargs, *node.args.args, *node.args.kwonlyargs]
    decorators = [ast.unparse(item).rsplit(".", 1)[-1] for item in node.decorator_list]
    positional = [*node.args.posonlyargs, *node.args.args]
    if (
        method
        and "staticmethod" not in decorators
        and positional
        and positional[0].arg in {"self", "cls"}
    ):
        args.remove(positional[0])
    args.extend(argument for argument in (node.args.vararg, node.args.kwarg) if argument)
    expected = {argument.arg for argument in args}
    missing = {
        f"annotation:param:{argument.arg}" for argument in args if argument.annotation is None
    }
    missing.update(f"doc:param:{name}" for name in expected - documented)
    missing.update(f"doc:stale-param:{name}" for name in documented - expected)
    facts = _BodyFacts()
    for statement in node.body:
        facts.visit(statement)
    if node.returns is None:
        missing.add("annotation:return")
    elif (
        ast.unparse(node.returns).strip("'\"") not in {"None", "NoReturn", "Never"}
        and not returns
        and not (facts.generator and yields)
    ):
        missing.add("doc:returns")
    missing.update(f"doc:raises:{name}" for name in facts.raises - raises)
    return missing


def inspect_module(source: str, module: str, path: str = "<source>") -> list[Symbol]:
    """Inspect module, public classes, and public callables from syntax alone."""
    tree = ast.parse(source, filename=path)
    symbols: list[Symbol] = []

    def add(
        node: ast.Module | ast.ClassDef | ast.FunctionDef | ast.AsyncFunctionDef, name: str
    ) -> None:
        issues: set[str] = set()
        docstring = ast.get_docstring(node) or ""
        first_line = docstring.splitlines()[0].strip() if docstring.strip() else ""
        if not first_line or _SECTION.fullmatch(first_line) or _FIELD.match(first_line):
            issues.add("doc:summary")
        if isinstance(node, ast.FunctionDef | ast.AsyncFunctionDef):
            issues.update(_callable_issues(node, name.count(".") > module.count(".") + 1))
            signature = _fingerprint(
                [
                    type(node).__name__,
                    ast.dump(node.args),
                    ast.dump(node.returns) if node.returns else None,
                    [ast.dump(item) for item in node.decorator_list],
                ]
            )
        elif isinstance(node, ast.ClassDef):
            signature = _fingerprint(
                [
                    [ast.dump(item) for item in node.bases],
                    [ast.dump(item) for item in node.keywords],
                    [ast.dump(item) for item in node.decorator_list],
                ]
            )
        else:
            signature = _fingerprint([module])
        symbols.append(
            Symbol(name, path, getattr(node, "lineno", 1), signature, tuple(sorted(issues)))
        )

    def walk(body: list[ast.stmt], prefix: str) -> None:
        for node in body:
            if isinstance(node, ast.ClassDef) and not node.name.startswith("_"):
                add(node, prefix + "." + node.name)
                walk(node.body, prefix + "." + node.name)
            elif isinstance(
                node, ast.FunctionDef | ast.AsyncFunctionDef
            ) and not node.name.startswith("_"):
                if any(
                    isinstance(decorator, ast.Attribute) and decorator.attr in {"setter", "deleter"}
                    for decorator in node.decorator_list
                ):
                    continue
                add(node, prefix + "." + node.name)

    add(tree, module)
    walk(tree.body, module)
    return symbols


def collect_symbols(source: Path = SOURCE, scope: Path = SCOPE) -> list[Symbol]:
    """Inspect exactly the module files registered for publication."""
    symbols = []
    for relative in read_scope(scope):
        path = source / relative
        module = "modsim_robotics." + relative.removesuffix(".py").replace("/", ".")
        if module.endswith(".__init__"):
            module = module.removesuffix(".__init__")
        symbols.extend(inspect_module(path.read_text(encoding="utf-8"), module, relative))
    return symbols


def evaluate(symbols: list[Symbol], baseline: dict) -> tuple[list[str], dict]:
    """Reject new debt and return a baseline containing only remaining debt."""
    if baseline.get("schema_version") != 1 or not isinstance(baseline.get("symbols"), dict):
        raise ValueError("unsupported documentation baseline format")
    old = baseline["symbols"]
    remaining = {}
    errors = []
    for symbol in symbols:
        if not symbol.missing:
            continue
        previous = old.get(symbol.name)
        allowed = (
            isinstance(previous, dict)
            and previous.get("signature") == symbol.signature
            and set(symbol.missing) <= set(previous.get("missing", []))
        )
        if not allowed:
            errors.append(
                f"{symbol.path}:{symbol.line}: {symbol.name}: " + ", ".join(symbol.missing)
            )
        else:
            remaining[symbol.name] = {
                "signature": symbol.signature,
                "missing": list(symbol.missing),
            }
    return errors, {"schema_version": 1, "symbols": dict(sorted(remaining.items()))}


def main() -> int:
    """Check the registered API; optionally prune resolved baseline debt."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--update-baseline",
        action="store_true",
        help="Remove resolved debt only; never add exemptions.",
    )
    args = parser.parse_args()
    try:
        symbols = collect_symbols()
        baseline = json.loads(BASELINE.read_text(encoding="utf-8"))
        errors, remaining = evaluate(symbols, baseline)
    except (OSError, ValueError, SyntaxError) as error:
        parser.exit(2, f"API documentation policy: {error}\n")
    if errors:
        for error in errors:
            print(error)
        print(
            "API documentation policy failed. Complete the contract for new/changed API; "
            "baseline updates cannot exempt it."
        )
        return 1
    if args.update_baseline:
        BASELINE.write_text(
            json.dumps(remaining, indent=2, sort_keys=True) + "\n", encoding="utf-8"
        )
    print(
        f"API documentation policy passed for {len(symbols)} objects; "
        f"{len(remaining['symbols'])} retain recorded legacy debt."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
