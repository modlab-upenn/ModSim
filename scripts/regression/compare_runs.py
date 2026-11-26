"""Compare a baseline run directory with a candidate run directory, file by file.

Every file in the baseline case directory must exist in the candidate one and be
identical, except files listed in IGNORED (wall-clock timings and interpreter
paths). For traces the report names the first differing record and line; for
JSON it names the first differing key path.

Usage::

    python scripts/regression/compare_runs.py BASELINE_DIR CANDIDATE_DIR [CASE ...]

Exit status is 0 when every case matches, 1 otherwise.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any

IGNORED = {"wall_seconds.txt", "modsim_path.txt", "stderr.txt"}
# Keys whose values are wall-clock measurements, not simulation results.
WALL_CLOCK_KEYS = {
    "planning_ms", "setup_wall_s", "wall_s", "physics_wall_s", "sync_wall_s",
    "controller_wall_s", "real_time_factor", "module_steps_per_s", "tick_ms_mean",
    "tick_ms_p95", "peak_rss_mb",
}  # fmt: skip


def _strip(value: Any) -> Any:
    if isinstance(value, dict):
        return {k: _strip(v) for k, v in value.items() if k not in WALL_CLOCK_KEYS}
    if isinstance(value, list):
        return [_strip(v) for v in value]
    return value


def _first_json_difference(a: Any, b: Any, path: str = "$") -> str | None:
    if type(a) is not type(b):
        return f"{path}: {type(a).__name__} != {type(b).__name__}"
    if isinstance(a, dict):
        for key in sorted(set(a) | set(b)):
            if key not in a or key not in b:
                return f"{path}.{key}: present in only one run"
            found = _first_json_difference(a[key], b[key], f"{path}.{key}")
            if found:
                return found
        return None
    if isinstance(a, list):
        if len(a) != len(b):
            return f"{path}: length {len(a)} != {len(b)}"
        for index, (x, y) in enumerate(zip(a, b, strict=True)):
            found = _first_json_difference(x, y, f"{path}[{index}]")
            if found:
                return found
        return None
    if a != b:
        return f"{path}: {a!r} != {b!r}"
    return None


def _compare_json_text(a: str, b: str) -> str | None:
    try:
        left, right = json.loads(a), json.loads(b)
    except json.JSONDecodeError:
        return None if a == b else "contents differ"
    return _first_json_difference(_strip(left), _strip(right))


def _compare_lines(a: str, b: str, *, jsonl: bool) -> str | None:
    left, right = a.splitlines(), b.splitlines()
    header = "(start)"
    for number, (x, y) in enumerate(zip(left, right, strict=False), start=1):
        if x.startswith("=== "):
            header = x.rsplit(" sha=", 1)[0]
        if x == y:
            continue
        if x.startswith("=== ") and y.startswith("=== ") and x.rsplit(" sha=", 1)[0] == header:
            continue  # same record, different digest: report the first differing body line
        if jsonl:
            found = _compare_json_text(x, y)
            if found is None:
                continue
            return f"line {number} (frame {number}): {found}"
        return (
            f"first difference at line {number}, in [{header}]:\n"
            f"      base: {x[:240]}\n      new:  {y[:240]}"
        )
    if len(left) != len(right):
        return f"line counts differ: {len(left)} != {len(right)}"
    return None


def compare_file(base: Path, new: Path) -> str | None:
    if not new.exists():
        return "missing in candidate run"
    a, b = base.read_text(), new.read_text()
    if base.suffix in {".json", ".txt"} and a.lstrip()[:1] in {"{", "["}:
        return _compare_json_text(a, b)
    if base.suffix == ".jsonl":
        return _compare_lines(a, b, jsonl=True)
    if a == b:
        return None
    return _compare_lines(a, b, jsonl=False)


def compare_case(base: Path, new: Path) -> list[str]:
    problems: list[str] = []
    if not new.is_dir():
        return ["candidate run produced no output directory"]
    for path in sorted(p for p in base.rglob("*") if p.is_file()):
        if path.name in IGNORED:
            continue
        relative = path.relative_to(base)
        found = compare_file(path, new / relative)
        if found:
            problems.append(f"{relative}: {found}")
    for path in sorted(p for p in new.rglob("*") if p.is_file()):
        relative = path.relative_to(new)
        if path.name not in IGNORED and not (base / relative).exists():
            problems.append(f"{relative}: only in candidate run")
    return problems


def main() -> int:
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    base_root, new_root = Path(sys.argv[1]), Path(sys.argv[2])
    names = sys.argv[3:]
    cases = (
        [base_root / name for name in names]
        if names
        else sorted(p for p in base_root.iterdir() if p.is_dir())
    )
    failures = 0
    for case in cases:
        problems = compare_case(case, new_root / case.name)
        if problems:
            failures += 1
            print(f"FAIL  {case.name}")
            for problem in problems:
                print(f"      {problem}")
        else:
            print(f"PASS  {case.name}")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
