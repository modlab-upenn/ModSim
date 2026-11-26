"""Run a ``modsim`` CLI command or a Python script with exact state tracing.

Usage::

    python scripts/regression/run_traced.py --out DIR [--every N] cli -- pack validate PACK
    python scripts/regression/run_traced.py --out DIR [--every N] script -- path/to/x.py --arg

Writes ``DIR/stdout.txt``, ``DIR/exit_code.txt`` and, when the command creates a
RuntimeSession, ``DIR/trace.txt`` (see state_trace.py). Paths written by the
command itself (``--snapshot``, ``--json``) should point inside DIR so that
compare_runs.py checks them too.
"""

from __future__ import annotations

import argparse
import contextlib
import io
import os
import runpy
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import state_trace


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument(
        "--every", type=int, default=250, help="Trace every Nth step (0 = final only)"
    )
    parser.add_argument("mode", choices=("cli", "script"))
    parser.add_argument("rest", nargs=argparse.REMAINDER)
    args = parser.parse_args()
    rest = args.rest[1:] if args.rest[:1] == ["--"] else args.rest
    args.out.mkdir(parents=True, exist_ok=True)
    tracer = state_trace.install(args.out / "trace.txt", args.every)

    import modsim

    (args.out / "modsim_path.txt").write_text(str(Path(modsim.__file__).resolve()) + "\n")
    # Stable rich/typer rendering regardless of terminal.
    os.environ.setdefault("COLUMNS", "200")
    os.environ["NO_COLOR"] = "1"
    os.environ["TERM"] = "dumb"

    buffer = io.StringIO()
    started = time.perf_counter()
    code = 0
    with contextlib.redirect_stdout(buffer):
        try:
            if args.mode == "cli":
                from modsim.cli import app

                app(args=rest, prog_name="modsim", standalone_mode=False)
            else:
                script = rest[0]
                sys.argv = [script, *rest[1:]]
                runpy.run_path(script, run_name="__main__")
        except SystemExit as exit_:
            code = exit_.code if isinstance(exit_.code, int) else (0 if exit_.code is None else 1)
        except Exception as error:  # click/typer exits arrive as exceptions here
            exit_code = getattr(error, "exit_code", None)
            if exit_code is None:
                raise
            code = int(exit_code)
    wall = time.perf_counter() - started
    tracer.close()
    (args.out / "stdout.txt").write_text(buffer.getvalue())
    (args.out / "exit_code.txt").write_text(f"{code}\n")
    (args.out / "wall_seconds.txt").write_text(f"{wall:.3f}\n")
    if tracer.records == 0:
        (args.out / "trace.txt").unlink(missing_ok=True)
    last = buffer.getvalue().strip().splitlines()[-1:] or [""]
    print(f"exit={code} trace_records={tracer.records} wall={wall:.1f}s | {last[0][:120]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
