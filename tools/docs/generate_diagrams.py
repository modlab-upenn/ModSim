"""Generate the first scoped Pyreverse diagrams from the current source tree.

Run with the documentation environment's Python. Graphviz's ``dot`` executable
must be on PATH. Outputs are ignored static assets staged for Docusaurus.
"""

from __future__ import annotations

import importlib.util
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
OUTPUT = ROOT / "website" / ".generated" / "site" / "generated" / "diagrams"
SOURCES = (
    "src/modsim_robotics/core/assemblies.py",
    "src/modsim_robotics/core/entities.py",
    "src/modsim_robotics/core/state.py",
)
PROJECT = "runtime_model"


def graphviz_renderer() -> tuple[str | None, dict[str, str]]:
    """Find system Graphviz or the ignored Linux checkout-local renderer."""
    env = os.environ.copy()
    dot = shutil.which("dot")
    if dot is not None:
        return dot, env
    local = ROOT / ".local-tools" / "graphviz" / "usr"
    candidate = local / "bin" / "dot"
    if candidate.is_file() and os.access(candidate, os.X_OK):
        libraries = str(local / "lib" / "x86_64-linux-gnu")
        env["LD_LIBRARY_PATH"] = libraries + (
            os.pathsep + env["LD_LIBRARY_PATH"] if env.get("LD_LIBRARY_PATH") else ""
        )
        env["GVBINDIR"] = str(local / "lib" / "x86_64-linux-gnu" / "graphviz")
        return str(candidate), env
    return None, env


def main() -> int:
    """Extract scoped DOT diagrams, render SVG, and stage both formats."""
    if importlib.util.find_spec("pylint") is None:
        print(
            "Pyreverse is missing. Install the locked documentation group with "
            "uv sync --locked --group docs --inexact.",
            file=sys.stderr,
        )
        return 1
    dot, dot_env = graphviz_renderer()
    if dot is None:
        print(
            "Graphviz's dot executable is missing from PATH. "
            "See docs/contribute/documentation-generation.md for installation.",
            file=sys.stderr,
        )
        return 1

    # Keep transient Pyreverse files outside the served directory. Publish only
    # a complete successful set so a failed render does not replace good assets.
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    scratch = ROOT / "website" / ".generated" / "diagrams"
    scratch.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="pyreverse-", dir=scratch) as temporary:
        work = Path(temporary)
        env = os.environ.copy()
        env["PYTHONPATH"] = str(ROOT / "src") + (
            os.pathsep + env["PYTHONPATH"] if env.get("PYTHONPATH") else ""
        )
        subprocess.run(
            [
                sys.executable,
                "-c",
                "from pylint import run_pyreverse; run_pyreverse()",
                "--output=dot",
                "--only-classnames",
                "--module-names=n",
                "--filter-mode=ALL",
                "--no-standalone",
                "--source-roots=src",
                f"--project={PROJECT}",
                f"--output-directory={work}",
                *SOURCES,
            ],
            cwd=ROOT,
            env=env,
            check=True,
        )
        for kind in ("classes", "packages"):
            dot_path = work / f"{kind}_{PROJECT}.dot"
            svg_path = dot_path.with_suffix(".svg")
            # Pyreverse's default bright-green relation labels have poor
            # contrast on white. Change presentation only, retaining every
            # generated node, edge, and field label.
            dot_path.write_text(
                dot_path.read_text().replace('fontcolor="green"', 'fontcolor="#475569"')
            )
            subprocess.run(
                [
                    dot,
                    "-Tsvg",
                    "-Gbgcolor=white",
                    "-Nfontname=Arial",
                    "-Ncolor=#1e293b",
                    "-Nfontcolor=#0f172a",
                    "-Ecolor=#475569",
                    str(dot_path),
                    "-o",
                    str(svg_path),
                ],
                env=dot_env,
                check=True,
            )
        OUTPUT.mkdir(parents=True, exist_ok=True)
        for path in sorted(work.glob("*")):
            shutil.copyfile(path, OUTPUT / path.name)
    print(f"Generated class and package diagrams in {OUTPUT.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
