"""Plot ``measurements.csv`` from bench_complexity.py as a one-column IEEE figure.

    python scripts/complexity/plot_complexity.py results/complexity docs/paper/fig_complexity.pdf

Top: per-tick stage cost vs. module count (M-Blocks pack, MuJoCo backend).
Bottom: one dock / one undock vs. module count at fixed assembly size; colour
is the event, line style the backend (solid MuJoCo, dashed core-only mock).
Every line is direct-labelled and carries a distinct marker, so the figure
reads in greyscale print.
"""

from __future__ import annotations

import csv
import sys
from collections import defaultdict
from pathlib import Path

import matplotlib
import matplotlib.ticker

matplotlib.use("Agg")
import matplotlib.pyplot as plt

INK, MUTED, GRID = "#1f1f1e", "#6b6a63", "#e4e3de"
SERIES = {  # categorical slots 1..7 of the validated palette, in fixed order
    "physics": ("#2a78d6", "o", "MuJoCo step"),
    "docking_pass": ("#eb6834", "s", "passive docking pass"),
    "ingest": ("#1baf7a", "^", "ingest"),
    "read_all_poses": ("#eda100", "D", "read every pose"),
    "snapshot": ("#e87ba4", "v", "snapshot"),
}
EVENTS = {"dock": ("#008300", "o"), "undock": ("#6250d6", "s")}


def _load(path: Path) -> dict[tuple[str, str, str], list[tuple[float, float]]]:
    series: dict[tuple[str, str, str], list[tuple[float, float]]] = defaultdict(list)
    with path.open() as handle:
        for row in csv.DictReader(handle):
            series[(row["experiment"], row["backend"], row["stage"])].append(
                (float(row["x"]), float(row["ms"]))
            )
    return {key: sorted(points) for key, points in series.items()}


def _style(ax: plt.Axes) -> None:
    ax.set_xscale("log", base=2)
    ax.set_yscale("log")
    ax.grid(True, which="major", color=GRID, linewidth=0.6)
    ax.set_axisbelow(True)
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(MUTED)
        ax.spines[side].set_linewidth(0.6)
    ax.tick_params(colors=MUTED, labelcolor=INK, labelsize=6.5, width=0.6, length=2.5)


def _label_end(ax: plt.Axes, points: list[tuple[float, float]], text: str, dy: float = 1.0) -> None:
    x, y = points[-1]
    ax.annotate(
        text,
        (x, y * dy),
        xytext=(4, 0),
        textcoords="offset points",
        va="center",
        fontsize=6.5,
        color=INK,
    )


def main() -> int:
    source, target = Path(sys.argv[1]), Path(sys.argv[2])
    data = _load(source / "measurements.csv")
    plt.rcParams.update({"font.family": "serif", "font.size": 7, "pdf.fonttype": 42})
    fig, (top, bottom) = plt.subplots(2, 1, figsize=(3.45, 5.0), gridspec_kw={"hspace": 0.95})

    # (a) per-tick stages
    for stage, (color, marker, name) in SERIES.items():
        points = data[("tick", "mujoco", stage)]
        ax_x = [p[0] for p in points]
        ax_y = [p[1] for p in points]
        top.plot(
            ax_x,
            ax_y,
            color=color,
            marker=marker,
            markersize=3.6,
            linewidth=1.2,
            markeredgecolor="white",
            markeredgewidth=0.5,
            label=name,
        )
    first = data[("tick", "mujoco", "physics")]
    x0, y0 = first[0]
    guide = [(x, y0 * x / x0) for x, _ in first]
    top.plot(
        [g[0] for g in guide],
        [g[1] * 0.12 for g in guide],
        color=MUTED,
        linewidth=0.7,
        linestyle=(0, (2, 2)),
        label="slope 1 (linear)",
    )
    _style(top)
    top.set_xlim(6, 700)
    top.set_xlabel("modules $N$ (M-Blocks, 14 connectors each)", fontsize=7, color=INK)
    top.set_ylabel("ms per tick", fontsize=7, color=INK)
    top.set_title("(a) Per-tick stages", fontsize=7.5, color=INK, loc="left")
    top.legend(
        fontsize=6,
        frameon=False,
        loc="upper center",
        bbox_to_anchor=(0.5, -0.3),
        ncol=3,
        handlelength=1.8,
        columnspacing=1.0,
    )

    # (b) per-event costs
    for event, (color, marker) in EVENTS.items():
        for backend, dash, label in (
            ("mujoco", "-", "MuJoCo"),
            ("mock", (0, (3, 1.6)), "core only"),
        ):
            points = data[("event_n", backend, event)]
            bottom.plot(
                [p[0] for p in points],
                [p[1] for p in points],
                color=color,
                marker=marker,
                markersize=3.6,
                linewidth=1.2,
                linestyle=dash,
                markeredgecolor="white",
                markeredgewidth=0.5,
                label=f"{event}, {label}",
            )
            # Spread the two end labels of each backend apart, upper one upwards.
            other = "undock" if event == "dock" else "dock"
            mine, theirs = points[-1][1], data[("event_n", backend, other)][-1][1]
            close = max(mine, theirs) / min(mine, theirs) < 1.6
            dy = (1.3 if mine >= theirs else 0.75) if close else 1.0
            _label_end(bottom, points, f"{event} ({label})", dy)
    _style(bottom)
    bottom.set_xlim(12, 9000)
    bottom.set_ylim(0.018, 1.6)
    ticks = [0.02, 0.05, 0.1, 0.2, 0.5, 1.0]
    bottom.yaxis.set_major_locator(matplotlib.ticker.FixedLocator(ticks))
    bottom.yaxis.set_minor_locator(matplotlib.ticker.NullLocator())
    bottom.yaxis.set_major_formatter(matplotlib.ticker.FixedFormatter([f"{t:g}" for t in ticks]))
    bottom.set_xlabel("modules $N$ (assembly of 4 fixed)", fontsize=7, color=INK)
    bottom.set_ylabel("ms per event", fontsize=7, color=INK)
    bottom.set_title("(b) One dock / one undock", fontsize=7.5, color=INK, loc="left")
    bottom.legend(
        fontsize=6,
        frameon=False,
        loc="upper center",
        bbox_to_anchor=(0.5, -0.3),
        ncol=2,
        handlelength=2.4,
        columnspacing=1.0,
    )

    target.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(target, bbox_inches="tight")
    fig.savefig(target.with_suffix(".png"), bbox_inches="tight", dpi=300)
    print(f"wrote {target} and {target.with_suffix('.png')}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
