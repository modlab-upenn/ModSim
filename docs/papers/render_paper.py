"""Build offline paper HTML and vector figures from the recorded reference run.

Documentation-only requirements: matplotlib, numpy, markdown-it-py.
The resulting HTML can be printed to PDF from a browser; no CDN is used.
"""

from __future__ import annotations

import base64
import csv
import io
import json
import re
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from markdown_it import MarkdownIt
from matplotlib.font_manager import FontProperties
from matplotlib.mathtext import math_to_image

HERE = Path(__file__).resolve().parent
STEM = "smores_3d_reconfiguration"


def figures() -> None:
    report = json.loads((HERE / "data/reference_run.json").read_text())
    with (HERE / "data/reference_trace.csv").open() as stream:
        rows = list(csv.DictReader(stream))

    def column(name: str) -> np.ndarray:
        return np.array([float(row[name]) for row in rows])

    plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 9, "svg.fonttype": "none"})
    fig, axes = plt.subplots(1, 4, figsize=(9, 3.15), layout="constrained")
    pos = {"H": (0, 0), "P": (0, 1), "R": (1, 0), "A1": (1, 1), "U": (1, 2)}
    modes = (
        ("(a) Initial", [("H", "P"), ("R", "A1"), ("A1", "U")]),
        ("(b) Capture", [("H", "P"), ("R", "A1"), ("A1", "U"), ("U", "P")]),
        ("(c) Release", [("R", "A1"), ("A1", "U"), ("U", "P")]),
    )
    for ax, (title, edges) in zip(axes, modes, strict=False):
        for a, b in edges:
            color = "#a6247b" if {a, b} == {"U", "P"} else "#384b60"
            ax.plot([pos[a][0], pos[b][0]], [pos[a][1], pos[b][1]], color=color, lw=2)
        for name, (x, y) in pos.items():
            ax.scatter(
                x,
                y,
                s=490,
                marker="s" if name in {"H", "R"} else "o",
                color="#eef2f6",
                edgecolor="#243a52",
                zorder=4,
            )
            ax.text(x, y, name, ha="center", va="center", zorder=5, weight="bold")
        ax.set(xlim=(-0.5, 1.5), ylim=(-0.35, 2.35), title=title)
        ax.axis("off")
    ax = axes[-1]
    centers = [(0.034458, z) for z in (0.060000, 0.150874, 0.241747, 0.331471)]
    ax.plot(*np.array(centers).T, color="#243a52", lw=2)
    for name, (x, z) in zip(("R", "A1", "U", "P", "H"), [*centers, (0.215056, 0.06)], strict=True):
        ax.scatter(
            x,
            z,
            s=130,
            marker="s" if name in {"H", "R"} else "o",
            color="#eef2f6",
            edgecolor="#243a52",
            zorder=4,
        )
        ax.annotate(name, (x, z), xytext=(8, 3), textcoords="offset points")
    ax.set(
        xlim=(0, 0.27),
        ylim=(0.02, 0.37),
        xlabel="World x (m)",
        ylabel="World z (m)",
        title="(d) Vertical target",
    )
    ax.set_xticks([0, 0.2])
    ax.spines[["top", "right"]].set_visible(False)
    fig.savefig(HERE / "figures/modes.svg")
    plt.close(fig)

    fig, axes = plt.subplots(2, 2, figsize=(9, 5.6), sharex=True, layout="constrained")
    t = column("time_s")
    colors = ("#0054a6", "#db6400", "#228833", "#aa3377")
    for module, color in zip(("receiver", "arm", "upper", "payload"), colors, strict=True):
        axes[0, 0].plot(t, column(f"{module}_z_m"), color=color, label=module, lw=1.5)
    axes[0, 0].set(ylabel="Geometric centre height (m)", title="(a) Vertical assembly")
    axes[0, 0].legend(loc="upper left", fontsize=8, ncol=2)
    axes[0, 1].plot(t, 1000 * column("target_error_m"), color="#0054a6", lw=1.5)
    axes[0, 1].axhline(4, color="#aa3377", ls="--", label="Completion: 4 mm")
    axes[0, 1].set(ylabel="Maximum root-position error (mm)", title="(b) Final-target error")
    axes[0, 1].legend(fontsize=8)
    axes[1, 0].plot(t, column("effort_nm"), color="#db6400", lw=1.4)
    axes[1, 0].axhline(1.2, color="#aa3377", ls="--", label="Actuator limit: 1.2 Nm")
    axes[1, 0].set(
        ylabel="Maximum actuator effort (Nm)", ylim=(0, 1.35), title="(c) Bounded actuation"
    )
    axes[1, 0].legend(fontsize=8, loc="lower right")
    axes[1, 1].plot(t, 1000 * column("penetration_m"), color="#0054a6", lw=1.4)
    axes[1, 1].axhline(0.5, color="#228833", ls="--", label="Planning: 0.5 mm")
    axes[1, 1].axhline(4, color="#aa3377", ls="--", label="Runtime abort: >4 mm")
    axes[1, 1].set(ylabel="Proxy penetration (mm)", ylim=(0, 4.5), title="(d) Contact diagnostics")
    axes[1, 1].legend(fontsize=8, loc="upper left")
    for ax in axes.flat:
        for state in report["history"]:
            if state["phase"] in {"transfer", "retreat", "verify"}:
                ax.axvline(state["time_s"], color="#818181", lw=0.8, ls=":")
        ax.grid(alpha=0.18)
        ax.set_xlim(0, report["time_s"])
        ax.spines[["top", "right"]].set_visible(False)
    for ax in axes[-1]:
        ax.set_xlabel("Simulation time (s)")
    fig.savefig(HERE / "figures/execution.svg")
    plt.close(fig)


def render() -> None:
    source = (HERE / f"{STEM}.md").read_text()
    replacements: dict[str, str] = {}
    counter = 0
    equation_numbers = []

    def maths(match: re.Match[str], *, display: bool) -> str:
        nonlocal counter
        formula = match.group(1).strip()
        number = re.search(r"\\tag\{(\d+)\}", formula)
        if number:
            equation_numbers.append(int(number[1]))
        formula = re.sub(r"\\tag\{\d+\}", "", formula).strip()
        formula = " ".join(formula.splitlines())
        stream = io.BytesIO()
        with matplotlib.rc_context({"mathtext.fontset": "stix", "svg.fonttype": "path"}):
            math_to_image(
                f"${formula}$",
                stream,
                format="svg",
                dpi=120,
                prop=FontProperties(size=12, family="STIXGeneral"),
            )
        svg = stream.getvalue().decode()
        svg = svg[svg.index("<svg") :]
        width, height = re.search(r'width="([\d.]+)pt" height="([\d.]+)pt"', svg).groups()
        if display:
            markup = f'<div class="equation"><div>{svg}</div>'
            markup += f"<span>({number[1]})</span></div>" if number else "</div>"
        else:
            svg = svg.replace(
                "<svg ",
                f'<svg style="width:{float(width) / 12:.3f}em;height:{float(height) / 12:.3f}em" ',
                1,
            )
            markup = f'<span class="inline-math">{svg}</span>'
        token = f"MATHPLACEHOLDER{counter}TOKEN"
        counter += 1
        replacements[token] = markup
        return token

    source = re.sub(r"\$\$(.*?)\$\$", lambda m: maths(m, display=True), source, flags=re.S)
    source = re.sub(r"(?<!\\)\$(.+?)(?<!\\)\$", lambda m: maths(m, display=False), source)
    body = MarkdownIt("commonmark").enable("table").render(source)
    for token, markup in replacements.items():
        body = body.replace(f"<p>{token}</p>", markup).replace(token, markup)

    def embed(match: re.Match[str]) -> str:
        path = HERE / match.group(1)
        content = base64.b64encode(path.read_bytes()).decode()
        return f'src="data:image/svg+xml;base64,{content}"'

    body = re.sub(r'src="(figures/[^\"]+\.svg)"', embed, body)
    css = """
    @page { size: A4; margin: 19mm 19mm 20mm;
      @bottom-center { content: counter(page); font: 9pt Georgia; color: #666; } }
    body { max-width: 760px; margin: 35px auto; padding: 0 15px; color: #18202a;
      font: 11pt/1.42 Georgia, 'Times New Roman', serif; }
    h1 { font-size: 23pt; line-height: 1.15; color: #173b5e; margin: 0 0 12px; }
    h1 + h2 { font-size: 14pt; border: 0; font-weight: normal; margin: 0 0 18px; }
    h2 { font-size: 15pt; line-height: 1.25; margin-top: 25px; padding-top: 5px;
      border-top: 1px solid #c8d4df; color: #173b5e; }
    h3 { font-size: 12pt; margin-top: 18px; color: #173b5e; }
    p { margin: 8px 0; orphans: 3; widows: 3; }
    h1,h2,h3 { break-after: avoid; }
    a { color: #164f80; text-decoration: none; }
    code { font: 8.7pt/1.4 'DejaVu Sans Mono', monospace; overflow-wrap: anywhere; }
    pre { font-size: 9pt; line-height: 1.4; background: #f3f5f7; border-left: 3px solid #547c9c;
      padding: 10px; white-space: pre-wrap; break-inside: avoid; }
    pre code { overflow-wrap: normal; }
    table { border-collapse: collapse; width: 100%; margin: 12px 0; font-size: 9.5pt; }
    th { color: #173b5e; background: #eff3f6; text-align: left; }
    th,td { padding: 5px 7px; border-bottom: 1px solid #d5dee6; vertical-align: top; }
    tr { break-inside: avoid; } thead { display: table-header-group; }
    img { display: block; width: 100%; height: auto; break-inside: avoid; }
    p:has(img) { break-after: avoid; }
    p:has(+ .equation), .equation:has(+ .equation) { break-after: avoid; }
    p:has(> strong:first-child) { break-after: avoid; }
    .equation { display: flex; align-items: center; justify-content: space-between;
      gap: 10px; margin: 13px 0; break-inside: avoid; }
    .equation > div { flex: 1; min-width: 0; text-align: center; }
    .equation svg { max-width: 100%; height: auto; }
    .equation > span { flex: 0 0 30px; text-align: right; font-size: 10pt; }
    .inline-math { display: inline-block; vertical-align: -0.16em; white-space: nowrap; }
    .inline-math svg { display: block; }
    @media print { body { max-width: none; margin: 0; padding: 0; } }
    """
    page = (
        '<!doctype html><html lang="en"><head><meta charset="utf-8">'
        f"<title>SMORES-EP Reconfiguration — Technical Draft</title><style>{css}</style>"
        f"</head><body>{body}</body></html>"
    )
    (HERE / f"{STEM}.html").write_text(page)
    assert equation_numbers == list(range(1, 35)), equation_numbers
    assert "MATHPLACEHOLDER" not in page
    print(f"Rendered {counter} math expressions, 34 numbered equations, and 2 vector figures.")


if __name__ == "__main__":
    figures()
    render()
