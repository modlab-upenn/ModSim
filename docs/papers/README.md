# SMORES spatial reconfiguration manuscript

- [Paper PDF](smores_3d_reconfiguration.pdf) — typeset equations and figures.
- [Editable manuscript](smores_3d_reconfiguration.md) — canonical text with LaTeX math.
- [Offline HTML](smores_3d_reconfiguration.html) — embedded vector mathematics and figures; no CDN.

The manuscript describes the current vertical-chain implementation and audits
its guarantees. Sections 3–7 contain the model, searches and controller;
Section 10 reviews the claims; Section 11 is explicitly proposed future work.
Appendix B maps equations to implementation functions. Bibliography links go
to the SMORES manuscripts and original publication records.

## Reference data

- `data/reference_run.json`: executed outcome, plans, events and extra measurements.
- `data/reference_trace.csv`: observations every 10 ms and at phase transitions.
- `data/reference_manifest.json`: environment versions, base commit, dirty-worktree
  flag and source/asset hashes. The commit alone does not reproduce this working tree.

The reference run is one deterministic nominal simulation. It contains no
perturbation sweep or hardware measurements. Saturation time means time with
at least one actuator at the 1.2 N·m limit, sampled every 1 ms with a numerical
tolerance of 1e-9 N·m. Reported maxima also use 1 ms observations. Recorded
wall times include instrumentation and are not a portable performance benchmark.

## Reproduce the measurements

With the project's MuJoCo extra installed, run from the repository root:

```bash
.venv/bin/python docs/papers/record_smores_run.py --output /tmp/modsim-paper-run
```

This uses the existing `SpatialExperiment` unchanged and writes separate
results, preserving the archived reference. No GUI is needed. Compare the
phase, path, events and physical metrics; timestamp, wall time and repository
status will naturally differ. If the implementation changes, do not silently
replace the archived data without updating the paper's results and provenance.

The archived run predates integration with the planar and M-Blocks branch.
The integrated spatial demo explicitly selects its URDF proxies, experimental
servos, capture tolerance, and contact behavior in memory; the shared Robot Pack
keeps its planar locomotion settings. The manifest records the original sources,
so use newly recorded results when evaluating subsequent implementation changes.

## Rebuild the paper

The documentation renderer requires `matplotlib`, `numpy`, and `markdown-it-py`.
They are available in the environment used for this draft, but are not added
as ModSim runtime requirements. Use a documentation environment with these
packages to rebuild elsewhere.

```bash
MPLCONFIGDIR=/tmp/modsim-paper-mpl .venv/bin/python docs/papers/render_paper.py
```

The renderer reads the archived data, regenerates two SVG figures, converts
the manuscript's math to vector SVG, and builds a self-contained HTML file.
It verifies that the numbered equations run from 1 to 34. Edit the Markdown
source rather than generated HTML or PDF.

Open the HTML in a browser and print to PDF using its CSS A4 page size, with
browser headers/footers disabled and background graphics enabled. With Chrome
installed, an equivalent repository-root command is:

```bash
google-chrome --headless --disable-gpu --no-pdf-header-footer \
  --print-to-pdf="$PWD/docs/papers/smores_3d_reconfiguration.pdf" \
  "file://$PWD/docs/papers/smores_3d_reconfiguration.html"
```

The committed draft was rendered with Chrome 152. All equations and plots
are local vectors. Paper-rendering tools do not change simulation code.
