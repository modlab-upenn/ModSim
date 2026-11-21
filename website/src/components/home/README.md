# Homepage demonstrations

`DockingDemo`, `TopologyPlayground`, and `ModelViewDemo` are React/SVG components
for the shared Docusaurus homepage. They use the site's `--modsim-*` palette,
local fonts, and scoped CSS. They require no Python process, physics server,
external graphics library, or external data request.

The interactive docking and topology widgets are illustrative browser models.
Their event labels explain the pack/runtime contracts; they do not report a
live ModSim backend's decisions. The docking diagram is planar and omits roll
acceptance and several policy guards. The playground uses cardinal face
alignment, release-time snapping, and a simple collision check.

`replayData.ts` preserves the `TOPO`, `MB`, and `SM` datasets from
`origin/gh-pages:index.html`, commit `c519375`, migrated on November 19, 2026:

- The four-action SMORES-EP Driver-to-Snake topology sequence.
- Twenty-one pivots from the six-cube planar M-Blocks planner output.
- 658 sampled frames, plan revisions, routes, and connection events from the
  historical SMORES-EP online-assembly recording.

These are historical recorded examples, not freshly executed benchmarks or
proof of present controller/hardware performance. Updating a replay requires
an intentional new data capture, source/version provenance, and review of the
description. Layout and screen interpolation are presentation choices.

Motion begins through user controls. Reduced-motion users can inspect the
same replay through its scrubber; the docking animation completes immediately
when that preference is enabled. Components cancel animation frames and
remove browser listeners when Docusaurus navigates away. No browser API runs
at module import or server render time.
