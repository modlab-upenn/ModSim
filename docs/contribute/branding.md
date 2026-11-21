---
title: Brand guidelines
sidebar_label: Branding
---

ModSim uses **brand asset release 5.0 / Final Honeycomb System**, prepared on
17 November 2026. That number identifies the artwork collection; it does not
change the Python package version.

The supplied theme palettes and outlined artwork define the identity. Use the
production files rather than sampling colors from a rendered cover or
recreating the wordmark.

## Choose the matching theme

| Surface | Theme | Direction |
| --- | --- | --- |
| Documentation light mode | Light | Glacier & Steel: white and icy-blue surfaces, navy text, steel-blue secondary elements |
| Documentation dark mode | Midnight | Deep Navy & Cyan: navy surfaces, pale text, teal actions |
| Additional desktop theme | Dark | Blue Charcoal: muted blue surfaces, pale text, teal actions |

The website and documentation share Docusaurus's light/dark switch. Studio and the
Runtime Inspector offer all three themes. Match fixed-color SVGs, PNGs, and
icons to the selected theme when switching colors.

## Preserve the logo

Use `logos/svg/modsim-horizontal.svg` for the primary lockup. Its wordmark is
outlined vector geometry and needs no font installation. Preserve the module
symbol, segmented reticle, proportions, and relative placement. Keep at least
one solid module's width of clear space where possible; suggested minimum
sizes are 160 px for the horizontal logo and 24 px for the symbol.

Use the supplied theme or monochrome variant. Keep the logo's original aspect
ratio and colors; do not retype its lettering, invert another theme's logo,
stretch it, or substitute a hexagon for the module symbol. The standard lockup
and covers have no subtitle.

The documentation keeps its existing self-hosted Archivo headings, IBM Plex
Sans body text, and IBM Plex Mono code. These fonts style interface copy; they
do not reconstruct the outlined logo. Retain their accompanying font licenses.

## Apply colors by role

Each pack's `theme/colors.json` is authoritative. Use functional roles rather
than choosing a decorative color because it looks similar.

| JSON role | Light | Dark | Midnight |
| --- | --- | --- | --- |
| `background` | `#E8F3FA` | `#132331` | `#071427` |
| `surface` | `#FFFFFF` | `#1C3042` | `#0D2339` |
| `raised` | `#D4E7F2` | `#263D51` | `#143550` |
| `text` | `#0B1E3A` | `#F1F6FB` | `#F2F6FB` |
| `muted` | `#425E7C` | `#B8CBDC` | `#B4CDDF` |
| `accent` | `#087F79` | `#35CDBF` | `#00BFAF` |
| `accent_text` | `#FFFFFF` | `#0B2228` | `#071427` |
| `secondary_accent` | `#425E7C` | `#93AEC5` | `#5592C1` |
| `border` | `#9AB9CE` | `#3D5B71` | `#245373` |
| `control_border` | `#526D82` | `#7E9DB6` | `#6896B7` |
| `focus` | `#1169A2` | `#78D8F2` | `#50DEF1` |
| `selection` | `#C5DFEF` | `#304E64` | `#16496B` |
| `disabled` | `#829AAC` | `#627C91` | `#466A88` |
| `blue` | `#188FE5` | `#53AEF3` | `#2AA8FF` |
| `teal` | `#00BFAF` | `#35CDBF` | `#00BFAF` |
| `cyan` | `#00BFD8` | `#67DAEC` | `#00D9EF` |

Use `control_border` around interactive controls and `border` for decorative
separators. Use `focus` for visible keyboard focus and `disabled` for inactive
controls. Primary actions pair `accent` with `accent_text`; the brighter
decorative `teal` is not the Light theme's button fill.

The web adapter uses Light's `link` token, `#126A9C`, and Midnight's `blue`,
`#2AA8FF`, for links. Keep ordinary reading surfaces white in Light and deep
navy in Midnight. Check links, focus rings, selected items, code, and table
boundaries on their actual backgrounds. The supplied contrast reports cover
individual color pairs, not the accessibility of a complete application.

## Use artwork with readable content

Honeycomb is a supporting motif. Keep it in branding artwork and cover areas;
documentation prose, API tables, code, and diagrams need calm reading surfaces.
Diagrams use the shared documentation tokens through the Mermaid adapter.

Use the approved SMORES cover and matching theme assets. The desktop loading
and welcome screens use `backgrounds/robot-cover-1920x1080.png` at full size,
with one logo already in the image. Live controls blend over its lower area
through a soft theme-colored fade. Do not add a second logo or a static progress
bar from a preview image. Preserve real progress and interaction widgets.

The robots and floating simulation window are illustrative brand artwork,
rather than verified CAD geometry or screenshots of running ModSim. Keep the
two approved configurations; do not redraw their bodies, invent connections,
or substitute another robot. Larger raster exports contain resampled source
detail. Keep the source artwork and its provenance intact.

### Approved cover examples

Light / Glacier & Steel:

![Approved Light cover with the outlined ModSim logo and two SMORES configurations.](/branding/light/backgrounds/robot-cover-1920x1080.png)

Midnight / Deep Navy & Cyan:

![Approved Midnight cover with the outlined ModSim logo and two SMORES configurations.](/branding/midnight/backgrounds/robot-cover-1920x1080.png)

These are production cover illustrations, not screenshots of the Studio.

## Update the repository consistently

| Location | Responsibility |
| --- | --- |
| `src/modsim_robotics_studio/assets/branding/` | Packaged desktop assets, production JSON palettes, source guidelines, provenance, and the curated hash manifest |
| `website/static/branding/{light,midnight}/` | Matching production assets served locally by the documentation site |
| `website/static/theme/tokens.css` | Framework-independent web color roles and light/dark values |
| `website/src/css/custom.css` | Docusaurus styling that consumes those roles |
| `website/src/theme/Mermaid/index.tsx` | Diagram colors derived from the same resolved tokens |
| `website/THEME.md` | Web theme implementation contract and review workflow |

Replace a theme's selected asset set together, keeping pack-relative filenames
and its manifest/provenance current. Switch fixed-color artwork alongside its
palette. Keep assets package-relative or site-relative; installed applications
and the published documentation must not depend on a developer's Documents
folder, a font CDN, or external image hosting.

Preview a changed documentation page in both modes and at desktop and narrow
widths. Review the navbar logo, focus states, links, code, API tables, and a
Mermaid diagram. Use [Local development](local-development.md) for editing and
preview commands, then run:

```bash
npm --prefix website run docs:check
npm --prefix website run typecheck
npm --prefix website run docs:build
```

The project homepage and its browser demonstrations use the same tokens, fonts,
and production wordmarks as the manual. Keep homepage/widget rules in CSS modules
and link to the authored installation instructions. Review the homepage, its
interactive controls, recordings, shared navigation, and a manual page together.
Use the [Pages guide](github-pages.md) to test an isolated deployment address;
assets must work under both `/ModSim/` and a preview repository's base path.

For desktop branding changes, preserve the system font size, shared window
sizing, file-dialog behavior, runtime state, and startup timing. Exercise the
affected application and its existing GUI regressions, including high-DPI
rendering and theme switching.
Check both Studio workspaces, the embedded Inspector, and the standalone
Inspector. Workspace switches must preserve unsaved edits and runtime results;
the embedded view uses Studio's shared header and sizing rather than adding
another application header. The
[Studio workflow checks](development-setup.md#exercise-the-studio-workflow)
cover the launch and shutdown paths to preserve during visual changes.
