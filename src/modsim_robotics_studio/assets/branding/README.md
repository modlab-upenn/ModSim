# Packaged ModSim Studio branding

This curated production subset comes from the user-provided
`Modsim_Branding/ModSim_Theme_Packs` collection, version 1.0, prepared
2026-11-16. The source pack READMEs are retained unchanged inside each theme
folder. Pack-relative filenames are preserved, and all copied assets are
byte-for-byte identical to the supplied production files.

| Studio theme | Pack directory | Source pack | Production canvas |
| --- | --- | --- | --- |
| ModSim Light (`light`) | `light/` | `ModSim_Light_Pack` | `#F8FBFE` |
| ModSim Dark (`dark`, legacy alias `graphite`) | `dark/` | `ModSim_Dark_Pack` | `#17191F` |
| ModSim Midnight (`midnight`) | `midnight/` | `ModSim_Midnight_Pack` | `#0B1521` |

`theme/colors.json` is the authoritative palette in each pack. The older
concept-board images are not palette references. Logos contain outlined paths
and need no additional font installation. Their proportions and wordmark
shapes must be preserved; use the theme-matched production assets rather than
inverting another theme's colors.

## Included assets

Each theme contains the horizontal logo and descriptor variant, transparent
symbol and app-tile SVGs, their 256/512 px PNG exports, 1024 px app PNGs,
Windows/macOS app icons,
24 toolbar glyphs in their original `currentColor` form, the accent check
glyph for checkboxes, the module-pattern
SVG, a theme-only SVG background, and the 1920×1080 robot background.

`backgrounds/robot-background-1920x1080.png` has no baked left-hand title or
progress indicator. Applications can place their own logo, status, and live
progress over it. The robot remains illustrative concept art, not a verified
robot model or screenshot of the application. Static progress previews,
reference boards, duplicate logo PNGs, web manifests, and HTML/demo catalogs
are intentionally outside this packaged subset.

`ui/svg/currentcolor/` glyphs require the rendering client to choose a stroke
color. They remain unmodified here. The theme-specific logo/symbol/app SVGs
already contain their production colors.

## Provenance and verification

[`asset-manifest.json`](asset-manifest.json) records the source pack version,
preparation date, SHA-256 of its supplied full manifest, and the byte count and
SHA-256 of every copied file. Each copied hash was checked against the
corresponding source manifest before inclusion. The inventory covers 41 source
files per theme, 123 in total; this README and the curated manifest are the only
new metadata files.

Runtime code uses package-relative paths through `_branding.py`. The original
Documents folder, source catalogs, and source manifests are not required by an
installed application. Curated assets are ordinary package data and do not add
Python or native dependencies.
