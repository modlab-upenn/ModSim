# Packaged ModSim Studio branding

This curated production subset comes from the user-provided
`Modsim_Branding/ModSim_{Light,Dark,Midnight}_Complete_Branding_Final_v5`
collection, brand asset release **5.0 / Final Honeycomb System**, prepared
2026-11-17. This asset version is independent of the Python package version.
The source pack READMEs are retained unchanged inside each theme
folder. Pack-relative filenames are preserved, and all copied assets are
byte-for-byte identical to the supplied production files.

| Studio theme | Pack directory | Source pack | Production canvas |
| --- | --- | --- | --- |
| ModSim Light (`light`) | `light/` | `ModSim_Light_Pack` | `#E8F3FA` |
| ModSim Dark (`dark`, legacy alias `graphite`) | `dark/` | `ModSim_Dark_Pack` | `#132331` |
| ModSim Midnight (`midnight`) | `midnight/` | `ModSim_Midnight_Pack` | `#071427` |

`theme/colors.json` is the authoritative palette in each pack. The older
concept-board images are not palette references. Functional controls use the
separate `control_border` token; decorative separators use `border`. Light is
Glacier & Steel, Dark is Blue Charcoal, and Midnight is Deep Navy & Cyan.
Read the supplied [final guidelines](light/docs/BRAND_GUIDELINES.md) for identity,
color roles, artwork preservation, and integration details. Logos contain outlined paths
and need no additional font installation. Their proportions and wordmark
shapes must be preserved; use the theme-matched production assets rather than
inverting another theme's colors.

## Included assets

Each theme contains the horizontal logo and descriptor variant, transparent
symbol and app-tile SVGs, their 256/512 px PNG exports, 1024 px app PNGs,
Windows/macOS app icons,
24 toolbar glyphs in their original `currentColor` form, the accent check
glyph for checkboxes, the module-pattern
SVG, theme-only SVG/1920×1080 PNG backgrounds, and the 1920×1080 robot cover with
its compatibility alias. The source
changelog, final brand guidelines, artwork provenance, and calculated contrast
checks accompany each curated theme.

`backgrounds/robot-cover-1920x1080.png` is used by the shared startup, welcome,
and loading panels. It includes the approved robots, logo, and illustrative
simulation window. The cover fills its panel; live status, progress, and welcome
actions blend over the lower artwork through a soft theme-colored fade. There
is no additional logo, separate card, or reserved band that shrinks the image.
`backgrounds/robot-background-1920x1080.png` is the identical compatibility
alias. The separate `backgrounds/theme-background.svg` and
`backgrounds/theme-background-1920x1080.png` supply an empty honeycomb
environment, but are not selected for these panels. These assets replace the
retired wave artwork and have no baked progress indicator.
The robot remains illustrative concept art, not a verified
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
corresponding source manifest before inclusion. The inventory covers 47 source
files per theme, 141 in total; this README and the curated manifest are the only
new metadata files.

Runtime code uses package-relative paths through `_branding.py`. The original
Documents folder, source catalogs, and source manifests are not required by an
installed application. Curated assets are ordinary package data and do not add
Python or native dependencies.
