# ModSim Light — complete final branding pack

**Brand asset release 5.0 / Honeycomb System.**

**Updated Light / Glacier & Steel.** Icy-blue and white surfaces; navy #0B1E3A; steel-blue #425E7C; a deeper #426E8B-family stage behind the white modules. Actual software assets and all cover aliases now use this direction.

Open **index.html** to browse all individual assets locally. Read **docs/ModSim_Brand_Guidelines_Final.pdf** or **docs/BRAND_GUIDELINES.md**. `PREVIEW.png` is a view of the actual exports, not the asset library itself.

## Included

| Folder | Contents |
| --- | --- |
| `logos` | Outlined horizontal, stacked, wordmark-only, monochrome and optional descriptor logos; SVG and transparent PNG. |
| `icons` | Symbols, app tiles, reticles and monochrome variants; 16–1024 px; ICO, ICNS, favicon, maskable and Apple touch formats. |
| `ui` | All 24 toolbar actions; five SVG states and four PNG states at 24/48/96 px. |
| `patterns` | Hex outlines, matte honeycomb, seamless tile, stage and straight connection motifs. |
| `backgrounds` | Native and 1920×1080 / 960×540 covers plus empty scalable backgrounds. |
| `artwork` | Original transparent SMORES configurations and isolated simulation-window illustration. |
| `theme` | JSON colors, CSS, all-theme CSS, Qt stylesheet, palette and calculated contrast reports. |
| `previews` | Actual asset board, toolbar state chart and static loading-screen example. |
| `docs`, `reference` | PDF/Markdown guidelines, source provenance, preservation mask and validation. |
| `examples` | Optional PySide6 styling demo. |

## Main entry points

```text
logos/svg/modsim-horizontal.svg
icons/svg/modsim-app.svg
ui/svg/default/play.svg
ui/png/24/play.png
backgrounds/robot-cover-960x540.png
theme/colors.json
theme/theme.css
theme/theme.qss
resources.qrc
```

All three packs use the same relative paths. Replace the whole older theme folder and rebuild any compiled Qt resources. Open `examples/qt_theme_demo.py` for a standalone styling demonstration; it is syntax-checked, not run inside ModSim.

## Artwork

The two SMORES robot configurations are reused from the approved honeycomb artwork, not generatively redrawn. Original fully opaque foreground pixels are preserved in the native adaptation; resizing and alpha edges are not pixel-identical guarantees. The simulation window is illustrative, not an actual ModSim screenshot. The wordmark is the supplied vector-outline form; its original font remains unidentified. No font files are distributed.

## Checks

See `docs/validation-report.json`, `theme/contrast-checks.json`, and `asset-manifest.json`. The manifest inventories file sizes and SHA-256 values. Native artwork preservation, dimensions, parsing and internal links are checked; no runtime or accessibility certification is implied.
