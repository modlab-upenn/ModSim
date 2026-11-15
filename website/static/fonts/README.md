# ModSim local font assets

These fonts match the project website's typography and are served locally so the
documentation does not need Google Fonts requests while reading or editing.

| Family | Local file | Exposed styles |
| --- | --- | --- |
| Archivo | `archivo-latin-variable.woff2` | Normal, weight 500–800, width 112–125% |
| IBM Plex Sans | `ibm-plex-sans-latin-variable.woff2` | Normal, weight 400–700 |
| IBM Plex Sans | `ibm-plex-sans-latin-italic.woff2` | Italic, weight 400 |
| IBM Plex Mono | `ibm-plex-mono-latin-400.woff2` | Normal, weight 400 |
| IBM Plex Mono | `ibm-plex-mono-latin-500.woff2` | Normal, weight 500 |
| IBM Plex Mono | `ibm-plex-mono-latin-600.woff2` | Normal, weight 600 |

`../theme/fonts.css` defines the font faces with `font-display: swap`. Each file is
the Latin subset supplied by the official Google Fonts CSS API. Other characters
use the fallback families specified in the shared theme tokens.

The font binaries are unmodified. Archivo includes variable weight and width
axes. The normal Plex Sans file includes a variable weight axis up to 700, which
allows real bold text without downloading another file. Plex Mono uses separate
static files for each weight.

## Sources and licenses

The files were retrieved from the official [Google Fonts CSS API request](https://fonts.googleapis.com/css2?family=Archivo:wdth,wght@112..125,500..800&family=IBM+Plex+Mono:wght@400;500;600&family=IBM+Plex+Sans:ital,wght@0,400..600;1,400&display=swap).
The served versions are Archivo v25, IBM Plex Sans v23, and IBM Plex Mono v23.

All three families use the SIL Open Font License 1.1. Keep these license files
with redistributed font assets:

- `OFL-Archivo.txt` from the [Archivo license](https://raw.githubusercontent.com/google/fonts/main/ofl/archivo/OFL.txt).
- `OFL-IBM-Plex-Sans.txt` from the [IBM Plex Sans license](https://raw.githubusercontent.com/google/fonts/main/ofl/ibmplexsans/OFL.txt).
- `OFL-IBM-Plex-Mono.txt` from the [IBM Plex Mono license](https://raw.githubusercontent.com/google/fonts/main/ofl/ibmplexmono/OFL.txt).
