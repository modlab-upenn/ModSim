# ModSim technical report

This directory contains the LaTeX source for the long-form ModSim technical
report. The report is intentionally split into section files so architecture,
experiments, and future-work material can evolve independently.

Build from this directory with a conventional TeX Live or MacTeX installation:

```bash
latexmk -pdf -interaction=nonstopmode -halt-on-error modsim_technical_report.tex
```

Or run the traditional BibTeX sequence:

```bash
pdflatex modsim_technical_report.tex
bibtex modsim_technical_report
pdflatex modsim_technical_report.tex
pdflatex modsim_technical_report.tex
```

Generated PDF and auxiliary files are not tracked. The current development
machine did not have a TeX distribution installed when this source was written,
so repository checks validate source structure but cannot render the PDF there.

The implementation snapshot described by the report is ModSim 0.1.0 at commit
`4a5ca00`. Published SMORES facts, CAD-derived quantities, provisional
simulation choices, and simulation observations are labeled separately.
