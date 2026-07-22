# Report Directory

The detailed technical report is available as both Markdown and a compiled
LaTeX PDF.  The LaTeX/PDF pair is the submission-ready version.
This report and its videos correspond to the `v1.1.0` branch, not `main`.

## Contents
- **report.tex** — Submission-ready LaTeX source (XeLaTeX + xeCJK)
- **report.pdf** — Compiled 10-page A4 technical report
- **videos/** — Git LFS video evidence for the acceptance scenarios
- **report.md** — Markdown source/reference version of the report
- **verification_guide.html** — Step-by-step guide with copy-paste commands for recording all 6 evaluation scenarios

## Related Documents
- `../README.md` — Project README (build, run, configure, troubleshoot)
- `../ai_usage.md` — AI-assisted programming disclosure
- `../drone_bringup/config/parameter_reference.yaml` — Unified parameter reference for all nodes

## Build the PDF

Use XeLaTeX so Chinese text wraps correctly:

```bash
cd report
xelatex report.tex
xelatex report.tex
```

The source uses the `Noto Serif SC` and `Noto Sans SC` fonts. Install those
fonts, or adjust the font declarations at the top of `report.tex` to fonts
available on the local system.
