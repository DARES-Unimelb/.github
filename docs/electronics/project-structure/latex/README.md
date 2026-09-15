# DARES Electronics — project structure proposal (LaTeX)

## Overleaf
1. Zip this folder (or upload the zip as-is): **New Project → Upload Project**.
2. Compiler: pdfLaTeX (default). Compile twice for the page count in the footer.
3. Fonts: uses Source Sans Pro on Overleaf; falls back to Latin Modern / Helvetica elsewhere.

## Contents
- `main.tex` — the document. Everything is in one file; project boxes are the `project` environment.
- `figures/form_interests.png` — technical and sub-system interest bars from the form.
- `figures/form_ownership_experience.png` — ownership and experience panels.
- `figures/project_interface_map.png` — **not included**: screenshot the interface map from the chat,
  save it under this name, and uncomment the block just above `\section{Cross-cutting}`.

## Editing
- Staffing lives in the `tabularx` table under "Staffing from the form" and in each `\begin{project}` title.
- Palette is defined in the preamble (`purple`, `coral`, `teal`, `blue`, `amber`, `slate`).
- To drop a project, delete its `\begin{project} ... \end{project}` block and its table row.
- Milestones are ordered, not dated, on purpose.
