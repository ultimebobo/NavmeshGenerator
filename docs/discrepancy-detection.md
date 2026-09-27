# Discrepancy detection (milestone 7)

`analysis.json` is a report-only artifact. It does not contain a replacement
NAVM, mutation operation, or plugin output path.

Each NAVM triangle is sampled at its centroid, near each vertex, and at three
edge-adjacent interior positions. This avoids treating a centroid hit as
coverage for a bridge edge, narrow gap, or partial floor. Samples query the
geometry spatial index and are clustered by source identity and compatible
height.

Source selection is deterministic: collision, terrain, render fallback, then
unknown. A lower-priority source may only be selected when no higher-priority
cluster covers the configured minimum share of samples. This lets collision
support on a bridge beat LAND below it while retaining terrain support where a
collision mesh is absent.

The selected evidence reports source confidence, number of samples, agreement,
distance, slope, and height delta. Aggregate confidence is source confidence
multiplied by coverage and a distance penalty. `ambiguous` means competing
same-priority surfaces have adequate but materially different support;
`out_of_coverage` means no usable scene evidence was present. Neither is a
defect and neither receives a repair candidate.

The report also records deterministic topology observations: invalid adjacency,
non-manifold edges, duplicate vertex/gap evidence, isolated components,
triangle overlaps, and exterior border edges needing cross-cell review. High
confidence findings and supported discrepancy classes may produce stable-ID
`manual_review` candidates. Candidates only preserve the evidence and the
affected polygon IDs; they do not alter source records.
