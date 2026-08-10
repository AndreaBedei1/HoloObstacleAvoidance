# Paper Outline (working — do not treat as final)

Working title (NOT frozen):
"Does Calibration Make Underwater Simulation Predictive? A Measured
Sim-to-Real Study of Uncertainty-Aware Visual Obstacle Avoidance on a
BlueROV2"

Alternative (method-first):
"Calibrated Perception Uncertainty for Camera-Only Obstacle Avoidance on a
BlueROV2: A Leveled HoloOcean-to-Pool Transfer Study"

## Sections

1. Introduction — the predictiveness question; one-sentence claim from
   `contributions.md`.
2. Related work — organized around the five closest papers (Mari 2026,
   Han 2026, DUViN 2025, Bergantin 2024, Kadian 2020) + adaptive-R MOT
   lineage + underwater classical planners.
3. System overview — same-code sim/real stack (infrastructure, not a claim).
4. Calibrated temporal obstacle estimation — T0/T1/T2/T3, residual-statistics
   calibration, NIS + coverage validation.
5. Avoidance policies — committed circumnavigation (cite Zhang 2022),
   uncertainty modulation, DWA baseline (Eriksen 2016).
6. Simulation calibration ladder S0–S3 — measured camera, latency, vehicle
   response; what is calibrated at each level.
7. Experimental protocol — pool, obstacles (anchor, torpedo), approaches,
   pre-registered validity criteria, overhead ground truth + error budget.
8. Simulation results — ablations A–F, ≥20 runs/condition, CIs + effect sizes.
9. Real pool results — selected 3–4 stacks × 2 obstacles × 3 approaches × 5 reps.
10. Sim-to-real analysis — per-level gap metrics, SRCC ranking preservation,
    estimator-consistency transfer; nulls reported as findings (Truong 2022).
11. Limitations — static obstacles, clear/shallow pool, known classes, no
    currents, no DVL navigation drift characterization.
12. Conclusion.

## Rules

- No claims not backed by an experiment listed in section 8-10.
- Every figure regenerable from committed scripts + experiment manifests.
- Negative/flat calibration results are findings, not failures.
