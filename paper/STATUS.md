# Manuscript Status

First rigorous IEEE draft, created 2026-08-11 while the Phase-8 planner
campaign was still running. **This is not a complete paper.** Its purpose is
to fix the argument, the terminology, and the evidence trail so that the
remaining experiments fill declared holes instead of reopening the story.

Rules that govern every edit to `paper/`:

1. every number traces to a committed experiment manifest — never to a
   conversation summary;
2. a result that does not exist is a `\PENDING{}` marker, never a plausible
   value;
3. Phase-8 and pool material stays out until the corresponding campaign is
   accepted and its decision entry recorded in `docs/SCIENTIFIC_DECISIONS.md`.

Build: `latexmk -pdf main.tex` (MiKTeX, IEEEtran present on this machine).

## Section status

| Section | File | Status | Blocking |
|---|---|---|---|
| Abstract | `sections/00_abstract.tex` | **WAIT_SIM_REAL** | skeleton only; needs S0–S3 + pool numbers |
| Introduction | `sections/01_introduction.tex` | **DRAFT** | full prose; contribution wording provisional |
| Related Work | `sections/02_related_work.tex` | **READY** | closed-list verification done 2026-08-11 |
| System Architecture | `sections/03_system.tex` | **READY** | — |
| Vehicle + Navigation | `sections/04_vehicle_model.tex` | **READY** | — |
| Temporal Estimation | `sections/05_perception_estimation.tex` | **READY** | — |
| Avoidance Policies | `sections/06_planners.tex` | **READY (methods)** | frozen D-013 parameters added 2026-08-12; results still WAIT_PHASE8 |
| Calibration Ladder | `sections/07_calibration.tex` | **DRAFT** | methodology fixed; measurements pending |
| Experimental Design | `sections/08_experimental_design.tex` | **DRAFT** | blocks D/E/F are plans |
| Results (avail.) | `sections/09_results.tex` | **READY** | Baseline-0 + Phase-7B only |
| Predictivity Analysis | `sections/10_predictivity_analysis.tex` | **DRAFT** | analysis plan pre-registered; no data |
| Discussion | `sections/11_discussion.tex` | **partial DRAFT** | planner + calibration subsections pending |
| Limitations | `sections/12_limitations.tex` | **READY** | revisit after pool measurements |
| Conclusion | `sections/13_conclusion.tex` | **WAIT_SIM_REAL** | skeleton only |

Additional gates: **WAIT_PHASE8** (planner results, DWA parameters),
**WAIT_YOLO** (visual-detector results — blockers B1/B2),
**WAIT_POOL** (pool geometry, obstacle dimensions, ground-truth error report),
**WAIT_CALIBRATION** (S1–S3 measured values, T3 coefficients).

## Figures

| Figure | File | Status |
|---|---|---|
| 1 Matched sim/real architecture | `figures/fig_architecture.tex` (TikZ) | READY |
| 2 BlueROV2 command path | `figures/fig_control.tex` (TikZ) | READY |
| 3 T1 vs T2 at maneuver onset | `figures/fig_onset.tex` + `fig_onset_D5.png` | READY (regenerated from `experiments/simulation/temporal_estimators_phase7b/onset_analysis/`) |
| — Estimator ladder concept | not drawn | optional; Table II may suffice |
| — Phase-7B outlier robustness | `fig_closedloop_comparison.png` copied, not yet included | optional; Table V carries the result |
| — DWA result figure | not created | **WAIT_PHASE8** |
| — Sim-real gap vs level, ranking | not created | **WAIT_SIM_REAL** |

## Tables

| Table | File | Status |
|---|---|---|
| I Related-work positioning | `tables/tab_related_work.tex` | READY |
| II Temporal estimators T0–T3 | `tables/tab_estimators.tex` | READY |
| III Sim/real sensing equivalence | `tables/tab_sensor_equivalence.tex` | READY |
| IV Calibration levels S0–S3 | `tables/tab_calibration_levels.tex` | DRAFT (structure final, values pending) |
| V Phase-7B success matrix | `tables/tab_phase7b.tex` | READY |
| VI Baseline-0 integration metrics | `tables/tab_baseline0.tex` | READY |
| VII S0 step responses | `tables/tab_step_response.tex` | READY |
| VIII Maneuver-onset prediction | `tables/tab_onset.tex` | READY |
| IX DWA frozen parameters | `tables/tab_dwa_parameters.tex` | READY (D-013, 2026-08-12) |

## Closed-list literature verification (2026-08-11)

Targeted DOI-level verification only. No broad search was run; no multi-agent
literature fan-out was launched.

| Ref | Paper | Outcome |
|---|---|---|
| A1 | Potokar 2022, HoloOcean, ICRA | bib correct; converted to `@inproceedings`, pages 3040–3046 |
| A2 | Potokar 2024, HoloOcean, JOE 49(4) | bib correct; volume/number/pages split out. **Now the primary HoloOcean citation** |
| A3 | Meyers & Mangelson 2025, HIL, OCEANS | already deep-read in the database; converted to `@inproceedings` |
| B1 | Kadian 2020, Sim2Real Predictivity, RA-L 5(4) 6670–6677 | verified; fields split out. SRCC attributed to them throughout |
| B2 | Truong 2022, Rethinking Sim2Real, CoRL | verified; converted to `@inproceedings`, PMLR 205, 859–870 |
| C1 | Zhang, Zhang & Chen 2025, Robotica 43(8) 2952–2974 | **NOT in the database — ADDED** (`zhang2025simtoreal`). Verified at publisher |
| C2 | Mari 2026, Sensors 26(7) 2179 | already full-text verified in the database |
| D1 | Bergantin 2024, Ocean Eng. 118674 | present and correct |
| E1 | Alinei-Poian\u{a} 2024, IFAC-PapersOnLine 58(20) | volume/issue/pages/DOI added |
| F1 | Li 2026, Applied Ocean Research 171:105095 | metadata verified via Crossref; abstract unavailable — substantive claims stay aggregator-level and the manuscript does not depend on them |
| F2 | Li 2024, JMSE 12(5):695 | metadata verified via Crossref; **author spelling corrected** (Chao Yin → Changyi Yin) |
| G1 | DUViN 2025 | still a preprint as of this check; cited as such |
| H1 | Han, Vahs & Tumova, CDC 2025 | **NOT in the database — ADDED** (`han2025riskaware`). Verified: its two underwater applications are **simulated**, confirming the user's correction |
| I1 | Fox, Burgard & Thrun 1997 | **was missing entirely — ADDED** (`fox1997thedynam`), IEEE RAM 4(1) 23–33 |
| I2 | Eriksen 2016, CCA | converted to `@inproceedings`, pages 499–506 |
| J1 | Naik 2026, covariance scheduling | **not pursued** (optional, off-topic for visual tracking) |

New papers added: 3 (`zhang2025simtoreal`, `han2025riskaware`,
`fox1997thedynam`) — within the ≤3 budget. Two were genuine discoveries; the
third was a canonical citation missing from the bibliography.

### Effect on the novelty story

- **No contribution had to be withdrawn.** The headline claim survives.
- **Zhang 2025 (C1) tightens the framing.** High-fidelity underwater
  simulation followed by physical pool obstacle avoidance is now explicitly
  published, so "we transfer to a pool" cannot be part of the claim. Their
  study performs no ranking analysis and no leveled calibration, so the
  predictivity framing is intact — but the Introduction and Related Work now
  cite them as an established transfer result.
- **Han disambiguation matters and is now correct.** Two distinct papers by
  overlapping author sets were at risk of being conflated: the CDC 2025 paper
  (simulated underwater applications) and the 2026 random-finite-set paper
  (real BlueROV2 tank trials). The manuscript cites both, for different
  purposes, and never attributes hardware validation to the CDC paper.
- **Unchanged from the earlier audit:** the paired pool/twin protocol and the
  integrated system remain non-contributions.

### Open tension to resolve with the supervisor

Provisional contribution 1 (the matched ROS 2 architecture) is close to what
`docs/LITERATURE_AND_NOVELTY.md` classifies as `already_done` (paired
protocol) and `too_weak` (integrated system), citing DUViN 2025,
Alinei-Poian\u{a} 2024, Meyers 2025 and Marinarium 2026. The draft therefore
states it as *enabling methodology* rather than as a novelty claim, and says
so explicitly in the Introduction. If reviewers must be given four
contributions, the safer fourth is the open release of code, configurations
and manifests, which none of the closest competitors provide.

## Title candidates (not frozen)

1. *How Predictive Is Underwater Simulation? A Calibration-Ladder Study of
   Visual Obstacle Avoidance from HoloOcean to a BlueROV2* — current working
   title in `main.tex`.
2. *Calibrating HoloOcean for Predictive Sim-to-Real Evaluation of Visual
   Obstacle Avoidance on a BlueROV2*
3. *Does Calibration Make Underwater Simulation Predictive? A Measured
   Sim-to-Real Study on a BlueROV2*
4. *From Transfer to Prediction: Measuring Simulator Predictivity for
   Underwater Obstacle Avoidance*
5. *What Should You Calibrate First? Isolating Camera, Timing and Vehicle
   Mismatch in Underwater Sim-to-Real Evaluation*
6. *A Measured Calibration Ladder for Underwater Simulation-to-Reality
   Evaluation of Obstacle Avoidance*
7. *Simulator Predictivity for Camera-Driven Underwater Obstacle Avoidance:
   A BlueROV2 Study* (most conservative)

Avoid "closing the sim-to-real gap" until a reduction is measured. Avoid
"first" everywhere.

## Venue

Not chosen. The manuscript is deliberately un-compressed and venue-neutral;
ICRA / IROS / OCEANS / RA-L all remain open per
`docs/PUBLICATION_POSITIONING.md`. Do not cut for page limit yet.

## Next manuscript action

When Phase-8 records D-013: add `tables/tab_dwa_parameters.tex` from the
frozen configuration, replace `\P8` in `sections/06_planners.tex`, and only
then write the planner-results subsection from the accepted campaign
manifest.

## Self-review (five dimensions, answered)

Run against `references/paper-review.md` of the writing skill. Unresolved
items are carried as work, not hidden.

**1. Contribution.** *Is any claim stronger than the evidence?* No claim in
the draft asserts a measured predictivity improvement; every such statement
is a `\PENDING` marker. The weakest contribution is provisional #1 (matched
architecture) — see "Open tension" above; it is stated as enabling
methodology and the Introduction says so explicitly. *Would a reviewer find
the paper's one-sentence claim elsewhere?* Not after the closed-list check:
Zhang 2025 and Mari 2026 supply transfer, Kadian 2020 supplies predictivity
in a different domain, nobody combines a measured calibration ladder with
underwater ranking preservation.

**2. Writing clarity.** Every section was written one-message-per-paragraph
with the message in the first sentence. Terminology is macro-locked in
`main.tex` (`\SZERO`–`\STHREE`, `\PC`, `\PD`) so it cannot drift. Remaining
weakness: Sec. VI-C has five `\subsubsection` blocks in a row, which reads as
a list; if space becomes tight, the two least load-bearing (mission-paced
objective, bounded obstacle memory) compress into one paragraph.

**3. Experimental strength.** The available evidence is oracle-perception
simulation only, and every table and the section opening say so. The honest
weakness the draft does *not* conceal: the temporal ablation separates the
methods in exactly one scenario (E4) out of five, and nominal conditions are
flat. That is reported as the result rather than smoothed over.

**4. Evaluation completeness.** The predictivity analysis is pre-registered
including its own failure modes: SRCC over two planners is called degenerate
in the text, pairwise direction agreement is declared as the primary ranking
statistic, and the primary ranking metric (minimum clearance) is fixed before
data. Threats to validity (repetition asymmetry, ceiling effects, instrument
error) are stated. Missing: a power analysis for the physical campaign — add
once the pool run budget is known.

**5. Method design soundness.** The DWA description follows the committed
implementation, and each departure from Fox is justified by the failure it
fixes. Only two of the ten pre-freeze formulation issues reached the
manuscript (response-aware rollout, directional stoppability) because only
those justify final mathematical choices; the rest stay in
`docs/DWA_IMPLEMENTATION_AND_PROTOCOL.md`, per the instruction not to turn
debugging history into claims.

### Claim–evidence map (major claims in Introduction and Results)

| Claim | Evidence | Status |
|---|---|---|
| Sim/physics clock consistency restored dead-reckoning accuracy (4.08 m → 0.117 m) | D-010, `experiments/simulation/baseline0_diag*` | supported |
| Integration baseline: 9/9 accepted, no collision, clearance 0.672–0.715 m | `docs/SCIENTIFIC_BASELINE_0.md`, `experiments/simulation/scientific_baseline_0/` | supported |
| S0 step responses ≤4.8 % SS error, no overshoot | `experiments/simulation/step_response_S0{,_run2,_run3}/manifest.json` | supported |
| T2 25/25 vs T0/T1 22/25; all separation in E4 | `docs/PHASE7B_RESULTS.md`, `.../temporal_estimators_phase7b/closedloop/aggregate.json` | supported |
| Hold out-predicts CV at maneuver onset (0.038 vs 0.084; 0.036 vs 0.071) | `.../onset_analysis/onset_analysis.json` | supported |
| Image-space velocity discontinuity −0.032 units/s at commitment | same file | supported |
| χ² gating wins on mature-track outliers (D9: 0.0074 vs 0.0102 vs 0.0199) | `.../replay_eval/metrics.json` | supported |
| RealSense sustains 29.81 fps at 1080p, host-clock stamps | `visualizations/realsense_inventory/`, D-007 | supported |
| Physical vehicle has no DVL / no velocity aiding | `docs/REAL_BLUEROV2_HARDWARE_INVENTORY.md` (read-only inventory) | supported |
| Kadian raised SRCC 0.18 → 0.844 by tuning | `docs/LITERATURE_AND_NOVELTY.md` deep-read #7 | supported |
| Zhang 2025 reports 94 % simulated success, pool transfer, no ranking analysis | verified at publisher 2026-08-11 | supported |
| Mari 2026 sea-trial obstacles are virtual | `docs/LITERATURE_AND_NOVELTY.md` (full-text verified) | supported |
| Han CDC 2025 underwater examples are simulated | verified at arXiv:2504.04097, 2026-08-11 | supported |
| Calibration improves predictivity | — | **needs evidence (WAIT_SIM_REAL)** |
| Planner C vs D comparison outcome | — | **needs evidence (WAIT_PHASE8)** |
| Ranking is preserved sim→real | — | **needs evidence (WAIT_SIM_REAL)** |
| T3 adaptive covariance helps | — | **needs evidence (WAIT_CALIBRATION); T3 ≡ T2 today** |

Note on one traceability detail: `docs/PHASE7B_RESULTS.md` quotes the onset
range as 0.028–0.036 (T1) vs 0.063–0.084 (T2). Those figures span an earlier
replay revision. The manuscript uses the values in the currently committed
artifacts (T1 0.038/0.036, T2 0.084/0.071) so every number matches a file in
the repository today.
