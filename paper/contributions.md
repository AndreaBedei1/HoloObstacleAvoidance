# Contribution Tracker

Living document. Updated after the 2026-08-10 novelty audit
(`docs/LITERATURE_AND_NOVELTY.md`). Order = current claim priority.

## C1 (HEADLINE) — Leveled sim-calibration predictiveness study (S0–S3)

- **Claim:** measuring whether progressively calibrating HoloOcean with
  *measured* camera, latency, and vehicle-response models improves its ability
  to predict real closed-loop avoidance outcomes and to preserve the ranking
  of estimator/planner stacks (SRCC, per Kadian 2020), underwater, with
  identical code in both domains.
- **Novelty status:** clearly novel AS AN UNDERWATER APPLICATION AND PROTOCOL
  (no underwater analogue found; simulator reviews name this as an open gap).
  To be explicit: SRCC and the sim-to-real predictivity question are Kadian
  et al.'s contribution, not ours — our candidate contribution is the
  underwater obstacle-avoidance instantiation with a progressive
  MEASURED-calibration ladder (S0–S3) and the quantitative study of whether
  calibration improves preservation of real safety/performance rankings
  between HoloOcean and a physical BlueROV2. Claim stays provisional until
  the remaining threat papers (Li AOR 2026, Li JMSE 2024) are read in full.
- **Supporting experiments:** S0–S3 paired campaigns (sim ladder + real pool),
  SRCC + per-level predictiveness metric (VEPD-spirit, Mahajan 2024).
- **Threats:** Kadian 2020 (ground, post-hoc tuning — cite and adopt SRCC);
  Truong 2022 (fidelity can be flat/harmful — pre-register that a null result
  at any level is a reported finding); Meyers 2025 (HoloOcean HIL, single
  fidelity, descriptive).
- **Evidence needed:** measured camera/latency/vehicle-response calibrations;
  ≥20 valid sim runs per condition; real campaign; ranking statistics.
- **Status:** infrastructure in progress (dynamics mode done on stock worlds).

## C2 — Calibrated, consistency-validated adaptive visual obstacle estimator

- **Claim:** measurement-noise model calibrated OFFLINE from residual
  statistics (sim oracle; separate real calibration trials), validated for
  statistical consistency (NIS, 1σ/2σ empirical coverage), and shown to beat
  heuristic adaptive-R baselines both in filter consistency AND in closed-loop
  avoidance outcomes on a real vehicle.
- **Novelty status:** potentially novel — ONLY with the calibrated-vs-heuristic
  delta demonstrated.
- **Mandatory ablations:** NSA-Kalman-style R=(1−c)·R₀ (GIAOTracker/StrongSORT
  lineage) and a UTrack-style empirical-variance R, same planner downstream.
- **Threats:** Bergantin 2024 (optic-flow + EKF distance on a real ROV);
  UncertaintyTrack 2024 (learned covariance, no filter-consistency check, no
  robot); Li AOR 2026 (sonar confidence-adaptive tracking → avoidance, real
  lake); NCT 2023 (documents confidence-driven-R gain chatter — motivation).
- **Status:** not started (Phase 7).

## C3 (component, NOT headline) — Calibrated-uncertainty-modulated committed circumnavigation

- **Claim (reframed):** the committed go-around's geometry (engagement
  distance, lateral clearance, speed, recovery timing) modulated by
  *calibrated* perception uncertainty; ablation calibrated-vs-heuristic
  covariance shows closed-loop safety/efficiency differences.
- **Novelty status:** too weak as standalone planner claim — do NOT claim
  chance-constraint novelty (de Groot IJRR 2025); cite Zhang 2022 for the
  committed-circumnavigation maneuver shape; Han 2026 for risk-bounded belief
  avoidance on BlueROV2.
- **Status:** baseline planner preserved (`committed_baseline`); uncertainty
  modulation pending estimator (Phase 9).

## Dropped as contributions (kept as methodology/system description)

- Paired pool/twin protocol with external overhead ground truth — already done
  in parts (DUViN 2025, Marinarium 2026, Alinei-Poiana 2024, Meyers 2025,
  Winkel 2023). Salvageable nugget: refraction-corrected overhead RGB ground
  truth with a quantified error budget, as a methodology subsection.
- The integrated system — no novelty weight (Yang 2022, UIVNAV 2024, DUViN
  2025). Compensate with an open code/data release (closest competitors have
  none).

## The one-sentence claim to build the paper around

> We measure whether progressively calibrating an underwater simulator with
> measured camera, latency, and vehicle-response models improves its ability
> to predict real closed-loop avoidance outcomes and to preserve the ranking
> of estimator/planner stacks, where those stacks include a
> residual-calibrated, NIS/coverage-validated perception-uncertainty pipeline.
