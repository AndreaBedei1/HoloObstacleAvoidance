# Phase 7B Results — Perception Qualification + Clean n=5 Benchmark

Campaign: `experiments/simulation/temporal_estimators_phase7b/closedloop/`
(100/100 runs, fresh engine/router/launch per run, parameters frozen per
D-012, protocol pre-registered in `docs/PHASE7B_CAMPAIGN_PROTOCOL.md`).
Label: `simulation dynamics integration baseline` (oracle perception).

## Success table (pre-registered success definition, n=5 per cell)

| Scenario | T0 | T1 | T2 | T3 |
|---|---|---|---|---|
| E0 clean | 5/5 | 5/5 | 5/5 | 5/5 |
| E1 early 2 s silence | 5/5 | 4/5 | 5/5 | 5/5 |
| E2 mid-maneuver 2 s silence | 4/5 | 5/5 | 5/5 | 5/5 |
| E3 repeated silences | 5/5 | 5/5 | 5/5 | 5/5 |
| E4 young-track outlier | 3/5 (1 collision) | 3/5 (2 collisions) | **5/5** | 4/5 |
| **Total** | 22/25 | 22/25 | **25/25** | 24/25 |

Per-method medians (all runs): clearance 0.708–0.713 m; final lateral
0.014–0.019 m; maneuver 69.5–69.7 s; confirmation delay 1.01–1.03 s;
odometry error max 0.173–0.185 m — the methods are indistinguishable on
every continuous metric in nominal conditions; ALL separation lives in E4.

## Failure anatomy (every non-success, none reclassified)

- **E4 grazing collisions (t0×1, t1×2): min clearance 0.288–0.297 m** vs
  the 0.30 m threshold — maneuvers of normal shape but ~0.4 m smaller
  lateral amplitude (2.04 vs 2.46 m). All three in the hold family; none in
  the KF family. In t1/3 the outlier passed the coherence gate (a startup
  frame-gap widened the dt-scaled bound) and the EMA absorbed a rightward
  bias; in the other two the outlier was gate-rejected but its confirmation
  penalty delayed planner-validity by ~1 s. Mechanism partially
  characterized; the family correlation (3/10 hold vs 0/10 KF) is the
  robust observation.
- **Residual vehicle-freeze infra mode (t0/E4, t3/E4 — 2/100):** vehicle
  stops ~1.9 m from start, never engages (clearance ~10 m, lateral 0.0) —
  same pinned/UE-sleep signature as E0/t3/3 in Phase 7. Does not meet the
  pre-registered technical-invalid list, so BOTH COUNT AS FAILURES per
  protocol; diagnosis noted for the infra backlog.
- **Single tail events (t1/E1 re-engagement; t0/E2 not-returned):** 1/25
  each — the new noise floor.

## Decision gate (the ten questions)

1. **Warm-up gate vs startup transients:** ELIMINATED — 0/100 early commits
   (Phase 7: 7/63); E0 row went 20/20.
2. **Track confirmation vs E4 young-track mechanism:** the Phase-7
   corrupted-commit collision mechanism is gone (commit distance stayed
   ~11.8 m, single normal-shaped maneuvers). E4 residual harm now appears
   only as grazing passes in the hold family; the gated KF family is clean.
3. **Confirmation latency:** ~1.0 s median (warm-up dominated), identical
   across methods.
4. **Clearance cost of that latency:** none measurable — clean-run
   clearance 0.71 m median, unchanged from Phase 7; distance at first
   planner-valid 11.97 m vs 12.0 m visible.
5. **T1 vs T2 at maneuver onset:** T1 remains superior offline (hold
   pred-RMSE 0.028–0.036 vs CV 0.063–0.084; GT image-velocity discontinuity
   −0.032 u/s at engagement quantified as the cause). In closed loop the
   nominal scenarios cannot separate them (both 19–20/20 on E0–E3).
6. **Does T2 justify its complexity?** YES, on one specific axis: outlier
   robustness for young tracks (E4: 5/5 vs 3/5) — mature-track gating plus
   covariance-weighted absorption. Elsewhere it buys nothing over T1 and
   remains WORSE at onset prediction. Honest summary: T2 is an outlier
   armor, not a better predictor.
7. **Ranking stability:** yes — T2 ≥ T3 > T0 ≈ T1, driven entirely by E4;
   E0–E3 rankings are flat (77/80 overall).
8. **Noise floor:** ~1 non-success per 25 runs (4%) from single-run tail
   events + ~2% residual vehicle-freeze infra mode. Low enough to compare
   planners on aggregate success and clearance; per-scenario differences
   below ~2/5 remain unresolvable at n=5.
9. **T2b (exploratory):** consistently between T1 and T2 at onset (pred
   RMSE 0.055/0.043) — an improvement over T2 but still behind the hold.
   DISCARD as a benchmark method; keep the damping idea as a candidate for
   a maneuver-aware model later (documented, not registered).
10. **DWA go/no-go: GO.** The integration noise floor is characterized and
    low; the estimator layer is stable and common; the benchmark harness,
    metrics and protocol are pre-registered and reusable as-is for a
    planner comparison.

## Scientific note (per the outcome rule)

T1's onset superiority and T2's E4-only advantage are BOTH preserved as
first-class results. The honest framing for the paper: *simple temporal
persistence is a stronger predictor through maneuver-induced image-motion
changes than a constant-velocity Kalman filter; the Kalman machinery earns
its place through gated outlier robustness, not through prediction
accuracy.* T3 remains `calibration pending` (θ=0 ≡ T2 throughout;
its 24/25 vs T2's 25/25 is one infra-freeze run, not a method difference).

## Carry-over infra backlog

Residual vehicle-freeze mode (~2%): pinned-at-start signature despite the
sleep-guard dither — next candidate mechanisms: contact with a transient
spawn artifact, or sleep re-entry between guard wake-ups. To be
instrumented before the DWA campaign (log guard activations per run).
