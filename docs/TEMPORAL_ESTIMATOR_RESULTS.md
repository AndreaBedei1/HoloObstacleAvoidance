# Temporal Estimator Results (Phase 7) and Decision Gate

Data: `experiments/simulation/temporal_estimators/`
(`replay_eval/` offline D-cases; `closedloop/` E-campaign 60 runs +
`closedloop_patch_e0t3/` 3 reruns after the t3 node-crash fix;
`shakeout*/` debugging artifacts, preserved). Framework commit `d2e0fca`.
All labeled `simulation dynamics integration baseline` — oracle perception,
NOT visual results.

## Offline D-series (deterministic replay — the clean comparison)

| Signal | Result |
|---|---|
| Availability under 2 s silence (D4/D5) | T1/T2/T3 ≈ 0.98 vs T0 0.91 — temporal methods bridge silence as designed |
| Fresh-empty vs silence (D8 vs D5) | correctly different: empty-streak drops the track (availability 0.91) while silence is bridged (0.98) |
| Outlier rejection (D9, mature track) | T2/T3 RMSE 0.0074 < T1 0.0102 < T0 0.0199 — χ² gating works on mature tracks |
| Prediction during maneuver-onset dropout (D5) | **T1 hold RMSE 0.028 beats T2 CV prediction 0.063** — constant-velocity extrapolation overshoots when image motion changes at maneuver start |
| Ghost tracks (D13 brief FP / D14 legit disappearance) | ≤ 0.23 s / ≤ 0.07 s for all temporal methods — no indefinite ghosts |
| NIS / coverage (known synthetic noise) | NIS ≈ 1.1–1.4 (target 4), 1σ coverage 0.99 (target 0.68): fixed R (0.015) is ~2× the true synthetic noise (0.008) — the consistency machinery correctly detects miscalibration; this is the machinery T3's real calibration will exploit |
| T3 ≡ T2 with θ=0 | verified bit-identical (unit test + identical metrics) |

## Closed-loop E-campaign (BlueROV2 dynamics, current planner, n=3/cell)

| Scenario | t0 | t1 | t2 | t3 |
|---|---|---|---|---|
| E0 normal | 2/3 (1 abort) | 2/3 (1 abort) | 3/3 | 1/3 (1 abort, 1 collision†) |
| E1 early 2 s silence (C_3 regression) | 3/3 | 2/3 (1 abort) | 3/3 | 2/3 (1 abort) |
| E2 mid-maneuver 2 s silence | 3/3 | 3/3 | 2/3 (1 abort) | 3/3 |
| E3 repeated 0.5 s silences | 3/3 | 2/3 (1 abort) | 2/3 (1 abort) | 2/3 (1 abort) |
| E4 giant outlier at first-det+0.15 s | 1/3 (1 collision) | 2/3 (1 collision) | 2/3 (1 collision) | 3/3 |

(E0×t3 = post-fix reruns; the original three runs had a dead estimator node
— `.warn` removed from the lyrical logger API — and are excluded as
technical-invalid, preserved in the main manifest. † the E0/t3/3 collision is
a no-strafe straight-in run, vehicle-pinned signature.)

### Failure anatomy (each analyzed, none hidden)

1. **Startup-transient early-commit (7 runs, ALL methods):** commitment fires
   at t < 0.5 s — while the vehicle is stationary 12 m away — on degenerate
   first oracle projections during graph bootstrap, then aborts within ~3 s
   and the reference line is lost (final lateral 7–11 m). Method-independent
   infrastructure noise; signature is unambiguous (commit `t < 1 s`).
   In clean runs commitment happens at ~28–60 s when the vehicle genuinely
   reaches the 9 m engage range.
2. **E4 outlier → collision (t0, t1, t2 once each):** the corrupted bbox
   (2.5× size ⇒ monocular range ~3.6 m) triggers an instant commit with
   corrupted geometry. χ² gating does NOT save young tracks: at
   first-det+0.15 s the covariance is still wide, the gate accepts. T3's
   3/3 on E4 is timing luck, not a method difference (θ=0 ⇒ T3 ≡ T2).
3. **E1 (C_3 regression) did NOT reproduce the original abort — for any
   method, including T0.** The original C_3 failure was tied to the previous
   12 m instant-commit configuration; with commitment at the designed 9 m
   engage range, the current planner already survives a 2 s silence 1 s
   after commit even with raw perception. Honest consequence: in THIS oracle
   configuration the dropout story alone does not justify estimator
   complexity; it becomes decisive with real YOLO noise/dropout behavior.

## Decision gate (per the seven questions)

1. **Does T1 already solve nearly everything?** Offline: yes for
   availability under silence; it even beats T2 on maneuver-onset dropout
   prediction. Closed loop: indistinguishable from T2/T3 at n=3 against the
   startup-transient noise floor.
2. **Does T2 measurably improve on T1?** Yes on outlier rejection for
   MATURE tracks (D9) and estimate smoothness; no on onset-dropout
   prediction (CV overshoot); no separable closed-loop difference at n=3.
3. **Model-based prediction without excess ghosts?** Yes — ghosts bounded
   (≤0.23 s offline; horizon-bounded 2.5 s in loop).
4. **Which variables correlate with measurement error?** Unknown and
   UNKNOWABLE on synthetic data (homoscedastic by construction — the
   calibration pipeline correctly finds ~zero structure). Requires real
   YOLO-vs-oracle residuals (blockers B1/B2).
5. **Is T3 likely to give a scientifically meaningful gain?** Undecided —
   depends entirely on whether real detector residuals show learnable
   heteroscedasticity. The consistency machinery (NIS/coverage) is proven to
   detect miscalibration. Keep T3 as framework; claim nothing yet.
6. **Does the early-dropout failure disappear for the right reason?** The
   original failure disappeared for a CONFIGURATION reason (9 m vs 12 m
   commit), not because of the estimators — documented, not claimed.
7. **Is estimator uncertainty statistically meaningful?** On synthetic data
   it is measurably (and expectedly) conservative; real assessment pending.

## Recommendation (before any DWA work)

1. **Fix the two exposed integration weaknesses first:** (a) perception
   warm-up gate (require ~1 s of consecutive coherent detections before the
   relay forwards — kills the startup-transient family); (b) track
   confirmation rule (publish only after M≥3 accepted updates — closes the
   young-track outlier hole E4 exposed). Both are estimator-layer changes;
   the planner stays untouched.
2. **Then rerun a tightened E-campaign with n=5** to get a usable noise
   floor before method ranking.
3. **T3 calibration proceeds ONLY after B1/B2** (custom world + YOLO
   weights) provide real residual data; if real residuals turn out
   homoscedastic too, T3 should be dropped and that negative result
   reported (paper/contributions.md already carries this contingency).
4. DWA implementation stays gated until 1–2 are done — comparing planners
   on top of an integration noise floor would be meaningless.

## Bug ledger for this phase (all fixed, all evidenced)

- `.warn` → `.warning` (lyrical logger API) — killed the t3 node at startup.
- Validator JSON now written atomically (kill-mid-write corruption).
- UE rigid-body sleep guard (dither + same-pose-teleport backstop) after the
  watchdog-stop freeze; one E0/t3 rerun still shows a pinned-vehicle
  signature under sustained contact — physically plausible after a failed
  avoidance, kept under observation.
- Estimator validity contract (published ⇒ planner-usable).
