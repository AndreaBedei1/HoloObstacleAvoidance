# Temporal Obstacle Estimation (Phase 7) — T0–T3

Purpose: rigorously isolate and quantify the effect of temporal obstacle
estimation BEFORE changing the avoidance algorithm. Motivating failure:
Baseline-0 attempt-1 run C_3 (early detector SILENCE ~1 s after avoidance
commitment → stale perception → maneuver abort → re-engagement → recovery to
a wrongly recaptured path). That failure is preserved
(`experiments/simulation/scientific_baseline_0_attempt1/`) and reproduced as
regression scenario E1.

## Common interface (package `rov_obstacle_tracking`)

```
detector / oracle / replay source
  -> /perception/obstacles_raw
  -> temporal_estimator_node  (method:=t0|t1|t2|t3)
  -> /perception/obstacles            (EXISTING planner, UNCHANGED)
  -> /tracking/obstacles_debug        (String JSON, validator-only)
```

All methods (T0 included) run inside the same node/launch so runtime
differences cannot bias the comparison. Planner-facing bearing / area / risk
are recomputed identically for every method with the YOLO-node formula.

**Framework note (documented behavioral difference):** the oracle relay's
own risk field was range-inflated; the estimator recomputes risk from
confidence/centrality/area only, so commitment now happens at the planner's
designed engage distance (~9 m monocular) instead of at first sight (~12 m)
as in the Baseline-0 campaign. This applies equally to all methods.

## The three upstream conditions (explicit, tested)

| Condition | Delivery | Meaning | Handling |
|---|---|---|---|
| A. fresh detection | `on_message([det...])` | new measurement | update |
| B. fresh EMPTY array | `on_message([])` | detector ran, saw nothing = ABSENCE EVIDENCE | miss counter; N=3 consecutive → drop track |
| C. SILENCE | no call; only `tick(t)` | no information | T0: silent; T1: hold; T2/T3: predict, bounded horizon |

This distinction is the direct lesson of C_3 (silence-staleness ≠ empty).

## Methods

- **T0 raw** — passthrough, one output per input message, silent during
  silence. Control baseline; deliberately NOT improved; must reproduce the
  early-dropout weakness.
- **T1 hold/EMA** — last-valid hold (EMA α=0.4) for `hold_duration_s`,
  fixed-rate output, held estimates marked predicted; expiry → fresh-empty
  output (no ghost).
- **T2 fixed KF** — linear CV model, state
  `[cx, cy, log w, log h, d/dt(...)]`, `z = [cx, cy, log w, log h]`, real
  dt; discrete white-accel Q (PSD q=0.30); fixed R (σ_c=0.015, σ_s=0.05);
  chi²(4, 99%) innovation gating (outliers rejected, prediction continues);
  bounded dropout prediction; deterministic expiry (horizon / empty-streak /
  out-of-image / max age); class consistency; single-track with
  multi-track-ready structure (id, association point).
- **T3 adaptive KF** — identical to T2 except
  `R = R0 · exp(θ·φ(features))`, features = (1−conf, 1/√area,
  border-proximity, frame interval). θ comes from an empirical calibration
  file. **Default θ=0 → T3 ≡ T2, `calibrated=false`** — verified by a unit
  test. No invented coefficients: status is
  `adaptive framework implemented, final coefficients pending B1/B2
  asset/model transfer`.

## Parameter tuning procedure (transparent)

Hold duration (T1) and prediction horizon (T2/T3) were set to **2.5 s** on
the D-series DEVELOPMENT cases only (max development dropout D4 = 2.0 s,
bridged with margin; ghost behavior bounded by D13/D14). KF q/R defaults were
sanity-checked on D0 (NIS ≈ 1.1–1.4 with known synthetic noise σ=0.008 vs
R σ=0.015 — the filter is deliberately conservative and the consistency
machinery correctly flags it). Nothing was tuned on the E-series closed-loop
evaluation scenarios. Dropouts longer than the horizon still defeat T1/T2 by
design.

## Deterministic replay framework

`rov_obstacle_tracking/replay.py`: JSONL records
`{t, message_present, detection_present, meas, gt, quality}` — the three
upstream conditions are first-class. `run_replay` interleaves message events
and fixed-rate ticks exactly like the ROS node. GT is consumed only by
`metrics.py` (RMSE overall/measured/predicted, availability, longest gap,
reacquisition delay, ghost-track time, NIS + 95% band, 1σ/2σ coverage).
Same file + same config ⇒ identical outputs (unit-tested).

Controlled cases D0–D14 (`test_cases.py`, seeded): clean; 0.25/0.5/1/2 s
silences; early-after-engagement silence (D5); mid-maneuver (D6); repeated
(D7); fresh-empty variant of D5 (D8); one large outlier (D9); noise burst
(D10); confidence collapse then dropout (D11); timestamp jitter (D12); brief
false positive (D13); legitimate disappearance with extended tail (D14).

Offline results: `experiments/simulation/temporal_estimators/replay_eval/`
(datasets, metrics.json, summary.md, fig_case_comparison.png). Headlines:
T1/T2 restore availability under silence (0.98 vs 0.91 for T0 on D4/D5);
T2 gating wins on the outlier case (D9 RMSE 0.0074 vs T1 0.0102, T0 0.0199);
T1 out-predicts T2 during maneuver-onset dropouts (D5 pred-RMSE 0.028 vs
0.063 — CV extrapolation overshoots when image velocity changes); ghost time
≤0.23 s on D13 and ≤0.07 s on D14 for all temporal methods.

## Calibration pipeline (framework now, coefficients later)

- `scripts/calibrate_visual_measurement_noise.py`: fits θ by least squares on
  log residual variance from paired detector-vs-oracle replay logs;
  round-robin calibration/validation split; refuses <500 samples; REFUSES
  synthetic D-cases unless `--allow-synthetic` (smoke test only). Smoke test
  on synthetic data correctly found ~zero feature correlation (the synthetic
  noise is homoscedastic — there is nothing to learn), confirming the
  pipeline does not hallucinate structure.
- `scripts/evaluate_uncertainty_calibration.py`: NIS distribution vs χ²(4),
  1σ/2σ coverage, predicted-σ-vs-error, residual-vs-confidence/area plots.
- No usable historical YOLO+oracle logs exist in the repo (checked: tracked
  logs are closed-loop validation summaries, not synchronized per-frame
  detector/oracle pairs). Real calibration waits for B1/B2.

## Safety properties

- Ghost obstacles: bounded by horizon/hold + empty-streak deletion +
  out-of-image deletion; measured by `ghost_track_time_s` (D13/D14 + E-runs).
- GT isolation: estimator node refuses ground-truth-like input topics;
  the core module never sees GT (replay evaluation layer only). Unit-tested.

## Closed-loop integration (Scientific Baseline 0 + estimator)

Everything identical to Baseline 0 (BlueROV2 Heavy dynamics, no teleport,
no-DVL commanded odometry, CURRENT committed planner, same nominal command,
same obstacle, same start; planner parameters untouched). Scenarios:

- E0 normal · E1 early 2 s silence 1 s after commit (**C_3 regression**) ·
  E2 mid-maneuver 2 s silence · E3 repeated 0.5 s silences · E4 one large
  outlier at first-detection+0.15 s.
- 3 fresh-process repetitions per method/scenario
  (`scripts/run_temporal_closedloop_campaign.py` →
  `experiments/simulation/temporal_estimators/closedloop/`).

## Infrastructure fixes discovered during Phase-7 integration (documented)

1. **Validator JSON atomic writes** — a hard kill mid-write corrupted a run
   file; now write-to-tmp + `os.replace`.
2. **UE rigid-body sleep** (D-011 class finding): after a watchdog stop the
   vehicle froze permanently — UE puts a resting body to sleep and HoloOcean
   `AddForceAtLocation` does not wake it; thrusters wound up to saturation
   with zero motion. Server-side fix: ±0.02 N alternating vertical dither
   (physically negligible, keeps the body awake) + frozen-state backstop
   (same-pose teleport wake + integrator reset). Evidence:
   `experiments/simulation/temporal_estimators/shakeout2/`.
3. **Estimator validity contract**: the planner drops
   `is_tracking_valid=false` obstacles, so published (incl. held/predicted)
   tracks must carry `true` — predicted-ness lives on the debug topic.

## Results

See `docs/TEMPORAL_ESTIMATOR_RESULTS.md` (written from the closed-loop
campaign manifest; includes the decision gate before DWA).
