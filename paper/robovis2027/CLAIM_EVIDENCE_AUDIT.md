# Claim-to-evidence audit

This internal audit is not part of the review PDF. Paths are relative to the
repository root. `PRE`, `REAL`, and `POST` follow the manuscript definitions.

| Class | Manuscript claim | Primary artifact or source | Audit conclusion |
|---|---|---|---|
| PRE | 80 runs: 4 calibration levels × 2 planners × 2 geometries × 5 repetitions | `experiments/simulation/phase10_predictions/predictions.json`; `frozen_analysis.json` | Cell counts, medians, collision count, and missing qualification/commitment values match the committed aggregate. |
| PRE | Frozen provenance precedes deployment | `experiments/simulation/phase10_predictions/PREDICTIONS.sha256`; freeze commit `d1cb7be` | The hash binds the predictions artifact; the manuscript reports a committed pre-deployment campaign, not a post-hoc reconstruction. |
| PRE | S0--S3 definitions and FOV mismatch | `src/rov_obstacle_sim_bridge/rov_obstacle_sim_bridge/calibrated_observation_relay_node.py`; `src/rov_obstacle_sim_bridge/launch/holoocean_baseline0.launch.py`; `src/rov_obstacle_avoidance/rov_obstacle_avoidance/planner.py` | Relay defaults to 60° while planner range inversion uses 90°; the distinction is now explicit. |
| REAL | Nine acquired runs, eight admissible | `experiments/real/PILOT_MANIFEST.json`; `experiments/real/command_trace_normalization.json` | Run 5 exceeds the 0.0107 contamination threshold (0.0349); eight runs pass. |
| REAL | The physical comparison is command-domain only | `experiments/command_domain_comparison.json`; `experiments/analysis/sim_real/sim_real.json` | All 8 admissible physical runs remain at the strafe setpoint; all 20 matched frozen runs reach the go-around setpoint. |
| REAL | No defensible physical clearance or trajectory metric | `experiments/real/perception_recovery/summary.json` | Six videos and 24 paired observations cannot recover externally referenced spatial ground truth; detector-derived distance is not relabelled as truth. |
| REAL | Detector acceptance corresponds to a 0.29--2.11 m range window | `scripts/real/anchor_detect.py`; planner range equation and 90° FOV source above | Image-height gates are 0.15 and 0.90; the reported window follows from the documented inversion. |
| REAL | The runs used a 1.8 m engagement range | Per-run `result.json` files under `experiments/real/quick_runs/` | Recorded configuration is 1.8 m; the manuscript does not repeat the session-note value of 5.0 m as executed. |
| POST | 80 diagnostic runs, A0--A7, ten per configuration | `experiments/simulation/diagnostic/ablation_index.json`; `ablation_analysis.json` | Counts and medians in the generated LaTeX table match the stored validation artifacts. |
| POST | A1--A6 are single-factor changes relative to A0 | `scripts/run_deployment_gap_ablation.py`; diagnostic index | Each rung changes one documented factor; A0 is the same-session baseline. |
| POST | A7 combines A1--A6 and was not flown | `scripts/run_deployment_gap_ablation.py`; diagnostic index entry historically stored as `A7_deployed` | A7 contains the FOV correction and geometry rescaling as well as A1--A4; its label is now *combined diagnostic configuration*. The historical directory name is retained only for artifact compatibility. |
| POST | A6 produced 4/10 unsafe runs | `experiments/simulation/diagnostic/ablation_analysis.json` | Supported only for the tested rescaling. The manuscript explicitly rejects extrapolation to every basin-compatible manoeuvre. |
| LIMITATION | One pool/day, one static anchor, one planner, eight admissible runs, K1/K1M n=2 | `experiments/real/PILOT_MANIFEST.json`; real campaign artifacts | Declared together near the start of the limitations section; no population-level generalisation is made. |
| LIMITATION | The deployed configuration changed during the session | Deployment source/config history and per-run results | Declared as a constraint on causal interpretation; POST results are diagnostic sensitivity evidence, not pre-registered prediction. |
