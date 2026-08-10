# Real Pool Experiment Protocol (pre-registered)

Written BEFORE the final experiments to prevent cherry-picking. Changes after
the campaign starts require a dated amendment note here.

## Factors

- Obstacles: anchor, torpedo (physical).
- Approaches: central, left-diagonal, right-diagonal (marked start points).
- Algorithms: 3–4 stacks selected from the simulation ablation (must include
  the calibrated estimator stack, one non-probabilistic baseline, and DWA).
- Repetitions: 5 VALID runs per condition (initial target).
- Operating depth: constant policy (from the bottom survey).
- Speed: conservative (≤0.3 m/s surge cap) given the ~8 m pool, stopping
  distance, tether, and wall safety zones; avoidance side must never point
  into the nearer wall (planner wall-awareness checked in sim first).

## Pre-registered run criteria

- **Valid run:** vehicle starts within ±0.15 m / ±10° of the marked start;
  all logs recording (onboard video, external GT video, detections, tracker,
  planner states, commands, depth, attitude); ground-truth marker visible
  ≥95% of the run; no operator input after the start signal.
- **Collision:** any physical contact with the obstacle (external video).
- **Near collision:** minimum clearance < 0.3 m without contact.
- **Successful avoidance:** no collision AND forward progress past the
  obstacle plane AND no wall/floor contact.
- **Successful recovery:** after passing, returns within 0.5 m lateral and
  10° heading of the nominal line before the end zone.
- **Operator intervention:** any manual command/disarm after start → run
  recorded as intervention (not silently discarded).
- **Technical invalidation (excluded from stats but archived):** stack crash,
  stream loss > 1 s, GT marker loss > 5% of run, wrong start placement.
  Exclusions are logged with reason; counts reported in the paper.

## Data per run

Commit SHA, config + calibration versions, seed(s), raw onboard video,
external RealSense video, all ROS topics (bag), interlock audit trail,
operator notes, outcome label per the criteria above. Structure:
`experiments/real_pool/<campaign_id>/runs/<run_id>/`.

## Order & blinding

Condition order randomized per block (obstacle fixed per block to limit
repositioning error); the operator does not see live algorithm identity
during a run (labels only in configs) to reduce intervention bias.
