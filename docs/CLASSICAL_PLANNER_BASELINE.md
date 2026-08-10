# Classical Obstacle-Avoidance Baseline Selection

Requirement (supervisor): compare against a known, famous obstacle-avoidance
algorithm from the literature. Selection is literature-driven (see
`docs/LITERATURE_AND_NOVELTY.md`), not preference-driven.

## Decision: Dynamic Window Approach (DWA), underwater-adapted

Implement DWA following **Eriksen, Breivik, Pettersen (2016), "A modified
dynamic window algorithm for horizontal collision avoidance for AUVs"** — the
canonical adaptation of DWA to second-order underwater vehicle dynamics.

## Why DWA over the alternatives

| Candidate | Assessment |
|---|---|
| **DWA** | **Chosen.** Mari et al. (Sensors 2026) already use DWA as THE baseline for BlueROV2 obstacle avoidance — it is the accepted comparator in exactly our setting. EROAS (IEEE JOE 2024) uses DWA and APF as baselines; VADWA (2024) and the 2025 marine path-planning review confirm DWA as the mainstream classical local planner underwater. Naturally handles the BlueROV2's velocity-controlled quasi-holonomic motion. |
| APF | Optional secondary baseline only (Ochoa 2022, EROAS precedent). Known local-minima/oscillation pathologies make it a weaker primary comparison. |
| VFH/VFH+ | Designed for dense range histograms (sonar/lidar); ill-matched to sparse monocular object detections. Underwater precedent thinner (Zhang et al. AUV-VFH variants). |
| Velocity Obstacle / ORCA | Aimed at dynamic obstacles; our primary study is static obstacles — VO reduces to a cone check and would be a strawman. Canonical underwater VO (Zhang et al. 2017) is simulation-only. |
| Local MPC | Strong but heavy: an honest MPC baseline (à la de Groot IJRR 2025) is a research project of its own; a weak MPC implementation would draw "under-tuned baseline" objections. |

## Implementation rules (pre-registered, anti-strawman)

1. **Same perception input:** DWA consumes the SAME estimator's mean state
   (uncertainty discarded), never oracle obstacle positions and never raw
   detections — so the comparison isolates the *use of uncertainty*, not
   estimation quality.
2. **Honest tuning:** document the tuning procedure and grid; report the tuned
   configuration. (Mari et al. reported 76% collisions for DWA — reviewers are
   primed to suspect under-tuned DWA baselines.)
3. Cost terms per Eriksen 2016: goal/heading progress, clearance, velocity
   admissibility under the measured/simulated acceleration limits of the
   BlueROV2, with our vehicle footprint + fixed safety margin.
4. Same command interface (`/planner/cmd_vel_safe`), same rate, same limits as
   the other planners.
5. Optional ranking-set extension: a DWA-with-uncertainty-inflated-margins
   variant (Dobrevski-style adaptive DWA) strengthens the S0–S3
   ranking-preservation analysis — implement only if time allows.

## Status

- [ ] Implement `planner_mode:=dwa_baseline` (Phase 8)
- [ ] Unit tests: admissible-window computation, clearance scoring, tuning grid
- [ ] Simulation validation vs committed baseline
