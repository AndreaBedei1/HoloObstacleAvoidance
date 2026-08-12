# Phase 8 — Planner Comparison Results (C committed vs D holonomic DWA)

STATUS: FILLED after the campaign completed (2026-08-12). The skeleton,
protocol, planner code, and D-013 freeze were committed at `b46492a`
BEFORE any F/K run. Numbers below come exclusively from
`experiments/simulation/planner_dwa/campaign_longrange/` and
`campaign_pool/` via `scripts/aggregate_planner_campaign.py`
(`aggregate_results.json`); no metric was added or removed after
unblinding.

Campaign execution: 200 protocol runs completed; 19 technical-invalid
occurrences were auto re-run per protocol §6 (12 `cmd_path_dead`
bridge-TCP outages, 5 double graph-liveness failures, 2 `infra_freeze`).
One tuple (F0/dwa rep 9) failed its re-run too and is excluded
(ledger n = 1); 219 valid runs analyzed.

## Headline table

success k/n (Wilson 95% CI) | median min GT clearance m | median max
lateral deviation m:

| Scen | Committed (C) | DWA (D) |
|---|---|---|
| F0 central | **10/10** [.72,1] \| 0.72 \| 2.48 | 8/9 [.56,.98] \| 1.65 \| 3.43 |
| F1 left 1.5 | 9/10 \| 2.22 \| 2.48 | 9/10 \| 2.11 \| 2.42 |
| F2 right 1.5 | 7/10 [.40,.89] \| 2.21 \| 2.48 | **9/10** \| 2.16 \| 2.47 |
| F5 small extent | 10/10 \| 1.46 \| 2.48 | 10/10 \| 1.09 \| 2.60 |
| F6 reduced left | 9/10 \| 0.71 \| 2.48 | **10/10** \| 0.99 \| 3.44 |
| F7 dropout 2 s | 10/10 \| 0.72 \| 2.48 | 10/10 \| 1.65 \| 3.44 |
| F8 outlier | 10/10 \| 0.72 \| 2.48 | 9/10 \| 1.65 \| 3.44 |
| F9 heading +15° | 5/10 \| 1.03 \| 5.92 | 5/10 \| 2.42 \| 6.44 |
| F10 speed 0.4 | **10/10** \| 0.72 \| 2.48 | **0/10** [0,.28] \| 1.80 \| 1.12 |
| K0 pool central | 5/5 \| 2.22 \| 2.48 | 5/5 \| 2.15 \| 3.02 |
| K1 pool left .75 | 5/5* \| **0.50** \| **0.00** | 5/5 \| 2.85 \| 2.90 |

\* K1/C "success" is a NON-MANEUVER: see Q9.

Long-range totals: C 80/90 (88.9%), D 70/89 (78.7%). Excluding F10
(off-tuning speed) and F9 (measurement artifact, Q5): C 65/70 (92.9%),
D 65/69 (94.2%) — statistically indistinguishable.

## Technical-invalid ledger

19 auto re-runs (12 `cmd_path_dead`, 5 liveness×2, 2 `infra_freeze`),
all replaced by valid runs except F0/dwa rep 9 (re-run also invalid →
excluded, n = 9 for that cell). Full per-run records in the two
manifests (`replaced_technical_invalid` fields). 30 single liveness
retries were absorbed by the full-environment restart and are not
exclusions.

## 10-question decision gate — answers

1. **Collisions: ZERO in all 219 valid runs, both planners.** The
   safety layers (C's commitment margins; D's admissibility +
   directional stoppability + 0.8 m margin) both hold everywhere,
   including dropout, outlier, heading-error, and pool scenarios.

2. **Success:** C dominates only through F10 (10/10 vs 0/10) and the F2
   asymmetry reverses it (7/10 vs 9/10). Outside F10/F9 the planners
   are equivalent (92.9% vs 94.2%, overlapping CIs everywhere). No
   overall dominance at the tuned mission speed.

3. **Safety margins:** D's GT min clearance is systematically LARGER
   (Holm-adjusted MWU p < 0.005 on F0, F5, F7, F8, F9, F10; p < 0.05 on
   K0, K1): D medians 1.65–2.85 m vs C's 0.71–0.72 m on central-type
   geometries. C's 0.72 m reflects its implicit 0.75 m offset; its
   worst case (K1, 0.50 m = 0.10 m from hull contact) is the tightest
   pass of the campaign.

4. **Efficiency:** C is tighter and cheaper — smaller lateral envelope
   (2.48 vs 3.43 m median on central geometries, p < 0.05), lower
   thruster peak/mean (p < 0.002 on six scenarios), smoother commands,
   lower odometry stress (p < 0.03 on six). D pays ~1 m of extra
   lateral excursion and ~35% more mean thrust for its wider margins.

5. **Robustness:** F7 dropout and F8 outlier: both planners 10/10 and
   9/10 — the shared T2 + Phase-7B qualification stack absorbs both
   perturbations; neither planner's memory mechanism is the
   differentiator there. F9: the 5/10-vs-5/10 split is BIMODAL AND
   IDENTICAL for both planners (final lateral either ≈0 or ≈−9.5 m =
   36.8·sin 15°): a nominal-line capture race with the 15° start yaw
   (the validator/planners capture the route before vs after the yaw
   settles), i.e. a measurement-frame artifact, not an algorithmic
   failure — the paired design keeps the comparison fair. F10 is the
   real robustness split: see Q8/Q10.

6. **Effort/smoothness:** C peak 3.84 N / mean 0.66 N vs D peak
   3.5–3.6 N but higher mean sway activity; D's command smoothness is
   ~5–10× coarser (sampled argmax vs continuous state machine).
   Transferability: both are far below the 28.8 N thruster cap; neither
   stresses actuation.

7. **Odometry stress:** median odo error 0.15–0.19 m (C) vs
   0.15–0.26 m (D); D's wider sway maneuvers cost ~0.07 m more drift
   (p < 0.03 where significant) — small in absolute terms; the
   commanded dead reckoning stays fit for purpose for both.

8. **DWA failure anatomy (F10):** at 0.4 m/s approach (tuning was at
   0.3), 9/10 runs end in a stable SAFE PARK ~0.6 m outside the
   inflated boundary (fwd 8.48 m, obstacle plane at 11 m): the
   admissibility window at the higher speed brakes the vehicle into the
   blocked-cone standstill, where the frozen objective holds station —
   the classical DWA local minimum. 1/10 escaped the minimum but then
   wandered off-route (41 m lateral, never near the obstacle). Zero
   collisions; the fallback stop behaves exactly as designed. C's
   commitment maneuver is speed-robust (10/10). Note C is not immune to
   sporadic non-successes elsewhere: F2 runs 1/4/6 (no return 2.46 m; a
   mid-run graph death mis-scored as a stop — diagnosed in the post-hoc
   section; a 15.3 m off-line drift).

9. **Pool scale (~8 m):** K0: both planners avoid and return inside a
   ~2.5–3.0 m lateral envelope — marginal but feasible in an 8 m pool
   (≈±4 m of maneuvering room), at 0.15 m/s with the 0.5 m obstacle
   class. K1 (obstacle 0.75 m off-path): **C never maneuvers at all**
   (0 avoidance entries in all 5 runs, perception valid from ~1.2 s):
   its image-space engagement threshold, tuned on the 3.5 m class,
   classifies the small offset obstacle as "passing clear" and grazes
   it at 0.50 m center-clearance = 0.10 m from hull contact — recorded
   as "success" by the metric but unacceptable in a real pool. D's
   explicit metric inflation (0.25 + 0.40 + 0.80 m) forces a proper
   detour (clearance 2.85 m). For the real pool: C needs its engagement
   thresholds re-scaled to the pool obstacle class; D transfers as-is
   but its 0.8 m frozen margin dominates the pool geometry (3 m
   excursions) and a pool-specific margin would tighten it.

10. **Verdict for the paper:** the committed planner's measurable
    advantages are (a) speed-robustness of the committed maneuver (F10)
    and (b) efficiency/precision (smaller envelope, less effort, less
    odometry stress). They are NOT attributable to memory under
    intermittent perception (F7/F8 show parity — the qualification
    stack, not the planner, carries those) and NOT to collision safety
    (zero for both; D actually runs wider margins). The honest claim:
    commitment provides robust maneuver COMPLETION where a myopic
    sampled objective can stall (off-tuning speed, blocked-cone
    geometries), while classical DWA generalizes better across obstacle
    classes (K1) because its geometry is explicit rather than
    threshold-tuned. The baseline is competitive at the tuned operating
    point — the comparison is fair, and the paper should report both
    directions of the trade.

## Known asymmetries declared before unblinding

- C's parameters have Baseline-0 lineage (not tuned for F/K); D was
  tuned on P-series with pre-declared ranges (D-013).
- D cruises with an explicit obstacle memory (TTL 30 s); C's memory is
  implicit in its commitment state machine.
- The K-series margin (0.80 m frozen) is large relative to the 0.25 m
  pool obstacle; feasibility was evaluated as-is by design.
- Baseline-0/Phase-7 infra was debugged under C; residual infra bias,
  if any, favors C.

## Post-hoc observations (flagged as post-hoc, not pre-registered)

- F9's success metric is confounded by the line-capture race; a fixed
  world-frame route definition would remove the artifact. Kept as-is
  here because the protocol was frozen; both planners are affected
  identically.
- F2/C run 4 (0.29 m forward) — trace-level diagnosis done: NOT a
  vehicle stop. The validator recorded only ~1.6 s of observation
  (49 dynamics messages, all healthy: setpoint 0.3, vehicle
  accelerating, state APPROACH_OBSTACLE) and then the ROS graph died;
  the atomic final write preserved a truncated window. This is an
  infrastructure death mis-scored as an algorithm failure. Objective
  signature for FUTURE protocols: validator `elapsed_s` far below the
  commanded run duration (here ~2 s vs 120 s). This campaign's ledger
  stands as frozen; re-scoring C's F2 with that rule would give 7/9
  (0.78) instead of 7/10 — direction unchanged (D 9/10 stays ahead on
  F2 either way).
