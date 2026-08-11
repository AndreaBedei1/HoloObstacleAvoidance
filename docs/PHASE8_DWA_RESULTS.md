# Phase 8 — Planner Comparison Results (C committed vs D holonomic DWA)

STATUS: PENDING — this skeleton was committed with the D-013 freeze,
BEFORE the campaign ran. Numbers are filled only from
`experiments/simulation/planner_dwa/campaign_longrange/` and
`campaign_pool/` via `scripts/aggregate_planner_campaign.py`; no metric
is added or removed after unblinding (protocol §7).

Campaign: per `docs/PHASE8_DWA_CAMPAIGN_PROTOCOL.md` (frozen). Planner D
frozen per D-013. Planner C unchanged since Baseline 0.

## Headline table (per scenario)

TBD — success k/n with Wilson 95% CI, collisions, min clearance median
(IQR), path length, max lateral deviation, odometry error, per planner.

## Technical-invalid ledger

TBD — every excluded run with its objective signature and replacement.

## 10-question decision gate

1. Did either planner collide in any valid run? Where and why (trace-level
   diagnosis for every collision)?
2. Does either planner dominate success rate overall, and is the
   difference outside the paired Wilson CIs per scenario?
3. Safety margins: how do GT min-clearance distributions compare —
   median, worst case, and tail (per scenario)?
4. Efficiency: path length, maneuver time, lateral-deviation envelope —
   who pays how much for its safety margin?
5. Robustness scenarios (F7 dropout, F8 outlier, F9 heading error,
   F10 speed): does the ranking change under perturbation?
6. Control effort and smoothness: thruster peak/mean, saturation
   fraction, command smoothness — transferability implications?
7. Odometry stress: does either planner degrade the shared no-DVL
   dead reckoning more (odo_err during maneuver), and does that correlate
   with its failures?
8. DWA-specific: how often did `no_admissible_candidate` fire, and did
   the safe-stop fallback ever strand the vehicle (park-style outcome)?
9. Pool-scale (K0/K1): do BOTH planners' maneuvers fit the ~8 m pool
   envelope (lateral excursion, return distance within 6 m route) at
   as-is frozen tuning? What must change for the real pool, if anything?
10. Verdict for the paper: is the committed planner's advantage (if any)
    attributable to commitment/memory under intermittent monocular
    perception — the mechanism the contribution claims — or to something
    else (tuning asymmetry, margin choice, scenario bias)?

## Answers

TBD after campaign.

## Known asymmetries declared before unblinding

- Planner C's parameters have Baseline-0 lineage (not tuned for F/K);
  Planner D was tuned on P-series with pre-declared ranges (D-013).
- Planner D cruises with an explicit obstacle memory (TTL 30 s); C's
  memory is implicit in its commitment state machine.
- The K-series margin (0.80 m frozen) is large relative to the 0.25 m
  pool obstacle; feasibility is evaluated as-is by design (transfer
  question), not re-tuned.
- Baseline-0/Phase-7 campaigns used the committed planner; the shared
  infra (odometry, validator, scenarios) was debugged under it. Residual
  infra bias, if any, favors C.
