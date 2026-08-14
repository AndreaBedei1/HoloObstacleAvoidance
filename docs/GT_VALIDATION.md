# Ground-Truth Validation and Abort Machinery (Phase 9)

Scope: the overhead RealSense pipeline that produces external ground
truth (GT) and supervises safety during real runs. The overhead camera is
**never** an input to the obstacle-avoidance planner — the scientific
boundary stays at `/perception/obstacles_raw`, produced onboard.

Everything below is dry-runnable. Nothing in this document requires the
vehicle, and none of it authorizes actuation.

## What exists

| File | Role | Hardware |
|---|---|---|
| `scripts/real/gt_guard.py` | `PoseGuard` — THE abort predicate, pure function over an explicit state | none (stdlib only) |
| `scripts/real/gt_validate.py` | offline harness: synthetic failure classes + replay of the recorded pilot trajectories | none |
| `scripts/real/test_gt_guard.py` | 36 unit tests of the predicate | none |
| `scripts/real/pool_remap.py` | rebuild the camera→pool transform after a camera move | RealSense (except `--self-test`) |

```
python -m pytest scripts/real/test_gt_guard.py -q     # 36 passed
python scripts/real/gt_validate.py                    # VERDICT: PASS
python scripts/real/pool_remap.py --self-test         # SELF-TEST: PASS
```

`avoid_mission.py` now calls the same `PoseGuard`. That is the point of
the refactor: on 2026-08-14 the wall guard lived inline in the mission,
so the only way to exercise it was to put the vehicle in the water, and
three wet runs (19:00, 19:02, 19:09) were spent discovering that an
absolute margin aborts at t=0 when the operator releases the vehicle from
the pool edge. The predicate is now qualified before anyone gets wet.

## The five abort classes

| Class | Trips when | Verdict |
|---|---|---|
| `wall_margin` | the vehicle leaves the safe box in frame fractions | ABORT |
| `no_pose` | GT never acquired within the acquisition budget | ABORT |
| `stale_pose` | no usable fix for longer than the staleness timeout | ABORT |
| `impossible_jump` | the reported position moved faster than the vehicle physically can | ABORT |
| `low_confidence` | blob area outside the plausible hull range | DEGRADED, escalates to `stale_pose` if sustained |

Aborts **latch**: once the guard has stopped a run it keeps reporting the
same reason, so a caller that misses a cycle cannot silently resume. An
abort always means "break into the active heading hold" — never a bare
disarm, because a disarmed BlueROV2 keeps coasting on tether pull
(measured 2026-08-14).

## Proposed acceptance thresholds, and why

Scale for every conversion below: **402 px/m** at the anchor plane
(2026-08-13 survey), colour frame 1920×1080. That makes the frame
4.78 m × 2.69 m; the measured pool width at the rod is 2.66 m, which is
an independent confirmation that 402 px/m is right to about 1 %.
`pool_remap.py` writes a freshly measured scale — use it once the camera
has been remapped.

### Wall margin — `margin = 0.10`, `release_tol = 0.03`, `hard_limit = 0.02`

The box is **relative to the release point**:

```
lo = min(margin, max(hard_limit, start_frac - release_tol))
hi = max(1 - margin, min(1 - hard_limit, start_frac + release_tol))
```

Rule in words: keep at least the standoff you were released with, minus a
small tolerance, and never cross the hard limit. A mid-frame start
collapses to the plain absolute box [0.10, 0.90].

* `margin = 0.10` → 0.478 m (x) / 0.269 m (y) from the frame edge. The
  pool walls sit just outside the field of view, so this is the working
  standoff for a run that starts in open water.
* `release_tol = 0.03` → 0.143 m (x) / 0.081 m (y). This is the entire
  allowance the vehicle has toward the edge it was released from, and it
  is deliberately tight: the operator hands the vehicle in at the rim, so
  "do not go further into the wall than where you started" is the only
  statement that is both safe and satisfiable. Replay over all eight
  recorded runs with GT confirms it holds: five of them were released
  hard against the left edge (frac_x 0.051 – 0.080) and **not one ever
  moved back toward that edge** — every headroom bottoms out at exactly
  0.030, the tolerance itself.
* `hard_limit = 0.02` → 0.096 m (x) / 0.054 m (y). Nothing relaxes past
  this, whatever the release point. It binds in practice: the 19:50
  release at frac_x 0.051 produced a box floor of 0.021.
* The binding case is not the release edge at all. Run 19:10 started
  mid-frame (0.187, 0.648) and came within **0.015** of the *near-rim*
  limit — the tightest wall approach in the whole pilot, and the one the
  operator did not know about.

Verified by: `test_safe_box_relaxes_for_an_edge_release`,
`test_edge_release_does_not_abort_at_t0`,
`test_edge_release_still_aborts_if_it_gets_worse`.

### Max plausible speed — `max_speed_m_s = 0.60`, `jump_tol_px = 25`

An overhead fix that implies more than 0.60 m/s is a detector fault, not
a vehicle motion. This threshold is the one with real empirical support,
because the eight recorded runs contain both populations.

The predicate allows `jump_tol_px + max_speed × dt` pixels per step, so
the honest statistic is the **step ratio** (observed step ÷ allowance,
abort at 1.00):

| Population | Worst step ratio | Implied speed |
|---|---|---|
| genuine vehicle motion, 8 runs | **0.54** (50 px in 0.28 s) | 0.45 m/s |
| detector re-lock, run 19:05 | **1.26** (123 px in 0.30 s) | 1.02 m/s |
| detector re-lock, run 19:10 | **1.41** (127 px in 0.27 s) | 1.17 m/s |

A threshold at 1.00 sits between the two populations with a factor 2.3
separation. The pool profile's `max_surge_ms` is 0.25 m/s, so 0.60 m/s is
also 2.4× the planned maximum.

Sweeping the threshold against the full corpus
(`gt_validate.py --max-speed X`) shows where it may sit:

| `max_speed_m_s` | Failures |
|---|---|
| 0.35 / 0.45 / 0.60 | 0 — both teleports caught, no genuine run stopped |
| 0.90 | 1 — the 1.02 m/s teleport slips through |
| 1.20 | 2 — both teleports slip through |

The admissible window is roughly 0.30–0.85 m/s; **0.60 is deliberately
placed in the middle of it**, not at an edge, so neither a slightly
faster vehicle nor a slightly smaller glitch flips the outcome.

* `jump_tol_px = 25` (6.2 cm) absorbs blob-centroid wander at dt → 0: the
  ROV silhouette changes shape with roll and ripple, and two fixes with
  the same timestamp must not read as infinite speed.
* The jump test runs **before** the wall test on purpose: a teleport that
  lands inside the box is still a bad measurement, and testing it against
  the wall would only launder it.

Verified by: `test_impossible_jump_aborts`,
`test_measured_peak_speed_does_not_trip_the_jump_test`,
`test_jump_check_precedes_the_wall_check`.

#### Finding: the pilot ground truth contains teleports

Replaying the archived runs turned up a defect that no one was looking
for. Runs **19:05** and **19:10** — the two the experiment log highlights
as passing the anchor plane on GT, with minimum clearances of 0.87 m and
0.80 m — each contain a single ~125 px (≈ 0.31 m) one-cycle jump in the
overhead position:

```
19:05  t=7.80 s   [1007, 288] -> [1130, 291]   123 px   median step 16 px, p90 28 px
19:10  t=13.47 s  [ 976, 956] -> [1103, 959]   127 px   median step 15 px, p90 21 px
```

Both are 5–6× the run's own p90 step, both are almost pure horizontal
translation with the vertical coordinate unchanged, and both are isolated
(the neighbouring steps are ~20 px). A BlueROV2 does not reach 1.1 m/s
for exactly one 0.27 s sample and return to 0.2 m/s. These are detector
re-locks.

Consequence for the reported numbers: in both runs the jump lands **at
the closest approach**. It happens to be nearly tangential to the
anchor direction, so the reported minimum distances move only ~8 px
(19:10: 334 → 326 px), and the 0.80 m / 0.87 m headline figures survive.
But the trajectory is wrong at exactly the moment the paper quotes, and
the same glitch rotated 90° would have corrupted the clearance directly.
**The pilot clearances should not be quoted without this caveat, and the
campaign runs must be screened with `gt_validate.py` before their
clearances are used.** These are pilot/calibration runs, so nothing in
the final statistics depends on them — but it is exactly the failure the
guard now stops in flight.

Separately, run 19:10 also came within **0.015 frame fractions** (~0.04 m
cross-pool) of the wall-margin box near the near rim. It was closer to
both guards than the log suggests.

### Staleness timeout — `stale_abort_s = 1.0` (warn at 0.45), `acquire_timeout_s = 12.0`

* Worst inter-fix gap over 160 fixes in three recorded runs: **0.31 s**.
  The overhead detector essentially never drops out once locked (the
  missing cycles are all before first acquisition). 1.0 s is 3.2× the
  worst observed gap, so spurious aborts are not a realistic risk.
* What it buys: the guard bounds unobserved travel at
  `max_speed × stale_abort_s = 0.60 m` worst case, and at the profile
  speed of 0.12 m/s it is **0.12 m** — inside the 0.143 m (x) release
  allowance and inside the 0.269 m (y) absolute margin.
* **Honest limitation.** At the pilot surge that produced 0.33 m/s the
  unobserved travel is 0.33 m, which exceeds the 0.081 m cross-pool
  release allowance. The clean fix is *not* a tighter timeout — the
  overhead loop only runs at 3.7 Hz, so anything below ~0.5 s starts
  aborting good runs. Either hold the mission surge at the 0.12 m/s pool
  profile, or raise the overhead rate before raising the speed.
* `acquire_timeout_s = 12.0`: the worst measured acquisition was **7.4 s**
  (mission 19:38 — the vehicle sat near the frame edge). 12 s gives 1.6×
  headroom. During acquisition `avoid_mission` now holds PREROLL, so the
  vehicle is stationary and the budget costs nothing.

Verified by: `test_measured_worst_gap_does_not_trip_staleness`,
`test_slow_acquisition_is_tolerated`,
`test_stale_pose_aborts_after_the_timeout`.

### Minimum blob area — `min_area_px = 4000`, `max_area_px = 60000`

The BlueROV2 Heavy footprint is 0.457 × 0.338 m = 0.1545 m², which at
402 px/m is **24 962 px** — matching the upper end of the 4k–25k range
measured on 2026-08-14 and recorded in `overhead_track.py`.

* `4000 px` = **16 %** of the nominal footprint. That tolerates heavy
  partial occlusion by glint and ripple and the vehicle partly leaving
  frame, while rejecting the specular fragments that a percentile
  threshold always produces.
* `60000 px` = **2.4×** the footprint. This catches the observed failure
  where the ROV blob merges with the shaded band along the pool rim; the
  merged centroid is meaningless, and the compactness filter added to
  `overhead_track.py` on 2026-08-14 exists for the same reason.

A rejected blob is **not** used as a fix — it does not update the wall
check and it does not reset the staleness clock. One bad blob is
DEGRADED; sustained bad blobs become `stale_pose`, with
`last_reject = low_confidence` recorded so the operator sees *why* the
pose went stale.

Note: `area_px` was not written to the mission log before today, so the
replayed pilot runs exercise every threshold **except** this one. The
mission now logs `area_px` per cycle, which makes tomorrow's runs fully
replayable.

## Regression status

```
13 synthetic cases, 8 replayed runs, 0 failure(s)
worst observed on real data:  box headroom 0.030 (abort at 0.000)
                            | jump 0.40x (abort at 1.00x)
                            | fix gap 0.31s (abort at 1.00s)
```

Each session is replayed **with the wall margin it was actually flown
with** — the pilot used 0.03, 0.08 and 0.10 as the operator hunted for a
workable value, and replaying them all at one margin compares the guard
against a box the run never had.

The expected outcome for a replayed run is "must complete" unless
`REPLAY_EXPECTATIONS` in `gt_validate.py` says otherwise. Four pilot runs
are expected to abort, each with its reason recorded in that table:

| Run | Expected | Why |
|---|---|---|
| 18:53 | `no_pose` | overhead locked on 7 of 86 cycles (8 %) — flew essentially without external position |
| 18:56 | `stale_pose` | 14 of 75 cycles (19 %), with a 1.32 s hole |
| 19:05 | `impossible_jump` | 123 px teleport (see the finding above) |
| 19:10 | `impossible_jump` | 127 px teleport (see the finding above) |

Those four are the guard doing its job, not false positives. Any run
**not** in that table that aborts is a genuine harness failure.

> **The replay corpus is incomplete in the working tree.** Eight of the
> eleven pilot mission directories under `experiments/real/missions/` are
> deleted in the working tree (they are still in git, and
> `docs/REAL_POOL_EXPERIMENT_LOG.md` still cites them). With only the
> three surviving sessions the harness reports "3 replayed runs" and
> never exercises the four known-bad cases. To validate against all of
> them, restore the directories:
>
> ```
> git restore experiments/real/missions/
> ```
>
> or replay them out-of-tree with `--replay-glob <pattern>` without
> touching the working tree. Deciding whether those recordings should be
> restored is Andrea's call; the numbers in this document were obtained
> by replaying them read-only from git.

Re-run the harness after any threshold change and after every session
(new sessions are picked up automatically).

## Camera remap procedure (`pool_remap.py`)

`config/real_pool/pool_geometry.yaml` is **stale in its camera-frame
entries**: the camera was moved twice on 2026-08-14, which invalidates the
transform, the surface plane and the anchor pixel. The pool dimensions in
that file are unaffected.

Pool frame produced (same convention as the old file, so downstream code
does not learn a second one):

```
origin = anchor attachment point on the rod
+Y     = along the rod, cross-pool, toward the FAR rim
+X     = perpendicular to the rod in the surface plane = approach direction
+Z     = up (surface normal, pointing back at the camera)
p_pool = R @ (p_cam - t)
```

1. Camera rigidly mounted, nothing else in the water, exposure will be
   forced to 5 (the value validated on 2026-08-14 against sun glare).
2. `python scripts/real/pool_remap.py --anchor-px 1069 635`
   — 30 aligned frames, median depth, median colour; the rod is found
   automatically as the long bright near-vertical structure; the anchor
   pixel is a parameter because nothing in the image marks it.
3. **Look at the reference image** (`pool_frame_axes.png`, also copied
   next to the JSON). Confirm: the rod line lies on the rod, the origin
   cross sits on the anchor attachment, +X points along-pool in the
   direction the vehicle will approach, +Y points at the far rim.
4. Check `warnings` and `frame_checks` in
   `config/real_pool/pool_frame_<date>.json`. Specifically:
   * `anchor.snap_distance_px` > 40 px means the supplied anchor pixel no
     longer matches the rod — the camera moved again.
   * `z_source = camera_axis_fallback` means the surface-plane fit failed
     and **+Z is not the pool vertical**; the file is marked
     `valid: false` and must not be used for metric GT.
   * `surface_plane_rms_m` above ~0.06 m means ripple contaminated the
     vertical, which is always the weakest axis of this frame.
5. Anything that touches the camera after this invalidates the file.
   Re-run it. It takes about a minute.

`avoid_mission.py` picks up `scale.px_per_m_anchor_plane` from the newest
valid `pool_frame_*.json` automatically and falls back to 402 px/m with a
printed warning if none exists.

The geometry path is verified offline against a synthetic scene with a
known answer (`--self-test`): axis errors 0.15°, origin error 3.1 mm, and
80/80 configurations across tilt 0–22°, rod yaw ±18° and four noise seeds
recover the frame to better than 1.5° and 30 mm.

## Quantifying localization error — what Andrea must physically do

The overhead GT is currently reported in **pixels** and converted with a
single scale. Before any number from it appears in the paper (minimum
clearance, trajectory error), its error must be measured. Budget about
45 minutes.

**Do the camera remap first, and do not touch the camera again all day.**

### A. Static stations (the core measurement)

1. Lay a tape measure along the near rim, zeroed at the rod (the rod
   plane is pool X = 0). Mark the pool X stations with tape on the rim:
   **−1.5, −1.0, −0.5, 0, +0.5, +1.0, +1.5 m**.
2. For cross-pool Y, use a second tape spanning rim to rim at the rod and
   at ±1.0 m. Mark **Y = −0.6, 0, +0.6 m** (the usable corridor; the
   anchor occupies Y ≈ 0 at the rod, so use Y = ±0.6 there).
3. Suspend the ROV from a pole or boat hook at each station, **at the
   mission keel depth of 0.55 m** — not at the surface. This is the
   single most important detail: the overhead camera looks through the
   air–water interface, so the apparent position of a submerged vehicle
   is refraction-displaced, and the error must be characterised at the
   depth the runs actually use.
4. Hold each station still for ~10 s (≈ 40 overhead fixes at 3.7 Hz).
   Record, per station: the true `(X, Y)` from the tapes, the keel depth
   from the vehicle's pressure sensor, and the overhead fixes.
5. Repeat **three** of the stations at keel depth 0.35 m to get the
   depth sensitivity of the refraction error.

Suggested capture format, one row per fix, so the analysis is trivial:

```
experiments/real/gt_stations/stations_<date>.csv
station_id, true_x_m, true_y_m, keel_depth_m, t, pixel_u, pixel_v, area_px
```

`OverheadTracker.detect()` already returns `pixel` and `area_px`; a
10-line capture loop around it is all that is needed.

### B. Dynamic pass (scale and straightness)

Drive the vehicle straight along +X at the mission speed from X = −1.5 m
to X = +1.5 m at the mission depth, with the overhead logging every
cycle. Two checks fall out: the along-track distance must match the 3.0 m
tape measurement (that validates the scale under motion), and the
cross-track scatter about the fitted straight line bounds the dynamic
noise.

### C. Mount rigidity

Re-run `pool_remap.py` at the **end** of the session. If the recovered
origin has moved more than ~10 px, or the axes by more than ~1°, the
mount is not rigid and every GT number of the day carries that drift.
Report the drift either way.

### D. What to report

Convert each fix to pool coordinates with the transform from
`pool_frame_<date>.json`, then report:

* bias and RMSE in X and Y, over all stations;
* 95th percentile error;
* error versus distance from the image centre (the camera is oblique, so
  error is expected to grow toward the frame edges — if it does, a single
  px/m scale is not good enough and a homography is required);
* error versus depth, from the 0.35 m repeat stations;
* the dynamic-pass scale residual and cross-track scatter;
* the end-of-day mount drift.

### E. Acceptance criteria (pre-registered here, before the data exists)

The real-pool clearances being claimed are ≈ 0.25–0.30 m.

| RMSE | Consequence |
|---|---|
| ≤ 0.05 m | GT usable as a metric result; clearances reported as measured |
| 0.05–0.10 m | usable, but every clearance must be reported with the error bar |
| > 0.10 m | overhead GT is a **safety supervisor only**; clearance claims need another method (a homography with more control points, or an onboard reference) |

Whatever the outcome, the number goes in `docs/REAL_POOL_EXPERIMENT_LOG.md`
next to the clearances it qualifies.

## Standing constraints

* The pilot wet runs of 2026-08-14 are **calibration only**. They are used
  here as a regression corpus for the guard; they never enter final
  statistics.
* The overhead camera is validation and safety. It is never a planner
  input.
* Imaging/side-scan sonar remains permanently excluded.
* Arming is operator-only.
