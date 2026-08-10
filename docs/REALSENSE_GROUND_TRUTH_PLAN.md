# RealSense External Ground-Truth Plan

Role: independent validation-only measurement of the BlueROV2's pool-frame
x, y, yaw during real experiments. **Never** an input to YOLO, tracking,
planning, or command generation (runtime guard in Phase 9).

## Inventoried device (2026-08-10, desk, read-only)

- **Intel RealSense D435** (RGB module present), USB. Full dump:
  `visualizations/realsense_inventory/realsense_inventory.json`
  (+ sample RGB/depth frames alongside), produced by `scripts/inspect_realsense.py`.
- RGB 1920×1080@30 requested → **29.81 fps measured** sustained; frame interval
  33.5 ± 2.8 ms, p95 36.1 ms.
- Timestamp domain: **system time** (host clock) → directly comparable with ROS
  host-clock stamps on the same PC; no cross-device clock transfer needed.
- Depth 848×480 with depth-to-color extrinsics recorded. **Depth and IR are
  recorded but NOT trusted through the air-water interface** until validated
  against known submerged references (refraction + IR projector behavior at the
  surface are unmodeled). RGB is the primary ground-truth channel.

## Physical configuration (when pool access exists)

- Rigid mount, as high as practical, near-nadir over the usable test area.
- Fixed focus/exposure during experiments (lock auto-exposure after warm-up;
  the inventory records the option ranges).
- Pool coordinate references: ≥4 non-collinear measured markers on the pool
  perimeter visible in-frame; define pool frame (x = length, y = width,
  z documented).
- High-contrast rigid marker board on top of the BlueROV2. Candidates to be
  tested through the water surface, in order: ArUco board, AprilTag (36h11),
  custom high-contrast asymmetric pattern (fallback if fiducial detection
  degrades through ripples). Board must yield x, y, yaw + detection confidence.

## Refraction-aware calibration (no naïve single homography)

Air-water refraction makes the pixel→pool mapping depth-dependent. Plan:

1. Dry intrinsic calibration of the RGB camera (checkerboard) — desk-doable now.
2. Extrinsics to pool frame from the perimeter markers (above water — no
   refraction on those rays).
3. **Multi-plane submerged calibration:** place a target at surveyed x/y at
   several known depths (using the V-bottom survey + measuring rod); fit
   per-depth pixel→pool-x/y maps; interpolate with the ROV's *measured pressure
   depth* at runtime:
   `(u, v) + z_rov  →  (x, y)_pool`.
4. If residuals warrant it, replace interpolation with an explicit flat-refraction
   model (Snell's law through the planar interface).

## Validation before use (pre-registered)

Report on static known positions at multiple depths and a moving-marker pass:
x RMSE, y RMSE, yaw RMSE, 95th-percentile errors, error vs depth, timestamp
uncertainty. Ground truth is usable for the campaign only after these numbers
are documented.

## Isolation rules

- Topics: `/ground_truth/external/pose`, `/ground_truth/external/confidence`
  (package `rov_ground_truth`, Phase 11).
- Consumers allowed: logger, validator, experiment analyzer. Planner/perception
  nodes must not subscribe (`/ground_truth/*` runtime guard + test).

## Open items (need pool access)

- Mounting height/pose, final FOV coverage check (whole usable area).
- Marker size selection vs height (target ≥ 30 px marker edge at max distance).
- Multi-depth calibration session + error report.
- Synchronization event procedure (e.g. LED flash visible to both cameras) to
  bound residual clock offset.
