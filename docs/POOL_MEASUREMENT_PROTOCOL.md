# Pool Measurement Protocol

Fills every `TODO_MEASURE_REAL_POOL` in `config/pool_digital_twin.yaml`.
Tools: tape measure / laser rangefinder, plumb line, marked measuring rod,
spirit level, waterproof markers, the RealSense once mounted.

## 1. Pool geometry

1. Length and width at the waterline (two measurements each, opposite sides;
   record both, use mean; disagreement > 2 cm → investigate squareness).
2. Define and physically mark the pool-frame origin corner + x direction.
3. Waterline height reference vs the origin marker.

## 2. V-bottom survey

Grid: every 1.0 m along the length × every 0.5 m across the width, measure
water depth with the marked rod (plumb, read at waterline). Record
`{x, y, depth}` triples. Densify to 0.25 m near the obstacle placement area.
Deliverable: `config/pool_bottom_survey.csv` + interpolated profile plot.

## 3. Obstacles

For each physical obstacle (anchor, torpedo): bounding dimensions with tape
(3 axes), mass if liftable/known, placement position (plumb from two measured
wall references), yaw vs pool x-axis, base depth at that location from the
survey.

## 4. Start points

Choose ≥3 repeatable start locations (central, left-diagonal, right-diagonal
relative to the obstacle area); mark physically (weighted floor markers /
edge references); measure their pool coordinates and intended headings.

## 5. External camera

After rigid mounting: photograph the mount; measure camera position vs pool
frame (tape + plumb); verify full usable-area coverage in the RGB preview;
lock focus/exposure; record the perimeter reference markers' pool coordinates
(≥4 non-collinear). Then run the dry extrinsic calibration and the
multi-depth submerged calibration (`docs/REALSENSE_GROUND_TRUTH_PLAN.md`).

## 6. Vehicle references

Marker board position/orientation on the ROV measured relative to the
vehicle body frame; operating depth chosen from the survey (constant-depth
policy, sufficient bottom clearance along all routes).

Every measurement gets: value, method, estimated uncertainty, date, initials.
Store raw notes as a scan/photo under `datasets/pool_survey/`.
