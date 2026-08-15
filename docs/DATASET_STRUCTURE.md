# Released dataset — structure and what each file proves

The claim of this work is a sim-to-real comparison. A comparison is only
checkable if a reader can recompute both sides from the released files,
so the dataset is organised around that: every number in the paper is
derived by a script in this repository from a file listed here.

## 1. Simulated predictions (80 runs)

```
experiments/simulation/
  phase10_S0/ phase10_S1/ phase10_S2/ phase10_S3/
    manifest.json                 every run: outcome, launch invocation,
                                  calibration level, relay/plant level
    runs/<K?>_<planner>_<n>/
      validation.json             metrics AND the full command trace
      relay_status.json           level the observation relay ran at
      plant_status.json           level the actuation model ran at
      ros2_launch_*.log           complete launch output
      sim_server_*.log            simulator side
  phase10_predictions/
    predictions.json              aggregated, hashed
    predictions.md                the tables
    PREDICTIONS.sha256            the hash, written before the real runs
```

`relay_status.json` and `plant_status.json` are not diagnostics. They
are the proof that the calibration a run CLAIMS is the calibration it
RAN: an earlier version of the relay died in its constructor and the run
still produced a complete, plausible set of metrics.

## 2. Real validation campaign (20 runs)

```
experiments/real/final_campaign/
  manifest.json                   per run: planner, geometry, repetition,
                                  MEASURED start pose, outcome, sha256 of
                                  both videos
  runs/<geometry>_<planner>_<n>/
    cmd_trace.json                /planner/cmd_vel_safe with the
                                  ground-truth distance at each sample
    ground_truth.jsonl            overhead pose, one line per frame
    perception.jsonl              /perception/obstacles_raw, INCLUDING
                                  the empty messages
    estimator.jsonl               T2 state
    telemetry.jsonl               vehicle attitude, depth, thruster PWM
    onboard.mp4 + onboard_index.jsonl
    overhead.mp4 + overhead_index.jsonl
    sync_markers.jsonl            light flashes visible in BOTH videos
    recording.json                frame counts, measured rate, clock
```

The two videos are the reason the dataset can be re-analysed rather than
merely read. A trajectory table cannot show a reviewer why a detection
failed, cannot be re-run through a different detector, and cannot be
checked against the claimed geometry. The videos can, and the sidecar
indices are the timing authority: container frame rates are nominal and
encoder timestamps are rewritten, so every join uses the index.

Empty perception messages are recorded deliberately. Downstream,
"nothing seen" and "silence" are different states, and a qualifier that
cannot tell them apart behaves differently in the two cases.

## 3. Calibration inputs

```
config/calibration/
  s1_observation_fit.json         from the paired dataset
  s2_timing_fit.json              from the pilot logs
  s3_vehicle.json                 from the actuator trials
config/real_pool/pool_frame_FROZEN.json
experiments/real/paired_dataset/  frames + positions + REPLAY files
experiments/real/actuator_id/     every trial, including excluded ones
experiments/real/missions/        the pilot runs S2 was fitted from
```

Excluded trials stay in the dataset with the reason attached. A trial in
which the vehicle struck the pool wall does not measure free motion, but
deleting it would hide that the exclusion happened at all; a trial in
which the vehicle did not move is the primary evidence for a deadband.

## 4. Analysis

```
scripts/analysis/
  fit_s1_observation.py           paired dataset -> S1
  fit_s2_timing.py                pilot logs     -> S2
  fit_actuator_model.py           trials         -> S3
  redetect_paired.py              replay of the corrected detector
  audit_rung_ownership.py         each rung changes only what it owns
  phase10_predictions.py          verify, aggregate, hash
  sim_real_compare.py             the pre-registered comparison
  phase10_figures.py              the figures
experiments/analysis/
  sim_real/                       tables and JSON
  figures/                        the figures
```

`sim_real_compare.py` reads the simulated and the real runs through the
SAME functions, including the commitment threshold. That is deliberate:
the two domains must not be analysed by two code paths, or a difference
in the analysis becomes indistinguishable from a difference in the
world.

## 5. What is NOT in git

Raw video is roughly 60 MB per run, about 1.2 GB for the campaign. It
lives with the release archive named in the campaign manifest, and the
manifest carries the sha256 of every file, so the dataset is complete
and verifiable even though git does not hold the bytes.

Also excluded: the imaging/side-scan sonar. It is physically connected
to the vehicle and was never activated, initialised, queried or
recorded, at any point in this work.
