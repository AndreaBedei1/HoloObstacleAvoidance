# experiments/real -- PILOT / CALIBRATION DATA ONLY

**Everything in this folder is pilot and calibration data from the 2026-08-14
wet runs, and none of it may enter final evaluation statistics.** These runs
were recorded while the pool geometry, the visual detector and the engagement
thresholds were still being tuned between runs, so the numbers in these logs
are not comparable to each other and are not measurements of any frozen
system. They are admissible only as calibration input, as qualitative
illustration, or as provenance history. Every reported result, table and plot
in the paper must come from a post-calibration campaign run against a frozen
protocol, recorded outside this folder.

## What is here

`PILOT_MANIFEST.json` is the immutable record: for every artifact it stores
the relative path, size, sha256, mtime, and one label.

| Label | Meaning |
| --- | --- |
| `PILOT_CALIBRATION` | Wet closed-loop and dataset-capture runs whose logs look like results but are calibration only (`missions`, `avoid_runs`, `anchor_dataset`, `approach_dataset`). |
| `BRINGUP` | Platform characterisation: thruster response, yaw authority, station keeping (`motion_tests`, `station_keep`, `yaw_authority`). |
| `DEVELOPMENT` | Software and algorithm iteration: detector checks, visual-servo tuning, supervised move tests (`servo`, `supervised`). |

The labels differ only in which pilot activity produced the artifact. No label
means "usable as a result". A directory added later with no labelling rule
falls back to `PILOT_CALIBRATION`, so new data can never default into looking
admissible.

## Regenerating and verifying

```
python scripts/analysis/build_pilot_manifest.py            # rebuild
python scripts/analysis/build_pilot_manifest.py --check    # verify, no write
```

`--check` re-hashes the folder and exits non-zero on any added, modified or
removed artifact. Both modes are read-only with respect to the data.

## Integrity status

The manifest reconciles the folder against git and reports three conditions in
its `integrity` block; consult that block rather than this paragraph, as it is
regenerated with the data. As of manifest generation at git
`3f2d784817c93929e2be4704d522b9de045c626d`:

- **46 artifacts (8 mission runs) are tracked in git HEAD but missing from the
  working tree.** The on-disk record is therefore an incomplete subset of the
  pilot session: the 8 absent runs include the 3 that aborted on the overhead
  safe-box wall margin, while all 3 runs still on disk passed. Any count taken
  from the filesystem alone is survivorship-filtered. The data is intact in
  git and restorable with `git checkout -- experiments/real`.
- **7 artifacts are untracked** (mission `20260814_195048`, not yet committed).
- **58 artifacts are gitignored** (bulky frames and video excluded by rule).
  Git holds no copy of these, so the sha256 in the manifest is their only
  integrity record; back them up outside git.
