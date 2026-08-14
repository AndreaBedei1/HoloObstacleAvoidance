# Phase 8 — Campaign Execution Log

Operational provenance for the frozen Phase-8 campaign. **This file records
what was executed and when. It is NOT the protocol** —
`docs/PHASE8_DWA_CAMPAIGN_PROTOCOL.md` is frozen and must not be edited.

Frozen commit: `b46492ad73abb66a725d777a46cf1d1cf810f433` (D-013).

## Execution session 1 — started 2026-08-11 21:23:36 (local)

Command (long-range set only):

```
python scripts/run_planner_campaign.py --planners committed,dwa \
  --scenarios F0,F1,F2,F5,F6,F7,F8,F9,F10 --runs 10 \
  --out experiments\simulation\planner_dwa\campaign_longrange
```

Runner PID 7484. The process **survived the end of the session that started
it** and was still executing when the audit below was taken.

## Audit — 2026-08-12 00:38 (local)

Taken from artifacts (`manifest.json` + per-run `validation.json`), not from
logs or elapsed time.

| Quantity | Value |
|---|---|
| Planned total (protocol §4) | **200** = 180 long-range + 20 pool-scale |
| Planned long-range | 180 (9 F-scenarios × 2 planners × 10 reps) |
| Planned pool-scale | 20 (2 K-scenarios × 2 planners × 5 reps) |
| Long-range tuples attempted | 68 |
| Long-range with valid outcome | **68** |
| Technical-invalid, re-run exhausted | **1** — `F0/dwa/9` |
| Duplicate tuple records | 0 |
| Corrupted manifests | 0 |
| Long-range still to execute | 112 + 1 re-run |
| Pool-scale executed | **0** (not part of the running invocation) |
| Manifest `commit_sha` | `b46492a…` — matches the freeze |

Per-scenario coverage at audit time: F0, F1, F2 complete (10/10 both
planners); F5 partial; F6–F10 not started.

### `F0/dwa/9` — technical-invalid, correctly excluded

Graph liveness failed twice on the first attempt, the runner performed its
one pre-registered re-run (protocol §4), and that attempt also failed
(`also_retry_invalid: true`). Evidence in
`runs/F0_dwa_9/validation.json`: `msg_counts.dynamics_debug == 0`,
`dyn_series == []`, `forward_progress_m == 0.0`, `final_state == NONE`.
Assessment recorded as `{"outcome": "no_data"}`.

This is **not** an algorithm failure and is excluded from the statistics. The
planned tuple still owes a valid outcome and must be re-run.

Known artifact issue: `run_once` reuses the same run directory, so the
re-run's `sim_server_*.log` / `ros2_launch_*.log` overwrote the first
attempt's logs. Fixed going forward by `preserve_partial()` (below).

## Freeze validation — 2026-08-12 00:36

| Check | Result |
|---|---|
| `HEAD` | `b46492a` ✔ |
| `git diff b46492a -- src scripts config` | empty ✔ |
| `install/…/dwa_planner.py` vs frozen source | md5 identical ✔ |
| `install/…/dwa_planner_node.py` | md5 identical ✔ |
| `install/…/local_avoidance_planner_node.py` | md5 identical ✔ |
| `planner_F0/F5/K0/K1.yaml` src vs install | md5 identical ✔ |
| `install/` build time (21:05) vs campaign start (21:23) | built before start ✔ |

**No rebuild was performed and none is required.** The running campaign is
behaviourally the frozen campaign.

## Resume support added — 2026-08-12 00:40 (no experimental change)

The runner had no resume path: `results` started empty and `manifest.json`
was opened with `"w"`, so any restart would have **destroyed the record of
every completed run**. Added, in `scripts/run_planner_campaign.py`:

- `--resume` (opt-in; **omitting it reproduces the previous behaviour
  exactly**) — loads prior results, executes only planned tuples that lack a
  valid outcome, merges into the manifest;
- `load_prior()` — manifest first, per-run `validation.json` as fallback if
  the manifest was lost or truncated;
- `has_valid_outcome()` — a tuple is done only if it completed and is not
  technical-invalid. **Algorithm failures count as valid outcomes and are
  never re-run** (protocol §4);
- `preserve_partial()` — moves an interrupted/technical-invalid attempt into
  `superseded_<n>/` with a `REASON.txt` before re-running the same tuple, so
  nothing is silently overwritten;
- `write_manifest()` — atomic write (tmp + `os.replace`).

No weight, margin, sampling, TTL, speed preference, admissibility rule,
scenario, acceptance criterion or technical-invalid rule was touched.

Dry-tested against the live artifacts: correctly reported 68 complete,
identified `F0/dwa/9` as owing a re-run, and 112 tuples remaining.

Regression suite: `scripts/test_planner_campaign_resume.py` (10 tests, all
passing; runs under pytest or standalone, since the experiment machine's
default interpreter has no pytest). It pins the invariants that protect the
campaign: an algorithm failure is a valid outcome and is never re-run, a
technical-invalid run is re-run, a corrupt manifest is recoverable from
per-run artifacts, a re-run never overwrites the previous attempt, and the
manifest write is atomic.

### Commit hold (important)

`git_sha()` resolves `HEAD` on **every** manifest write, so committing while
the campaign runs would make subsequent writes record a different SHA and
break the frozen-provenance record. The resume change is therefore held
uncommitted until the long-range campaign finishes. `git_sha()` carries no
dirty flag, so the uncommitted edit does not affect the recorded SHA.

## Continuation commands

Only after the current runner (PID 7484) has exited:

```
python scripts/run_planner_campaign.py --planners committed,dwa \
  --scenarios F0,F1,F2,F5,F6,F7,F8,F9,F10 --runs 10 --resume \
  --out experiments\simulation\planner_dwa\campaign_longrange
```

Then the pool-scale profile. **Use the directory the pre-registered results
skeleton already names (`campaign_pool`)**, protocol §4: n=5:

```
python scripts/run_planner_campaign.py --planners committed,dwa \
  --scenarios K0,K1 --runs 5 --duration 90 --resume \
  --out experiments\simulation\planner_dwa\campaign_pool
```

Aggregation runs only when every planned tuple has a valid final outcome:

```
python scripts/aggregate_planner_campaign.py \
  experiments\simulation\planner_dwa\campaign_longrange\manifest.json \
  experiments\simulation\planner_dwa\campaign_pool\manifest.json \
  --out experiments\simulation\planner_dwa\aggregate.json
```

Gate question 9 (pool envelope) additionally needs the lateral excursion and
return distance, which the aggregator does not compute:

```
python scripts/analyze_poolscale_envelope.py \
  experiments\simulation\planner_dwa\campaign_pool\manifest.json \
  --scenarios K0,K1 --out experiments\simulation\planner_dwa\poolscale_envelope.json
```

`scripts/analyze_poolscale_envelope.py` was added this session as a SEPARATE
script so the pre-registered aggregator stays untouched. It never assumes a
pool dimension: it reports required workspace unconditionally and only
evaluates fit against `--pool-length` / `--pool-width` supplied from an
actual survey.

## Correction recorded 2026-08-12 00:52

`docs/PHASE8_DWA_RESULTS.md` was **overwritten in error** during this session
with an invented "proposed" decision gate. That file is pre-registered: it
was committed WITH the D-013 freeze and contains the authoritative
10-question gate and the "known asymmetries declared before unblinding"
section. It was restored from `b46492a` (`git checkout --`) and is byte-clean
against the freeze. **The pre-registered gate stands; no question in it was
authored this session.** Answers are still to be filled from the completed
campaign only.
