"""What do the EXISTING trials already identify, and what is missing?

Phase 10 rule: 8-12 trials per axis is an UPPER BOUND, not a target. The
cheapest defensible calibration reuses every valid pilot trial and
collects only the trials that are still needed to make the command
mapping identifiable OVER THE OPERATING RANGE THE PLANNERS WILL USE.

This script reads the recorded trials, reports per signed axis what is
already estimable and with what confidence, and prints the SHORTEST list
of missing trials. It touches no hardware.

Identifiability, per signed axis, needs:
  * at least one trial that did NOT move the vehicle and one that did,
    bracketing the deadband;
  * at least two distinct levels that DID move it, to separate the
    deadband offset from the slope;
  * at least two repetitions at one level, to have any estimate of
    repeatability.

Yaw uses the ATTITUDE gyro, which is a direct rate measurement, so yaw
trials are self-sufficient. Translations need the overhead ground truth
for speed, which the early pilot trials did NOT record - that is
reported rather than silently assumed.
"""

from __future__ import annotations

import glob
import json
import math
import os
import sys
from collections import defaultdict

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
MOTION = os.path.join(ROOT, "experiments", "real", "motion_tests")
YAWDIR = os.path.join(ROOT, "experiments", "real", "yaw_authority")

# The range the planners will actually command in the pool. Levels far
# outside it are not worth a trial: extrapolation there is never used.
OPERATING = {"surge": (0.08, 0.30), "sway": (0.08, 0.25),
             "yaw": (0.05, 0.30)}

MOVED_YAW_DEG = 3.0        # a turn below this is indistinguishable from
                           # drift over a few seconds


def load_jsonl(path):
    out = []
    with open(path) as f:
        for line in f:
            try:
                out.append(json.loads(line))
            except json.JSONDecodeError:
                pass
    return out


def analyse_motion_file(path):
    recs = load_jsonl(path)
    meta = next((r for r in recs if r.get("kind") == "meta"), None)
    if meta is None:
        return None
    axis = str(meta.get("seq", "")).replace("step_", "")
    base = axis.rstrip("+-") or axis
    sign = "+" if axis.endswith("+") else ("-" if axis.endswith("-")
                                           else "+")
    att = [r for r in recs if r.get("kind") == "ATTITUDE"]
    pulse_t = next((r["t"] for r in recs
                    if r.get("kind") == "cmd" and isinstance(
                        r.get("pulse"), dict)), None)
    end_t = next((r["t"] for r in recs
                  if r.get("kind") == "cmd" and r.get("pulse_end")), None)
    turned = None
    if att and pulse_t and end_t:
        during = [a for a in att if pulse_t <= a["t"] <= end_t]
        if len(during) > 2:
            acc = 0.0
            prev = during[0]["yaw"]
            for a in during[1:]:
                d = a["yaw"] - prev
                acc += abs(math.atan2(math.sin(d), math.cos(d)))
                prev = a["yaw"]
            turned = math.degrees(acc)
    return {"file": os.path.basename(path), "axis": base, "sign": sign,
            "power": meta.get("power"), "dur": meta.get("dur"),
            "turned_deg": None if turned is None else round(turned, 1),
            "has_overhead_gt": False}


def main() -> int:
    trials = defaultdict(list)
    for p in sorted(glob.glob(os.path.join(MOTION, "*.jsonl"))):
        r = analyse_motion_file(p)
        if r and r["axis"] in ("surge", "sway", "yaw"):
            trials[(r["axis"], r["sign"])].append(r)
    # CONTAMINATED SESSION. The 18:28 yaw authority sweep was run while
    # the vehicle was grounded on the shallow bottom (operator: "il rover
    # era bloccato dal fondale"), and the numbers show it: 140 deg at
    # 15 % authority but 2.5 deg at 30 % and 2.8 deg at 45 %. A monotone
    # actuator cannot do that, so the sweep measures the grounding, not
    # the vehicle. Excluded from identifiability accounting.
    CONTAMINATED = ("yaw_authority_20260814_182830.json",)
    for p in sorted(glob.glob(os.path.join(YAWDIR, "*.json"))):
        if os.path.basename(p) in CONTAMINATED:
            print(f"[excluded] {os.path.basename(p)}: vehicle grounded "
                  "during the sweep; non-monotone response")
            continue
        with open(p) as f:
            for rec in json.load(f):
                trials[("yaw", "+")].append({
                    "file": os.path.basename(p), "axis": "yaw", "sign": "+",
                    "power": rec.get("level"), "dur": 3.0,
                    "turned_deg": rec.get("turned_deg"),
                    "rate_deg_s": rec.get("rate_deg_s"),
                    "coast_deg": rec.get("coast_deg_3s"),
                    "has_overhead_gt": False})

    print("EXISTING TRIALS")
    missing = []
    for axis in ("surge", "sway", "yaw"):
        lo, hi = OPERATING[axis]
        for sign in ("+", "-"):
            ts = trials.get((axis, sign), [])
            in_range = [t for t in ts if t.get("power") is not None
                        and lo <= t["power"] <= hi]
            moved, still = [], []
            for t in ts:
                if axis == "yaw" and t.get("turned_deg") is not None:
                    (moved if t["turned_deg"] >= MOVED_YAW_DEG
                     else still).append(t)
            levels = sorted({t["power"] for t in ts
                             if t.get("power") is not None})
            print(f"\n  {axis}{sign}: {len(ts)} trial(s), "
                  f"levels {levels}, {len(in_range)} inside the operating "
                  f"range {lo}-{hi}")
            for t in ts:
                extra = ""
                if t.get("turned_deg") is not None:
                    extra = f"turned {t['turned_deg']:.1f} deg"
                if t.get("rate_deg_s") is not None:
                    extra += f", {t['rate_deg_s']:.1f} deg/s"
                print(f"      {t['file']:38s} p={t.get('power')} "
                      f"dur={t.get('dur')} {extra}")

            need = []
            if axis == "yaw":
                if not moved:
                    need.append("a level that visibly turns the vehicle")
                if not still:
                    need.append("a level that does NOT turn it "
                                "(deadband bracket)")
                if len({t["power"] for t in moved}) < 2:
                    need.append("a second distinct level that turns it "
                                "(separates offset from slope)")
                reps = defaultdict(int)
                for t in moved:
                    reps[t["power"]] += 1
                if not any(v >= 2 for v in reps.values()):
                    need.append("one repetition at a moving level "
                                "(repeatability)")
            else:
                need.append("overhead ground-truth speed: the recorded "
                            "translation trials logged attitude only, so "
                            "no speed is recoverable from them")
                if len(levels) < 2:
                    need.append("at least two distinct levels")
            if need:
                missing.append((f"{axis}{sign}", need))
                for n in need:
                    print(f"      MISSING: {n}")
            else:
                print("      SUFFICIENT: parameters identifiable from "
                      "existing data")

    print("\n\nSHORTEST REMAINING EXPERIMENT")
    if not missing:
        print("  none: every axis is already identifiable.")
    for axis, need in missing:
        print(f"  {axis}: {len(need)} gap(s)")
        for n in need:
            print(f"     - {n}")
    print("\nNote: yaw has a dedicated authority sweep (4 levels with "
          "rate and coast); surge has one step; SWAY has only short "
          "cross-coupling pulses and no speed measurement at all, so "
          "sway is the real gap.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
