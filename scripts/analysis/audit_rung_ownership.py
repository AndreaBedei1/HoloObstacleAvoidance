"""Verify that each calibration rung changes ONLY what it owns.

The S0 -> S3 ladder is only interpretable if every step adds exactly one
effect class. If a rung quietly also changed the scenario geometry, a
planner gain or the engagement distance, then any difference between two
levels could be attributed to the wrong cause -- and nothing in the
results would reveal it. An earlier version of S3 did precisely that: it
lowered the planner's sway limit, changing the controller and the
simulator at the same rung.

This audit reads the launch invocation recorded with every run and
checks, level by level, that the only arguments which differ are the
ones that level owns. It fails loudly rather than reporting a caveat,
because a violated ladder is not a weaker result: it is a different
experiment.

Usage:  python scripts/analysis/audit_rung_ownership.py <campaign_root>
"""

from __future__ import annotations

import json
import os
import sys

# What each rung is allowed to change with respect to the previous one.
OWNED = {
    "S0": {"calibration_level"},
    "S1": {"calibration_level", "s1_fit_path"},
    "S2": {"calibration_level", "s1_fit_path", "s2_fit_path"},
    "S3": {"calibration_level", "s1_fit_path", "s2_fit_path",
           "s3_fit_path"},
}
LEVELS = ["S0", "S1", "S2", "S3"]


def load(root: str, level: str):
    path = os.path.join(root, "phase10_%s" % level, "manifest.json")
    if not os.path.isfile(path):
        return None
    with open(path) as f:
        return json.load(f).get("results", [])


def args_by_scenario(results):
    """Launch arguments per (scenario, planner), as {key: value}."""
    out = {}
    for r in results:
        if not r.get("launch_args"):
            continue
        key = (r["scenario"], r["planner"])
        d = {}
        for a in r["launch_args"]:
            k, _, v = a.partition(":=")
            d[k] = v
        prev = out.get(key)
        if prev is not None and prev != d:
            raise SystemExit(
                "INCONSISTENT within a level: %s %s has two different "
                "launch configurations" % key)
        out[key] = d
    return out


def main() -> int:
    root = (sys.argv[1] if len(sys.argv) > 1
            else os.path.join(os.path.dirname(os.path.abspath(__file__)),
                              "..", "..", "experiments", "simulation"))
    per_level = {}
    for lv in LEVELS:
        res = load(root, lv)
        if res is None:
            print("%s: manifest assente" % lv)
            continue
        per_level[lv] = args_by_scenario(res)
        print("%s: %d run, %d configurazioni scenario/planner"
              % (lv, len(res), len(per_level[lv])))

    if len(per_level) < 2:
        print("\nservono almeno due livelli per l'audit")
        return 1

    ok = True
    present = [lv for lv in LEVELS if lv in per_level]
    base_lv = present[0]
    for lv in present[1:]:
        allowed = OWNED[lv]
        print("\n--- %s contro %s ---" % (lv, base_lv))
        for key in sorted(set(per_level[base_lv]) & set(per_level[lv])):
            a, b = per_level[base_lv][key], per_level[lv][key]
            changed = {k for k in set(a) | set(b) if a.get(k) != b.get(k)}
            illegal = changed - allowed
            label = "%s/%s" % key
            if illegal:
                ok = False
                print("  VIOLAZIONE %-18s cambia anche: %s"
                      % (label, ", ".join(sorted(illegal))))
                for k in sorted(illegal):
                    print("      %s: %r -> %r" % (k, a.get(k), b.get(k)))
            else:
                print("  ok %-18s cambia solo: %s"
                      % (label, ", ".join(sorted(changed)) or "nulla"))

    # The pool benchmark must be identical everywhere: it defines the
    # matched experiment and is not itself a calibration rung.
    print("\n--- geometria e configurazione di vasca, identiche ovunque ---")
    frozen = ("engage_distance_m", "nominal_surge",
              "target_obstacle_height_m", "dwa_obstacle_radius_m",
              "dwa_goal_lookahead_m")
    for key in sorted(per_level[base_lv]):
        vals = {lv: per_level[lv].get(key, {}) for lv in present
                if key in per_level[lv]}
        for k in frozen:
            seen = {lv: v.get(k) for lv, v in vals.items()}
            if len(set(seen.values())) > 1:
                ok = False
                print("  VIOLAZIONE %s/%s: %s differisce fra livelli: %s"
                      % (key[0], key[1], k, seen))
    if ok:
        print("  tutte le configurazioni di vasca coincidono")

    print("\nAUDIT:", "SUPERATO" if ok else "FALLITO")
    return 0 if ok else 2


if __name__ == "__main__":
    raise SystemExit(main())
