"""Freeze the simulated PREDICTIONS of the Phase-10 campaign.

These 80 runs are predictions, not results: they are produced and hashed
BEFORE the 20 real runs exist, so the sim-to-real comparison cannot be
tuned after the fact. That ordering is the whole reason the comparison
means anything, and a hash is what makes it checkable rather than
promised.

The script also VERIFIES the predictions before freezing them, because a
prediction set with a silently uncalibrated run in it is worse than none:

  * every level has the full 2 planners x 2 geometries x 5 repetitions;
  * every run's relay sentinel and plant sentinel report the level that
    was requested -- proof that the calibration was actually applied,
    not merely asked for;
  * no run is a technical invalid.

Usage:  python scripts/analysis/phase10_predictions.py
"""

from __future__ import annotations

import hashlib
import json
import os
import statistics

_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
SIM = os.path.join(_ROOT, "experiments", "simulation")
LEVELS = ["S0", "S1", "S2", "S3"]
OUT = os.path.join(SIM, "phase10_predictions")

# Metrics compared against reality. Chosen before the real runs exist.
# Chosen before the real runs exist. Every one of these is defined on
# quantities BOTH planners produce and that the real runs record, so no
# comparison silently rests on a metric only one side has: the earlier
# set included two derived from planner-internal states, which the DWA
# node never publishes.
METRICS = [
    ("min_clearance_m", "distanza minima"),
    ("lateral_commit_dist_m", "distanza all'ingaggio laterale"),
    ("max_lat_dev_m", "escursione laterale"),
    ("path_length_m", "lunghezza percorso"),
    ("lateral_maneuver_s", "durata manovra laterale"),
    ("lateral_peak_m_s", "picco di velocita laterale"),
]


def load_level(level):
    path = os.path.join(SIM, "phase10_%s" % level, "manifest.json")
    with open(path) as f:
        return json.load(f).get("results", [])


def sentinel(run_dir, name):
    try:
        with open(os.path.join(run_dir, name)) as f:
            return json.load(f)
    except (OSError, json.JSONDecodeError):
        return None


def verify(level, results):
    """Every claim about a run is checked against the run's own files."""
    problems = []
    if len(results) != 20:
        problems.append("%d run invece di 20" % len(results))
    seen = set()
    for r in results:
        key = (r.get("scenario"), r.get("planner"), r.get("run"))
        seen.add(key)
        rd = os.path.join(SIM, "phase10_%s" % level, "runs",
                          "%s_%s_%d" % key)
        if r.get("technical_invalid"):
            problems.append("%s: tecnicamente invalida" % (key,))
        rel = sentinel(rd, "relay_status.json")
        pla = sentinel(rd, "plant_status.json")
        if not rel or rel.get("level") != level:
            problems.append("%s: relay riporta %s" % (key, rel))
        if not pla or pla.get("level") != level:
            problems.append("%s: plant riporta %s" % (key, pla))
        # The plant must be active at S3 and inactive below it.
        if pla and bool(pla.get("plant_active")) != (level == "S3"):
            problems.append("%s: plant_active=%s a %s"
                            % (key, pla.get("plant_active"), level))
        if rel:
            want_s1 = level in ("S1", "S2", "S3")
            want_s2 = level in ("S2", "S3")
            if bool(rel.get("s1_loaded")) != want_s1:
                problems.append("%s: s1_loaded=%s a %s"
                                % (key, rel.get("s1_loaded"), level))
            if bool(rel.get("s2_loaded")) != want_s2:
                problems.append("%s: s2_loaded=%s a %s"
                                % (key, rel.get("s2_loaded"), level))
    if len(seen) != 20:
        problems.append("tuple duplicate o mancanti: %d distinte" % len(seen))
    return problems


def stat(values):
    v = [x for x in values if isinstance(x, (int, float))]
    if not v:
        return None
    return {"n": len(v), "median": round(statistics.median(v), 4),
            "mean": round(statistics.mean(v), 4),
            "sd": (round(statistics.stdev(v), 4) if len(v) > 1 else 0.0),
            "min": round(min(v), 4), "max": round(max(v), 4)}


def main() -> int:
    os.makedirs(OUT, exist_ok=True)
    all_results, problems = {}, {}
    for lv in LEVELS:
        res = load_level(lv)
        all_results[lv] = res
        p = verify(lv, res)
        problems[lv] = p
        print("%s: %d run, %s" % (lv, len(res),
                                  "OK" if not p else "%d PROBLEMI" % len(p)))
        for x in p[:6]:
            print("    ", x)

    if any(problems.values()):
        print("\nLe previsioni NON sono congelabili finche' questi "
              "problemi restano.")
        return 2

    # ---- aggregate -----------------------------------------------------
    pred = {}
    for lv in LEVELS:
        for r in all_results[lv]:
            a = r.get("assessment") or {}
            key = "%s|%s|%s" % (lv, r["scenario"], r["planner"])
            pred.setdefault(key, {"runs": []})["runs"].append(a)
    summary = {}
    for key, blk in pred.items():
        rows = blk["runs"]
        s = {"n": len(rows),
             "collisions": sum(1 for a in rows if a.get("collision")),
             "returned": sum(1 for a in rows if a.get("returned")),
             "aborted": sum(1 for a in rows
                            if a.get("maneuver_aborted_to_normal"))}
        for m, _ in METRICS:
            s[m] = stat([a.get(m) for a in rows])
        summary[key] = s

    # ---- tables --------------------------------------------------------
    lines = ["# Phase 10 — previsioni simulate (80 run)", "",
             "Generate e congelate PRIMA dei 20 run reali.", ""]
    for m, label in METRICS:
        lines += ["## %s (mediana, m o s)" % label, "",
                  "| geometria | planner | S0 | S1 | S2 | S3 |",
                  "|---|---|---|---|---|---|"]
        for scen in ("K0", "K1"):
            for pl in ("committed", "dwa"):
                cells = []
                for lv in LEVELS:
                    st = summary.get("%s|%s|%s" % (lv, scen, pl), {}).get(m)
                    cells.append("%.3f" % st["median"] if st else "-")
                lines.append("| %s | %s | %s |"
                             % (scen, pl, " | ".join(cells)))
        lines.append("")
    lines += ["## collisioni e ritorni in rotta", "",
              "| geometria | planner | livello | collisioni | ritorni | "
              "manovre abortite |", "|---|---|---|---|---|---|"]
    for scen in ("K0", "K1"):
        for pl in ("committed", "dwa"):
            for lv in LEVELS:
                s = summary.get("%s|%s|%s" % (lv, scen, pl))
                if s:
                    lines.append("| %s | %s | %s | %d/%d | %d/%d | %d |"
                                 % (scen, pl, lv, s["collisions"], s["n"],
                                    s["returned"], s["n"], s["aborted"]))
    table = "\n".join(lines) + "\n"

    payload = {"levels": LEVELS, "metrics": [m for m, _ in METRICS],
               "design": "2 planners x 2 geometries x 5 repetitions "
                         "per level",
               "summary": summary,
               "runs": {lv: [{"scenario": r["scenario"],
                              "planner": r["planner"], "run": r["run"],
                              "assessment": r.get("assessment"),
                              "launch_args": r.get("launch_args")}
                             for r in all_results[lv]] for lv in LEVELS}}
    body = json.dumps(payload, indent=2, sort_keys=True)
    digest = hashlib.sha256(body.encode()).hexdigest()

    with open(os.path.join(OUT, "predictions.json"), "w") as f:
        f.write(body)
    with open(os.path.join(OUT, "predictions.md"), "w") as f:
        f.write(table)
    with open(os.path.join(OUT, "PREDICTIONS.sha256"), "w") as f:
        f.write("%s  predictions.json\n" % digest)

    print("\nsha256 delle previsioni: %s" % digest)
    print("->", OUT)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
