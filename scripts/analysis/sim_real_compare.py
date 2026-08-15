"""Pre-registered sim-to-real comparison.

Written and committed BEFORE the 20 real runs exist. That ordering is
what makes the comparison a test rather than a description: every
choice below -- which metrics, which statistic, which threshold, what
counts as agreement -- is fixed while the real numbers are still
unknown, so none of them can be selected afterwards for producing a
tidier answer.

WHAT IS COMPARED. For each metric, each geometry and each planner, the
median over the 5 repetitions in simulation against the median over the
5 real repetitions. Medians rather than means: with n=5 and a physical
disturbance that produced 16-second blind runs in the pilots, a single
degraded run would move a mean and not a median.

WHAT IS ASKED, in order of how much the data can support:

 1. ABSOLUTE ERROR per calibration level. Does |sim - real| shrink from
    S0 to S3? This is the paper's question and it needs no assumption
    about the shape of the distributions.

 2. MONOTONICITY. Does each added effect class improve prediction, or
    does one of them overshoot? An improvement that is not monotone is
    still a result; it says the levels interact.

 3. RANKING AGREEMENT between the two planners. With only two planners a
    rank correlation is not meaningful and is NOT reported: what is
    reported is whether the simulation puts the same planner ahead on
    the same metric as reality does, and whether the sign of the
    difference matches.

WHAT IS NOT DONE. No parameter is fitted here; the calibration was
frozen before the real runs. No metric is dropped after seeing the real
data. If a metric turns out to be uninformative -- as the commitment
distance did for DWA in simulation, saturating at the start distance --
that is reported, not quietly removed.

THE COMMITMENT THRESHOLD is applied here, offline, to the recorded
command trace, identically in both domains. It is a parameter of the
ANALYSIS, declared below, not a constant buried in the node that
produced the numbers.

Usage:
    python scripts/analysis/sim_real_compare.py \
        --real experiments/real/final_campaign
"""

from __future__ import annotations

import argparse
import json
import os
import statistics

_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
SIM = os.path.join(_ROOT, "experiments", "simulation")
LEVELS = ["S0", "S1", "S2", "S3"]
GEOMETRIES = ["K0", "K1"]
PLANNERS = ["committed", "dwa"]

# Pre-registered. A lateral command must exceed this AND persist for the
# hold time before the vehicle counts as having committed to a
# manoeuvre. The persistence requirement is what stops a planner that
# always trims laterally from registering a commitment on its first
# cycle.
COMMIT_LATERAL_M_S = 0.05
COMMIT_HOLD_S = 1.0

# LEFT CENSORING. If the vehicle is already commanding laterally at the
# FIRST recorded sample, the manoeuvre began at or before the start of
# the window and its true commitment distance is unknown: all we know is
# that it is at least the start distance. Reporting that as an exact
# measurement would be wrong in a specific and misleading direction --
# it would look like the vehicle committed exactly where the recording
# happened to begin.
#
# This is not hypothetical. In the frozen predictions DWA commits at
# 3.500 m in K0 and 3.517 m in K1 at every calibration level: exactly the
# simulated start distance, because it trims laterally from its first
# cycle. The real runs start at 1.86 m, so the same behaviour there will
# produce a censored value of >= 1.86 m, and comparing 3.50 against 1.86
# as if both were measurements would manufacture a sim-to-real error of
# 1.6 m out of two numbers that are both just "the start".
#
# The censoring threshold is LOWER than the commitment threshold on
# purpose: a lower bar for "already moving" flags more runs as censored,
# which is the conservative direction.
CENSOR_LATERAL_M_S = 0.02

# COMMON OBSERVABLE WINDOW. The simulated geometries start the vehicle
# 3.5 m from the obstacle; the real runs start at 1.86 m, because that
# is the furthest a wholly visible start fits in the overhead frame. The
# extra 1.6 m of simulated approach is not observable in reality, so a
# second view of the SAME simulated traces is computed with everything
# beyond this distance discarded. That view and the real runs cover the
# same window and can be compared directly; the frozen predictions are
# left exactly as they are and are reported alongside it.
COMMON_WINDOW_M = 1.86

METRICS = [
    ("min_clearance_m", "distanza minima", "m", "higher_is_safer"),
    ("commit_distance_m", "distanza all'ingaggio", "m", "neutral"),
    ("max_lat_dev_m", "escursione laterale", "m", "neutral"),
    ("path_length_m", "lunghezza percorso", "m", "lower_is_better"),
    ("maneuver_s", "durata manovra (span)", "s", "neutral"),
    ("maneuver_active_s", "tempo di comando laterale", "s", "neutral"),
    # CONTROL VARIABLE, not an outcome. It is measured on the frozen
    # boundary, so it is the COMMANDED lateral speed and cannot change
    # with calibration: at S3 the plant saturates downstream of it. Its
    # constancy across all four levels is evidence that the planner
    # configuration really was identical, which is the property the
    # ladder depends on.
    ("lateral_peak_m_s", "picco laterale comandato (controllo)",
     "m/s", "control"),
]


def commitment(trace):
    """(distance, time, duration, peak) at the pre-registered threshold.

    Applied to the raw command trace so simulation and reality go
    through the SAME code path. Returns None where the vehicle never
    committed, which is itself an outcome and must not be silently
    turned into a zero.
    """
    if not trace:
        return {}
    start = None
    last = None
    peak = 0.0
    active = 0.0
    prev_t = None
    # Already manoeuvring at the first sample: the commitment happened at
    # or before the window opened and its distance is a LOWER BOUND.
    censored = abs(trace[0].get("y") or 0.0) > CENSOR_LATERAL_M_S
    for s in trace:
        lat = abs(s.get("y") or 0.0)
        peak = max(peak, lat)
        if lat > COMMIT_LATERAL_M_S:
            if start is None:
                start = s
            last = s
            if prev_t is not None:
                # capped so one long gap in the trace cannot be counted
                # as continuous lateral command
                active += min(s["t"] - prev_t, 0.5)
        prev_t = s["t"]
    if start is None or last is None or \
            (last["t"] - start["t"]) < COMMIT_HOLD_S:
        return {"commit_distance_m": None, "commit_t_s": None,
                "commit_censored": censored,
                "maneuver_s": None, "maneuver_active_s": None,
                "maneuver_censored_start": censored,
                "lateral_peak_m_s": round(peak, 4)}
    return {"commit_distance_m": start.get("d"),
            "commit_t_s": round(start["t"], 3),
            # True = the value above is ">= that distance", not a
            # measurement of where the manoeuvre began.
            "commit_censored": censored,
            # The span shares the censoring: if the start is unknown the
            # duration is a lower bound too.
            "maneuver_censored_start": censored,
            # SPAN from first commitment to the last lateral command. An
            # earlier version ended the manoeuvre at the first gap longer
            # than the hold time, which under S2 -- where the vehicle is
            # blind for 2.6 s at a stretch -- chopped every manoeuvre
            # systematically and reported DWA at 1.4 s against 22.3 s at
            # S0. The gap is a property of the perception being modelled,
            # not the end of the manoeuvre.
            "maneuver_s": round(last["t"] - start["t"], 3),
            # Time actually spent commanding laterally, which separates a
            # long intermittent manoeuvre from a long continuous one.
            "maneuver_active_s": round(active, 3),
            "lateral_peak_m_s": round(peak, 4)}


def clip_to_window(trace, max_d=COMMON_WINDOW_M):
    """The part of a trace inside the commonly observable window.

    Everything before the vehicle first comes within `max_d` of the
    obstacle is discarded, so a simulated run that started at 3.5 m is
    reduced to the stretch a real run could have been watched over.
    Samples without a ground-truth distance are dropped: they carry no
    information about which side of the window they fall on.
    """
    out, inside = [], False
    for s in trace:
        d = s.get("d")
        if d is None:
            continue
        if not inside and d <= max_d:
            inside = True
        if inside:
            out.append(s)
    if not out:
        return []
    t0 = out[0]["t"]
    return [dict(s, t=round(s["t"] - t0, 3)) for s in out]


def run_metrics(assessment, trace):
    """One run's metric row, from the frozen boundary output only.

    Two views of the same trace: the full one, which reproduces the
    frozen predictions unchanged, and the common-window one, restricted
    to the stretch both domains can observe.
    """
    m = dict(commitment(trace))
    for k in ("min_clearance_m", "max_lat_dev_m", "path_length_m",
              "collision"):
        m[k] = (assessment or {}).get(k)
    clipped = clip_to_window(trace)
    # A run can miss the common window entirely: at S0 and S1 the DWA
    # planner keeps 2.1-2.5 m of clearance, so the simulated vehicle
    # never comes as close as the real one STARTS. That is not missing
    # data, it is a fact about the trajectory, and it means the two
    # domains cannot be compared on this window for that cell at all.
    m["cw_in_window"] = bool(clipped)
    for k, v in commitment(clipped).items():
        m["cw_" + k] = v
    return m


def load_sim(level):
    path = os.path.join(SIM, "phase10_%s" % level, "manifest.json")
    with open(path) as f:
        results = json.load(f)["results"]
    rows = []
    for r in results:
        rd = os.path.join(SIM, "phase10_%s" % level, "runs",
                          "%s_%s_%d" % (r["scenario"], r["planner"],
                                        r["run"]))
        trace = []
        try:
            with open(os.path.join(rd, "validation.json")) as f:
                trace = json.load(f).get("cmd_trace") or []
        except (OSError, json.JSONDecodeError):
            pass
        rows.append({"scenario": r["scenario"], "planner": r["planner"],
                     "run": r["run"],
                     **run_metrics(r.get("assessment"), trace)})
    return rows


def load_real(root):
    """Real runs, in the same shape. Absent until the pool session."""
    path = os.path.join(root, "manifest.json")
    if not os.path.isfile(path):
        return None
    with open(path) as f:
        results = json.load(f)["results"]
    rows = []
    for r in results:
        trace = r.get("cmd_trace") or []
        if not trace:
            rd = os.path.join(root, "runs", r.get("run_dir", ""))
            try:
                with open(os.path.join(rd, "cmd_trace.json")) as f:
                    trace = json.load(f)
            except (OSError, json.JSONDecodeError):
                pass
        rows.append({"scenario": r["scenario"], "planner": r["planner"],
                     "run": r["run"],
                     **run_metrics(r.get("assessment"), trace)})
    return rows


def med(rows, scen, plan, key):
    v = [r[key] for r in rows
         if r["scenario"] == scen and r["planner"] == plan
         and isinstance(r.get(key), (int, float))]
    return (statistics.median(v), len(v)) if v else (None, 0)


# Metrics whose value is a LOWER BOUND when the manoeuvre was already
# under way at the first sample, and the flag that says so.
CENSORED_BY = {"commit_distance_m": "commit_censored",
               "maneuver_s": "maneuver_censored_start",
               "cw_commit_distance_m": "cw_commit_censored",
               "cw_maneuver_s": "cw_maneuver_censored_start"}


def censored_count(rows, scen, plan, key):
    flag = CENSORED_BY.get(key)
    if flag is None:
        return 0, 0
    sel = [r for r in rows
           if r["scenario"] == scen and r["planner"] == plan
           and isinstance(r.get(key), (int, float))]
    return sum(1 for r in sel if r.get(flag)), len(sel)


def cell(rows, scen, plan, key):
    """A table cell that shows censoring instead of hiding it.

    A median taken over lower bounds is itself a lower bound. When every
    contributing run is censored the cell is prefixed ">="; when only
    some are, the count is shown, because mixing bounds and measurements
    in one median is exactly the sort of quiet averaging that turns an
    artefact into a finding.
    """
    v, n = med(rows, scen, plan, key)
    if v is None:
        if key.startswith("cw_"):
            sel = [r for r in rows if r["scenario"] == scen
                   and r["planner"] == plan]
            if sel and not any(r.get("cw_in_window") for r in sel):
                return "mai in finestra"
        return "-"
    cens, tot = censored_count(rows, scen, plan, key)
    if cens and cens == tot:
        return ">=%.3f" % v
    if cens:
        return "%.3f (%d/%d cens.)" % (v, cens, tot)
    return "%.3f" % v


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--real", default=os.path.join(
        _ROOT, "experiments", "real", "final_campaign"))
    ap.add_argument("--out", default=os.path.join(
        _ROOT, "experiments", "analysis", "sim_real"))
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    sim = {lv: load_sim(lv) for lv in LEVELS}
    real = load_real(args.real)

    lines = ["# Confronto sim-reale (pre-registrato)", "",
             "Soglia di ingaggio %.2f m/s mantenuta %.1f s, applicata "
             "offline alla stessa traccia di comando in entrambi i "
             "domini." % (COMMIT_LATERAL_M_S, COMMIT_HOLD_S), ""]

    # ---- simulation side is always reportable -------------------------
    lines += ["## Previsioni simulate (mediane su 5 ripetizioni)", "",
              "`>=` indica una misura CENSURATA A SINISTRA: il veicolo "
              "gia manovrava al primo campione, quindi l'inizio e "
              "precedente alla finestra e il valore e un limite "
              "inferiore, non una misura.", ""]
    for key, label, unit, _ in METRICS:
        lines += ["### %s (%s)" % (label, unit), "",
                  "| geometria | planner | " + " | ".join(LEVELS) + " |",
                  "|---|---|" + "---|" * len(LEVELS)]
        for g in GEOMETRIES:
            for p in PLANNERS:
                cells = [cell(sim[lv], g, p, key) for lv in LEVELS]
                lines.append("| %s | %s | %s |" % (g, p, " | ".join(cells)))
        lines.append("")

    # ---- common observable window -------------------------------------
    lines += ["## Finestra osservabile comune (<= %.2f m)"
              % COMMON_WINDOW_M, "",
              "Le STESSE tracce simulate, con tutto cio che sta oltre "
              "%.2f m scartato. La simulazione parte a 3.5 m e il reale "
              "a %.2f m, quindi 1.6 m di avvicinamento simulato non sono "
              "osservabili in vasca: confrontare l'ingaggio su finestre "
              "diverse fabbricherebbe un errore sim-reale a partire da "
              "due numeri che sono entrambi solo \"la partenza\". Le "
              "previsioni congelate restano invariate e sono riportate "
              "sopra."
              % (COMMON_WINDOW_M, COMMON_WINDOW_M), ""]
    for key, label, unit, kind in METRICS:
        cw_key = "cw_" + key
        if kind == "control" or not any(
                cw_key in r for r in sim[LEVELS[0]]):
            continue
        lines += ["### %s, finestra comune (%s)" % (label, unit), "",
                  "| geometria | planner | " + " | ".join(LEVELS) + " |",
                  "|---|---|" + "---|" * len(LEVELS)]
        for g in GEOMETRIES:
            for p in PLANNERS:
                cells = [cell(sim[lv], g, p, cw_key) for lv in LEVELS]
                lines.append("| %s | %s | %s |" % (g, p, " | ".join(cells)))
        lines.append("")

    payload = {"threshold": {"lateral_m_s": COMMIT_LATERAL_M_S,
                             "hold_s": COMMIT_HOLD_S},
               "sim": sim, "real": real}

    if real is None:
        lines += ["## Confronto con la realta", "",
                  "I 20 run reali non esistono ancora. Questo file "
                  "contiene solo le previsioni; il confronto viene "
                  "prodotto dallo stesso script, senza modifiche, "
                  "quando la campagna reale sara completa.", ""]
        print("run reali assenti: prodotte solo le previsioni")
    else:
        lines += ["## Errore assoluto di previsione |sim - reale|", "",
                  "| metrica | geometria | planner | " +
                  " | ".join(LEVELS) + " | migliora S0->S3 |",
                  "|---|---|---|" + "---|" * (len(LEVELS) + 1)]
        errors = {}
        for key, label, unit, _ in METRICS:
            for g in GEOMETRIES:
                for p in PLANNERS:
                    rv, rn = med(real, g, p, key)
                    cells, errs = [], []
                    for lv in LEVELS:
                        sv, sn = med(sim[lv], g, p, key)
                        if sv is None or rv is None:
                            cells.append("-")
                            errs.append(None)
                        else:
                            e = abs(sv - rv)
                            errs.append(e)
                            cells.append("%.3f" % e)
                    ok = (all(e is not None for e in errs)
                          and errs[-1] < errs[0])
                    errors["%s|%s|%s" % (key, g, p)] = errs
                    lines.append("| %s | %s | %s | %s | %s |"
                                 % (label, g, p, " | ".join(cells),
                                    "si" if ok else "no"))
        payload["absolute_error"] = errors

        lines += ["", "## Accordo di ordinamento fra i due planner", "",
                  "Con due soli planner una correlazione di rango non e "
                  "significativa e non viene riportata: si riporta se la "
                  "simulazione mette avanti lo stesso planner della "
                  "realta, e con lo stesso segno.", "",
                  "| metrica | geometria | reale | " +
                  " | ".join(LEVELS) + " |",
                  "|---|---|---|" + "---|" * len(LEVELS)]
        agree = {}
        for key, label, unit, _ in METRICS:
            for g in GEOMETRIES:
                rc, _ = med(real, g, "committed", key)
                rd, _ = med(real, g, "dwa", key)
                if rc is None or rd is None:
                    continue
                r_sign = "C" if rc > rd else ("D" if rd > rc else "=")
                cells = []
                for lv in LEVELS:
                    sc, _ = med(sim[lv], g, "committed", key)
                    sd, _ = med(sim[lv], g, "dwa", key)
                    if sc is None or sd is None:
                        cells.append("-")
                        continue
                    s_sign = "C" if sc > sd else ("D" if sd > sc else "=")
                    cells.append(s_sign + ("" if s_sign == r_sign
                                           else " (discorde)"))
                agree["%s|%s" % (key, g)] = {"real": r_sign,
                                             "sim": cells}
                lines.append("| %s | %s | %s | %s |"
                             % (label, g, r_sign, " | ".join(cells)))
        payload["ranking_agreement"] = agree

    with open(os.path.join(args.out, "sim_real.md"), "w",
              encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    with open(os.path.join(args.out, "sim_real.json"), "w") as f:
        json.dump(payload, f, indent=2)
    print("->", args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
