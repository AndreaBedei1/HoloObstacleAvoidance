"""Figures for the Phase-10 paper.

Written before the real runs exist, so the figures that will carry the
result are decided now rather than chosen afterwards from whichever view
flatters the data. Every figure degrades gracefully: with simulation
only it draws the predictions, and the same code adds the real series
once the pool campaign is in.

FIGURES

 1. calibration ladder -- each metric against S0..S3, one line per
    planner and geometry. This is the paper's main claim in one picture:
    if calibration helps, the simulated value should move toward the
    real one from left to right.
 2. observation model -- detection probability against true range, and
    the shared estimator against ground truth. These are measurements of
    the real sensor, not of the simulator, and they explain WHY S1
    changes the behaviour it changes.
 3. blind-run structure -- the distribution of consecutive misses,
    measured against independent dropout at the same marginal. This is
    the effect a simulator that delivers an observation every tick
    cannot reproduce at all.
 4. command traces -- lateral command and true distance against time for
    one representative run per level, which shows the manoeuvre itself
    rather than a summary of it.

Usage:  python scripts/analysis/phase10_figures.py
"""

from __future__ import annotations

import json
import os
import statistics

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
SIM = os.path.join(_ROOT, "experiments", "simulation")
CAL = os.path.join(_ROOT, "config", "calibration")
OUT = os.path.join(_ROOT, "experiments", "analysis", "figures")
LEVELS = ["S0", "S1", "S2", "S3"]
GEOM = ["K0", "K1"]
PLAN = ["committed", "dwa"]
STYLE = {("K0", "committed"): ("o-", "#1f77b4"),
         ("K0", "dwa"): ("s--", "#ff7f0e"),
         ("K1", "committed"): ("^-", "#2ca02c"),
         ("K1", "dwa"): ("v--", "#d62728")}
METRICS = [("min_clearance_m", "distanza minima (m)"),
           ("max_lat_dev_m", "escursione laterale (m)"),
           ("path_length_m", "lunghezza percorso (m)"),
           ("lateral_peak_m_s", "picco velocita laterale (m/s)")]


def load_predictions():
    path = os.path.join(SIM, "phase10_predictions", "predictions.json")
    if not os.path.isfile(path):
        return None
    with open(path) as f:
        return json.load(f)


def fig_ladder(pred):
    """One panel per metric: the prediction against calibration level."""
    fig, axes = plt.subplots(2, 2, figsize=(11, 8))
    for ax, (key, label) in zip(axes.ravel(), METRICS):
        drew = False
        for g in GEOM:
            for p in PLAN:
                ys = []
                for lv in LEVELS:
                    s = pred["summary"].get("%s|%s|%s" % (lv, g, p), {})
                    st = s.get(key)
                    ys.append(st["median"] if st else None)
                if all(y is None for y in ys):
                    continue
                mk, col = STYLE[(g, p)]
                ax.plot(range(len(LEVELS)), ys, mk, color=col,
                        label="%s %s" % (g, p), linewidth=1.8,
                        markersize=6)
                drew = True
        ax.set_xticks(range(len(LEVELS)))
        ax.set_xticklabels(LEVELS)
        ax.set_ylabel(label)
        ax.grid(alpha=0.3)
        if not drew:
            ax.text(0.5, 0.5, "nessun dato", ha="center",
                    transform=ax.transAxes)
    axes[0][0].legend(fontsize=8, loc="best")
    fig.suptitle("Previsioni simulate lungo la scala di calibrazione "
                 "(mediane su 5 ripetizioni)")
    fig.tight_layout()
    path = os.path.join(OUT, "fig1_calibration_ladder.png")
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def fig_observation():
    """What the real sensor does, which is why S1 changes anything."""
    path_fit = os.path.join(CAL, "s1_observation_fit.json")
    if not os.path.isfile(path_fit):
        return None
    with open(path_fit) as f:
        s1 = json.load(f)
    fig, (a, b) = plt.subplots(1, 2, figsize=(11, 4))

    curve = s1.get("detection_probability_vs_range") or []
    xs = [0.5 * (c["range_lo_m"] + min(c["range_hi_m"], 3.0))
          for c in curve]
    ys = [c["p_detect"] for c in curve]
    ns = [c["n"] for c in curve]
    a.bar(xs, ys, width=0.35, color="#1f77b4")
    for x, y, n in zip(xs, ys, ns):
        a.text(x, y + 0.02, "n=%d" % n, ha="center", fontsize=8)
    a.set_xlabel("distanza vera (m)")
    a.set_ylabel("probabilita di rilevamento")
    a.set_ylim(0, 1.05)
    a.set_title("Rilevamento contro distanza: non monotono")
    a.grid(alpha=0.3, axis="y")

    est = s1.get("shared_estimator_vs_truth") or {}
    if est:
        lo, hi = est.get("gt_range_m", [0.3, 2.2])
        ratio = est.get("ratio_median", 1.0)
        b.plot([lo, hi], [lo, hi], "k--", label="stima corretta")
        b.plot([lo, hi], [lo * ratio, hi * ratio], "r-", linewidth=2,
               label="misurato: %.3f x il vero" % ratio)
        b.set_xlabel("distanza vera (m)")
        b.set_ylabel("distanza dallo stimatore condiviso (m)")
        b.set_title("Errore mediano %.2f m su n=%d"
                    % (est.get("abs_error_median_m", 0.0),
                       est.get("n", 0)))
        b.legend(fontsize=8)
        b.grid(alpha=0.3)
    fig.tight_layout()
    path = os.path.join(OUT, "fig2_observation_model.png")
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def fig_bursts():
    """Blind runs: the effect an every-tick simulator cannot produce."""
    path_fit = os.path.join(CAL, "s2_timing_fit.json")
    if not os.path.isfile(path_fit):
        return None
    with open(path_fit) as f:
        s2 = json.load(f)
    fig, ax = plt.subplots(figsize=(7, 4.2))
    p = s2.get("marginal_p_accept", 0.26)
    meas = s2.get("mean_blind_run_cycles", 0.0)
    indep = 1.0 / max(p, 1e-6)
    longest = s2.get("longest_blind_run_cycles", 0)
    per = s2.get("cycle_period_s", 0.285)
    ax.bar(["indipendente", "misurato"], [indep, meas],
           color=["#999999", "#d62728"], width=0.5)
    ax.set_ylabel("tratto cieco medio (cicli)")
    ax.set_title("Le osservazioni mancano A RAFFICHE\n"
                 "p=%.2f a %.1f Hz; tratto piu lungo %d cicli = %.1f s"
                 % (p, 1.0 / per if per else 0.0, longest, longest * per))
    for i, v in enumerate([indep, meas]):
        ax.text(i, v + 0.15, "%.1f cicli = %.1f s" % (v, v * per),
                ha="center")
    ax.grid(alpha=0.3, axis="y")
    fig.tight_layout()
    path = os.path.join(OUT, "fig3_blind_runs.png")
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def fig_traces():
    """The manoeuvre itself, one representative run per level."""
    fig, axes = plt.subplots(len(LEVELS), 1, figsize=(9, 10),
                             sharex=True)
    drew = False
    for ax, lv in zip(axes, LEVELS):
        rd = os.path.join(SIM, "phase10_%s" % lv, "runs",
                          "K0_committed_1", "validation.json")
        if not os.path.isfile(rd):
            ax.text(0.5, 0.5, "%s assente" % lv, ha="center",
                    transform=ax.transAxes)
            continue
        with open(rd) as f:
            tr = json.load(f).get("cmd_trace") or []
        if not tr:
            continue
        t = [s["t"] for s in tr]
        y = [s["y"] for s in tr]
        d = [s["d"] for s in tr]
        ax.plot(t, y, color="#d62728", linewidth=1.4,
                label="comando laterale (m/s)")
        ax2 = ax.twinx()
        ax2.plot([tt for tt, dd in zip(t, d) if dd is not None],
                 [dd for dd in d if dd is not None],
                 color="#1f77b4", linewidth=1.2, alpha=0.8,
                 label="distanza vera (m)")
        ax2.set_ylabel("distanza (m)", color="#1f77b4")
        ax.set_ylabel("%s\nlaterale (m/s)" % lv, color="#d62728")
        ax.grid(alpha=0.3)
        drew = True
    axes[-1].set_xlabel("tempo (s)")
    fig.suptitle("K0, planner committed, ripetizione 1: comando "
                 "laterale e distanza vera")
    fig.tight_layout()
    path = os.path.join(OUT, "fig4_command_traces.png")
    if drew:
        fig.savefig(path, dpi=150)
    plt.close(fig)
    return path if drew else None


def main() -> int:
    os.makedirs(OUT, exist_ok=True)
    made = []
    pred = load_predictions()
    if pred:
        made.append(fig_ladder(pred))
    else:
        print("previsioni assenti: figura 1 saltata")
    for fn in (fig_observation, fig_bursts, fig_traces):
        p = fn()
        if p:
            made.append(p)
    for p in made:
        print("->", os.path.relpath(p, _ROOT))
    return 0 if made else 1


if __name__ == "__main__":
    raise SystemExit(main())
