"""Build the presentation package from the real avoidance missions.

Produces, per successful mission and as a combined summary:
  * range-vs-time plot with the trigger and the maneuver phases marked
  * overhead trajectory with the anchor and the closest approach
  * a contact sheet of the onboard detector view (approach -> pass)
  * a one-page summary figure suitable for a management presentation

Pure matplotlib/OpenCV; no vehicle interaction.
"""

import glob
import json
import os
import sys

import cv2
import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
MISSIONS = os.path.join(ROOT, "experiments", "real", "missions")
OUTDIR = os.path.join(ROOT, "visualizations", "real_mission_package")

PX_PER_M = 402.0     # overhead scale (from the metric survey)


def load(sess):
    with open(os.path.join(sess, "log.json")) as f:
        return json.load(f)


def plot_range(d, sess, out):
    log = d["log"]
    t = [r["t"] for r in log]
    rng = [r["range_m"] for r in log]
    gt = [(r["gt_dist_px"] / PX_PER_M if r.get("gt_dist_px") else None)
          for r in log]
    states = [r["state"] for r in log]
    fig, ax = plt.subplots(figsize=(9, 4.2))
    tv = [ti for ti, r in zip(t, rng) if r is not None]
    rv = [r for r in rng if r is not None]
    ax.plot(tv, rv, "o-", color="#c62828", label="camera range (monocular)")
    tg = [ti for ti, g in zip(t, gt) if g is not None]
    gv = [g for g in gt if g is not None]
    ax.plot(tg, gv, ".-", color="#1565c0", alpha=0.8,
            label="ground truth distance (overhead)")
    # phase shading
    colors = {"APPROACH": "#e8f5e9", "AVOID": "#fff8e1",
              "STRAIGHT": "#e3f2fd", "PASSED": "#f3e5f5"}
    prev, t_prev = states[0], t[0]
    for ti, st in zip(t, states):
        if st != prev:
            ax.axvspan(t_prev, ti, color=colors.get(prev, "#eee"), zorder=0)
            ax.text((t_prev + ti) / 2, ax.get_ylim()[1] * 0.97, prev,
                    ha="center", va="top", fontsize=8, color="#555")
            prev, t_prev = st, ti
    ax.axvspan(t_prev, t[-1], color=colors.get(prev, "#eee"), zorder=0)
    trig = d["summary"].get("trigger")
    if trig:
        for ti, r in zip(t, rng):
            if r is not None and abs(r - trig["range_m"]) < 1e-6:
                ax.axvline(ti, color="k", ls="--", lw=1.2)
                ax.annotate(f"TRIGGER {trig['range_m']:.2f} m\n"
                            f"bearing {trig['bearing_deg']:+.0f}°",
                            (ti, trig["range_m"]),
                            textcoords="offset points", xytext=(12, 18),
                            fontsize=9,
                            arrowprops=dict(arrowstyle="->", lw=1))
                break
    ax.set_xlabel("time (s)")
    ax.set_ylabel("distance to anchor (m)")
    ax.set_title("Real BlueROV2 — camera-based anchor avoidance")
    ax.grid(alpha=0.3)
    ax.legend(loc="upper right", fontsize=9)
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    plt.close(fig)


def contact_sheet(sess, out):
    keys = sorted(glob.glob(os.path.join(sess, "key_*.png")))
    if not keys:
        return False
    imgs = [cv2.resize(cv2.imread(k), (640, 360)) for k in keys]
    labels = ["approach", "trigger / avoid", "clearing", "passed"]
    for im, lb in zip(imgs, labels):
        cv2.rectangle(im, (0, 0), (640, 36), (0, 0, 0), -1)
        cv2.putText(im, lb, (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (255, 255, 255), 2)
    while len(imgs) < 4:
        imgs.append(np.zeros_like(imgs[0]))
    sheet = np.vstack([np.hstack(imgs[:2]), np.hstack(imgs[2:4])])
    cv2.imwrite(out, sheet)
    return True


def main():
    os.makedirs(OUTDIR, exist_ok=True)
    sessions = sorted(glob.glob(os.path.join(MISSIONS, "*")))
    rows = []
    for sess in sessions:
        if not os.path.isfile(os.path.join(sess, "log.json")):
            continue
        d = load(sess)
        s = d["summary"]
        tag = os.path.basename(sess)
        ok = s.get("final_state") == "PASSED" and not s.get("aborted")
        rows.append({
            "session": tag, "passed": ok,
            "trigger_m": (s.get("trigger") or {}).get("range_m"),
            "bearing_deg": (s.get("trigger") or {}).get("bearing_deg"),
            "min_gt_px": s.get("min_gt_dist_px"),
            "min_gt_m": (round(s["min_gt_dist_px"] / PX_PER_M, 2)
                         if s.get("min_gt_dist_px") else None),
            "detect_rate": s.get("accepted_frac"),
            "samples": s.get("samples"),
        })
        if ok and d["log"]:
            plot_range(d, sess, os.path.join(OUTDIR, f"range_{tag}.png"))
            contact_sheet(sess, os.path.join(OUTDIR, f"sheet_{tag}.png"))
            src = os.path.join(sess, "overhead_trajectory.png")
            if os.path.isfile(src):
                img = cv2.imread(src)
                cv2.imwrite(os.path.join(OUTDIR, f"traj_{tag}.png"), img)
            src = os.path.join(sess, "onboard_detector.mp4")
            if os.path.isfile(src):
                import shutil
                shutil.copy(src, os.path.join(OUTDIR, f"video_{tag}.mp4"))
    with open(os.path.join(OUTDIR, "missions_summary.json"), "w") as f:
        json.dump(rows, f, indent=2)
    print(f"{'session':18s} {'passed':7s} {'trig m':7s} {'brg':6s} "
          f"{'minGT m':8s} {'det%':5s}")
    for r in rows:
        print(f"{r['session']:18s} {str(r['passed']):7s} "
              f"{str(r['trigger_m']):7s} {str(r['bearing_deg']):6s} "
              f"{str(r['min_gt_m']):8s} "
              f"{'' if r['detect_rate'] is None else round(100*r['detect_rate'])}")
    print("package ->", OUTDIR)
    return 0


if __name__ == "__main__":
    sys.exit(main())
