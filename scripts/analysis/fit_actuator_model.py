"""Fit the S3 vehicle profile from the recorded actuator trials.

ESTIMATOR. Speed is fitted by least squares on the POWERED SEGMENT of
the ground-truth track, not from the difference between the endpoints.
The endpoint difference was measured after the coast, so it mixed three
things -- the powered motion, the deceleration, and four more seconds of
pool drift -- and it is the quantity that produced non-monotone sway
speeds (1000 counts appearing slower than 600). Fitting the slope while
the thrusters are actually running, after the rise transient, uses the
same recordings and is far less sensitive to a slowly varying
disturbance.

DRIFT. Each trial carries the drift measured immediately before it. The
pool disturbance is thruster-induced recirculation and is NOT stationary
-- two measurements minutes apart pointed in opposite directions -- so
the local vector is subtracted, never a global mean.

TRIALS EXCLUDED FROM THE FIT (recorded, never silently dropped):
  * pulse shorter than 1.5 s: a run cut off at the frame edge divides a
    displacement by a fraction of a second and reported 5.4 m/s;
  * drift comparable to the signal;
  * WALL CONTACT: the operator observed the vehicle touching the pool
    wall during 20260815_105604 surge +1000, so that displacement is not
    free motion. It is kept with `wall_contact: true` and excluded.

Yaw is fitted from the IMU, which the pool drift does not affect, and is
therefore the best-identified axis here.

Usage:  python scripts/analysis/fit_actuator_model.py
"""

from __future__ import annotations

import glob
import json
import os
import sys

import numpy as np

_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
SESSIONS = os.path.join(_ROOT, "experiments", "real", "actuator_id")
PX_PER_M = 452.8

# Operator-reported wall contact: (session, axis, level).
WALL_CONTACT = {("20260815_105604", "surge", 1000)}


def fit_speed(trial):
    """Speed over the powered segment, drift removed. m/s, or None."""
    track = [p for p in trial.get("track", []) if p.get("powered")]
    if len(track) < 8:
        return None, "too few samples"
    t = np.array([p["t"] for p in track])
    held = float(t.max())
    if held < 1.5:
        return None, "pulse too short"
    # Drop the rise transient: the first-order response needs time to
    # reach steady speed, and including it biases the slope low.
    keep = t >= 0.45 * held
    if keep.sum() < 6:
        return None, "too few steady samples"
    t = t[keep]
    x = np.array([p["x"] for p in track])[keep]
    y = np.array([p["y"] for p in track])[keep]
    A = np.vstack([t, np.ones_like(t)]).T
    vx, _ = np.linalg.lstsq(A, x, rcond=None)[0]
    vy, _ = np.linalg.lstsq(A, y, rcond=None)[0]
    dx, dy = trial.get("drift_px_s", [0.0, 0.0])
    v = np.array([vx - dx, vy - dy])
    # residual scatter about the fitted line, as a quality measure
    rx = x - (vx * t + np.linalg.lstsq(A, x, rcond=None)[0][1])
    resid_px = float(np.sqrt(np.mean(rx ** 2)))
    speed = float(np.linalg.norm(v)) / PX_PER_M
    drift_mag = float(np.hypot(dx, dy)) / PX_PER_M
    return {"speed_m_s": round(speed, 4),
            "drift_m_s": round(drift_mag, 4),
            "drift_ratio": round(drift_mag / max(speed, 1e-6), 2),
            "resid_px": round(resid_px, 1),
            "n": int(keep.sum()), "held_s": round(held, 2)}, None


def main() -> int:
    rows = []
    for path in sorted(glob.glob(os.path.join(SESSIONS, "*", "trials.json"))):
        sess = os.path.basename(os.path.dirname(path))
        with open(path) as f:
            data = json.load(f)
        for tr in data["trials"]:
            if tr.get("aborted"):
                continue
            axis, level = tr["axis"], tr["level"]
            wall = (sess, axis, level) in WALL_CONTACT
            fit, why = fit_speed(tr)
            row = {"session": sess, "axis": axis, "level": level,
                   "end_reason": tr.get("end_reason"),
                   "wall_contact": wall,
                   "dyaw_deg": tr.get("dyaw_deg"),
                   "yaw_rate_deg_s": tr.get("yaw_rate_deg_s")}
            if fit is None:
                row["excluded"] = why
            else:
                row.update(fit)
                if wall:
                    row["excluded"] = "wall contact (operator observed)"
                elif fit["drift_ratio"] > 0.6:
                    row["excluded"] = "drift comparable to signal"
            rows.append(row)

    print("%-16s %-6s %+6s %8s %8s %6s  %s"
          % ("sessione", "asse", "liv", "v m/s", "deriva", "rap.", "stato"))
    for r in sorted(rows, key=lambda r: (r["axis"], r["level"])):
        state = r.get("excluded", "USATO")
        print("%-16s %-6s %+6d %8s %8s %6s  %s"
              % (r["session"], r["axis"], r["level"],
                 r.get("speed_m_s", "-"), r.get("drift_m_s", "-"),
                 r.get("drift_ratio", "-"), state))

    # ---- per signed axis summary -------------------------------------
    print("\n--- traslazione: prove utilizzabili ---")
    profile = {}
    for axis in ("surge", "sway"):
        for sign in (+1, -1):
            use = [r for r in rows
                   if r["axis"] == axis and np.sign(r["level"]) == sign
                   and "excluded" not in r]
            key = "%s%s" % (axis, "+" if sign > 0 else "-")
            if not use:
                print("%-8s NON IDENTIFICATO (0 prove valide)" % key)
                profile[key] = {"identified": False,
                                "reason": "no trial survived the drift and "
                                          "frame-size limits"}
                continue
            pts = [(abs(r["level"]), r["speed_m_s"]) for r in use]
            print("%-8s %d prove: %s" % (key, len(pts), pts))
            if len({p[0] for p in pts}) >= 2:
                L = np.array([p[0] for p in pts], float)
                V = np.array([p[1] for p in pts], float)
                A = np.vstack([L, np.ones_like(L)]).T
                k, b = np.linalg.lstsq(A, V, rcond=None)[0]
                db = -b / k if k > 1e-9 else None
                # A NEGATIVE deadband is not a small deadband: it means
                # the fitted line predicts motion at zero command, which
                # the vehicle cannot do. With two command levels and this
                # much scatter the intercept is simply not resolved, so
                # it is clamped to zero and FLAGGED rather than passed to
                # the simulator, where a negative deadband would make the
                # simulated vehicle creep with the thrusters idle.
                bad_db = db is not None and db < 0
                if bad_db:
                    # Refit through the origin rather than keep a slope
                    # that was fitted alongside an impossible intercept:
                    # the constraint deadband >= 0 is physics, and the
                    # unconstrained slope inherits the error the bad
                    # intercept absorbed.
                    k = float(np.sum(L * V) / np.sum(L * L))
                profile[key] = {"identified": True,
                                "m_s_per_count": round(float(k), 6),
                                "deadband_counts": (0 if bad_db else
                                                    None if db is None
                                                    else round(float(db))),
                                "deadband_resolved": not bad_db,
                                "n_trials": len(pts), "points": pts}
                if bad_db:
                    profile[key]["note"] = (
                        "deadband not separable from these levels (free fit "
                        "gave a non-physical %.0f counts); refitted through "
                        "the origin with deadband 0" % db)
                print("         pendenza %.5f m/s per conteggio, "
                      "banda morta ~%s conteggi%s"
                      % (k, "n/d" if db is None else round(db),
                         "  <- NON FISICA, azzerata e segnalata"
                         if bad_db else ""))
            else:
                # One command level cannot separate slope from deadband,
                # but it does pin the slope IF the deadband is assumed
                # zero. That assumption is stated, not hidden, and the
                # other three axes support it: their free fits all drove
                # the intercept to zero or below.
                L = np.array([p[0] for p in pts], float)
                V = np.array([p[1] for p in pts], float)
                k = float(np.sum(L * V) / np.sum(L * L))
                profile[key] = {"identified": True,
                                "m_s_per_count": round(k, 6),
                                "deadband_counts": 0,
                                "deadband_resolved": False,
                                "note": "single command level: slope fitted "
                                        "through the origin, deadband "
                                        "ASSUMED zero and not measured",
                                "n_trials": len(pts), "points": pts}
                print("         un solo livello: pendenza %.5f attraverso "
                      "l'origine, banda morta ASSUNTA nulla" % k)

    # ---- yaw, from the IMU -------------------------------------------
    print("\n--- imbardata (IMU, immune alla deriva) ---")
    for sign in (+1, -1):
        pts = sorted({(abs(r["level"]), r["yaw_rate_deg_s"]) for r in rows
                      if r["axis"] == "yaw"
                      and r.get("yaw_rate_deg_s") is not None
                      and np.sign(r["level"]) == sign})
        key = "yaw%s" % ("+" if sign > 0 else "-")
        if len(pts) >= 2:
            L = np.array([p[0] for p in pts], float)
            R = np.array([abs(p[1]) for p in pts], float)
            A = np.vstack([L, np.ones_like(L)]).T
            k, b = np.linalg.lstsq(A, R, rcond=None)[0]
            db = -b / k if k > 1e-9 else None
            profile[key] = {"identified": True,
                            "deg_s_per_count": round(float(k), 5),
                            "deadband_counts": (None if db is None
                                                else round(float(db))),
                            "points": pts}
            print("%-8s %s -> %.4f deg/s per conteggio, banda morta ~%s"
                  % (key, pts, k, "n/d" if db is None else round(db)))
        else:
            profile[key] = {"identified": False, "points": pts}
            print("%-8s dati insufficienti: %s" % (key, pts))

    out = os.path.join(_ROOT, "config", "calibration", "s3_vehicle.json")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    with open(out, "w") as f:
        json.dump({"source": "experiments/real/actuator_id/*",
                   "estimator": "least squares on the powered segment of "
                                "the overhead track, local drift removed",
                   "command_authority": 0.20,
                   "authority_warning": "ArduSub scales MANUAL_CONTROL by a "
                                        "runtime joystick gain, measured at "
                                        "0.20. A reboot resets it to "
                                        "JS_GAIN_DEFAULT (0.5) and silently "
                                        "invalidates every number here. "
                                        "Re-measure before any run.",
                   "px_per_m": PX_PER_M,
                   "profile": profile, "trials": rows}, f, indent=2)
    print("\n->", out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
