"""Actuator identification for S3, with the overhead camera as truth.

WHY THIS EXISTS SEPARATELY FROM actuator_id.py. Two defects invalidated
every earlier attempt, and both are corrected here.

1. LAGGED TELEMETRY. `recv_match` pops the OLDEST queued message, so a
   fast stream read in a slow loop returns state from seconds ago. A
   step response fitted against it yields a wrong time constant with no
   outward sign of error. Every read here uses `RovLink.newest`.

2. SIGNAL BELOW DRIFT. At 30 % for 2 s the vehicle moved about as far as
   it drifts unpowered, so the "identified" speed was mostly drift. The
   drift is therefore MEASURED FIRST, armed and commanded neutral, and
   subtracted; the ratio is reported so a contaminated trial is visible
   instead of assumed away.

COMMAND AUTHORITY IS PART OF THE CALIBRATION. ArduSub scales
MANUAL_CONTROL by a runtime joystick gain. On 2026-08-15 that gain was
0.20, so a full command reached only 20 % of thruster authority. The
operator judged that motion correct for a 6 m pool, so it is kept and
folded into the calibration: this script fits command counts -> m/s
directly. The gain is MEASURED and recorded in every output file, and it
must be re-checked before the validation runs, because a reboot resets
it to JS_GAIN_DEFAULT (0.5) and would silently invalidate these numbers.

The full track is sampled during the pulse AND during the coast, so the
steady speed, the rise constant and the decay all come from the same
trial instead of from dedicated runs.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")

from camera_stream import ensure_env  # noqa: E402

ensure_env()

from overhead_track import OverheadTracker  # noqa: E402
from rovlink import RovLink, Z_NEUTRAL  # noqa: E402
from pymavlink import mavutil  # noqa: E402

PX_PER_M = 452.8            # from the frozen pool remap
# Frame is 1920x1080 and the vehicle blob is about 200x226 px, so the
# CENTRE must stay this far from the edges to stay wholly visible.
XMIN, XMAX, YMIN, YMAX = 150, 1770, 150, 920

OUT = os.path.join(_ROOT, "experiments", "real", "actuator_id")


def in_frame(p) -> bool:
    return (p is not None and XMIN < p[0] < XMAX and YMIN < p[1] < YMAX)


def in_frame_soft(p) -> bool:
    """Whole blob still visible, ignoring the working margin.

    The start gate uses this, not `in_frame`: the vehicle drifts several
    hundred pixels between runs, and refusing to start merely because it
    sits in the outer margin would stall the experiment when recentring
    can fix it in a few seconds.
    """
    return (p is not None and 115 < p[0] < 1805 and 115 < p[1] < 965)


class Truth:
    """Overhead ground truth, averaged to beat the per-frame jitter."""

    def __init__(self):
        self.tr = OverheadTracker()

    def one(self):
        d = self.tr.detect()
        return np.array(d["pixel"], float) if (d and d.get("found")) else None

    def mean(self, n=5):
        pts = [p for p in (self.one() for _ in range(n)) if p is not None]
        return np.mean(pts, axis=0) if pts else None

    def close(self):
        self.tr.close()


def measure_gain(rov) -> float:
    """Effective command authority: |servo deflection| at full command
    divided by the 400 us half-range. Recorded with every dataset."""
    rov.set_message_interval(
        mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 10)
    t0, best = time.time(), 0
    while time.time() - t0 < 2.5:
        rov.manual(x=1000, z=Z_NEUTRAL)
        time.sleep(0.06)
        m = rov.newest("SERVO_OUTPUT_RAW", timeout=0.0)
        if m is not None and time.time() - t0 > 1.0:
            best = max(best, abs(m.servo1_raw - 1500))
    rov.neutral()
    t1 = time.time()
    while time.time() - t1 < 1.5:
        rov.manual(z=Z_NEUTRAL)
        time.sleep(0.06)
    return round(best / 400.0, 3)


def measure_drift(rov, truth, seconds=15.0):
    """Armed, commanded neutral: whatever moves the vehicle now is not
    the actuators, and it is the error floor of every trial below."""
    track = []
    t0 = time.time()
    while time.time() - t0 < seconds:
        rov.manual(z=Z_NEUTRAL)
        p = truth.one()
        if p is not None:
            track.append((time.time() - t0, p[0], p[1]))
        time.sleep(0.05)
    if len(track) < 6:
        return np.zeros(2), track
    a = np.array(track)
    # least squares slope in px/s per axis
    t = a[:, 0]
    A = np.vstack([t, np.ones_like(t)]).T
    vx = np.linalg.lstsq(A, a[:, 1], rcond=None)[0][0]
    vy = np.linalg.lstsq(A, a[:, 2], rcond=None)[0][0]
    return np.array([vx, vy]), track


CENTRE = np.array([960.0, 540.0])


def learn_basis(rov, truth, axis, drift, level=1000, pulse_s=2.5):
    """Image-plane direction of one commanded axis, drift removed.

    The vehicle's heading in the overhead image is not known a priori
    (the compass frame has no fixed relation to the camera), so the
    direction of every axis is MEASURED rather than assumed.

    The drift MUST be subtracted here. Without it, a short pulse against
    a 0.05 m/s disturbance mostly measures the disturbance: on
    2026-08-15 surge came out as (+1.00, -0.01) and sway as (+0.87,
    +0.50) -- nearly PARALLEL, when two orthogonal thruster axes cannot
    be -- because the drift was (+23.8, -0.4) px/s and dominated both.
    Recentring then steered along fictitious directions and pushed the
    vehicle out of frame. Near-parallel axes are reported so the failure
    is visible rather than silent.
    """
    kw = {"surge": "x", "sway": "y", "yaw": "r"}[axis]
    p0 = truth.mean(4)
    if p0 is None:
        return None
    t0 = time.time()
    while time.time() - t0 < pulse_s:
        rov.manual(z=Z_NEUTRAL, **{kw: int(level)})
        time.sleep(0.05)
    rov.neutral()
    t1 = time.time()
    while time.time() - t1 < 1.5:
        rov.manual(z=Z_NEUTRAL)
        time.sleep(0.05)
    p1 = truth.mean(4)
    if p1 is None:
        return None
    d = (p1 - p0) - np.asarray(drift, float) * (time.time() - t0)
    n = float(np.linalg.norm(d))
    return None if n < 25 else d / n


def recenter(rov, truth, basis, drift=(0.0, 0.0), tol_px=160, max_s=14.0):
    """Drive the vehicle back toward the middle of the overhead frame.

    Closed loop on the ground truth, in short bursts along whichever
    learned axis best points at the error. The pool drift alone would
    carry the vehicle out of frame within a minute, so without this the
    experiment loses the vehicle after one or two trials -- which is
    exactly what happened on the first attempts.
    """
    t0 = time.time()
    while time.time() - t0 < max_s:
        p = truth.mean(3)
        if p is None:
            return False
        err = CENTRE - p
        if float(np.linalg.norm(err)) < tol_px:
            return True
        best, bdot = None, 0.0
        for ax, u in basis.items():
            d = float(np.dot(u, err / np.linalg.norm(err)))
            if abs(d) > abs(bdot):
                best, bdot = ax, d
        if best is None or abs(bdot) < 0.25:
            return False
        kw = {"surge": "x", "sway": "y"}[best]
        lvl = int(math.copysign(1000, bdot))
        t1 = time.time()
        while time.time() - t1 < 1.2:
            rov.manual(z=Z_NEUTRAL, **{kw: lvl})
            time.sleep(0.05)
        rov.neutral()
        t2 = time.time()
        while time.time() - t2 < 1.0:
            rov.manual(z=Z_NEUTRAL)
            time.sleep(0.05)
        # SELF-CORRECTING BASIS. The vehicle yaws freely between trials,
        # so an axis direction learned once goes stale and recentring
        # then pushes the vehicle the wrong way -- which is how the
        # experiment kept losing it at the frame edge. Each burst is
        # itself a measurement, so use it: update the direction from
        # what the burst actually achieved.
        p2 = truth.mean(3)
        if p2 is not None:
            moved = (p2 - p) - np.asarray(drift, float) * (time.time() - t1)
            n = float(np.linalg.norm(moved))
            if n > 40:
                basis[best] = moved / n * math.copysign(1, bdot)
    return True


def trial(rov, truth, axis, level, pulse_s, coast_s, drift,
          local_drift_s=5.0):
    """One pulse: sample the track during the pulse and the coast.

    The drift is re-measured IMMEDIATELY BEFORE this pulse, not taken
    from a single global estimate. In a pool this small the disturbance
    is thruster-induced recirculation, and it is not stationary: two
    measurements minutes apart gave (-9, +14) and (+15, -6) px/s, i.e.
    opposite directions. Subtracting a global mean therefore removes the
    wrong vector and can make a trial worse. The local window doubles as
    the settling pause after recentring.
    """
    kw = {"surge": "x", "sway": "y", "yaw": "r"}[axis]
    if local_drift_s > 0:
        drift_local, _ = measure_drift(rov, truth, local_drift_s)
        if float(np.linalg.norm(drift_local)) > 0.0:
            drift = drift_local
    p_start = truth.mean(4)
    a0 = rov.newest("ATTITUDE", timeout=1.0)
    if not in_frame(p_start):
        return {"axis": axis, "level": level, "end_reason": "start_at_edge",
                "aborted": True}
    track, end = [], "duration"
    t0 = time.time()
    while time.time() - t0 < pulse_s:
        rov.manual(z=Z_NEUTRAL, **{kw: int(level)})
        p = truth.one()
        a = rov.newest("ATTITUDE", timeout=0.0)
        if p is not None:
            track.append({"t": round(time.time() - t0, 3),
                          "x": round(float(p[0]), 1),
                          "y": round(float(p[1]), 1),
                          "yaw": (round(float(a.yaw), 4) if a else None),
                          "powered": True})
            if not in_frame(p):
                end = "frame_edge"
                break
            if np.linalg.norm(p - p_start) / PX_PER_M > 0.9:
                end = "displacement_limit"
                break
        time.sleep(0.04)
    held = time.time() - t0
    rov.neutral()
    t1 = time.time()
    while time.time() - t1 < coast_s:      # decay, same trial
        rov.manual(z=Z_NEUTRAL)
        p = truth.one()
        a = rov.newest("ATTITUDE", timeout=0.0)
        if p is not None:
            track.append({"t": round(held + time.time() - t1, 3),
                          "x": round(float(p[0]), 1),
                          "y": round(float(p[1]), 1),
                          "yaw": (round(float(a.yaw), 4) if a else None),
                          "powered": False})
        time.sleep(0.04)
    p_end = truth.mean(4)
    a1 = rov.newest("ATTITUDE", timeout=1.0)

    rec = {"axis": axis, "level": int(level), "pulse_s": round(held, 2),
           "coast_s": coast_s, "end_reason": end, "aborted": False,
           "px_start": None if p_start is None else
           [round(float(v), 1) for v in p_start],
           "px_end": None if p_end is None else
           [round(float(v), 1) for v in p_end],
           "drift_px_s": [round(float(v), 3) for v in drift],
           "track": track}
    if p_start is not None and p_end is not None:
        elapsed = held + coast_s
        raw = p_end - p_start
        net = raw - drift * elapsed
        rec["disp_raw_m"] = round(float(np.linalg.norm(raw)) / PX_PER_M, 4)
        rec["disp_net_m"] = round(float(np.linalg.norm(net)) / PX_PER_M, 4)
        rec["disp_net_px"] = [round(float(v), 1) for v in net]
        rec["speed_m_s"] = round(rec["disp_net_m"] / max(held, 0.1), 4)
        drift_m = float(np.linalg.norm(drift)) * elapsed / PX_PER_M
        rec["drift_share"] = round(drift_m / max(rec["disp_net_m"], 1e-6), 2)
        # A pulse cut short at the frame edge divides a full displacement
        # by a fraction of a second and reports an impossible speed
        # (5.4 m/s was produced this way). Such trials are recorded but
        # flagged: they inform the deadband, not the slope.
        rec["speed_valid"] = bool(held >= 1.5 and rec["drift_share"] < 0.6)
        if not rec["speed_valid"]:
            rec["invalid_reason"] = ("pulse too short" if held < 1.5
                                     else "drift dominates")
    if a0 is not None and a1 is not None:
        d = math.degrees(a1.yaw - a0.yaw)
        rec["dyaw_deg"] = round((d + 180) % 360 - 180, 1)
        rec["yaw_rate_deg_s"] = round(rec["dyaw_deg"] / max(held, 0.1), 2)
    return rec


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--levels", default="600,1000",
                    help="MANUAL_CONTROL counts to test per signed axis")
    ap.add_argument("--axes", default="surge,sway,yaw")
    ap.add_argument("--pulse-s", type=float, default=3.0)
    ap.add_argument("--coast-s", type=float, default=3.0)
    ap.add_argument("--settle-s", type=float, default=3.0)
    ap.add_argument("--drift-s", type=float, default=15.0)
    ap.add_argument("--wait-s", type=float, default=40.0,
                    help="seconds to wait for the operator to bring the "
                         "vehicle back into frame before skipping a trial")
    args = ap.parse_args()

    levels = [int(v) for v in args.levels.split(",")]
    axes = args.axes.split(",")
    stamp = time.strftime("%Y%m%d_%H%M%S")
    sess = os.path.join(OUT, stamp)
    os.makedirs(sess, exist_ok=True)

    truth = Truth()
    trials = []
    try:
        p = truth.mean(6)
        print("start pixel:", None if p is None else np.round(p, 0),
              flush=True)
        if not in_frame_soft(p):
            # Wait for the operator instead of exiting: they reposition
            # the vehicle by hand during the session, and quitting here
            # wasted several attempts that were otherwise ready to run.
            print("rover fuori inquadratura: RICENTRALO, attendo...",
                  flush=True)
            t_w = time.time()
            while time.time() - t_w < 90.0:
                time.sleep(2.0)
                p = truth.mean(4)
                if in_frame_soft(p):
                    break
            if not in_frame_soft(p):
                print("ABORT: ancora fuori dopo 90 s")
                return 2
            print("ok, rover a", np.round(p, 0), flush=True)
        with RovLink() as rov:
            rov.set_mode("MANUAL")
            time.sleep(0.4)
            rov.arm(mode="MANUAL")
            rov.set_mode("MANUAL")
            time.sleep(0.4)

            gain = measure_gain(rov)
            print("autorita comando misurata: %.2f" % gain, flush=True)

            drift, drift_track = measure_drift(rov, truth, args.drift_s)
            print("deriva %.2f px/s = %.3f m/s (dx %+.2f dy %+.2f)"
                  % (float(np.linalg.norm(drift)),
                     float(np.linalg.norm(drift)) / PX_PER_M,
                     drift[0], drift[1]), flush=True)

            # Learn where each translation axis points in the image
            # BEFORE using it to recentre, otherwise recentring could
            # drive the vehicle out of frame.
            basis = {}
            for ax in ("surge", "sway"):
                if basis:
                    recenter(rov, truth, basis, drift, tol_px=120)
                u = learn_basis(rov, truth, ax, drift)
                if u is not None:
                    basis[ax] = u
                    print("direzione %s nell'immagine: (%+.2f, %+.2f)"
                          % (ax, u[0], u[1]), flush=True)
                else:
                    print("direzione %s NON misurabile" % ax, flush=True)
            if len(basis) == 2:
                ang = math.degrees(math.acos(max(-1.0, min(1.0, float(
                    np.dot(basis["surge"], basis["sway"]))))))
                print("angolo fra gli assi: %.0f gradi "
                      "(ortogonali attesi ~90)" % ang, flush=True)
                if ang < 45 or ang > 135:
                    print("  ATTENZIONE: assi quasi paralleli, la deriva "
                          "domina ancora la stima delle direzioni",
                          flush=True)

            # signed pairs, so each trial partly undoes the previous one
            plan = [(ax, s * lv) for ax in axes for lv in levels
                    for s in (+1, -1)]
            for ax, lv in plan:
                # Drift window FIRST, recentring LAST: measuring the
                # drift after recentring gave it 5 s to carry the vehicle
                # a quarter metre back toward the edge, which is what
                # aborted the trial that followed.
                d_local, _ = measure_drift(rov, truth, 4.0)
                if float(np.linalg.norm(d_local)) == 0.0:
                    d_local = drift
                if basis:
                    recenter(rov, truth, basis, d_local, tol_px=120)
                r = trial(rov, truth, ax, lv, args.pulse_s,
                          args.coast_s, d_local, local_drift_s=0.0)
                if r.get("aborted"):
                    # The operator is repositioning the vehicle by hand
                    # during the run, so WAIT for it to come back rather
                    # than abandoning the remaining trials: aborting the
                    # whole batch on one edge event is what left every
                    # earlier session with two or three usable trials.
                    print("%-6s %+5d  fuori inquadratura, attendo il "
                          "ricentraggio..." % (ax, lv), flush=True)
                    t_wait = time.time()
                    while time.time() - t_wait < args.wait_s:
                        if basis:
                            recenter(rov, truth, basis, d_local, tol_px=140,
                                     max_s=8.0)
                        if in_frame(truth.mean(3)):
                            break
                        t3 = time.time()
                        while time.time() - t3 < 2.0:
                            rov.manual(z=Z_NEUTRAL)
                            time.sleep(0.05)
                    if not in_frame(truth.mean(3)):
                        print("   ancora fuori dopo %.0f s, salto"
                              % args.wait_s, flush=True)
                        trials.append(r)
                        continue
                    r = trial(rov, truth, ax, lv, args.pulse_s,
                              args.coast_s, d_local, local_drift_s=0.0)
                trials.append(r)
                if r.get("aborted"):
                    print("%-6s %+5d  saltata" % (ax, lv), flush=True)
                    continue
                extra = ""
                if ax == "yaw" and r.get("dyaw_deg") is not None:
                    extra = "  dyaw %+.0f deg (%.0f deg/s)" % (
                        r["dyaw_deg"], r["yaw_rate_deg_s"])
                print("%-6s %+5d  netto %.3f m  v=%.3f m/s  "
                      "deriva/segnale %.2f  fine:%s%s"
                      % (ax, lv, r.get("disp_net_m", float("nan")),
                         r.get("speed_m_s", float("nan")),
                         r.get("drift_share", float("nan")),
                         r["end_reason"], extra), flush=True)
                t2 = time.time()
                while time.time() - t2 < args.settle_s:
                    rov.manual(z=Z_NEUTRAL)
                    time.sleep(0.05)
            rov.disarm()
    finally:
        truth.close()

    out = {"session": stamp,
           "purpose": "S3 vehicle identification. CALIBRATION DATA: never "
                      "part of the final validation statistics.",
           "px_per_m": PX_PER_M,
           "command_authority_measured": gain,
           "authority_note": "ArduSub scales MANUAL_CONTROL by a runtime "
                             "joystick gain; a reboot resets it to "
                             "JS_GAIN_DEFAULT and invalidates this fit. "
                             "Re-measure before any validation run.",
           "drift_px_s": [round(float(v), 3) for v in drift],
           "drift_m_s": round(float(np.linalg.norm(drift)) / PX_PER_M, 4),
           "drift_track": drift_track,
           "pulse_s": args.pulse_s, "coast_s": args.coast_s,
           "trials": trials}
    path = os.path.join(sess, "trials.json")
    with open(path, "w") as f:
        json.dump(out, f, indent=2)
    print("\n%d prove ->" % len(trials), path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
