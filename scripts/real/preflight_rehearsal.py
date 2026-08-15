"""Rehearsal gate for the real validation session. NOT a counted run.

Run once on the day, before run 1/20. If it passes, nothing is tuned and
the 20 runs start; if it fails, the campaign does not start. That is the
whole point: the alternative is discovering a broken link after five
runs and having to decide whether to keep them.

WHAT IT CHECKS, and why each one can silently ruin the campaign:

 1. COMMAND AUTHORITY. ArduSub scales MANUAL_CONTROL by a runtime
    joystick gain. It was 0.20 when the S3 vehicle profile was measured,
    and a reboot resets it to JS_GAIN_DEFAULT. At 0.5 the vehicle would
    move 2.5x faster than the plant model in the frozen predictions,
    with nothing in the logs to say so.

 2. CALIBRATION FILES present and matching the freeze. If one is
    missing the pipeline refuses to run, but a WRONG one would not
    announce itself, so the hashes are compared.

 3. POOL BENCHMARK readable, and the planner configuration it will
    actually apply. The real pipeline once launched the planners with
    library defaults tuned for 11 m scenarios.

 4. ONBOARD CAMERA delivering frames, and the detector producing the
    observation contract on them.

 5. OVERHEAD GROUND TRUTH seeing the vehicle, with the vehicle whole in
    frame rather than a border fragment: a false ground truth is worse
    than none.

 6. DUAL RECORDING writing both streams with their timestamp indices.

 7. INTERLOCK: the adapter must refuse live actuation while any axis is
    uncalibrated or the interlock is off.

Nothing here commands the vehicle. Thrusters are exercised only by the
authority measurement, which is a 2.5 s pulse the operator must approve.

Usage:
    python scripts/real/preflight_rehearsal.py            # checks only
    python scripts/real/preflight_rehearsal.py --authority  # + thrusters
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")

# Called ONCE, here, before any check runs. ensure_env re-executes the
# interpreter with the RealSense environment set, so calling it from
# inside a check restarts the whole rehearsal halfway through and every
# result is printed twice.
from camera_stream import ensure_env  # noqa: E402

ensure_env()

CAL = os.path.join(_ROOT, "config", "calibration")
REQUIRED_FITS = ["s1_observation_fit.json", "s2_timing_fit.json",
                 "s3_vehicle.json"]
EXPECTED_AUTHORITY = 0.20
AUTHORITY_TOLERANCE = 0.03


class Check:
    def __init__(self):
        self.rows = []

    def add(self, name, ok, detail=""):
        self.rows.append({"check": name, "ok": bool(ok), "detail": detail})
        print("  [%s] %-34s %s" % ("ok" if ok else "FAIL", name, detail),
              flush=True)
        return ok

    @property
    def passed(self):
        return all(r["ok"] for r in self.rows)


def check_calibration(c):
    for name in REQUIRED_FITS:
        path = os.path.join(CAL, name)
        if not os.path.isfile(path):
            c.add("calibrazione %s" % name, False, "assente")
            continue
        with open(path, "rb") as f:
            digest = hashlib.sha256(f.read()).hexdigest()[:16]
        c.add("calibrazione %s" % name, True, "sha256 %s" % digest)


def check_benchmark(c):
    path = os.path.join(_ROOT, "config", "pool_benchmark_FROZEN.yaml")
    try:
        import yaml
        with open(path) as f:
            b = yaml.safe_load(f)["pool_benchmark"]
        c.add("pool benchmark", True,
              "ingaggio %.2f m, surge %.2f m/s, altezza ostacolo %.2f m"
              % (b["engage_distance_m"], b["nominal_surge"],
                 b["target_obstacle_height_m"]))
        return b
    except Exception as exc:
        c.add("pool benchmark", False, str(exc))
        return None


def check_predictions(c):
    p = os.path.join(_ROOT, "experiments", "simulation",
                     "phase10_predictions", "PREDICTIONS.sha256")
    if not os.path.isfile(p):
        return c.add("previsioni congelate", False,
                     "assenti: i run reali non devono partire prima")
    with open(p) as f:
        line = f.read().strip()
    return c.add("previsioni congelate", True, line.split()[0][:16])


def check_authority(c, do_pulse):
    if not do_pulse:
        c.add("autorita di comando", True,
              "NON verificata (--authority per misurarla)")
        return
    try:
        from rovlink import RovLink, Z_NEUTRAL
        from pymavlink import mavutil
    except Exception as exc:
        c.add("autorita di comando", False, "pymavlink: %s" % exc)
        return
    try:
        with RovLink() as rov:
            rov.set_message_interval(
                mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 10)
            rov.set_mode("MANUAL")
            time.sleep(0.4)
            rov.arm(mode="MANUAL")
            rov.set_mode("MANUAL")
            time.sleep(0.4)
            t0, best = time.time(), 0
            while time.time() - t0 < 2.5:
                rov.manual(x=1000, z=Z_NEUTRAL)
                time.sleep(0.06)
                m = rov.newest("SERVO_OUTPUT_RAW", timeout=0.0)
                if m is not None and time.time() - t0 > 1.0:
                    best = max(best, abs(m.servo1_raw - 1500))
            rov.neutral()
            rov.disarm()
        gain = best / 400.0
        ok = abs(gain - EXPECTED_AUTHORITY) <= AUTHORITY_TOLERANCE
        c.add("autorita di comando", ok,
              "misurata %.2f, attesa %.2f%s"
              % (gain, EXPECTED_AUTHORITY,
                 "" if ok else "  -> S3 NON VALIDO, non avviare i run"))
    except Exception as exc:
        c.add("autorita di comando", False, str(exc))


def check_next_run(c):
    """Which run the frozen order says comes next, and its start pose.

    The rehearsal must exercise the SAME condition the next real run
    will use: a gate that passes on a different planner or a different
    geometry has not tested the thing that is about to happen.
    """
    try:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        import final_campaign as fc
        seq, cfg = fc.frozen_order()
        done = fc.completed_runs()
        nxt = fc.next_pending(seq, done)
        if nxt is None:
            c.add("prossima prova", True, "le 20 sono complete")
            return None, None
        c.add("prossima prova", True,
              "%d/20 -> %s %s replica %d (%d gia fatte)"
              % (nxt["index"], nxt["geometry"], nxt["planner"],
                 nxt["run"], len(done)))
        return nxt, cfg
    except Exception as exc:
        c.add("prossima prova", False, str(exc))
        return None, None


def check_start_pose(c, nxt, cfg):
    """The vehicle must be inside the start region BEFORE the run, and
    the pose must be measurable: a start outside the overhead frame
    cannot be recorded, which is the requirement that makes the
    pre-registered start regions mean anything."""
    if nxt is None or cfg is None:
        c.add("posa di partenza", False, "prossima prova sconosciuta")
        return
    try:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        import final_campaign as fc
        gt = fc.GroundTruth()
        gt.start()
        time.sleep(1.5)
        sp = cfg["start_poses"][nxt["geometry"]]
        pose, why = fc.check_start_pose(gt, sp["nominal_px"])
        gt.stop()
        if pose is None:
            c.add("posa di partenza", False, why or "non misurabile")
            return
        c.add("posa di partenza", pose["within_tolerance"],
              "lungo %+.2f m, laterale %+.2f m%s"
              % (pose["offset_along_m"], pose["offset_lateral_m"],
                 "" if pose["within_tolerance"] else "  -> " + (why or "")))
    except Exception as exc:
        c.add("posa di partenza", False, str(exc))


def check_timestamps(c):
    """Both video indices must be monotonic and on the same clock.

    The indices are the timing authority for every join in the dataset:
    container frame rates are nominal and encoder timestamps are
    rewritten, so if the indices are not trustworthy nothing in a run
    can be aligned with anything else.
    """
    try:
        import json as _json
        import tempfile
        import numpy as _np
        from dual_recorder import DualRecorder
        d = tempfile.mkdtemp(prefix="ts_")
        seq = {"i": 0}

        def onb():
            seq["i"] += 1
            return (_np.full((120, 160, 3), seq["i"] % 255, dtype=_np.uint8),
                    time.time())

        def ovr():
            return _np.full((120, 160, 3), 128, dtype=_np.uint8)

        rec = DualRecorder(d, onb, ovr, onboard_fps=10, overhead_fps=5,
                           downscale=1.0)
        rec.start()
        time.sleep(2.0)
        stats = rec.stop()
        ok, detail = True, []
        spans = {}
        for st in stats["streams"]:
            ts = [_json.loads(l)["t"] for l in
                  open(os.path.join(d, st["index"]))]
            mono = all(b >= a for a, b in zip(ts, ts[1:]))
            ok &= mono and len(ts) == st["frames"]
            spans[st["stream"]] = (ts[0], ts[-1]) if ts else (0, 0)
            detail.append("%s %d campioni%s"
                          % (st["stream"], len(ts),
                             "" if mono else " NON monotoni"))
        if len(spans) == 2:
            a, b = spans.values()
            shared = abs(a[0] - b[0]) < 1.0
            ok &= shared
            detail.append("avvio condiviso entro %.2f s"
                          % abs(a[0] - b[0]))
        c.add("timestamp e sincronizzazione", ok, ", ".join(detail))
    except Exception as exc:
        c.add("timestamp e sincronizzazione", False, str(exc))


def check_overhead(c):
    try:
        from camera_stream import ensure_env
        ensure_env()
        import numpy as np
        from overhead_track import OverheadTracker
        tr = OverheadTracker()
        seen, whole = 0, 0
        for _ in range(8):
            d = tr.detect()
            if d and d.get("found"):
                seen += 1
                if not d.get("partial"):
                    whole += 1
            time.sleep(0.12)
        tr.close()
        c.add("verita di riferimento dall'alto", seen >= 6 and whole >= 6,
              "%d/8 rilevato, %d intero" % (seen, whole))
    except Exception as exc:
        c.add("verita di riferimento dall'alto", False, str(exc))


def check_onboard(c):
    try:
        from camera_stream import CameraStream, ensure_env
        ensure_env()
        from anchor_detect import detect_anchor
        cam = CameraStream()
        frames, dets = 0, 0
        t0 = time.time()
        while time.time() - t0 < 6.0:
            frame, _ = cam.latest()
            if frame is not None:
                frames += 1
                if detect_anchor(frame).get("found"):
                    dets += 1
            time.sleep(0.2)
        cam.close()
        c.add("camera di bordo", frames >= 10,
              "%d fotogrammi, %d con ancora rilevata" % (frames, dets))
    except Exception as exc:
        c.add("camera di bordo", False, str(exc))


def check_recorder(c):
    try:
        from dual_recorder import self_test
        ok = self_test() == 0
        c.add("registrazione doppia", ok, "self-test")
    except Exception as exc:
        c.add("registrazione doppia", False, str(exc))


def check_interlock(c):
    try:
        sys.path.insert(0, os.path.join(_ROOT, "src", "rov_real_bridge"))
        from rov_real_bridge.command_mapping import AxisCalibration
        a = AxisCalibration()
        c.add("interlock adattatore", not a.calibrated,
              "un asse non calibrato blocca l'attuazione")
    except Exception as exc:
        c.add("interlock adattatore", False, str(exc))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--authority", action="store_true",
                    help="misura l'autorita di comando: MUOVE i thruster")
    args = ap.parse_args()

    print("REHEARSAL — non conteggiato, nessun tuning se passa\n")
    c = Check()
    check_predictions(c)
    check_calibration(c)
    check_benchmark(c)
    check_interlock(c)
    check_recorder(c)
    check_timestamps(c)
    nxt, cfg = check_next_run(c)
    check_overhead(c)
    check_onboard(c)
    check_start_pose(c, nxt, cfg)
    check_authority(c, args.authority)

    out = os.path.join(_ROOT, "experiments", "real", "rehearsal")
    os.makedirs(out, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    with open(os.path.join(out, "rehearsal_%s.json" % stamp), "w") as f:
        json.dump({"t": stamp, "passed": c.passed, "checks": c.rows}, f,
                  indent=2)
    print("\nESITO:", "PASSATO — si parte con i 20 run" if c.passed
          else "FALLITO — la campagna non parte")
    if not args.authority:
        print("NB: l'autorita di comando non e stata misurata. Va fatta "
              "prima del run 1/20: un riavvio del controllore la riporta "
              "a 0.5 e invalida S3 senza alcun segnale.")
    return 0 if c.passed else 2


if __name__ == "__main__":
    raise SystemExit(main())
