"""Record onboard camera + full telemetry while executing a gentle
scripted view sweep (Phase 9 anchor dataset: multiple bearings/distances).

Sequence (ALT_HOLD, all bounded, surface trim):
  1. 8 s static hold            (baseline frames)
  2. yaw sweep: +15% 2 s, -15% 4 s, +15% 2 s   (bearing variation)
  3. surge +20% 2.5 s           (approach ~0.2-0.3 m)
  4. 5 s hold
  5. surge -20% 2.5 s           (back off)
  6. 5 s hold, disarm

Camera: spawns camera_grab.py --video in a subprocess (UDP 5600).
Telemetry: same JSONL logger as rov_motion_test.
"""

import os
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rovlink import RovLink  # noqa: E402
from rov_motion_test import TelemetryLogger  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..", "..")
OUT = os.path.join(ROOT, "experiments", "real", "anchor_dataset")


def main() -> int:
    os.makedirs(OUT, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    video_dir = os.path.join(OUT, f"session_{stamp}")
    os.makedirs(video_dir, exist_ok=True)

    total_s = 70 if "--panorama" in sys.argv else 8 + 8 + 2.5 + 5 + 2.5 + 6 + 4
    cam = subprocess.Popen(
        [sys.executable, os.path.join(HERE, "camera_grab.py"),
         "--video", str(total_s + 5), "--out", video_dir],
        stdout=open(os.path.join(video_dir, "camera.log"), "w"),
        stderr=subprocess.STDOUT)

    with RovLink() as rov:
        log = TelemetryLogger(rov, os.path.join(video_dir,
                                                "telemetry.jsonl"))
        from pymavlink import mavutil
        for mid, hz in [
                (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 15),
                (mavutil.mavlink.MAVLINK_MSG_ID_VFR_HUD, 15),
                (mavutil.mavlink.MAVLINK_MSG_ID_DISTANCE_SENSOR, 10)]:
            rov.set_message_interval(mid, hz)
        log.start()
        log.write("meta", {"session": "anchor_dataset_sweep"})

        mode = "STABILIZE" if "--panorama" in sys.argv else "ALT_HOLD"
        hb = rov.set_mode(mode)
        hb = rov.arm(mode=mode)
        print("armed:", hb)
        if not hb or not hb["armed"]:
            print("ARMING FAILED")
            log.stop()
            cam.wait()
            return 1
        if hb["mode"] != mode:
            hb = rov.set_mode(mode)
            print("post-arm mode:", hb)

        def hold(sec):
            t0 = time.time()
            while time.time() - t0 < sec:
                rov.neutral()
                time.sleep(0.1)

        try:
            import argparse
            mode_panorama = "--panorama" in sys.argv
            if mode_panorama:
                log.write("cmd", {"phase": "panorama_360"})
                # slow continuous rotation, ~2 x 30 s at 12%
                rov.pulse(r=+120, duration_s=30.0)
                rov.pulse(r=+120, duration_s=30.0)
                hold(3)
            else:
                log.write("cmd", {"phase": "static_hold"})
                hold(8)
                log.write("cmd", {"phase": "yaw_sweep"})
                rov.pulse(r=+150, duration_s=2.0)
                rov.pulse(r=-150, duration_s=4.0)
                rov.pulse(r=+150, duration_s=2.0)
                log.write("cmd", {"phase": "approach"})
                rov.pulse(x=+200, duration_s=2.5)
                hold(5)
                log.write("cmd", {"phase": "backoff"})
                rov.pulse(x=-200, duration_s=2.5)
                hold(6)
        finally:
            hb = rov.disarm()
            print("disarmed:", hb)
            log.write("cmd", {"disarm": True, "result": hb})
            log.stop()
    cam.wait(timeout=30)
    print("session dir:", video_dir)
    return 0


if __name__ == "__main__":
    sys.exit(main())
