"""One run of the 20-run real validation campaign.

Produces exactly the file shape `scripts/analysis/sim_real_compare.py`
reads, so the real and the simulated side go through the SAME analysis
code. Two domains analysed by two code paths would make a difference in
the analysis indistinguishable from a difference in the world.

WHAT ONE RUN DOES

 1. Refuses to start unless the predictions are frozen and hashed. The
    comparison only means anything if the prediction predates the
    measurement.
 2. Measures the start pose from the overhead ground truth and checks it
    against the PRE-REGISTERED tolerance. Outside tolerance the operator
    nudges the vehicle and it is measured again; no run is scored from a
    pose outside tolerance, and the measured pose is recorded as a
    covariate, never used to correct the result afterwards.
 3. Starts both video recorders with their timestamp indices.
 4. Launches the real pipeline with the frozen pool benchmark, the same
    planner code the simulation ran, and live actuation.
 5. Records /planner/cmd_vel_safe paired with the ground-truth distance
    at the same instant, which is what every manoeuvre metric is
    computed from, offline, at the pre-registered threshold.
 6. Stops on the pre-registered end conditions and writes the run.

THE OVERHEAD CAMERA IS EVALUATION AND SAFETY ONLY. It never reaches the
planner. That separation is what lets it serve as ground truth at all.

Usage:
    python scripts/real/final_campaign.py --geometry K0 \
        --planner committed --run 1
"""

from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")

from camera_stream import ensure_env  # noqa: E402

ensure_env()

import numpy as np  # noqa: E402

from overhead_track import OverheadTracker  # noqa: E402
from dual_recorder import DualRecorder  # noqa: E402

OUT = os.path.join(_ROOT, "experiments", "real", "final_campaign")
PX_PER_M = 452.8
ANCHOR_PX = (1044.0, 626.0)

# Pre-registered start tolerance (docs/PHASE10_FINAL_CAMPAIGN_PROTOCOL).
TOL_LATERAL_M = 0.30
TOL_ALONG_M = 0.40
TOL_HEADING_DEG = 12.0


def frozen_predictions_hash():
    p = os.path.join(_ROOT, "experiments", "simulation",
                     "phase10_predictions", "PREDICTIONS.sha256")
    if not os.path.isfile(p):
        return None
    with open(p) as f:
        return f.read().split()[0]


class GroundTruth:
    """Overhead tracker in its own thread: pose, and distance to the
    anchor, at whatever rate the camera sustains."""

    def __init__(self):
        self.tr = OverheadTracker()
        self.lock = threading.Lock()
        self.latest = None
        self.samples = []
        self._stop = threading.Event()
        self._t = threading.Thread(target=self._loop, daemon=True)

    def _loop(self):
        while not self._stop.is_set():
            d = self.tr.detect()
            if d and d.get("found"):
                px = np.array(d["pixel"], float)
                dist = float(np.linalg.norm(px - np.array(ANCHOR_PX))) \
                    / PX_PER_M
                rec = {"t": round(time.time(), 4),
                       "px": [round(float(v), 1) for v in px],
                       "distance_m": round(dist, 4),
                       "partial": bool(d.get("partial"))}
                with self.lock:
                    self.latest = rec
                    self.samples.append(rec)
            time.sleep(0.05)

    def start(self):
        self._t.start()

    def distance(self):
        with self.lock:
            return None if self.latest is None else self.latest["distance_m"]

    def pose(self, n=6):
        pts = []
        t0 = time.time()
        while len(pts) < n and time.time() - t0 < 4.0:
            with self.lock:
                if self.latest is not None:
                    pts.append(self.latest["px"])
            time.sleep(0.12)
        return None if not pts else np.mean(pts, axis=0)

    def stop(self):
        self._stop.set()
        self._t.join(timeout=3)
        self.tr.close()


def check_start_pose(gt, nominal_px):
    """Measured pose against the pre-registered tolerance."""
    p = gt.pose()
    if p is None:
        return None, "verita di riferimento non disponibile"
    d = (p - np.array(nominal_px, float)) / PX_PER_M
    along, lateral = float(d[0]), float(d[1])
    ok = abs(lateral) <= TOL_LATERAL_M and abs(along) <= TOL_ALONG_M
    return ({"px": [round(float(v), 1) for v in p],
             "offset_along_m": round(along, 3),
             "offset_lateral_m": round(lateral, 3),
             "within_tolerance": ok},
            None if ok else
            "fuori tolleranza: lungo %.2f m (max %.2f), laterale %.2f m "
            "(max %.2f)" % (along, TOL_ALONG_M, lateral, TOL_LATERAL_M))


def record_cmd_trace(run_dir, gt, stop_event):
    """Subscribe to the frozen boundary output and pair every sample
    with the ground-truth distance at that instant."""
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node

    trace = []
    t0 = time.time()

    class Rec(Node):
        def __init__(self):
            super().__init__("final_campaign_recorder")
            self.create_subscription(
                Twist, "/planner/cmd_vel_safe", self.on_cmd, 20)

        def on_cmd(self, msg):
            trace.append({"t": round(time.time() - t0, 3),
                          "x": round(msg.linear.x, 4),
                          "y": round(msg.linear.y, 4),
                          "r": round(msg.angular.z, 4),
                          "d": gt.distance()})

    rclpy.init()
    node = Rec()
    try:
        while not stop_event.is_set():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    with open(os.path.join(run_dir, "cmd_trace.json"), "w") as f:
        json.dump(trace, f)
    return trace


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--geometry", required=True, choices=["K0", "K1"])
    ap.add_argument("--planner", required=True,
                    choices=["committed", "dwa"])
    ap.add_argument("--run", required=True, type=int)
    ap.add_argument("--duration-s", type=float, default=90.0)
    ap.add_argument("--nominal-px", nargs=2, type=float,
                    default=[300.0, 540.0],
                    help="nominal start pixel for this geometry")
    ap.add_argument("--dry-run", action="store_true",
                    help="check the gates and the start pose, actuate "
                         "nothing")
    args = ap.parse_args()

    digest = frozen_predictions_hash()
    if digest is None:
        print("ABORT: le previsioni simulate non sono congelate. I run "
              "reali non devono precedere la previsione che devono "
              "verificare.")
        return 2
    print("previsioni congelate: %s" % digest[:16])

    run_dir = os.path.join(OUT, "runs", "%s_%s_%d"
                           % (args.geometry, args.planner, args.run))
    os.makedirs(run_dir, exist_ok=True)

    gt = GroundTruth()
    gt.start()
    time.sleep(1.5)

    pose, why = check_start_pose(gt, args.nominal_px)
    if pose is None or not pose["within_tolerance"]:
        print("posa di partenza NON valida: %s" % (why or "sconosciuto"))
        print("sposta il rover e rilancia; nessun run viene valutato da "
              "una posa fuori tolleranza")
        gt.stop()
        return 3
    print("posa di partenza entro tolleranza: lungo %+.2f m, laterale "
          "%+.2f m" % (pose["offset_along_m"], pose["offset_lateral_m"]))

    if args.dry_run:
        gt.stop()
        print("dry-run: nessuna attuazione")
        return 0

    rec = DualRecorder(run_dir, overhead_source=lambda: gt.tr.last_frame)
    rec.start()
    rec.mark("run_start", geometry=args.geometry, planner=args.planner,
             run=args.run)

    stop_event = threading.Event()
    trace_holder = {}
    th = threading.Thread(
        target=lambda: trace_holder.update(
            trace=record_cmd_trace(run_dir, gt, stop_event)),
        daemon=True)
    th.start()

    env = dict(os.environ)
    env.setdefault("RMW_IMPLEMENTATION", "rmw_zenoh_cpp")
    env["HOLO_REPO_ROOT"] = os.path.abspath(_ROOT)
    cmd = ["ros2", "launch", "rov_real_bridge", "real_pipeline.launch.py",
           "planner:=%s" % args.planner,
           "estimator_method:=t2",
           "real_control_mode:=live",
           "vehicle_in_water:=true",
           "allow_real_actuation:=true"]
    print("avvio pipeline reale:", " ".join(cmd), flush=True)
    log = open(os.path.join(run_dir, "pipeline.log"), "w")
    proc = subprocess.Popen(cmd, env=env, stdout=log,
                            stderr=subprocess.STDOUT)
    t0 = time.time()
    end_reason = "duration"
    try:
        while time.time() - t0 < args.duration_s:
            if proc.poll() is not None:
                end_reason = "pipeline_exited"
                break
            time.sleep(0.2)
    except KeyboardInterrupt:
        end_reason = "operator_stop"
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()
        log.close()
        stop_event.set()
        th.join(timeout=5)
        rec.mark("run_stop", reason=end_reason)
        stats = rec.stop()
        with gt.lock:
            samples = list(gt.samples)
        gt.stop()

    with open(os.path.join(run_dir, "ground_truth.jsonl"), "w") as f:
        for s in samples:
            f.write(json.dumps(s) + "\n")

    dists = [s["distance_m"] for s in samples]
    result = {
        "geometry": args.geometry, "scenario": args.geometry,
        "planner": args.planner, "run": args.run,
        "run_dir": os.path.basename(run_dir),
        "predictions_sha256": digest,
        "start_pose": pose,
        "end_reason": end_reason,
        "duration_s": round(time.time() - t0, 2),
        "recording": stats,
        "assessment": {
            "min_clearance_m": (round(min(dists), 3) if dists else None),
            "collision": None,     # scored from the video and the radius
            "gt_samples": len(samples),
        },
        "cmd_trace_samples": len(trace_holder.get("trace") or []),
    }
    with open(os.path.join(run_dir, "result.json"), "w") as f:
        json.dump(result, f, indent=2)

    man_path = os.path.join(OUT, "manifest.json")
    manifest = {"results": []}
    if os.path.isfile(man_path):
        with open(man_path) as f:
            manifest = json.load(f)
    manifest["results"] = [r for r in manifest["results"]
                           if (r["scenario"], r["planner"], r["run"])
                           != (args.geometry, args.planner, args.run)]
    manifest["results"].append(result)
    manifest["predictions_sha256"] = digest
    with open(man_path, "w") as f:
        json.dump(manifest, f, indent=2)

    print("\nrun %s/%s/%d: fine=%s, distanza minima %s m, %d campioni GT, "
          "%d campioni comando"
          % (args.geometry, args.planner, args.run, end_reason,
             result["assessment"]["min_clearance_m"], len(samples),
             result["cmd_trace_samples"]))
    print("->", run_dir)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
