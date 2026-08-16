"""One real run. No RealSense in this process; data saved incrementally.

Two lessons paid for in pool time are built in here.

NO OVERHEAD CAMERA. Opening the RealSense inside this process -- even
briefly, even closed again before the run -- left the process dying
partway through with no traceback, every time, while the otherwise
identical chain probe completed reliably. The overhead camera only ever
served as tracking and safety, never as planner input, so the run does
not need it: the start pose is measured by a separate process before the
run and the clearance comes from the overhead video afterwards.

DATA IS WRITTEN AS IT ARRIVES. Several completed runs were lost because
everything was saved at the end and the end never came. The command
trace is appended to disk continuously, so whatever the run reached
survives however it ends.
"""
from __future__ import annotations
import json, os, subprocess, sys, time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_zenoh_cpp")
os.environ["ZENOH_ROUTER_CHECK_ATTEMPTS"] = "20"

planner = sys.argv[1] if len(sys.argv) > 1 else "committed"
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 45.0
tag = sys.argv[3] if len(sys.argv) > 3 else "run"
engage = sys.argv[4] if len(sys.argv) > 4 else "1.8"
out = os.path.join(_ROOT, "experiments", "real", "quick_runs",
                   time.strftime("%Y%m%d_%H%M%S") + "_" + tag)
os.makedirs(out, exist_ok=True)
print("cartella:", out, flush=True)

# CLEAN SLATE FIRST. Nodes from previous runs survive the launch being
# terminated, and they keep publishing: after three runs there were 26
# ROS nodes alive, several of them planners writing to the same command
# topic. The vehicle obeys whichever message arrived last, so it moves
# in jerks that get worse with every run -- which is exactly what the
# operator saw, and what cost the whole morning in a different guise
# (sixty-six orphaned MAVLink senders).
# Kill the previous bridge by pid: it is a python process and the
# name-based sweep below cannot see it.
_pidfile = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        ".cmd_bridge.pid")
try:
    with open(_pidfile) as _f:
        _old = _f.read().strip()
    if _old:
        subprocess.run(["taskkill", "/F", "/PID", _old],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
except OSError:
    pass

for _name in ("real_detector_node.exe", "temporal_estimator_node.exe",
              "nominal_cmd_publisher_node.exe",
              "local_avoidance_planner_node.exe", "dwa_planner_node.exe",
              "real_control_live_node.exe", "rmw_zenohd.exe"):
    subprocess.run(["taskkill", "/F", "/IM", _name],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

# The name sweep above is NOT enough, and believing it was cost four
# runs. Each of those .EXE files is only a stub: it spawns a python.exe
# child holding "install\lib\<pkg>\<node>-script.py", and that child is
# what subscribes and publishes. taskkill /IM reaps the stub and leaves
# the child running. The survivors accumulated one full node set per
# run -- six planners on the same command topic by run 6, visible as the
# command rate climbing 19 -> 36 -> 56 -> 74 -> 91 Hz -- and one orphan
# detector kept UDP 5600 bound so the NEW detector could not start at
# all. Sweep by command line, which sees the children.
def _sweep_orphans():
    ps = (
        "Get-CimInstance Win32_Process | "
        "Where-Object { $_.CommandLine -and ("
        # 'install.lib' and not the repo name: every installed ROS node
        # child lives under install\lib\<pkg>\, the dot absorbs the
        # backslash, and matching the repo name alone would also match
        # this very script's own launcher.
        "$_.CommandLine -match 'install.lib' -or "
        "$_.CommandLine -match 'ros2-script' -or "
        "$_.CommandLine -match 'rmw_zenohd') } | "
        "ForEach-Object { $_.ProcessId }"
    )
    try:
        r = subprocess.run(["powershell", "-NoProfile", "-Command", ps],
                           capture_output=True, text=True, timeout=40)
    except (OSError, subprocess.SubprocessError):
        return 0
    n = 0
    for line in r.stdout.split():
        if line.strip().isdigit() and int(line) != os.getpid():
            subprocess.run(["taskkill", "/F", "/PID", line.strip()],
                           stdout=subprocess.DEVNULL,
                           stderr=subprocess.DEVNULL)
            n += 1
    return n


_killed = _sweep_orphans()
time.sleep(2.0)
_left = _sweep_orphans()
if _left:
    time.sleep(1.5)
print("[quick_run] orfani terminati: %d (+%d)" % (_killed, _left), flush=True)

# REFUSE to start on an occupied camera port. Run 6 launched anyway, its
# detector died on bind, and the planner silently consumed an orphan's
# detections for the whole run -- a run that looked like data and was
# not. A run that cannot see is worse than a run that does not happen.
def _port_5600_busy():
    try:
        r = subprocess.run(["netstat", "-ano", "-p", "UDP"],
                           capture_output=True, text=True, timeout=30)
    except (OSError, subprocess.SubprocessError):
        return False
    return any(":5600" in ln for ln in r.stdout.splitlines())


if _port_5600_busy():
    time.sleep(2.0)
    if _port_5600_busy():
        print("[quick_run] ABORT: la porta UDP 5600 e' ancora occupata; "
              "il rilevatore non potrebbe partire", flush=True)
        raise SystemExit(3)

env = dict(os.environ)
env["HOLO_REPO_ROOT"] = os.path.abspath(_ROOT)
env["ROV_IN_WATER"] = "1"
env["ROV_ALLOW_ACTUATION"] = "1"
env["ROV_CONTROL_MODE"] = "live"
zl = open(os.path.join(out, "zenoh.log"), "w")
zen = subprocess.Popen(["ros2", "run", "rmw_zenoh_cpp", "rmw_zenohd"],
                       env=env, stdout=zl, stderr=subprocess.STDOUT)
time.sleep(6.0)

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rov_obstacle_msgs.msg import Obstacle2DArray

tf = open(os.path.join(out, "cmd_trace.jsonl"), "w")
stats = {"cmd": 0, "lat": 0, "raw": 0, "raw_obs": 0, "qual": 0,
         "qual_obs": 0, "surge_max": 0.0, "lat_max": 0.0}
t0 = time.time()


class Rec(Node):
    def __init__(self):
        super().__init__("quick_run_recorder")
        self.create_subscription(Twist, "/planner/cmd_vel_safe",
                                 self.on_cmd, 20)
        self.create_subscription(Obstacle2DArray,
                                 "/perception/obstacles_raw", self.on_raw, 20)
        self.create_subscription(Obstacle2DArray,
                                 "/perception/obstacles", self.on_qual, 20)

    def on_cmd(self, m):
        stats["cmd"] += 1
        stats["surge_max"] = max(stats["surge_max"], m.linear.x)
        lat = abs(m.linear.y)
        stats["lat_max"] = max(stats["lat_max"], lat)
        if lat > 0.02:
            stats["lat"] += 1
        tf.write(json.dumps({"t": round(time.time() - t0, 3),
                             "x": round(m.linear.x, 4),
                             "y": round(m.linear.y, 4),
                             "r": round(m.angular.z, 4)}) + "\n")
        if stats["cmd"] % 40 == 0:
            tf.flush()

    def on_raw(self, m):
        stats["raw"] += 1
        if m.obstacles:
            stats["raw_obs"] += 1

    def on_qual(self, m):
        stats["qual"] += 1
        if m.obstacles:
            stats["qual_obs"] += 1


rclpy.init()
node = Rec()

pl = open(os.path.join(out, "pipeline.log"), "w")
proc = subprocess.Popen(
    ["ros2", "launch", "rov_real_bridge", "real_pipeline.launch.py",
     "planner:=%s" % planner, "estimator_method:=t2",
     "real_control_mode:=shadow", "vehicle_in_water:=true",
     "allow_real_actuation:=false", "engage_distance_m:=%s" % engage,
     "annotated_video_path:=" + os.path.join(
         out, "onboard_detections.mp4").replace("\\", "/")],
    env=env, stdout=pl, stderr=subprocess.STDOUT)
bl = open(os.path.join(out, "cmd_bridge.log"), "w")
brg = subprocess.Popen(
    [sys.executable, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                  "cmd_bridge.py"), str(secs + 5)],
    env=env, stdout=bl, stderr=subprocess.STDOUT)
print("CORSA planner=%s ingaggio=%s m, %.0f s" % (planner, engage, secs),
      flush=True)

try:
    t0 = time.time()
    while time.time() - t0 < secs:
        rclpy.spin_once(node, timeout_sec=0.1)
finally:
    tf.flush()
    tf.close()
    res = {"planner": planner, "tag": tag, "engage_distance_m": engage,
           "duration_s": round(time.time() - t0, 1), **stats}
    with open(os.path.join(out, "result.json"), "w") as f:
        json.dump(res, f, indent=2)
    print(json.dumps(res), flush=True)
    for p in (brg, proc, zen):
        try:
            p.terminate()
            p.wait(timeout=8)
        except Exception:
            try:
                p.kill()
            except Exception:
                pass
    for h in (pl, bl, zl):
        try:
            h.close()
        except Exception:
            pass
