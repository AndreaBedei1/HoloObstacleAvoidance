"""Is the perception -> planner -> command chain alive? No actuation.

Runs the real pipeline in SHADOW with the vehicle disarmed and counts
what actually flows on each topic. Every failure so far has been a link
that was silently dead while the run looked healthy, and each one cost a
repositioning of the vehicle to discover. This finds them with the ROV
standing still.
"""
from __future__ import annotations
import os, subprocess, sys, threading, time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")

os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_zenoh_cpp")
os.environ["ZENOH_ROUTER_CHECK_ATTEMPTS"] = "20"

SECS = float(sys.argv[1]) if len(sys.argv) > 1 else 40.0
log_dir = os.path.join(_ROOT, "experiments", "real", "shadow_check")
os.makedirs(log_dir, exist_ok=True)

env = dict(os.environ)
env["HOLO_REPO_ROOT"] = os.path.abspath(_ROOT)
zl = open(os.path.join(log_dir, "zenoh.log"), "w")
zen = subprocess.Popen(["ros2", "run", "rmw_zenoh_cpp", "rmw_zenohd"],
                       env=env, stdout=zl, stderr=subprocess.STDOUT)
print("router avviato", flush=True)
time.sleep(6.0)

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rov_obstacle_msgs.msg import Obstacle2DArray

counts = {"obstacles_raw": 0, "obstacles_raw_nonempty": 0,
          "obstacles": 0, "obstacles_nonempty": 0, "cmd_vel_safe": 0,
          "cmd_lateral": 0}
ranges = []


class Probe(Node):
    def __init__(self):
        super().__init__("shadow_check_probe")
        self.create_subscription(Obstacle2DArray,
                                 "/perception/obstacles_raw", self.raw, 20)
        self.create_subscription(Obstacle2DArray,
                                 "/perception/obstacles", self.qual, 20)
        self.create_subscription(Twist, "/planner/cmd_vel_safe",
                                 self.cmd, 20)

    def raw(self, m):
        counts["obstacles_raw"] += 1
        if m.obstacles:
            counts["obstacles_raw_nonempty"] += 1
            ranges.append(max(o.height for o in m.obstacles))

    def qual(self, m):
        counts["obstacles"] += 1
        if m.obstacles:
            counts["obstacles_nonempty"] += 1

    def cmd(self, m):
        counts["cmd_vel_safe"] += 1
        if abs(m.linear.y) > 0.02:
            counts["cmd_lateral"] += 1


rclpy.init()
node = Probe()
time.sleep(1.0)

pl = open(os.path.join(log_dir, "pipeline.log"), "w")
proc = subprocess.Popen(
    ["ros2", "launch", "rov_real_bridge", "real_pipeline.launch.py",
     "planner:=committed", "estimator_method:=t2",
     "real_control_mode:=shadow", "vehicle_in_water:=true",
     "allow_real_actuation:=false"],
    env=env, stdout=pl, stderr=subprocess.STDOUT)
print("pipeline in SHADOW (nessuna attuazione), %.0f s..." % SECS, flush=True)
t0 = time.time()
while time.time() - t0 < SECS:
    rclpy.spin_once(node, timeout_sec=0.1)
proc.terminate()
try:
    proc.wait(timeout=10)
except subprocess.TimeoutExpired:
    proc.kill()
pl.close()
node.destroy_node()
rclpy.shutdown()
zen.terminate()
try:
    zen.wait(timeout=8)
except subprocess.TimeoutExpired:
    zen.kill()
zl.close()

print("\n--- cosa e passato in %.0f s ---" % SECS)
print("  /perception/obstacles_raw   %4d messaggi, %4d con ostacolo"
      % (counts["obstacles_raw"], counts["obstacles_raw_nonempty"]))
print("  /perception/obstacles       %4d messaggi, %4d con ostacolo"
      % (counts["obstacles"], counts["obstacles_nonempty"]))
print("  /planner/cmd_vel_safe       %4d messaggi, %4d con comando laterale"
      % (counts["cmd_vel_safe"], counts["cmd_lateral"]))
ok = (counts["obstacles_raw"] > 0 and counts["cmd_vel_safe"] > 0)
if counts["obstacles_raw"] == 0:
    print("\nIL RILEVATORE NON PUBBLICA: catena morta a monte")
elif counts["obstacles_raw_nonempty"] == 0:
    print("\nil rilevatore pubblica ma NON VEDE MAI l'ancora")
elif counts["obstacles_nonempty"] == 0:
    print("\nil rilevatore vede ma il qualificatore SCARTA TUTTO")
elif counts["cmd_vel_safe"] == 0:
    print("\nil pianificatore NON PUBBLICA comandi")
else:
    print("\nCATENA VIVA")
sys.exit(0 if ok else 3)
