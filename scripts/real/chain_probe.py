"""Where does the command stop? Probe every link, ROV stays put.

The command trace showed 0.15 m/s leaving the planner while the operator
saw the thrusters never turn. One of those is wrong, and the only way to
tell is to read the thruster commands themselves. Servo output is read
over the BlueOS HTTP API rather than MAVLink, because the adapter owns
the MAVLink port and a second binding would fight it.

Nothing here repositions or requires a valid start pose: the vehicle can
sit wherever it is.
"""
from __future__ import annotations
import json, os, subprocess, sys, threading, time, urllib.request

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_zenoh_cpp")
os.environ["ZENOH_ROUTER_CHECK_ATTEMPTS"] = "20"
SECS = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0
out = os.path.join(_ROOT, "experiments", "real", "chain_probe")
os.makedirs(out, exist_ok=True)

env = dict(os.environ)
env["HOLO_REPO_ROOT"] = os.path.abspath(_ROOT)
zl = open(os.path.join(out, "zenoh.log"), "w")
zen = subprocess.Popen(["ros2", "run", "rmw_zenoh_cpp", "rmw_zenohd"],
                       env=env, stdout=zl, stderr=subprocess.STDOUT)
time.sleep(6.0)

servo = {"samples": 0, "moved": 0, "max_dev": 0}


def watch_servo(stop):
    url = ("http://192.168.2.2:6040/v1/mavlink/vehicles/1/components/1"
           "/messages/SERVO_OUTPUT_RAW")
    while not stop.is_set():
        try:
            with urllib.request.urlopen(url, timeout=1.0) as r:
                m = json.loads(r.read().decode())["message"]
            dev = max(abs(m["servo%d_raw" % i] - 1500) for i in range(1, 5))
            servo["samples"] += 1
            servo["max_dev"] = max(servo["max_dev"], dev)
            if dev > 5:
                servo["moved"] += 1
        except Exception:
            pass
        time.sleep(0.2)


import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

n = {"nominal": 0, "nominal_fwd": 0, "safe": 0, "safe_fwd": 0, "dbg": 0}
dbg_last = []


class P(Node):
    def __init__(self):
        super().__init__("chain_probe")
        self.create_subscription(Twist, "/cmd_vel_nominal", self.nom, 20)
        self.create_subscription(Twist, "/planner/cmd_vel_safe",
                                 self.safe, 20)
        self.create_subscription(String, "/real/adapter_debug",
                                 self.dbg, 20)

    def nom(self, m):
        n["nominal"] += 1
        if m.linear.x > 0.01:
            n["nominal_fwd"] += 1

    def safe(self, m):
        n["safe"] += 1
        if m.linear.x > 0.01:
            n["safe_fwd"] += 1

    def dbg(self, m):
        n["dbg"] += 1
        # LAST message, not the first three: the first ones can come
        # from a node left over from an earlier launch, which is how a
        # stale payload was read as the current one.
        del dbg_last[:]
        dbg_last.append(m.data[:600])


rclpy.init()
node = P()
stop = threading.Event()
th = threading.Thread(target=watch_servo, args=(stop,), daemon=True)
th.start()

pl = open(os.path.join(out, "pipeline.log"), "w")
proc = subprocess.Popen(
    ["ros2", "launch", "rov_real_bridge", "real_pipeline.launch.py",
     "planner:=committed", "estimator_method:=t2",
     "real_control_mode:=live", "vehicle_in_water:=true",
     "allow_real_actuation:=true"],
    env=env, stdout=pl, stderr=subprocess.STDOUT)
# The bridge does the actuating; the packaged adapter is inert.
env["ROV_IN_WATER"] = "1"
env["ROV_ALLOW_ACTUATION"] = "1"
env["ROV_CONTROL_MODE"] = "live"
bl = open(os.path.join(out, "cmd_bridge.log"), "w")
brg = subprocess.Popen(
    [sys.executable, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                  "cmd_bridge.py"), str(SECS + 5)],
    env=env, stdout=bl, stderr=subprocess.STDOUT)
print("pipeline + PONTE DI COMANDO, %.0f s. Il rover puo' muoversi."
      % SECS, flush=True)
t0 = time.time()
while time.time() - t0 < SECS:
    rclpy.spin_once(node, timeout_sec=0.1)
proc.terminate()
try:
    proc.wait(timeout=10)
except subprocess.TimeoutExpired:
    proc.kill()
pl.close()
brg.terminate()
try:
    brg.wait(timeout=10)
except subprocess.TimeoutExpired:
    brg.kill()
bl.close()
stop.set()
node.destroy_node()
rclpy.shutdown()
zen.terminate()
try:
    zen.wait(timeout=8)
except subprocess.TimeoutExpired:
    zen.kill()
zl.close()

print("\n--- dove si ferma il comando ---")
print("  /cmd_vel_nominal        %5d msg, %5d con surge>0" % (n["nominal"], n["nominal_fwd"]))
print("  /planner/cmd_vel_safe   %5d msg, %5d con surge>0" % (n["safe"], n["safe_fwd"]))
print("  /real/adapter_debug     %5d msg" % n["dbg"])
print("  thruster (via BlueOS)   %5d letture, %5d con deviazione, max %d us"
      % (servo["samples"], servo["moved"], servo["max_dev"]))
for d in dbg_last:
    print("     adapter:", d)
if n["nominal_fwd"] == 0:
    print("\n-> il COMANDO NOMINALE non chiede mai avanti")
elif n["safe_fwd"] == 0:
    print("\n-> il PIANIFICATORE azzera il comando")
elif servo["moved"] == 0:
    print("\n-> l'ADATTATORE non arriva ai thruster")
else:
    print("\n-> i thruster RICEVONO comando (max %d us)" % servo["max_dev"])
