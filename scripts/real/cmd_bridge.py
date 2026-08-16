"""Command bridge: /planner/cmd_vel_safe -> MANUAL_CONTROL, live.

WHY THIS EXISTS. The packaged adapter node would not pick up the
measured calibration in this build environment: every edit reached the
source and the installed copy, the node loaded that exact file, and the
running process still reported `calibration_id: uncalibrated` and stayed
in shadow through six real attempts with the thrusters idle. With one
pool day left, chasing the build was the wrong trade.

WHAT IS PRESERVED. This is not a shortcut around the science. It reads
the SAME frozen boundary topic, uses the SAME tested conversion
(`command_mapping.twist_to_manual_control`, 19 tests covering the sign
conventions, the deadband, saturation and the fail-closed behaviour on
NaN), and the SAME measured calibration file the simulated plant model
reads. What it drops is the ROS packaging around that conversion, not
the conversion.

SAFETY. Fail-closed: a non-finite command, a stale command or a lost
link all produce neutral, never a held setpoint. Heave is never
commanded -- depth belongs to the vehicle's own controller.
"""
from __future__ import annotations
import json, math, os, sys, threading, time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
sys.path.insert(0, os.path.join(_ROOT, "src", "rov_real_bridge"))

from rov_real_bridge.command_mapping import (      # noqa: E402
    AxisCalibration, CommandMapping, twist_to_manual_control)
from rovlink import RovLink, Z_NEUTRAL             # noqa: E402


def mapping_from_file():
    path = os.path.join(_ROOT, "config", "calibration", "s3_vehicle.json")
    with open(path) as f:
        prof = json.load(f)["profile"]
    r = math.pi / 180.0
    su = float((prof.get("surge_symmetric") or {}).get("m_s_per_count") or 0)
    sw = float((prof.get("sway_symmetric") or {}).get("m_s_per_count") or 0)
    yp, yn = prof.get("yaw+") or {}, prof.get("yaw-") or {}
    def ax(kp, kn, dbp, dbn, mx):
        return AxisCalibration(k_pos=kp, k_neg=kn, db_pos=dbp, db_neg=dbn,
                               min_command=0.0, max_command=mx,
                               calibrated=kp > 0 and kn > 0,
                               source="s3_vehicle_20260815")
    return CommandMapping(
        surge=ax(su, su, 0.0, 0.0, 0.30),
        sway=ax(sw, sw, 0.0, 0.0, 0.25),
        yaw=ax(float(yp.get("deg_s_per_count") or 0) * r,
               float(yn.get("deg_s_per_count") or 0) * r,
               float(yp.get("deadband_counts") or 0),
               float(yn.get("deadband_counts") or 0), 0.60))


PIDFILE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       ".cmd_bridge.pid")


def main() -> int:
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
    # Record this process so the next run can kill it. The bridge is a
    # plain python process, so the runner's node cleanup (which matches
    # executable names) never touched it: eleven of them accumulated,
    # each streaming MANUAL_CONTROL at 20 Hz, and the vehicle obeyed
    # whichever arrived last. That is the jerking, and it is the third
    # time today the same cause wore a different disguise.
    try:
        with open(PIDFILE, "w") as f:
            f.write(str(os.getpid()))
    except OSError:
        pass
    # The same interlock the packaged adapter enforces, kept explicitly
    # rather than lost with the ROS wrapper: actuation requires the
    # vehicle to be declared in water AND actuation to be allowed, and
    # both must be passed deliberately.
    from rov_real_bridge.safety_interlock import SafetyInterlock
    decision = SafetyInterlock.from_params({
        "vehicle_in_water": os.environ.get("ROV_IN_WATER") == "1",
        "allow_real_actuation": os.environ.get("ROV_ALLOW_ACTUATION") == "1",
        "real_control_mode": os.environ.get("ROV_CONTROL_MODE", "shadow"),
    }).evaluate()
    if not decision.may_actuate:
        print("ABORT interlock:", decision.reason)
        return 4
    print("interlock:", decision.reason, flush=True)
    mapping = mapping_from_file()
    print("calibrazione: surge k=%.6f  sway k=%.6f  yaw k+=%.6f k-=%.6f"
          % (mapping.surge.k_pos, mapping.sway.k_pos,
             mapping.yaw.k_pos, mapping.yaw.k_neg), flush=True)
    if not mapping.fully_calibrated:
        print("ABORT: assi non calibrati", mapping.uncalibrated_axes())
        return 2

    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node

    state = {"twist": None, "t": 0.0, "sent": 0, "moved": 0}
    lock = threading.Lock()

    class Sub(Node):
        def __init__(self):
            super().__init__("cmd_bridge")
            self.create_subscription(Twist, "/planner/cmd_vel_safe",
                                     self.on_cmd, 20)

        def on_cmd(self, m):
            with lock:
                state["twist"] = (m.linear.x, m.linear.y, m.angular.z)
                state["t"] = time.time()

    rclpy.init()
    node = Sub()
    stop = threading.Event()

    def tx():
        with RovLink() as rov:
            rov.set_mode("ALT_HOLD")
            time.sleep(0.5)
            rov.arm(mode="ALT_HOLD")
            t_arm = time.time()
            # 0 = disabled: back to the behaviour the operator approved.
            straight_s = float(os.environ.get("ROV_STRAIGHT_S", "0"))
            min_surge = float(os.environ.get("ROV_MIN_SURGE", "0.12"))
            latch = {"sign": 0.0, "t": 0.0}
            # 0 = DISABLED. Latching the side for 8 s stopped the
            # 400-per-run oscillation but made the vehicle insist on one
            # direction long after the planner wanted the other, and it
            # drove into the pool wall. The oscillation is the lesser
            # problem: it wastes lateral travel, the wall does damage.
            latch_hold = float(os.environ.get("ROV_SIDE_LATCH_S", "0"))
            print("armato, invio a 20 Hz (dritto i primi %.1f s)"
                  % straight_s, flush=True)
            while not stop.is_set():
                with lock:
                    tw, age = state["twist"], time.time() - state["t"]
                if tw is None or age > 1.0:      # fail closed
                    rov.manual(z=Z_NEUTRAL)
                else:
                    # STRAIGHT RUN-IN. The vehicle goes straight for the
                    # first second whatever the planner says, so every
                    # run starts from the same clean approach instead of
                    # side-stepping off the line before it has moved.
                    # Operationally requested and applied identically to
                    # both planners, so it cannot favour either.
                    if time.time() - t_arm < straight_s:
                        tw = (tw[0], 0.0, 0.0)
                    else:
                        # SIDE LATCH. The detector's box jumps between
                        # frames, so the planner re-picks the avoidance
                        # side constantly: run 1 changed side 400 times
                        # in 25 s, the lateral motion cancelled itself
                        # out and the vehicle drove into the anchor.
                        # Once a side is chosen it is held, which is what
                        # "committed" is supposed to mean.
                        lat = tw[1]
                        now = time.time()
                        if latch_hold > 0 and abs(lat) > 0.02:
                            if (latch["sign"] == 0
                                    or now - latch["t"] > latch_hold):
                                latch["sign"] = 1.0 if lat > 0 else -1.0
                                latch["t"] = now
                            tw = (tw[0], abs(lat) * latch["sign"], tw[2])
                    # COMMON FORWARD FLOOR, applied to both planners at
                    # the actuation layer. The committed planner has a
                    # min-surge parameter of its own and DWA has none, so
                    # raising the committed planner's parameter -- which
                    # is what made it move this morning -- quietly
                    # advantaged it. DWA brakes to zero 47 % of the time
                    # on intermittent perception and never reached the
                    # anchor. One floor, one place, same for both.
                    # The floor applies only when the planner is ALIVE
                    # and commanding. A dead planner publishes zeros, and
                    # a floor over that drove the vehicle straight into
                    # the anchor at 0.12 m/s with nothing steering it.
                    if abs(tw[0]) > 0.005 or abs(tw[1]) > 0.005:
                        tw = (max(tw[0], min_surge), tw[1], tw[2])
                    mc = twist_to_manual_control(tw[0], tw[1], tw[2],
                                                 mapping)
                    rov.manual(x=mc["x"], y=mc["y"], z=Z_NEUTRAL,
                               r=mc["r"])
                    state["sent"] += 1
                    if abs(mc["x"]) + abs(mc["y"]) + abs(mc["r"]) > 0:
                        state["moved"] += 1
                time.sleep(0.05)
            rov.neutral()
            time.sleep(0.3)
            rov.disarm()

    th = threading.Thread(target=tx, daemon=True)
    th.start()
    t0 = time.time()
    while time.time() - t0 < secs:
        rclpy.spin_once(node, timeout_sec=0.1)
    stop.set()
    th.join(timeout=8)
    node.destroy_node()
    rclpy.shutdown()
    print("inviati %d comandi, di cui %d non nulli"
          % (state["sent"], state["moved"]))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
