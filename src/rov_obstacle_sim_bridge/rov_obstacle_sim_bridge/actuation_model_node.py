"""S3: the measured PLANT, strictly downstream of the frozen boundary.

WHERE THIS SITS, AND WHY IT MATTERS. The scientific boundary is

    /perception/obstacles_raw -> Phase7B -> T2 -> Planner C/D
                              -> /planner/cmd_vel_safe

and it is IDENTICAL in simulation and reality. S3 is the calibration of
what happens AFTER that topic: the actuator chain and the vehicle.

An earlier version of S3 lowered the planner's own sway limit. That was
wrong. It changed the simulator and the planner at the same time, so a
difference between S2 and S3 could no longer be attributed to the
vehicle model rather than to a different controller, and the causal
reading of the S0 -> S3 ladder collapsed. The planner configuration is
frozen and IDENTICAL at every level and in the real campaign; if the
planner asks for 0.30 m/s of sway and the vehicle can only deliver
0.12, that shortfall must appear HERE, exactly as it does on the real
vehicle, where the planner also asks for more than the thrusters give.

THE MODEL. Each axis is passed through the SAME affine deadband map the
real adapter uses (`rov_real_bridge/command_mapping.py`), forward and
then back:

    counts   = sign(v) * (deadband + |v| / k)      clamped to +/- 1000
    achieved = sign(counts) * k * (|counts| - deadband)   or 0

so saturation, the deadband and the per-sign asymmetry all fall out of
the measured numbers instead of being re-stated as separate rules. A
first-order lag with the measured rise and decay constants then turns
the achievable steady speed into what the vehicle is doing right now.

WHAT IS NOT INJECTED. Command latency was never measured on the real
vehicle, so it is exposed as a parameter and left at zero rather than
invented; a plausible-looking transport delay would be indistinguishable
from a real one in the results and could not be defended. The rise
constants are weakly identified (their spread is as large as their
value) and that is recorded with them.

At S0, S1 and S2 this node is an identity pass-through. It stays in the
graph at every level so the topology, and therefore the timing, is the
same throughout the ladder.
"""

from __future__ import annotations

import json
import math
import os

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

FULL_SCALE = 1000.0          # MANUAL_CONTROL counts at full command


class Axis:
    """One signed degree of freedom of the measured actuator chain."""

    def __init__(self, k_pos, k_neg, db_pos, db_neg,
                 tau_rise, tau_decay):
        self.k_pos = float(k_pos)
        self.k_neg = float(k_neg)
        self.db_pos = float(db_pos)
        self.db_neg = float(db_neg)
        self.tau_rise = max(1e-3, float(tau_rise))
        self.tau_decay = max(1e-3, float(tau_decay))
        self.state = 0.0

    def achievable(self, v: float) -> float:
        """Steady speed the vehicle can actually hold for request `v`."""
        if v == 0.0:
            return 0.0
        k = self.k_pos if v > 0 else self.k_neg
        db = self.db_pos if v > 0 else self.db_neg
        if k <= 0.0:
            return v
        counts = db + abs(v) / k
        counts = min(counts, FULL_SCALE)
        if counts <= db:
            return 0.0
        return math.copysign(k * (counts - db), v)

    def step(self, v_req: float, dt: float) -> float:
        """First-order approach to the achievable speed."""
        target = self.achievable(v_req)
        tau = self.tau_rise if abs(target) >= abs(self.state) \
            else self.tau_decay
        alpha = 1.0 - math.exp(-max(0.0, dt) / tau)
        self.state += alpha * (target - self.state)
        return self.state


def axes_from_profile(path: str):
    """Build the three axes from the measured S3 fit file."""
    with open(path) as f:
        prof = json.load(f)["profile"]

    def sym(name, default_k):
        e = prof.get("%s_symmetric" % name) or {}
        k = float(e.get("m_s_per_count") or default_k)
        db = float(e.get("deadband_counts") or 0.0)
        return k, k, db, db

    def dyn(name, d_rise, d_decay):
        e = prof.get("%s_dynamics" % name) or {}
        return (float(e.get("tau_rise_s") or d_rise),
                float(e.get("tau_decay_s") or d_decay))

    ks, kn, dp, dn = sym("surge", 1.45e-4)
    tr, td = dyn("surge", 1.0, 1.0)
    surge = Axis(ks, kn, dp, dn, tr, td)

    ks, kn, dp, dn = sym("sway", 1.23e-4)
    tr, td = dyn("sway", 1.0, 1.0)
    sway = Axis(ks, kn, dp, dn, tr, td)

    # Yaw keeps its per-sign parameters: two levels per direction from
    # the IMU, which the pool disturbance does not affect, so the
    # asymmetry there is measured rather than inferred.
    yp = prof.get("yaw+") or {}
    yn = prof.get("yaw-") or {}
    rad = math.radians(1.0)
    yaw = Axis(float(yp.get("deg_s_per_count") or 0.065) * rad,
               float(yn.get("deg_s_per_count") or 0.054) * rad,
               float(yp.get("deadband_counts") or 0.0),
               float(yn.get("deadband_counts") or 0.0),
               *dyn("surge", 1.0, 1.0))
    return surge, sway, yaw


class ActuationModel(Node):

    def __init__(self) -> None:
        super().__init__("actuation_model")
        self.declare_parameter("input_topic", "/planner/cmd_vel_safe")
        self.declare_parameter("output_topic", "/rov/cmd_vel_plant")
        self.declare_parameter("calibration_level", "S0")
        self.declare_parameter("s3_fit_path", "")
        self.declare_parameter("status_path", "")
        # Never measured on the real vehicle; left at zero rather than
        # invented. See the module docstring.
        self.declare_parameter("command_latency_s", 0.0)

        self.level = str(self.get_parameter(
            "calibration_level").value).upper().strip()
        self.latency = float(self.get_parameter("command_latency_s").value)
        self.axes = None
        if self.level == "S3":
            path = str(self.get_parameter("s3_fit_path").value or "")
            if not path or not os.path.isfile(path):
                raise RuntimeError(
                    "calibration_level=S3 requires s3_fit_path; refusing "
                    "to run an uncalibrated plant that would be reported "
                    "as a calibrated one (got %r)" % path)
            self.axes = axes_from_profile(path)

        self._t_last = None
        self._queue = []
        self.pub = self.create_publisher(
            Twist, str(self.get_parameter("output_topic").value), 10)
        self.create_subscription(
            Twist, str(self.get_parameter("input_topic").value),
            self.on_cmd, 10)
        self.get_logger().info(
            "actuation model: level=%s plant=%s latency=%.3f s"
            % (self.level, bool(self.axes), self.latency))
        status = str(self.get_parameter("status_path").value or "")
        if status:
            try:
                with open(status, "w") as f:
                    json.dump({"level": self.level,
                               "plant_active": bool(self.axes),
                               "command_latency_s": self.latency}, f)
            except OSError as exc:
                self.get_logger().error("cannot write status: %s" % exc)

    def on_cmd(self, msg: Twist) -> None:
        if self.axes is None:
            self.pub.publish(msg)       # identity at S0, S1, S2
            return
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = 0.05 if self._t_last is None else max(1e-3, now - self._t_last)
        self._t_last = now
        surge, sway, yaw = self.axes
        out = Twist()
        out.linear.x = surge.step(msg.linear.x, dt)
        out.linear.y = sway.step(msg.linear.y, dt)
        # Heave is not modelled: depth is held by the vehicle's own
        # controller in both domains and the planners never command it.
        out.linear.z = msg.linear.z
        out.angular.z = yaw.step(msg.angular.z, dt)
        self.pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ActuationModel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
