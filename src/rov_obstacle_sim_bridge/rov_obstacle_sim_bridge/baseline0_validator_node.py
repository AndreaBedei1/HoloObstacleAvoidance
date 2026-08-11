"""Scientific Baseline 0 validator: physical metrics from GT + runtime topics.

VALIDATOR-ONLY consumer: this node legitimately subscribes ground-truth and
sim-debug topics (pose GT, obstacle world geometry, dynamics debug) because it
never feeds the control path — it only measures it.

Metrics written (JSON, overwritten every write_period_s so any teardown order
is safe): collision, min clearance, forward progress, max lateral deviation,
final lateral/yaw error, return-to-line, avoidance cycles, side switches,
maneuver time, path length, thruster peak/mean utilization + saturation time,
command smoothness, odometry error stats (aligned frames) incl. max during
maneuver, watchdog/dropout observations, planner state sequence.

Nominal line: captured from GT pose at the time the first nonzero nominal
command arrives (start point + heading), like the pre-existing validator.
"""

from __future__ import annotations

import json
import math
import os

import rclpy
from geometry_msgs.msg import PoseStamped, Twist, Vector3Stamped
from rclpy.node import Node
from rov_obstacle_msgs.msg import AvoidanceDebug, Obstacle2DArray
from std_msgs.msg import Float32, String


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class Baseline0ValidatorNode(Node):
    def __init__(self) -> None:
        super().__init__("baseline0_validator")
        self.declare_parameter("output_path", "logs/baseline0_validation.json")
        self.declare_parameter("write_period_s", 2.0)
        self.declare_parameter("vehicle_radius_m", 0.3)
        self.declare_parameter("return_lateral_tol_m", 0.5)
        self.declare_parameter("return_yaw_tol_deg", 10.0)
        self.declare_parameter("max_thrust_n", 28.75)
        self.declare_parameter("label", "baseline0")

        self._out = str(self.get_parameter("output_path").value)
        self._veh_r = float(self.get_parameter("vehicle_radius_m").value)
        self._max_thrust = float(self.get_parameter("max_thrust_n").value)

        # GT state
        self._gt = None                 # latest (x, y, z, yaw, t)
        self._gt_first = None
        self._gt_prev = None
        self._path_len = 0.0
        self._min_center_dist = float("inf")
        self._min_clear_point = None
        self._max_lat_dev = 0.0
        self._max_fwd = 0.0
        self._depth_min = float("inf")
        self._depth_max = -float("inf")
        self._roll_max = 0.0
        self._pitch_max = 0.0

        # Nominal line
        self._line = None               # (x0, y0, yaw0)

        # Odometry
        self._odo = None
        self._odo_first = None
        self._odo_err_sum = 0.0
        self._odo_err_n = 0
        self._odo_err_max = 0.0
        self._odo_err_max_maneuver = 0.0

        # Planner state machine observation
        self._states = []               # (t, state, side)
        self._state = "NONE"
        self._avoid_entries = 0
        self._side_switches = 0
        self._last_side = ""
        self._maneuver_t0 = None
        self._maneuver_t1 = None
        self._in_maneuver = False

        # Commands
        self._nominal_count = 0
        self._safe_prev = None
        self._smooth_sum = [0.0, 0.0, 0.0]
        self._smooth_n = 0
        self._safe_count = 0

        # Dynamics debug
        self._thr_peak = 0.0
        self._thr_mean_sum = 0.0
        self._thr_mean_n = 0
        self._sat_ticks = 0
        self._dyn_ticks = 0
        self._watchdog_ticks = 0
        # Decimated diagnostic time series (1 sample per 8 messages).
        self._dyn_series = []
        self._odo_series = []

        # Perception
        self._det_count = 0
        self._first_det_t = None
        self._dropout_events = []

        # Temporal estimator observation (Phase 7)
        self._trk_msgs = 0
        self._trk_pred_ticks = 0
        self._trk_max_since_meas = 0.0
        self._trk_method = None
        self._raw_det_count = 0
        # Phase 7B qualification observation
        self._qual_last = None
        self._first_raw_det_t = None
        self._planner_valid_first_t = None
        self._dist_first_planner_valid = None
        self._dist_at_commit = None

        # Obstacles (world geometry)
        self._obstacles = []

        self.create_subscription(PoseStamped, "/rov/pose_ground_truth",
                                 self._on_gt, 20)
        self.create_subscription(PoseStamped, "/rov/odom_estimated",
                                 self._on_odo, 20)
        self.create_subscription(Twist, "/cmd_vel_nominal", self._on_nominal, 10)
        self.create_subscription(Twist, "/planner/cmd_vel_safe", self._on_safe, 20)
        self.create_subscription(AvoidanceDebug, "/avoidance/debug",
                                 self._on_state, 20)
        self.create_subscription(Obstacle2DArray, "/perception/obstacles",
                                 self._on_det, 20)
        self.create_subscription(String, "/sim/dynamics_debug", self._on_dyn, 20)
        self.create_subscription(String, "/sim/obstacles_world",
                                 self._on_obstacles, 10)
        self.create_subscription(String, "/sim/dropout_debug",
                                 self._on_dropout, 10)
        self.create_subscription(String, "/tracking/obstacles_debug",
                                 self._on_tracking_debug, 20)
        self.create_subscription(String, "/tracking/qualification_debug",
                                 self._on_qual_debug, 20)
        self.create_subscription(Obstacle2DArray, "/perception/obstacles_raw",
                                 self._on_raw_det, 20)
        self.create_subscription(Float32, "/rov/depth", self._on_depth, 10)
        self.create_subscription(Vector3Stamped, "/rov/attitude_measured",
                                 self._on_att, 10)

        period = float(self.get_parameter("write_period_s").value)
        self.create_timer(period, self._write)
        self._t0 = self._now()
        self.get_logger().info(f"baseline0 validator -> {self._out}")

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # --- callbacks ---------------------------------------------------------
    def _on_gt(self, msg: PoseStamped) -> None:
        t = self._now()
        x, y, z = (msg.pose.position.x, msg.pose.position.y,
                   msg.pose.position.z)
        yaw = yaw_from_quaternion(msg.pose.orientation)
        if self._gt_first is None:
            self._gt_first = (x, y, z, yaw)
        if self._gt_prev is not None:
            self._path_len += math.hypot(x - self._gt_prev[0],
                                         y - self._gt_prev[1])
        self._gt_prev = (x, y)
        self._gt = (x, y, z, yaw, t)

        for ob in self._obstacles:
            d = math.hypot(x - ob["x"], y - ob["y"])
            if d < self._min_center_dist:
                self._min_center_dist = d
                self._min_clear_point = {"x": x, "y": y, "t": t,
                                         "obstacle": ob.get("name")}

        if self._line is not None:
            x0, y0, yaw0 = self._line
            dx, dy = x - x0, y - y0
            fwd = dx * math.cos(yaw0) + dy * math.sin(yaw0)
            lat = -dx * math.sin(yaw0) + dy * math.cos(yaw0)
            self._max_fwd = max(self._max_fwd, fwd)
            self._max_lat_dev = max(self._max_lat_dev, abs(lat))

        self._update_odo_error()

    def _on_odo(self, msg: PoseStamped) -> None:
        x, y = msg.pose.position.x, msg.pose.position.y
        yaw = yaw_from_quaternion(msg.pose.orientation)
        if self._odo_first is None:
            self._odo_first = (x, y, yaw)
        self._odo = (x, y, yaw)
        self._update_odo_error()

    def _update_odo_error(self) -> None:
        if self._gt is None or self._odo is None \
                or self._gt_first is None or self._odo_first is None:
            return
        gx = self._gt[0] - self._gt_first[0]
        gy = self._gt[1] - self._gt_first[1]
        ox = self._odo[0] - self._odo_first[0]
        oy = self._odo[1] - self._odo_first[1]
        err = math.hypot(gx - ox, gy - oy)
        self._odo_err_sum += err
        self._odo_err_n += 1
        self._odo_err_max = max(self._odo_err_max, err)
        if self._in_maneuver:
            self._odo_err_max_maneuver = max(self._odo_err_max_maneuver, err)
        if self._odo_err_n % 16 == 0:
            self._odo_series.append({
                "t": round(self._now() - self._t0, 2),
                "gt_dx": round(gx, 3), "gt_dy": round(gy, 3),
                "odo_dx": round(ox, 3), "odo_dy": round(oy, 3),
                "err": round(err, 3),
            })

    def _on_nominal(self, msg: Twist) -> None:
        self._nominal_count += 1
        if self._line is None and abs(msg.linear.x) > 1e-3 and self._gt:
            x, y, _, yaw, _ = self._gt
            self._line = (x, y, yaw)
            self.get_logger().info(
                f"nominal line captured at ({x:.2f},{y:.2f}) "
                f"yaw {math.degrees(yaw):.1f} deg")

    def _on_safe(self, msg: Twist) -> None:
        self._safe_count += 1
        cur = (msg.linear.x, msg.linear.y, msg.angular.z)
        if self._safe_prev is not None:
            for i in range(3):
                self._smooth_sum[i] += abs(cur[i] - self._safe_prev[i])
            self._smooth_n += 1
        self._safe_prev = cur

    def _on_state(self, msg: AvoidanceDebug) -> None:
        t = self._now()
        state = msg.current_state
        side = msg.selected_side
        if state != self._state:
            self._states.append({"t": round(t - self._t0, 3), "state": state,
                                 "side": side})
            if state.startswith("AVOIDING"):
                self._avoid_entries += 1
                if self._maneuver_t0 is None:
                    self._maneuver_t0 = t
                    self._dist_at_commit = self._gt_obstacle_distance()
            if state != "NORMAL" and not self._in_maneuver \
                    and state != "NONE":
                self._in_maneuver = True
            if state == "NORMAL" and self._in_maneuver:
                self._in_maneuver = False
                self._maneuver_t1 = t
            self._state = state
        # Count only genuine left<->right reversals; 'none'/'' mean no side.
        if side in ("left", "right"):
            if self._last_side in ("left", "right") and side != self._last_side:
                self._side_switches += 1
            self._last_side = side

    def _on_det(self, msg: Obstacle2DArray) -> None:
        if msg.obstacles:
            self._det_count += 1
            if self._first_det_t is None:
                self._first_det_t = self._now() - self._t0
            # With Phase-7B gating, the first nonempty planner-side message
            # IS the first planner-valid moment.
            if self._planner_valid_first_t is None:
                self._planner_valid_first_t = self._now() - self._t0
                self._dist_first_planner_valid = self._gt_obstacle_distance()

    def _on_dyn(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self._dyn_ticks += 1
        if data.get("watchdog_active"):
            self._watchdog_ticks += 1
        dyn = data.get("dynamics") or {}
        forces = dyn.get("thruster_forces") or []
        if forces:
            peak = max(abs(f) for f in forces)
            self._thr_peak = max(self._thr_peak, peak)
            self._thr_mean_sum += sum(abs(f) for f in forces) / len(forces)
            self._thr_mean_n += 1
            if peak >= self._max_thrust - 0.05:
                self._sat_ticks += 1
        if self._dyn_ticks % 8 == 0:
            sp = dyn.get("setpoint") or {}
            ach = dyn.get("achieved_body_velocity") or [0, 0, 0]
            tgt = dyn.get("target") or {}
            self._dyn_series.append({
                "t": round(self._now() - self._t0, 2),
                "target_surge": round(float(tgt.get("surge", 0.0)), 3),
                "sp_surge": round(float(sp.get("surge", 0.0)), 3),
                "ach_surge": round(float(ach[0]), 3),
                "ach_sway": round(float(ach[1]), 3),
                "peak_force": round(peak if forces else 0.0, 2),
            })

    def _on_obstacles(self, msg: String) -> None:
        if self._obstacles:
            return
        try:
            raw = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        for ob in raw:
            pos = ob.get("position") or [0, 0, 0]
            self._obstacles.append({
                "name": ob.get("name", "?"),
                "class": ob.get("class_name", "?"),
                "x": float(pos[0]), "y": float(pos[1]), "z": float(pos[2]),
                "radius_m": float(ob.get("radius_m", 1.0)),
            })
        self.get_logger().info(f"obstacle geometry: {self._obstacles}")

    def _on_dropout(self, msg: String) -> None:
        try:
            self._dropout_events.append(json.loads(msg.data))
        except json.JSONDecodeError:
            pass

    def _on_tracking_debug(self, msg: String) -> None:
        try:
            d = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self._trk_msgs += 1
        self._trk_method = d.get("method", self._trk_method)
        for tr in d.get("tracks", []):
            if tr.get("is_predicted"):
                self._trk_pred_ticks += 1
            self._trk_max_since_meas = max(
                self._trk_max_since_meas,
                float(tr.get("time_since_meas_s", 0.0)))

    def _on_raw_det(self, msg: Obstacle2DArray) -> None:
        if msg.obstacles:
            self._raw_det_count += 1
            if self._first_raw_det_t is None:
                self._first_raw_det_t = self._now() - self._t0

    def _on_qual_debug(self, msg: String) -> None:
        try:
            self._qual_last = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _gt_obstacle_distance(self):
        if self._gt is None or not self._obstacles:
            return None
        x, y = self._gt[0], self._gt[1]
        return min(math.hypot(x - ob["x"], y - ob["y"])
                   for ob in self._obstacles)

    def _on_depth(self, msg: Float32) -> None:
        self._depth_min = min(self._depth_min, msg.data)
        self._depth_max = max(self._depth_max, msg.data)

    def _on_att(self, msg: Vector3Stamped) -> None:
        self._roll_max = max(self._roll_max, abs(msg.vector.x))
        self._pitch_max = max(self._pitch_max, abs(msg.vector.y))

    # --- report ------------------------------------------------------------
    def _report(self) -> dict:
        obstacle_r = self._obstacles[0]["radius_m"] if self._obstacles else None
        min_clear = None
        collision = None
        if self._obstacles and math.isfinite(self._min_center_dist):
            min_clear = self._min_center_dist - obstacle_r
            collision = bool(min_clear <= self._veh_r)

        final_lat = final_yaw_deg = returned = None
        if self._line is not None and self._gt is not None:
            x0, y0, yaw0 = self._line
            x, y, _, yaw, _ = self._gt
            dx, dy = x - x0, y - y0
            final_lat = -dx * math.sin(yaw0) + dy * math.cos(yaw0)
            final_yaw_deg = math.degrees(wrap_pi(yaw - yaw0))
            returned = bool(
                abs(final_lat)
                <= float(self.get_parameter("return_lateral_tol_m").value)
                and abs(final_yaw_deg)
                <= float(self.get_parameter("return_yaw_tol_deg").value)
                and self._state == "NORMAL")

        maneuver_time = None
        if self._maneuver_t0 is not None and self._maneuver_t1 is not None:
            maneuver_time = self._maneuver_t1 - self._maneuver_t0

        return {
            "label": str(self.get_parameter("label").value),
            "elapsed_s": round(self._now() - self._t0, 2),
            "test_type": "simulation dynamics integration baseline",
            "perception_source": "oracle relay (NOT a visual result)",
            "collision": collision,
            "min_center_distance_m": (None if not math.isfinite(self._min_center_dist)
                                      else round(self._min_center_dist, 3)),
            "min_clearance_m": (None if min_clear is None
                                else round(min_clear, 3)),
            "min_clearance_point": self._min_clear_point,
            "obstacle_geometry": self._obstacles,
            "vehicle_radius_m": self._veh_r,
            "forward_progress_m": round(self._max_fwd, 3),
            "max_lateral_deviation_m": round(self._max_lat_dev, 3),
            "final_lateral_error_m": (None if final_lat is None
                                      else round(final_lat, 3)),
            "final_yaw_error_deg": (None if final_yaw_deg is None
                                    else round(final_yaw_deg, 2)),
            "returned_to_original_line": returned,
            "avoidance_entries": self._avoid_entries,
            "side_switches": self._side_switches,
            "maneuver_time_s": (None if maneuver_time is None
                                else round(maneuver_time, 2)),
            "path_length_m": round(self._path_len, 3),
            "state_sequence": self._states,
            "final_state": self._state,
            "thruster_peak_n": round(self._thr_peak, 2),
            "thruster_peak_utilization": round(
                self._thr_peak / self._max_thrust, 3),
            "thruster_mean_abs_n": (round(self._thr_mean_sum / self._thr_mean_n, 3)
                                    if self._thr_mean_n else None),
            "saturation_ticks": self._sat_ticks,
            "saturation_fraction": (round(self._sat_ticks / self._dyn_ticks, 4)
                                    if self._dyn_ticks else None),
            "watchdog_ticks": self._watchdog_ticks,
            "cmd_smoothness_mean_delta": (
                [round(s / self._smooth_n, 5) for s in self._smooth_sum]
                if self._smooth_n else None),
            "odo_err_mean_m": (round(self._odo_err_sum / self._odo_err_n, 3)
                               if self._odo_err_n else None),
            "odo_err_max_m": round(self._odo_err_max, 3),
            "odo_err_max_during_maneuver_m": round(
                self._odo_err_max_maneuver, 3),
            "depth_min": (None if not math.isfinite(self._depth_min)
                          else round(self._depth_min, 3)),
            "depth_max": (None if not math.isfinite(self._depth_max)
                          else round(self._depth_max, 3)),
            "roll_max_deg": round(math.degrees(self._roll_max), 2),
            "pitch_max_deg": round(math.degrees(self._pitch_max), 2),
            "first_detection_t_s": self._first_det_t,
            "nonempty_detection_msgs": self._det_count,
            "raw_nonempty_detection_msgs": self._raw_det_count,
            "time_to_first_raw_detection_s": self._first_raw_det_t,
            "planner_valid_first_t_s": self._planner_valid_first_t,
            "confirmation_delay_s": (
                round(self._planner_valid_first_t - self._first_raw_det_t, 3)
                if self._planner_valid_first_t is not None
                and self._first_raw_det_t is not None else None),
            "distance_at_first_planner_valid_m": (
                round(self._dist_first_planner_valid, 3)
                if self._dist_first_planner_valid is not None else None),
            "distance_at_commitment_m": (
                round(self._dist_at_commit, 3)
                if self._dist_at_commit is not None else None),
            "qualification": self._qual_last,
            "estimator_method": self._trk_method,
            "estimator_debug_msgs": self._trk_msgs,
            "estimator_prediction_ticks": self._trk_pred_ticks,
            "estimator_max_time_since_meas_s": round(
                self._trk_max_since_meas, 3),
            "dropout_events": self._dropout_events,
            "msg_counts": {"nominal": self._nominal_count,
                           "safe": self._safe_count,
                           "dynamics_debug": self._dyn_ticks},
            "dyn_series": self._dyn_series,
            "odo_series": self._odo_series,
        }

    def _write(self) -> None:
        os.makedirs(os.path.dirname(self._out) or ".", exist_ok=True)
        # Atomic write: a hard kill mid-write must never leave a truncated
        # JSON (it corrupted a campaign run before this fix).
        tmp = self._out + ".tmp"
        with open(tmp, "w") as f:
            json.dump(self._report(), f, indent=2)
        os.replace(tmp, self._out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Baseline0ValidatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._write()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
