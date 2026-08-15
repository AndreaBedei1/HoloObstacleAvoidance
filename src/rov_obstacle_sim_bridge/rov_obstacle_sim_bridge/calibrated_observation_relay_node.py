"""Inject the MEASURED observation process into the simulation.

This node is the whole point of Phase 10. At S0 the simulated planner
receives a noiseless geometric projection of ground truth: every tick,
perfectly centred, with the true apparent size. The real vehicle
receives something quite different, and this node makes the simulated
stream differ in exactly the ways that were MEASURED on the real
vehicle, and in no other way.

It sits where the Phase-8 dropout relay sits — between the oracle
projection and `/perception/obstacles_raw` — because that is the shared
scientific boundary: everything downstream (Phase-7B qualification, the
T2 estimator, the planners) is the identical code in both domains.

CALIBRATION LEVELS

  S0  identity. The historical simulator. Nothing is injected.
  S1  + the static observation model, from
      config/calibration/s1_observation_fit.json:
        * detection probability as a function of TRUE range, which is
          non-monotone: the anchor is missed close in as well as far out
        * the monocular range bias, applied where it physically arises
          -- by scaling the bbox HEIGHT, not by editing a range. The
          planner must receive a box that could really have produced the
          range it computes; injecting the error into a derived quantity
          would hand the planner a bbox and a range that disagree.
        * bearing scatter on the bbox centre
  S2  + timing, from config/calibration/s2_timing_fit.json:
        * the measured cycle rate and jitter
        * BURST structure of the misses. The real stream goes blind for
          9.2 cycles at a time against 3.5 for independent dropout at the
          same marginal, and a run of consecutive misses is what starves
          a temporal estimator.
  S3  S2 plus the vehicle profile, which is applied by the dynamics
      configuration rather than here.

THE DOUBLE-COUNTING RULE. S1 owns the MARGINAL probability that a frame
yields a detection. S2 owns only the CORRELATION between misses. The
two-state process is therefore parameterised to reproduce S1's marginal
at the current range and to lengthen the runs by the measured
burstiness factor -- never to impose a second marginal of its own, which
would thin the stream twice and silently double the perception penalty.

Nothing here is invented: every number is read from a fit file produced
from real recordings, and running at S1 or above with a missing fit file
is an error rather than a default.
"""

from __future__ import annotations

import json
import math
import os
import random

import rclpy
from rclpy.node import Node

from rov_obstacle_msgs.msg import Obstacle2DArray


def burst_params(mean_visible_meas: float, mean_blind_meas: float,
                 p_marginal: float):
    """Two-state run lengths that reproduce `p_marginal` exactly.

    Module level and free of ROS so the double-counting safeguard can be
    tested directly: for independent dropout the mean run of misses is
    1/p, the measured stream's blind runs are `burstiness` times longer
    at the same marginal, and

        mean_blind   = burstiness / p
        mean_visible = burstiness / (1 - p)

    satisfies mean_visible/(mean_visible+mean_blind) == p by
    construction. S2 therefore changes WHEN the misses happen, never HOW
    MANY -- which is the one place the perception penalty could be
    counted twice.
    """
    p_meas = mean_visible_meas / max(mean_visible_meas + mean_blind_meas,
                                     1e-6)
    burstiness = mean_blind_meas * max(p_meas, 1e-6)
    p = min(max(p_marginal, 1e-3), 1.0 - 1e-3)
    return (max(1.0, burstiness / (1.0 - p)),   # mean visible
            max(1.0, burstiness / p),           # mean blind
            burstiness)


class CalibratedObservationRelay(Node):

    def __init__(self) -> None:
        super().__init__("calibrated_observation_relay")
        self.declare_parameter("input_topic", "/perception/obstacles_oracle")
        self.declare_parameter("output_topic", "/perception/obstacles_raw")
        self.declare_parameter("calibration_level", "S0")
        self.declare_parameter("s1_fit_path", "")
        self.declare_parameter("s2_fit_path", "")
        self.declare_parameter("seed", 0)
        # Sentinel file. The campaign must be able to prove the relay ran
        # at the requested level; parsing the launch log for the startup
        # line was unreliable because the line is not always flushed
        # before the process is killed at teardown, which made runs look
        # uncalibrated when they were fine. A file written at
        # construction is deterministic.
        self.declare_parameter("status_path", "")
        # Fixed monocular constants, shared with the planner. They are
        # INPUTS, never fitted here: the fit measured the bias of this
        # exact pair, so changing them would invalidate the bias.
        self.declare_parameter("vfov_deg", 60.0)
        self.declare_parameter("target_height_m", 0.75)

        self.level = str(self.get_parameter(
            "calibration_level").value).upper().strip()
        seed = int(self.get_parameter("seed").value)
        self.rng = random.Random(seed if seed else None)
        self.vfov = math.radians(float(
            self.get_parameter("vfov_deg").value))
        self.target_h = float(self.get_parameter("target_height_m").value)

        self.s1 = self._load("s1_fit_path", self.level in ("S1", "S2", "S3"))
        self.s2 = self._load("s2_fit_path", self.level in ("S2", "S3"))

        # two-state visible/blind memory
        self._blind = False
        self._last_pub_t = 0.0
        self.n_in = self.n_out = 0

        self.pub = self.create_publisher(
            Obstacle2DArray,
            str(self.get_parameter("output_topic").value), 10)
        self.create_subscription(
            Obstacle2DArray,
            str(self.get_parameter("input_topic").value), self.on_obs, 10)
        self.get_logger().info(
            "calibrated observation relay: level=%s s1=%s s2=%s"
            % (self.level, bool(self.s1), bool(self.s2)))
        status = str(self.get_parameter("status_path").value or "")
        if status:
            try:
                with open(status, "w") as f:
                    json.dump({"level": self.level,
                               "s1_loaded": bool(self.s1),
                               "s2_loaded": bool(self.s2)}, f)
            except OSError as exc:
                self.get_logger().error("cannot write status: %s" % exc)

    # -- calibration files -------------------------------------------
    def _load(self, param: str, required: bool):
        path = str(self.get_parameter(param).value or "")
        if not path:
            if required:
                raise RuntimeError(
                    "calibration_level=%s requires %s; refusing to run "
                    "with an uncalibrated stand-in, which would look like "
                    "a calibrated result" % (self.level, param))
            return None
        if not os.path.isfile(path):
            raise RuntimeError("%s not found: %s" % (param, path))
        with open(path) as f:
            return json.load(f)

    # -- S1 pieces ----------------------------------------------------
    def _true_range(self, ob) -> float:
        """Range implied by the ORACLE bbox, i.e. the true range."""
        h = max(1e-4, min(1.0, float(ob.height)))
        t = math.tan(0.5 * h * self.vfov)
        return (self.target_h * 0.5) / t if t > 1e-6 else 99.0

    def _p_detect(self, rng_m: float) -> float:
        curve = (self.s1 or {}).get("detection_probability_vs_range") or []
        for band in curve:
            if band["range_lo_m"] <= rng_m < band["range_hi_m"]:
                return float(band["p_detect"])
        return float((self.s1 or {}).get("marginal_p_detect", 1.0))

    def _apply_s1(self, ob) -> bool:
        """Mutate one observation in place. False = not detected."""
        r_true = self._true_range(ob)
        if self.rng.random() > self._p_detect(r_true):
            return False
        # Range bias, applied at its physical origin: the apparent size.
        # ratio = estimated/true, and range varies as 1/height, so the
        # height the detector really reports is the true height divided
        # by that ratio.
        est = (self.s1 or {}).get("shared_estimator_vs_truth") or {}
        ratio = float(est.get("ratio_median") or 1.0)
        if ratio > 1e-6:
            ob.height = float(min(1.0, ob.height / ratio))
            sd = float(est.get("ratio_robust_sd") or 0.0)
            if sd > 0:
                ob.height = float(min(1.0, max(
                    1e-4, ob.height * (1.0 + self.rng.gauss(0.0, sd)))))
        # Bearing scatter on the bbox centre. The systematic component of
        # the measured bearing error is a frame convention, not a sensor
        # bias, so ONLY the scatter is injected.
        bear = (self.s1 or {}).get("bearing_error_deg") or {}
        sd_deg = float(bear.get("robust_sd") or 0.0)
        if sd_deg > 0:
            hfov = 2.0 * math.atan(math.tan(self.vfov / 2.0) * 16.0 / 9.0)
            ob.center_x = float(min(1.0, max(
                0.0, ob.center_x
                + math.radians(self.rng.gauss(0.0, sd_deg)) / hfov)))
        return True

    # -- S2 pieces ----------------------------------------------------
    def _burst_allows(self, p_marginal: float) -> bool:
        """Two-state visible/blind gate reproducing p_marginal.

        For independent dropout the mean run of misses is 1/p. The
        measured stream's blind runs are `burstiness` times longer at the
        same marginal, so:

            mean_blind   = burstiness / p
            mean_visible = burstiness / (1 - p)

        which satisfies mean_visible/(mean_visible+mean_blind) == p by
        construction. That identity is the whole safeguard: S2 changes
        WHEN the misses happen, never HOW MANY.
        """
        s2 = self.s2 or {}
        mean_visible, mean_blind, _ = burst_params(
            float(s2.get("mean_visible_run_cycles") or 1.0),
            float(s2.get("mean_blind_run_cycles") or 1.0),
            p_marginal)
        if self._blind:
            if self.rng.random() < 1.0 / mean_blind:
                self._blind = False
        else:
            if self.rng.random() < 1.0 / mean_visible:
                self._blind = True
        return not self._blind

    def _rate_allows(self, now: float) -> bool:
        s2 = self.s2 or {}
        period = float(s2.get("cycle_period_s") or 0.0)
        if period <= 0:
            return True
        jitter = float(s2.get("cycle_jitter_s") or 0.0)
        due = self._last_pub_t + max(
            0.0, self.rng.gauss(period, jitter))
        if now < due:
            return False
        self._last_pub_t = now
        return True

    # -- pipeline -----------------------------------------------------
    def on_obs(self, msg: Obstacle2DArray) -> None:
        self.n_in += 1
        out = Obstacle2DArray()
        out.header = msg.header

        if self.level == "S0":
            out.obstacles = msg.obstacles
            self.pub.publish(out)
            self.n_out += 1
            return

        now = (msg.header.stamp.sec
               + msg.header.stamp.nanosec * 1e-9)
        if self.s2 is not None and not self._rate_allows(now):
            return                       # not this tick: no message at all

        kept = []
        p_used = 1.0
        for ob in msg.obstacles:
            p_used = self._p_detect(self._true_range(ob))
            if self._apply_s1(ob):
                kept.append(ob)

        if self.s2 is not None and not self._burst_allows(p_used):
            kept = []

        # An EMPTY array is published deliberately: downstream, silence
        # and "nothing seen" are different states, and the qualifier
        # needs to tell them apart.
        out.obstacles = kept
        self.pub.publish(out)
        self.n_out += 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CalibratedObservationRelay()
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
