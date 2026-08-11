"""Holonomic Dynamic Window Approach — classical literature baseline.

Formulation: Fox, Burgard & Thrun (1997) dynamic-window principle —
sample DYNAMICALLY REACHABLE commands, forward-simulate a short horizon,
discard inadmissible (unsafe/unstoppable) candidates, maximize a small
classical objective — adapted to the NATIVE horizontal control space of the
BlueROV2 Heavy, which is horizontally holonomic:

    command = [u, v, r]   (body surge, body sway, yaw rate)

This adaptation is a fairness requirement (a nonholonomic (v, omega) DWA
would be handicapped against the sway-using committed planner), follows the
spirit of Eriksen et al. (2016)'s adaptation of DWA to AUV dynamics, and is
NOT claimed as a contribution.

TRANSFERABLE INPUTS ONLY (identical information to the committed planner):
  - estimated pose (x, y, yaw)  <- /rov/odom_estimated (no-DVL dead reckoning)
  - current body-velocity ESTIMATE from the planner's own commanded-response
    model (same first-order taus as the commanded odometry; NO VelocitySensor,
    NO DVL)
  - obstacles from the T2 (+qualification) output: image bbox -> bearing +
    MONOCULAR range with the same known-height assumption as the committed
    planner (target_obstacle_height_m / VFOV); class-reference radius
  - nominal route captured from the first nonzero nominal command + pose
    (same convention as the committed planner)
No ground truth, no oracle position, no simulator sensors.

Classical objective (5 weights):
    J = w_clearance * clearance_norm
      + w_progress  * route_progress_norm
      + w_speed     * forward_speed_norm
      - w_route     * cross_track_norm
      - w_smooth    * command_change_norm

Admissibility (classical stop-distance): a candidate is admissible only if
its minimum predicted clearance exceeds the braking distance
v^2 / (2 * stop_decel) — i.e. the vehicle can still stop before contact.
No admissible candidate => SAFE FALLBACK (zero command), logged as a
first-class `no_admissible_candidate` event.

Pure Python module (numpy only) for deterministic unit testing; the ROS node
wrapper lives in dwa_planner_node.py.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

import numpy as np


@dataclass
class DWAConfig:
    # --- velocity limits (same envelope as the committed planner/vehicle) ---
    max_surge: float = 0.5
    min_surge: float = 0.0          # no reverse in the primary baseline
    max_sway: float = 0.3
    max_yaw_rate: float = 0.3
    # --- acceleration limits (<= controller setpoint rate limits) ----------
    accel_lin: float = 0.5          # m/s^2 planning accel bound
    accel_yaw: float = 0.8          # rad/s^2
    # CLASSICAL fidelity: the dynamic window spans what is reachable within
    # ONE control interval (Fox 1997). With 10 Hz replanning this bounds the
    # effective command slew to accel_lin (0.5 m/s^2), matching the
    # downstream controller limits and keeping the commanded-response
    # odometry assumption valid (a 0.5 s window at 10 Hz replanning produced
    # 2.5 m/s^2 effective slew, thruster saturation and odometry divergence
    # in the first closed-loop smoke).
    window_dt: float = 0.1          # = control interval at 10 Hz
    # --- sampling -----------------------------------------------------------
    n_u: int = 7
    n_v: int = 7
    n_r: int = 5
    # --- forward simulation -------------------------------------------------
    horizon_s: float = 3.0
    sim_dt: float = 0.2
    # --- footprint / safety -------------------------------------------------
    # BlueROV2 Heavy documented horizontal extent 0.576 x 0.457 m ->
    # circumscribed radius ~0.37; declared conservative 0.40 m.
    vehicle_radius_m: float = 0.40
    # Margin must absorb the measured optimism stack: no-DVL odometry
    # drift (<=0.2 m), close-range monocular bias from image-edge bbox
    # clipping, response mismatch and rollout discretization. The committed
    # planner carries an equivalent implicit margin of 0.75 m
    # (clearance_offset 2.5 - obstacle radius 1.75).
    safety_margin_m: float = 0.80
    stop_decel: float = 0.5         # available braking deceleration m/s^2
    # --- objective weights (classical, small set) ---------------------------
    w_clearance: float = 1.0
    clearance_saturation_m: float = 2.0
    w_progress: float = 1.0
    w_speed: float = 0.3
    w_route: float = 0.5
    w_smooth: float = 0.1
    # --- perception geometry (same assumptions as committed planner) -------
    camera_vertical_fov_deg: float = 90.0
    camera_horizontal_fov_deg: float = 90.0
    target_obstacle_height_m: float = 3.5
    obstacle_radius_m: float = 1.75   # class-reference (anchor surrogate)
    max_range_m: float = 40.0
    # --- response model for the internal velocity estimate ------------------
    tau_surge_s: float = 1.2
    tau_sway_s: float = 1.15
    tau_yaw_s: float = 0.3


@dataclass
class ObstacleEstimate:
    """Local metric obstacle estimate from the visual detection (planner
    frame = odom). Produced by `obstacle_from_detection`."""
    x: float
    y: float
    radius: float
    bearing_rad: float
    range_m: float


@dataclass
class DWAResult:
    u: float
    v: float
    r: float
    admissible_count: int
    candidate_count: int
    no_admissible: bool
    best_cost: Optional[float]
    planning_time_ms: float
    min_predicted_clearance: Optional[float]
    debug: Dict[str, Any] = field(default_factory=dict)


def monocular_range(bbox_height: float, cfg: DWAConfig) -> float:
    """Same known-height pinhole model as the committed planner."""
    h = max(1e-4, min(1.0, bbox_height))
    r = cfg.target_obstacle_height_m / (
        2.0 * h * math.tan(math.radians(cfg.camera_vertical_fov_deg) / 2.0))
    return min(cfg.max_range_m, max(0.1, r))


def obstacle_from_detection(cx: float, bbox_h: float, pose_x: float,
                            pose_y: float, pose_yaw: float,
                            cfg: DWAConfig) -> ObstacleEstimate:
    """bbox -> bearing + monocular range -> odom-frame obstacle estimate.

    Image convention (project-wide): cx in [0,1], 0.5 = center; positive
    bearing = object RIGHT of center. Body frame x fwd, y LEFT =>
    obstacle body position (range*cos(b), -range*sin(b))."""
    bearing = (cx - 0.5) * math.radians(cfg.camera_horizontal_fov_deg)
    rng = monocular_range(bbox_h, cfg)
    bx = rng * math.cos(bearing)
    by = -rng * math.sin(bearing)
    ox = pose_x + bx * math.cos(pose_yaw) - by * math.sin(pose_yaw)
    oy = pose_y + bx * math.sin(pose_yaw) + by * math.cos(pose_yaw)
    return ObstacleEstimate(x=ox, y=oy, radius=cfg.obstacle_radius_m,
                            bearing_rad=bearing, range_m=rng)


class ResponseVelocityEstimator:
    """Planner-internal transferable body-velocity estimate: first-order
    response of the planner's OWN commanded outputs (same concept/taus as the
    no-DVL commanded odometry). No DVL, no VelocitySensor."""

    def __init__(self, cfg: DWAConfig) -> None:
        self.cfg = cfg
        self.u = 0.0
        self.v = 0.0
        self.r = 0.0

    def update(self, cmd_u: float, cmd_v: float, cmd_r: float,
               dt: float) -> None:
        if dt <= 0:
            return
        c = self.cfg
        self.u += (cmd_u - self.u) * min(1.0, dt / c.tau_surge_s)
        self.v += (cmd_v - self.v) * min(1.0, dt / c.tau_sway_s)
        self.r += (cmd_r - self.r) * min(1.0, dt / c.tau_yaw_s)


def _simulate(x: float, y: float, yaw: float, u: float, v: float, r: float,
              cfg: DWAConfig,
              vel0: Optional[Tuple[float, float, float]] = None) -> np.ndarray:
    """RESPONSE-AWARE holonomic rollout (Eriksen-style marine adaptation):
    the actual velocity converges first-order (tau from the identified
    vehicle response) toward the candidate command instead of jumping to it.
    A constant-command rollout over-predicted lateral clearance by ~0.3-0.5 m
    near the obstacle (closed-loop smoke graze at 0.299 m) because the
    BlueROV2 needs ~1.2 s to realize a commanded velocity.
    Returns Nx3 [x, y, yaw]."""
    n = max(1, int(round(cfg.horizon_s / cfg.sim_dt)))
    cu, cv, cr = (vel0 if vel0 is not None else (u, v, r))
    out = np.empty((n, 3))
    for i in range(n):
        cu += (u - cu) * min(1.0, cfg.sim_dt / cfg.tau_surge_s)
        cv += (v - cv) * min(1.0, cfg.sim_dt / cfg.tau_sway_s)
        cr += (r - cr) * min(1.0, cfg.sim_dt / cfg.tau_yaw_s)
        yaw = yaw + cr * cfg.sim_dt
        x = x + (cu * math.cos(yaw) - cv * math.sin(yaw)) * cfg.sim_dt
        y = y + (cu * math.sin(yaw) + cv * math.cos(yaw)) * cfg.sim_dt
        out[i] = (x, y, yaw)
    return out


class HolonomicDWA:
    """Deterministic core: plan() maps state -> best admissible command."""

    def __init__(self, config: Optional[DWAConfig] = None) -> None:
        self.cfg = config or DWAConfig()
        self._last_cmd = (0.0, 0.0, 0.0)

    def _window(self, vel: Tuple[float, float, float]):
        c = self.cfg
        du = c.accel_lin * c.window_dt
        dr = c.accel_yaw * c.window_dt
        u0, v0, r0 = vel
        us = np.linspace(max(c.min_surge, u0 - du),
                         min(c.max_surge, u0 + du), c.n_u)
        vs = np.linspace(max(-c.max_sway, v0 - du),
                         min(c.max_sway, v0 + du), c.n_v)
        rs = np.linspace(max(-c.max_yaw_rate, r0 - dr),
                         min(c.max_yaw_rate, r0 + dr), c.n_r)
        return us, vs, rs

    def plan(self,
             pose: Tuple[float, float, float],
             vel_est: Tuple[float, float, float],
             obstacles: List[ObstacleEstimate],
             route: Optional[Tuple[float, float, float]],
             nominal_surge: float) -> DWAResult:
        """Vectorized over all candidates: the original per-candidate Python
        loop cost enough CPU to drag the ENGINE below real time (25.3 vs
        30 Hz), dilating sim time and biasing the wall-clock commanded
        odometry ~20% (the D-010 trap, load-induced). All-candidate numpy
        keeps planning around ~1 ms."""
        t0 = time.perf_counter()
        c = self.cfg
        x0, y0, yaw0 = pose
        us, vs, rs = self._window(vel_est)
        inflate = c.vehicle_radius_m + c.safety_margin_m

        UU, VV, RR = np.meshgrid(us, vs, rs, indexing="ij")
        cu_cmd = UU.ravel()
        cv_cmd = VV.ravel()
        cr_cmd = RR.ravel()
        n = cu_cmd.size

        steps = max(1, int(round(c.horizon_s / c.sim_dt)))
        au = min(1.0, c.sim_dt / c.tau_surge_s)
        av = min(1.0, c.sim_dt / c.tau_sway_s)
        ar = min(1.0, c.sim_dt / c.tau_yaw_s)
        cu = np.full(n, float(vel_est[0]), dtype=float)
        cv = np.full(n, float(vel_est[1]), dtype=float)
        cr = np.full(n, float(vel_est[2]), dtype=float)
        x = np.full(n, float(x0), dtype=float)
        y = np.full(n, float(y0), dtype=float)
        yaw = np.full(n, float(yaw0), dtype=float)
        min_d = np.full(n, np.inf)
        for _ in range(steps):
            cu += (cu_cmd - cu) * au
            cv += (cv_cmd - cv) * av
            cr += (cr_cmd - cr) * ar
            yaw = yaw + cr * c.sim_dt
            cos_y = np.cos(yaw)
            sin_y = np.sin(yaw)
            x = x + (cu * cos_y - cv * sin_y) * c.sim_dt
            y = y + (cu * sin_y + cv * cos_y) * c.sim_dt
            for ob in obstacles:
                d = np.hypot(x - ob.x, y - ob.y) - ob.radius
                np.minimum(min_d, d, out=min_d)

        if obstacles:
            clear = min_d - inflate
            speed = np.hypot(cu_cmd, cv_cmd)
            stop_dist = speed * speed / (2.0 * c.stop_decel)
            admissible_mask = (clear > 0.0) & (clear >= stop_dist)
            clear_norm = np.minimum(clear, c.clearance_saturation_m)                 / c.clearance_saturation_m
        else:
            admissible_mask = np.ones(n, dtype=bool)
            clear = np.full(n, np.inf)
            clear_norm = np.ones(n)

        speed_ref = max(0.05, min(nominal_surge, c.max_surge))
        if route is not None:
            rx, ry, ryaw = route
            cos_r, sin_r = math.cos(ryaw), math.sin(ryaw)
            along = (x - rx) * cos_r + (y - ry) * sin_r
            along0 = (x0 - rx) * cos_r + (y0 - ry) * sin_r
            progress = along - along0
            cross = np.abs(-(x - rx) * sin_r + (y - ry) * cos_r)
            dyaw = yaw - ryaw
            yaw_err = np.abs(np.arctan2(np.sin(dyaw), np.cos(dyaw)))
        else:
            progress = np.zeros(n)
            cross = np.zeros(n)
            yaw_err = np.zeros(n)
        progress_norm = progress / max(1e-6, speed_ref * c.horizon_s)
        # LINEAR, UNCAPPED route term: any cap flattens the return-to-route
        # gradient beyond the cap distance and the vehicle wanders off
        # indefinitely after the pass (observed twice in closed loop, 10 m
        # drift). Safety near the obstacle is protected by the HARD
        # admissibility constraint, not by this weight, so an unbounded
        # route penalty is safe by construction.
        cross_norm = cross / 1.5 + 1.0 * (yaw_err / math.pi)
        speed_norm = cu_cmd / c.max_surge
        lu, lv, lr = self._last_cmd
        smooth_norm = (np.abs(cu_cmd - lu) / max(c.max_surge, 1e-6)
                       + np.abs(cv_cmd - lv) / max(c.max_sway, 1e-6)
                       + np.abs(cr_cmd - lr) / max(c.max_yaw_rate, 1e-6)
                       ) / 3.0

        cost = (c.w_clearance * clear_norm
                + c.w_progress * progress_norm
                + c.w_speed * speed_norm
                - c.w_route * cross_norm
                - c.w_smooth * smooth_norm)
        cost = np.where(admissible_mask, cost, -np.inf)

        admissible = int(admissible_mask.sum())
        dt_ms = (time.perf_counter() - t0) * 1000.0
        if admissible == 0:
            self._last_cmd = (0.0, 0.0, 0.0)
            return DWAResult(0.0, 0.0, 0.0, 0, int(n), True, None, dt_ms,
                             None, {"event": "no_admissible_candidate"})
        i = int(np.argmax(cost))
        best = (float(cu_cmd[i]), float(cv_cmd[i]), float(cr_cmd[i]))
        best_clear = None if not obstacles else float(clear[i])
        self._last_cmd = best
        return DWAResult(best[0], best[1], best[2], admissible, int(n),
                         False, float(cost[i]), dt_ms, best_clear)
