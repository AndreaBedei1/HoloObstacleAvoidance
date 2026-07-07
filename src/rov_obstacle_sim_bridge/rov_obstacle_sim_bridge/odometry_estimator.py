"""Realistic dead-reckoning odometry estimator (pure, testable core).

Estimates planar pose (x, y, yaw) by integrating a DVL-like body-frame velocity
and a gyro-like yaw rate -- exactly the sensors a real underwater vehicle has
(a Doppler velocity log + an IMU/AHRS), never the simulator ground-truth pose.

Configurable bias / scale-factor / white-noise terms model real sensor error,
so the estimate accumulates drift over time the way real DVL+gyro odometry does.
The class is deterministic given its RNG, so it can be unit-tested exactly.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Callable


@dataclass(frozen=True)
class OdometryNoiseConfig:
    """Sensor error model for the estimator (all default to zero == perfect)."""

    gyro_bias_rad_s: float = 0.0
    gyro_noise_std_rad_s: float = 0.0
    gyro_scale_error: float = 0.0
    dvl_bias_x_ms: float = 0.0
    dvl_bias_y_ms: float = 0.0
    dvl_noise_std_ms: float = 0.0
    dvl_scale_error: float = 0.0


@dataclass(frozen=True)
class OdometryState:
    x: float
    y: float
    z: float
    yaw: float


class OdometryEstimator:
    """Integrate body velocity + yaw rate into a drifting pose estimate.

    ``rng`` is an optional callable returning standard-normal samples (mean 0,
    std 1); pass ``None`` for a noise-free estimate (used in tests).
    """

    def __init__(
        self,
        config: OdometryNoiseConfig | None = None,
        *,
        x0: float = 0.0,
        y0: float = 0.0,
        z0: float = 0.0,
        yaw0: float = 0.0,
        rng: Callable[[], float] | None = None,
    ) -> None:
        self.config = config or OdometryNoiseConfig()
        self.x = float(x0)
        self.y = float(y0)
        self.z = float(z0)
        self.yaw = float(yaw0)
        self._rng = rng

    def update(
        self,
        body_vx: float,
        body_vy: float,
        body_vz: float,
        yaw_rate: float,
        dt: float,
    ) -> OdometryState:
        """Advance the estimate by ``dt`` seconds and return the new state."""
        if dt <= 0.0:
            return self.state()
        c = self.config

        # Gyro: scale error + constant bias + white noise, then integrate.
        omega = (
            yaw_rate * (1.0 + c.gyro_scale_error)
            + c.gyro_bias_rad_s
            + self._noise(c.gyro_noise_std_rad_s)
        )
        self.yaw = _wrap_to_pi(self.yaw + omega * dt)

        # DVL: scale error + axis bias + white noise on the body velocity.
        mvx = (
            body_vx * (1.0 + c.dvl_scale_error)
            + c.dvl_bias_x_ms
            + self._noise(c.dvl_noise_std_ms)
        )
        mvy = (
            body_vy * (1.0 + c.dvl_scale_error)
            + c.dvl_bias_y_ms
            + self._noise(c.dvl_noise_std_ms)
        )

        # Rotate the measured body velocity into the world by the ESTIMATED yaw
        # (so heading error couples into position drift, as in reality).
        cos_y = math.cos(self.yaw)
        sin_y = math.sin(self.yaw)
        world_vx = cos_y * mvx - sin_y * mvy
        world_vy = sin_y * mvx + cos_y * mvy

        self.x += world_vx * dt
        self.y += world_vy * dt
        self.z += body_vz * dt
        return self.state()

    def state(self) -> OdometryState:
        return OdometryState(self.x, self.y, self.z, self.yaw)

    def _noise(self, std: float) -> float:
        if std <= 0.0 or self._rng is None:
            return 0.0
        return self._rng() * std


def _wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))
