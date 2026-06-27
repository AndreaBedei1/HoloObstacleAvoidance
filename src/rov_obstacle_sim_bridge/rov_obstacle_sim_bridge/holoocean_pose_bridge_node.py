"""Read or simulate ROV pose and publish /sim/rov_pose.

Optional HoloOcean dependency: when installed the node opens a scenario,
steps each timer tick, reads agent pose and publishes a PoseStamped.
When HoloOcean is unavailable (or use_holoocean=False) the node falls
back to a deterministic fake pose so the rest of the pipeline can run.

Does NOT send /planner/cmd_vel_safe, does NOT control thrusters, does NOT
connect to MAVLink or the real ROV.
"""

from __future__ import annotations

import math
from typing import Any

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from rclpy.node import Node


# ---------------------------------------------------------------------------
# Pure-Python helpers (no external dependencies, testable standalone)
# ---------------------------------------------------------------------------

def quaternion_from_yaw(yaw_rad: float) -> Quaternion:
    """Return a yaw-only quaternion (zero pitch/roll)."""
    q = Quaternion()
    q.w = math.cos(yaw_rad / 2.0)
    q.z = math.sin(yaw_rad / 2.0)
    return q


def pose_from_holoocean_state(
    state: dict[str, Any],
    agent_name: str,
) -> tuple[float, float, float, float]:
    """Extract (x, y, z, yaw_rad) from a HoloOcean environment state dict.

    Robustly handles multiple HoloOcean state dict layouts:
      - state[agent_name]['position'] / 'orientation'  (dict with euler keys)
      - state['agents'][agent_name]['pose']            (flat list of 7 floats)
      - state[agent_name]['pose']                       (flat list of 7 floats)

    Returns (x, y, z, yaw_rad).  Raises KeyError when the agent cannot be
    found in the state dictionary.
    """
    agents = state.get("agents", state)

    if agent_name not in agents:
        raise KeyError(
            f"Agent '{agent_name}' not found in state. "
            f"Available keys: {list(agents.keys())}"
        )

    agent = agents[agent_name]

    # Layout 1: flat 'pose' list [x, y, z, qx, qy, qz, qw]
    if "pose" in agent and isinstance(agent["pose"], (list, tuple)):
        pose = agent["pose"]
        if len(pose) >= 7:
            x, y, z = pose[0], pose[1], pose[2]
            qx, qy, qz, qw = pose[3], pose[4], pose[5], pose[6]
        else:
            x, y, z = pose[:3]
            qx, qy, qz, qw = 0.0, 0.0, 0.0, 1.0
    # Layout 2: separate 'position' and 'orientation' dicts
    elif "position" in agent and "orientation" in agent:
        pos = agent["position"]
        ori = agent["orientation"]
        x = pos.get("x", pos.get(0, 0.0))
        y = pos.get("y", pos.get(1, 0.0))
        z = pos.get("z", pos.get(2, 0.0))
        qx = ori.get("x", ori.get(0, 0.0))
        qy = ori.get("y", ori.get(1, 0.0))
        qz = ori.get("z", ori.get(2, 0.0))
        qw = ori.get("w", ori.get(3, 1.0))
    else:
        raise KeyError(
            f"Unrecognised agent state layout for '{agent_name}': "
            f"keys = {list(agent.keys())}"
        )

    # Convert quaternion to yaw (rotation about Z)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw_rad = math.atan2(siny_cosp, cosy_cosp)

    return x, y, z, yaw_rad


def fake_pose_at_time(
    t: float,
    start_x: float = 0.0,
    start_y: float = 0.0,
    start_z: float = 0.0,
    velocity_x: float = 0.0,
    velocity_y: float = 0.0,
    yaw_rad: float = 0.0,
) -> tuple[float, float, float, float]:
    """Return a deterministic fake (x, y, z, yaw) at elapsed time *t*.

    Simple linear drift so the pipeline receives moving poses even without
    HoloOcean.  Yaw is constant.
    """
    x = start_x + velocity_x * t
    y = start_y + velocity_y * t
    z = start_z
    return x, y, z, yaw_rad


# ---------------------------------------------------------------------------
# ROS 2 Node
# ---------------------------------------------------------------------------

class HolooceanPoseBridgeNode(Node):
    """Read pose from HoloOcean (or fake) and publish PoseStamped."""

    def __init__(
        self,
        *,
        context: rclpy.context.Context | None = None,
        parameter_overrides: list[rclpy.parameter.Parameter] | None = None,
    ) -> None:
        super().__init__(
            "holoocean_pose_bridge",
            context=context,
            parameter_overrides=parameter_overrides,
        )

        self._declare_parameters()

        output_topic = str(self.get_parameter("output_topic").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._scenario_name = str(self.get_parameter("scenario_name").value)
        self._agent_name = str(self.get_parameter("agent_name").value)
        self._use_holoocean = bool(self.get_parameter("use_holoocean").value)
        self._fallback_to_fake = bool(
            self.get_parameter("fallback_to_fake_pose").value
        )

        # Fake-pose defaults
        self._fake_start_x = float(self.get_parameter("fake_start_x").value)
        self._fake_start_y = float(self.get_parameter("fake_start_y").value)
        self._fake_start_z = float(self.get_parameter("fake_start_z").value)
        self._fake_velocity_x = float(self.get_parameter("fake_velocity_x").value)
        self._fake_velocity_y = float(self.get_parameter("fake_velocity_y").value)
        self._fake_yaw_rad = math.radians(
            float(self.get_parameter("fake_yaw_deg").value)
        )

        self._publisher = self.create_publisher(PoseStamped, output_topic, 10)

        rate_hz = max(0.1, float(self.get_parameter("publish_rate_hz").value))
        self._timer = self.create_timer(1.0 / rate_hz, self._timer_callback)

        self._start_time = self.get_clock().now()

        # --- Optional HoloOcean environment ---
        self._env: Any | None = None
        self._holoocean_error: str | None = None

        if self._use_holoocean:
            self._env, self._holoocean_error = self._try_create_environment()
        else:
            self.get_logger().info("HoloOcean disabled by parameter.")

        mode = "HoloOcean" if self._env is not None else "fake-pose fallback"
        self.get_logger().info(
            f"Pose bridge publishing {output_topic} ({mode})."
        )

    # ------------------------------------------------------------------
    # Parameters
    # ------------------------------------------------------------------
    def _declare_parameters(self) -> None:
        self.declare_parameter("output_topic", "/sim/rov_pose")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("frame_id", "world")
        self.declare_parameter("scenario_name", "OpenWater-Hovering")
        self.declare_parameter("agent_name", "auv0")
        self.declare_parameter("use_holoocean", True)
        self.declare_parameter("fallback_to_fake_pose", True)
        # Fake-pose defaults
        self.declare_parameter("fake_start_x", 0.0)
        self.declare_parameter("fake_start_y", 0.0)
        self.declare_parameter("fake_start_z", 0.0)
        self.declare_parameter("fake_velocity_x", 0.2)
        self.declare_parameter("fake_velocity_y", 0.0)
        self.declare_parameter("fake_yaw_deg", 0.0)

    # ------------------------------------------------------------------
    # HoloOcean environment lifecycle
    # ------------------------------------------------------------------
    @staticmethod
    def _try_create_environment(
        scenario_name: str | None = None,
    ) -> tuple[Any | None, str | None]:
        """Try to import holoocean and create an environment.

        Returns (env, error_message).  On success env is non-None and
        error_message is None.  On failure env is None and error_message
        explains what went wrong.
        """
        try:
            from holoocean import Environment  # noqa: TID251
        except ImportError as exc:
            return None, f"HoloOcean not installed: {exc}"
        except Exception as exc:
            return None, f"HoloOcean import failed: {exc}"

        try:
            scenario = scenario_name or "OpenWater-Hovering"
            env = Environment(scenario)
            return env, None
        except Exception as exc:
            return None, f"Environment('{scenario}') failed: {exc}"

    # ------------------------------------------------------------------
    # Timer callback
    # ------------------------------------------------------------------
    def _elapsed(self) -> float:
        return (self.get_clock().now() - self._start_time).nanoseconds * 1e-9

    def _timer_callback(self) -> None:
        t = self._elapsed()

        if self._env is not None:
            x, y, z, yaw_rad = self._try_read_holoocean_pose(t)
        else:
            x, y, z, yaw_rad = fake_pose_at_time(
                t,
                self._fake_start_x,
                self._fake_start_y,
                self._fake_start_z,
                self._fake_velocity_x,
                self._fake_velocity_y,
                self._fake_yaw_rad,
            )

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation = quaternion_from_yaw(yaw_rad)

        self._publisher.publish(msg)

    def _try_read_holoocean_pose(
        self, t: float
    ) -> tuple[float, float, float, float]:
        """Step the environment and read agent pose; fall back to fake."""
        try:
            # Send zero/neutral action so the agent hovers in place.
            action = {}
            state, _, _, _, _ = self._env.step(action)

            try:
                x, y, z, yaw_rad = pose_from_holoocean_state(
                    state, self._agent_name
                )
                return x, y, z, yaw_rad
            except (KeyError, TypeError, IndexError) as exc:
                self.get_logger().warn_throttle(
                    5.0,
                    f"Failed to parse agent pose: {exc}. "
                    "Falling back to fake pose.",
                )
        except Exception as exc:
            self.get_logger().warn_throttle(
                5.0, f"HoloOcean step failed: {exc}. Falling back to fake pose."
            )

        # Fallback
        if self._fallback_to_fake:
            return fake_pose_at_time(
                t,
                self._fake_start_x,
                self._fake_start_y,
                self._fake_start_z,
                self._fake_velocity_x,
                self._fake_velocity_y,
                self._fake_yaw_rad,
            )

        # If fallback is disabled, return zero pose.
        self.get_logger().error_throttle(
            2.0, "Cannot read HoloOcean pose and fallback is disabled."
        )
        return 0.0, 0.0, 0.0, 0.0


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = HolooceanPoseBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
