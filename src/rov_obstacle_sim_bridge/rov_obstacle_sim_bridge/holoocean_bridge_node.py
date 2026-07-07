"""ROS 2 bridge to the HoloOcean sim server (runs in the pixi ROS 2 env).

This is the *ROS 2 side* of the two-process bridge.  It connects to the
``holoocean_sim_server`` (which runs HoloOcean in the separate conda ``ocean``
Python 3.9 environment) over a localhost TCP socket and republishes the
simulator state as standard ROS 2 topics:

  * ``/camera/front/image_raw``      sensor_msgs/Image  (rgb8)
  * ``/rov/pose_ground_truth``       geometry_msgs/PoseStamped
      -- SIMULATION-ONLY ground-truth pose.  For validation / debug only; the
         runtime planner must NOT consume this.  The planner uses the estimated
         odometry (``/rov/odom_estimated`` from ``odometry_estimator_node``).
  * ``/rov/velocity``                geometry_msgs/TwistStamped
      -- BODY-frame velocity (linear) + yaw rate (angular.z) from pose finite
         difference: the realistic DVL + gyro signal the estimator integrates.
  * ``/rov/depth``                   std_msgs/Float32
  * ``/perception/obstacles_oracle`` rov_obstacle_msgs/Obstacle2DArray
      -- SIMULATION-ONLY ground-truth projection.  This is NOT a real sensor.
         It exists for debugging, evaluation, dataset labels and planner
         validation.  Never wire it into the real-rover runtime.

It subscribes to ``/planner/cmd_vel_safe`` (geometry_msgs/Twist) and forwards
the abstract body-frame velocity command to the sim server.  It does NOT talk to
thrusters, MAVLink or any real vehicle.

If the sim server is not reachable, the node keeps retrying the connection and
publishes nothing until it succeeds -- HoloOcean being absent must never crash
the ROS 2 graph.
"""

from __future__ import annotations

import array
import math

import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion, Twist, TwistStamped
from rclpy.node import Node
from rov_obstacle_msgs.msg import Obstacle2D, Obstacle2DArray
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

from .oracle_geometry import (
    CameraConfig,
    ObstacleConfig,
    RoverPose2D,
    project_obstacles,
)
from .sim_bridge_protocol import (
    DEFAULT_HOST,
    DEFAULT_PORT,
    MSG_STATE,
    FrameStream,
    connect,
    make_command_header,
)


def quaternion_from_yaw(yaw_rad: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw_rad / 2.0)
    q.z = math.sin(yaw_rad / 2.0)
    return q


def _wrap_to_pi(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class HolooceanBridgeNode(Node):
    def __init__(
        self,
        *,
        context: rclpy.context.Context | None = None,
        parameter_overrides: list[rclpy.parameter.Parameter] | None = None,
    ) -> None:
        super().__init__(
            "holoocean_bridge",
            context=context,
            parameter_overrides=parameter_overrides,
        )
        self._declare_parameters()

        self._host = str(self.get_parameter("host").value)
        self._port = int(self.get_parameter("port").value)
        self._frame_id_world = str(self.get_parameter("world_frame_id").value)
        self._frame_id_camera = str(self.get_parameter("camera_frame_id").value)

        self._min_range = float(self.get_parameter("min_detection_range_m").value)
        self._max_range = float(self.get_parameter("max_detection_range_m").value)
        self._risk_area_gain = float(self.get_parameter("risk_area_gain").value)

        # Publishers
        self._pub_image = self.create_publisher(Image, str(self.get_parameter("image_topic").value), 5)
        self._pub_pose = self.create_publisher(PoseStamped, str(self.get_parameter("pose_topic").value), 10)
        self._pub_vel = self.create_publisher(TwistStamped, str(self.get_parameter("velocity_topic").value), 10)
        self._pub_depth = self.create_publisher(Float32, str(self.get_parameter("depth_topic").value), 10)
        self._pub_oracle = self.create_publisher(
            Obstacle2DArray, str(self.get_parameter("oracle_topic").value), 10)
        relay_oracle_topic = str(self.get_parameter("relay_oracle_topic").value)
        self._pub_oracle_relay = (
            self.create_publisher(Obstacle2DArray, relay_oracle_topic, 10)
            if relay_oracle_topic
            else None
        )

        # Subscriber: forward safe command to the sim
        self.create_subscription(
            Twist, str(self.get_parameter("cmd_vel_topic").value), self._on_cmd_vel, 10)
        self._pending_cmd: dict | None = None

        # Socket state
        self._stream: FrameStream | None = None
        self._publish_image = bool(self.get_parameter("publish_image").value)
        self._logged_first_oracle_detection = False
        # Previous pose sample (x, y, z, yaw, t) for body-velocity finite diff.
        self._prev_pose: tuple[float, float, float, float, float] | None = None

        poll_hz = max(1.0, float(self.get_parameter("poll_rate_hz").value))
        self._timer = self.create_timer(1.0 / poll_hz, self._on_timer)
        self.get_logger().info(
            f"HoloOcean bridge: will connect to sim server {self._host}:{self._port}. "
            "/perception/obstacles_oracle is SIMULATION-ONLY ground truth."
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("host", DEFAULT_HOST)
        self.declare_parameter("port", DEFAULT_PORT)
        self.declare_parameter("poll_rate_hz", 60.0)
        self.declare_parameter("world_frame_id", "world")
        self.declare_parameter("camera_frame_id", "front_camera")
        self.declare_parameter("image_topic", "/camera/front/image_raw")
        self.declare_parameter("pose_topic", "/rov/pose_ground_truth")
        self.declare_parameter("velocity_topic", "/rov/velocity")
        self.declare_parameter("depth_topic", "/rov/depth")
        self.declare_parameter("oracle_topic", "/perception/obstacles_oracle")
        self.declare_parameter("relay_oracle_topic", "")
        self.declare_parameter("cmd_vel_topic", "/planner/cmd_vel_safe")
        self.declare_parameter("publish_image", True)
        self.declare_parameter("min_detection_range_m", 0.2)
        self.declare_parameter("max_detection_range_m", 40.0)
        self.declare_parameter("risk_area_gain", 4.0)

    # -- connection ----------------------------------------------------------
    def _ensure_connected(self) -> bool:
        if self._stream is not None and not self._stream.closed:
            return True
        try:
            self._stream = connect(self._host, self._port, timeout_s=0.5)
            self.get_logger().info("connected to sim server")
            return True
        except OSError:
            self._stream = None
            self.get_logger().warning(
                f"sim server {self._host}:{self._port} unavailable; retrying...",
                throttle_duration_sec=5.0,
            )
            return False

    # -- ROS callbacks -------------------------------------------------------
    def _on_cmd_vel(self, msg: Twist) -> None:
        self._pending_cmd = make_command_header(
            surge=msg.linear.x,
            sway=msg.linear.y,
            heave=msg.linear.z,
            roll_rate=msg.angular.x,
            pitch_rate=msg.angular.y,
            yaw_rate=msg.angular.z,
        )

    def _on_timer(self) -> None:
        if not self._ensure_connected():
            return
        assert self._stream is not None

        # Forward the most recent command, if any.
        if self._pending_cmd is not None:
            try:
                self._stream.send(self._pending_cmd)
            except ConnectionError:
                self._handle_disconnect()
                return
            self._pending_cmd = None

        # Read the freshest state frame (drop stale).
        try:
            frame = self._stream.read_latest()
        except ConnectionError:
            self._handle_disconnect()
            return
        if frame is None:
            if self._stream.closed:
                self._handle_disconnect()
            return

        header, blob = frame
        if header.get("type") != MSG_STATE:
            return
        self._publish_state(header, blob)

    def _handle_disconnect(self) -> None:
        self.get_logger().warning("sim server connection lost; will reconnect")
        if self._stream is not None:
            self._stream.close()
        self._stream = None

    # -- publishing ----------------------------------------------------------
    def _publish_state(self, header: dict, blob: bytes) -> None:
        stamp = self.get_clock().now().to_msg()
        pose = header.get("pose", {})
        yaw = float(pose.get("yaw", 0.0))

        # Pose
        pmsg = PoseStamped()
        pmsg.header.stamp = stamp
        pmsg.header.frame_id = self._frame_id_world
        pmsg.pose.position.x = float(pose.get("x", 0.0))
        pmsg.pose.position.y = float(pose.get("y", 0.0))
        pmsg.pose.position.z = float(pose.get("z", 0.0))
        pmsg.pose.orientation = quaternion_from_yaw(yaw)
        self._pub_pose.publish(pmsg)

        # Velocity: BODY-frame linear velocity + yaw rate from pose finite
        # difference.  Under the kinematic teleport model this reproduces the
        # true motion, and it is the realistic DVL (body velocity) + gyro (yaw
        # rate) signal the odometry estimator dead-reckons from.  The sim
        # VelocitySensor reads ~0 under teleport, so it is intentionally unused.
        now_s = self.get_clock().now().nanoseconds * 1e-9
        px = float(pose.get("x", 0.0))
        py = float(pose.get("y", 0.0))
        pz = float(pose.get("z", 0.0))
        body_vx = body_vy = body_vz = yaw_rate = 0.0
        if self._prev_pose is not None:
            xp, yp, zp, yawp, tp = self._prev_pose
            dt = now_s - tp
            if dt > 1e-6:
                wvx = (px - xp) / dt
                wvy = (py - yp) / dt
                body_vz = (pz - zp) / dt
                yaw_rate = _wrap_to_pi(yaw - yawp) / dt
                cos_y = math.cos(yaw)
                sin_y = math.sin(yaw)
                body_vx = cos_y * wvx + sin_y * wvy
                body_vy = -sin_y * wvx + cos_y * wvy
        self._prev_pose = (px, py, pz, yaw, now_s)

        vmsg = TwistStamped()
        vmsg.header.stamp = stamp
        vmsg.header.frame_id = "base_link"
        vmsg.twist.linear.x = body_vx
        vmsg.twist.linear.y = body_vy
        vmsg.twist.linear.z = body_vz
        vmsg.twist.angular.z = yaw_rate
        self._pub_vel.publish(vmsg)

        # Depth
        dmsg = Float32()
        dmsg.data = float(header.get("depth", 0.0))
        self._pub_depth.publish(dmsg)

        # Camera image
        img_meta = header.get("image")
        if self._publish_image and img_meta and blob:
            imsg = Image()
            imsg.header.stamp = stamp
            imsg.header.frame_id = self._frame_id_camera
            imsg.height = int(img_meta["height"])
            imsg.width = int(img_meta["width"])
            imsg.encoding = str(img_meta.get("encoding", "rgb8"))
            imsg.is_bigendian = 0
            imsg.step = int(img_meta.get("step", imsg.width * 3))
            imsg.data = array.array("B", blob)
            self._pub_image.publish(imsg)

        # Oracle obstacles (SIMULATION-ONLY ground-truth projection)
        self._publish_oracle(header, stamp, yaw)

    def _publish_oracle(self, header: dict, stamp, yaw: float) -> None:
        cam = header.get("camera", {})
        camera = CameraConfig(
            horizontal_fov_deg=float(cam.get("horizontal_fov_deg", 90.0)),
            vertical_fov_deg=float(cam.get("vertical_fov_deg", 60.0)),
            min_detection_range_m=self._min_range,
            max_detection_range_m=self._max_range,
            confidence=1.0,
            risk_area_gain=self._risk_area_gain,
        )
        # HoloOcean world is REP-103 (x-fwd, y-LEFT, z-up); oracle_geometry uses
        # the opposite lateral sign (+y = right).  Negate y and yaw so projected
        # center_x matches where the obstacle actually appears in the RGB image.
        pose = header.get("pose", {})
        velocity = header.get("velocity", {})
        rover = RoverPose2D(
            x=float(pose.get("x", 0.0)),
            y=-float(pose.get("y", 0.0)),
            z=float(pose.get("z", 0.0)),
            yaw_rad=-yaw,
            velocity_x=float(velocity.get("x", 0.0)),
            velocity_y=-float(velocity.get("y", 0.0)),
            velocity_z=float(velocity.get("z", 0.0)),
        )

        def _to_oracle_xyz(p) -> tuple[float, float, float]:
            return (float(p[0]), -float(p[1]), float(p[2]))

        def _to_oracle_bounds(raw_bounds):
            if not raw_bounds:
                return None
            mn = raw_bounds.get("min")
            mx = raw_bounds.get("max")
            if mn is None or mx is None:
                return None
            corners = [
                _to_oracle_xyz((x, y, z))
                for x in (float(mn[0]), float(mx[0]))
                for y in (float(mn[1]), float(mx[1]))
                for z in (float(mn[2]), float(mx[2]))
            ]
            return (
                (
                    min(p[0] for p in corners),
                    min(p[1] for p in corners),
                    min(p[2] for p in corners),
                ),
                (
                    max(p[0] for p in corners),
                    max(p[1] for p in corners),
                    max(p[2] for p in corners),
                ),
            )

        raw_obstacles = header.get("obstacles", [])
        obstacles = [
            ObstacleConfig(
                name=str(o.get("name", "obstacle")),
                class_name=str(o.get("class_name", "unknown_obstacle")),
                position=_to_oracle_xyz(o.get("position", [0.0, 0.0, 0.0])),
                radius_m=float(o.get("radius_m", 1.0)),
                bounds=_to_oracle_bounds(o.get("bounds")),
            )
            for o in raw_obstacles
        ]
        projected = project_obstacles(obstacles, rover, camera)
        if projected and not self._logged_first_oracle_detection:
            self.get_logger().info(
                f"oracle projected {len(projected)} obstacle(s); "
                f"first class={projected[0].class_name} risk={projected[0].risk:.3f}"
            )
            self._logged_first_oracle_detection = True
        elif raw_obstacles and not projected:
            self.get_logger().warning(
                "server sent obstacle metadata but oracle projection is empty; "
                f"seq={header.get('seq')} pose={pose} first_obstacle={raw_obstacles[0]}",
                throttle_duration_sec=5.0,
            )

        arr = Obstacle2DArray()
        arr.header.stamp = stamp
        arr.header.frame_id = self._frame_id_camera
        for p in projected:
            m = Obstacle2D()
            m.header = arr.header
            m.class_name = p.class_name
            m.confidence = float(p.confidence)
            m.center_x = float(p.center_x)
            m.center_y = float(p.center_y)
            m.width = float(p.width)
            m.height = float(p.height)
            m.bearing_rad = float(p.bearing_rad)
            m.apparent_area = float(p.apparent_area)
            m.risk = float(p.risk)
            m.is_tracking_valid = True
            arr.obstacles.append(m)
        self._pub_oracle.publish(arr)
        if self._pub_oracle_relay is not None:
            self._pub_oracle_relay.publish(arr)

    def destroy_node(self) -> bool:
        if self._stream is not None:
            self._stream.close()
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = HolooceanBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
