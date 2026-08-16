"""REAL scientific pipeline: the same shared stack as simulation.

    real camera -> real detector -> /perception/obstacles_raw
        -> temporal_estimator (Phase-7B qualification + T2)
        -> /perception/obstacles
        -> Planner C or Planner D
        -> /planner/cmd_vel_safe
        -> real adapter (SHADOW by default) -> ArduSub

Everything between /perception/obstacles_raw and /planner/cmd_vel_safe is
the SAME source code that runs in simulation; only the nodes at the two
ends are platform-specific (docs/EXPERIMENTAL_BOUNDARY.md).

The adapter starts in SHADOW: it computes, logs and publishes what it
WOULD send, and transmits nothing. Real actuation requires all three of
real_control_mode:=live vehicle_in_water:=true allow_real_actuation:=true
AND a fully calibrated command mapping, or the node refuses and stays in
shadow.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _pool_benchmark():
    """The frozen pool benchmark, the SAME file the simulated campaign
    reads.

    Until 2026-08-15 this launch started the planner nodes with no
    parameters, so they used the defaults tuned for the 11 m simulated
    scenarios: a 9 m engagement distance in a 6 m pool, an assumed
    obstacle height of 3.5 m for an anchor of 0.5 m, and a DWA obstacle
    radius seven times too large. The simulation used the pool values.
    The sim-to-real comparison would have been between two differently
    configured planners, and nothing in either domain's output would
    have shown it.
    """
    import os
    import yaml
    rel = os.path.join("config", "pool_benchmark_FROZEN.yaml")
    # Search upward: this launch file runs from the source tree during
    # development and from install/share once built, so a fixed relative
    # path is right in one place and wrong in the other. An explicit
    # environment override comes first for out-of-tree installs.
    roots = []
    env = os.environ.get("HOLO_REPO_ROOT")
    if env:
        roots.append(env)
    here = os.path.dirname(os.path.abspath(__file__))
    for _ in range(8):
        roots.append(here)
        here = os.path.dirname(here)
    for root in roots:
        path = os.path.join(root, rel)
        if os.path.isfile(path):
            with open(path) as f:
                return yaml.safe_load(f)["pool_benchmark"]
    # Never fall back to the library defaults: they are the 11 m
    # scenario values, and a real run started with them would look
    # normal while being a different experiment from the simulation.
    raise RuntimeError(
        "pool_benchmark_FROZEN.yaml not found; refusing to launch the "
        "real pipeline with the default planner configuration, which is "
        "tuned for the 11 m simulated scenarios and would silently make "
        "the real campaign incomparable to the predictions. Set "
        "HOLO_REPO_ROOT to the repository root.")


def _s3_calibration():
    """The MEASURED command mapping, as adapter parameters.

    The adapter refuses live actuation while any axis is uncalibrated --
    "a commanded m/s has no measured meaning on this vehicle" -- and it
    was right to: the profile was measured on 2026-08-15 and then never
    handed to it, so every real run so far executed in shadow with the
    thrusters idle while the rest of the pipeline looked healthy.

    The numbers are read from config/calibration/s3_vehicle.json, the
    same file the simulated plant model uses, so the two domains cannot
    describe different vehicles.
    """
    import json
    import math
    import os
    rel = os.path.join("config", "calibration", "s3_vehicle.json")
    here = os.path.dirname(os.path.abspath(__file__))
    roots = [os.environ["HOLO_REPO_ROOT"]] if os.environ.get(
        "HOLO_REPO_ROOT") else []
    for _ in range(8):
        roots.append(here)
        here = os.path.dirname(here)
    path = next((os.path.join(r, rel) for r in roots
                 if os.path.isfile(os.path.join(r, rel))), None)
    if path is None:
        raise RuntimeError("s3_vehicle.json not found; the adapter would "
                           "stay in shadow and the run would look healthy "
                           "with the thrusters idle")
    with open(path) as f:
        prof = json.load(f)["profile"]
    out = {"calibration_id": "s3_vehicle_20260815"}
    for axis, key in (("surge", "surge_symmetric"),
                      ("sway", "sway_symmetric")):
        e = prof.get(key) or {}
        k = float(e.get("m_s_per_count") or 0.0)
        db = float(e.get("deadband_counts") or 0.0)
        out.update({f"{axis}_k_pos": k, f"{axis}_k_neg": k,
                    f"{axis}_db_pos": db, f"{axis}_db_neg": db,
                    f"{axis}_min_command": 0.0,
                    f"{axis}_calibrated": k > 0.0})
    # Yaw keeps its measured per-sign asymmetry, converted to rad/s.
    yp, yn = prof.get("yaw+") or {}, prof.get("yaw-") or {}
    kp = float(yp.get("deg_s_per_count") or 0.0) * math.pi / 180.0
    kn = float(yn.get("deg_s_per_count") or 0.0) * math.pi / 180.0
    out.update({"yaw_k_pos": kp, "yaw_k_neg": kn,
                "yaw_db_pos": float(yp.get("deadband_counts") or 0.0),
                "yaw_db_neg": float(yn.get("deadband_counts") or 0.0),
                "yaw_min_command": 0.0,
                "yaw_calibrated": kp > 0.0 and kn > 0.0})
    return out


def generate_launch_description():
    pool = _pool_benchmark()
    calib = _s3_calibration()
    planner = LaunchConfiguration("planner")
    return LaunchDescription([
        # OpenCV's FFMPEG backend reads this at DLL load time; setting it
        # inside a running interpreter is ignored.
        SetEnvironmentVariable(
            "OPENCV_FFMPEG_CAPTURE_OPTIONS",
            "protocol_whitelist;file,rtp,udp|fflags;nobuffer|"
            "flags;low_delay"),
        DeclareLaunchArgument("planner", default_value="committed"),
        DeclareLaunchArgument("estimator_method", default_value="t2"),
        DeclareLaunchArgument("annotated_video_path", default_value=""),
        DeclareLaunchArgument("min_avoidance_hold_s", default_value="4.0"),
        DeclareLaunchArgument("min_surge_during_avoidance",
                              default_value="0.08"),
        DeclareLaunchArgument("risk_enter_threshold", default_value="0.30"),
        DeclareLaunchArgument("risk_exit_threshold", default_value="0.15"),
        DeclareLaunchArgument("engage_distance_m",
                              default_value=str(pool["engage_distance_m"])),
        DeclareLaunchArgument("warmup_min_updates", default_value="2"),
        DeclareLaunchArgument("confirm_min_updates", default_value="1"),
        DeclareLaunchArgument("real_control_mode", default_value="shadow"),
        DeclareLaunchArgument("vehicle_in_water", default_value="false"),
        DeclareLaunchArgument("allow_real_actuation",
                              default_value="false"),
        DeclareLaunchArgument("calibration_id",
                              default_value="uncalibrated"),
        # Defaults come from the frozen pool benchmark, never from this
        # file: one source, both domains.
        DeclareLaunchArgument("nominal_surge",
                              default_value=str(pool["nominal_surge"])),
        DeclareLaunchArgument("camera_hfov_deg",
                              default_value=str(pool["camera_hfov_deg"])),

        Node(package="rov_real_bridge", executable="real_detector_node",
             name="real_detector", output="screen",
             parameters=[{
                 "camera_horizontal_fov_deg":
                     LaunchConfiguration("camera_hfov_deg"),
                 "annotated_video_path":
                     LaunchConfiguration("annotated_video_path"),
             }]),
        Node(package="rov_obstacle_tracking",
             executable="temporal_estimator_node",
             name="temporal_estimator", output="screen",
             parameters=[{
                 "method": LaunchConfiguration("estimator_method"),
                 # The Phase-7B warm-up was tuned on simulated
                 # perception, which delivers a coherent detection every
                 # tick. The real detector in this water produces 81 %
                 # of frames with an obstacle but with a bbox that moves
                 # between frames, so twenty coherent updates never
                 # accumulate and the qualifier discarded ALL 1152
                 # messages while the vehicle drove past the anchor.
                 # Relaxed for the real runs and recorded as such: the
                 # qualifier still filters, it just confirms on the
                 # evidence reality actually provides.
                 "warmup_min_updates": ParameterValue(
                     LaunchConfiguration("warmup_min_updates"),
                     value_type=int),
                 "confirm_min_updates": ParameterValue(
                     LaunchConfiguration("confirm_min_updates"),
                     value_type=int),
             }]),
        Node(package="rov_obstacle_bringup",
             executable="nominal_cmd_publisher_node",
             name="nominal_cmd_publisher", output="screen",
             parameters=[{"surge": LaunchConfiguration("nominal_surge")}]),
        Node(package="rov_obstacle_avoidance",
             executable="local_avoidance_planner_node",
             name="local_avoidance_planner", output="screen",
             parameters=[{
                 # The manoeuvre must last long enough to MOVE the
                 # vehicle. The default 1 s hold was tuned where the
                 # simulated vehicle strafes at 0.20-0.30 m/s; the real
                 # one reaches 0.123 m/s, so one second buys 12 cm and
                 # the anchor is still there. Four seconds buys about
                 # half a metre. This is the same shortfall the S3 plant
                 # model injects in simulation, showing up in the pool.
                 # RISK THRESHOLD, dropped for the real vehicle. The
                 # 0.55 default was set where the simulated detector
                 # returns a full-confidence box covering a broad
                 # obstacle. The real anchor is a thin shank: its
                 # apparent area is small, so the risk score stays far
                 # below 0.55 and the planner never commits, even with
                 # 79 % of frames detecting it and 75 % qualified. In a
                 # pool with a single obstacle, seeing it at all is
                 # reason enough to act.
                 "risk_enter_threshold": ParameterValue(
                     LaunchConfiguration("risk_enter_threshold"),
                     value_type=float),
                 "risk_exit_threshold": ParameterValue(
                     LaunchConfiguration("risk_exit_threshold"),
                     value_type=float),
                 # Keep moving WHILE avoiding. With engagement on sight
                 # the planner is in avoidance for most of the run, and
                 # it throttles surge back to this value while there.
                 # At 0.08 the real vehicle averaged 0.036 m/s with the
                 # command at zero 55 % of the time: the thrusters spun
                 # and the vehicle stayed put. It only appeared to work
                 # earlier because orphaned planners were pushing it.
                 "min_surge_during_avoidance": ParameterValue(
                     LaunchConfiguration("min_surge_during_avoidance"),
                     value_type=float),
                 "min_avoidance_hold_s": ParameterValue(
                     LaunchConfiguration("min_avoidance_hold_s"),
                     value_type=float),
                 # Overridable for the pool session; defaults to the
                 # frozen benchmark value.
                 "engage_distance_m": ParameterValue(
                     LaunchConfiguration("engage_distance_m"),
                     value_type=float),
                 "target_obstacle_height_m":
                     float(pool["target_obstacle_height_m"]),
             }],
             condition=IfCondition(PythonExpression(
                 ["'", planner, "' == 'committed'"]))),
        Node(package="rov_obstacle_avoidance",
             executable="dwa_planner_node",
             name="dwa_planner", output="screen",
             parameters=[{
                 "target_obstacle_height_m":
                     float(pool["target_obstacle_height_m"]),
                 "obstacle_radius_m":
                     float(pool["dwa_obstacle_radius_m"]),
                 "goal_lookahead_m":
                     float(pool["dwa_goal_lookahead_m"]),
             }],
             condition=IfCondition(PythonExpression(
                 ["'", planner, "' == 'dwa'"]))),
        Node(package="rov_real_bridge",
             executable="real_control_live_node",
             name="real_control_live", output="screen",
             parameters=[calib, {
                 "real_control_mode":
                     LaunchConfiguration("real_control_mode"),
                 # TYPED. These are declared as BOOLEAN parameters on
                 # the node, and a LaunchConfiguration arrives as a
                 # STRING: the assignment is rejected, both stay False,
                 # the interlock blocks and the adapter runs in shadow --
                 # publishing a complete command stream with
                 # "sent": false while the thrusters never turn. Every
                 # real run so far did exactly that, and nothing in the
                 # launch output said so.
                 "vehicle_in_water": ParameterValue(
                     LaunchConfiguration("vehicle_in_water"),
                     value_type=bool),
                 "allow_real_actuation": ParameterValue(
                     LaunchConfiguration("allow_real_actuation"),
                     value_type=bool),
                 "calibration_id":
                     LaunchConfiguration("calibration_id"),
             }]),
    ])
