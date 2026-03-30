# Full stack bringup: Aurora, Nav2, perception (segment_3d), inspection manager, motor driver.
# **Memory-constrained Jetsons:** Prefer `ugv_bringup/minimal_tyre_inspection.launch.py` for a lighter
# stack. Use this file for full integration on hardware with enough unified memory (e.g. 16 GB+)
# or after tuning; on 8 GB it often needs reduced perception or throttling.
# Starts everything in order with delays so dependencies are up before clients.
# Inspection manager is delayed until after Nav2 lifecycle (120s in nav_aurora) so the
# NavigateToPose action server is available when the mission tries to rotate/drive.
# Usage: ros2 launch ugv_nav full_bringup.launch.py
#   [ip_address:=192.168.11.1] [use_motor_driver:=true] [uart_port:=/dev/ttyTHS1]
#   [publish_wheel_odom:=true]  # /wheel/odometry from ESP32 T:1001 (for EKF)
#   [use_ekf:=false]  # true = fuse Aurora+wheel via robot_localization; Nav2 uses /odometry/filtered
#   [perception_only_mode:=false]  # set true to validate without motion

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.utilities import perform_substitutions
from launch_ros.actions import Node


def generate_launch_description():
    ugv_nav_share = get_package_share_directory("ugv_nav")
    seg3d_share = get_package_share_directory("segmentation_3d")
    inspection_share = get_package_share_directory("inspection_manager")

    ip_address_arg = DeclareLaunchArgument(
        "ip_address",
        default_value="192.168.11.1",
        description="IP address of the SLAMTEC Aurora device",
    )
    use_bridge_arg = DeclareLaunchArgument(
        "use_bridge",
        default_value="false",
        description="Legacy: use aurora_sdk_bridge. Default: Aurora 2.11 native depth.",
    )
    perception_only_arg = DeclareLaunchArgument(
        "perception_only_mode",
        default_value="false",
        description="Inspection manager: disable all movement (Nav2, visual_servo, cmd_vel).",
    )
    use_motor_driver_arg = DeclareLaunchArgument(
        "use_motor_driver",
        default_value="true",
        description="Launch motor driver (ugv_base_driver) to send cmd_vel to base",
    )
    sim_no_move_arg = DeclareLaunchArgument(
        "sim_no_move",
        default_value="false",
        description="When true, use stub_motor_node instead of motor_driver (no physical motion)",
    )
    sim_tyre_detections_arg = DeclareLaunchArgument(
        "sim_tyre_detections",
        default_value="false",
        description="When true, launch verify_system.py --simulate --publish-objects-segment",
    )
    publish_wheel_odom_arg = DeclareLaunchArgument(
        "publish_wheel_odom",
        default_value="true",
        description="Motor driver: publish /wheel/odometry from ESP32 T:1001 feedback (for EKF fusion)",
    )
    use_ekf_arg = DeclareLaunchArgument(
        "use_ekf",
        default_value="false",
        description="Fuse Aurora + wheel odom via robot_localization EKF; Nav2 uses /odometry/filtered",
    )
    uart_port_arg = DeclareLaunchArgument(
        "uart_port",
        default_value="/dev/ttyTHS1",
        description="UART port for motor driver (e.g. /dev/ttyTHS1 on Jetson)",
    )
    workspace_root = os.environ.get("UGV_WS", os.path.expanduser("~/ugv_ws"))
    production_config = os.path.join(workspace_root, "PRODUCTION_CONFIG.yaml")
    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=production_config,
        description="Path to PRODUCTION_CONFIG.yaml for inspection_manager and photo_capture_service",
    )
    reset_map_arg = DeclareLaunchArgument(
        "reset_map_on_startup",
        default_value="true",
        description="Reset Aurora map 6s after startup so each mission uses a fresh map. Aurora then builds from the live scene (LiDAR, depth, point cloud); no loaded/stale map to confuse costmap or vehicle detection. Set false to keep existing map.",
    )
    nav2_log_level_arg = DeclareLaunchArgument(
        "nav2_log_level",
        default_value="info",
        description="Log level for Nav2 nodes (info, warn, error, debug). Use info to avoid rcl DEBUG spam from smoother/planner/velocity_smoother.",
    )
    require_nav_permitted_arg = DeclareLaunchArgument(
        "require_nav_permitted",
        default_value="true",
        description="Gate goals on depth gate. Set false for headless runs without /stereo/navigation_permitted.",
    )
    use_tyre_3d_positions_arg = DeclareLaunchArgument(
        "use_tyre_3d_positions",
        default_value="true",
        description="inspection_manager: navigate from /tyre_3d_positions (tyre_3d_projection_node) when true.",
    )
    inspection_delay_s_arg = DeclareLaunchArgument(
        "inspection_delay_s",
        default_value="90.0",
        description="Seconds after launch before starting inspection_manager (Nav2 lifecycle must be up). 60–120 typical; lower for quick bench tests.",
    )
    require_detection_topic_at_startup_arg = DeclareLaunchArgument(
        "require_detection_topic_at_startup",
        default_value="true",
        description="inspection_manager: wait for tire merge topic at startup. Set false for minimal_perception + tyre_3d only.",
    )
    demo_mode_arg = DeclareLaunchArgument(
        "demo_mode",
        default_value="false",
        description="inspection_manager: bypass photo distance gates (stable_viz / stub motor).",
    )
    demo_simulate_nav_success_topic_arg = DeclareLaunchArgument(
        "demo_simulate_nav_success_topic",
        default_value="",
        description="inspection_manager: Empty on this topic → synthetic Nav2 arrival in INSPECT_TIRE when demo_mode.",
    )
    use_tyre_geometry_arg = DeclareLaunchArgument(
        "use_tyre_geometry",
        default_value="true",
        description="inspection_manager: infer vehicle frame from /tyre_3d_positions when possible.",
    )
    use_batch_waypoints_arg = DeclareLaunchArgument(
        "use_batch_waypoints",
        default_value="false",
        description="inspection_manager: Nav2 followWaypoints with full perimeter+standoff list in map frame (vs sequential goals).",
    )
    enable_tyre_3d_projection_arg = DeclareLaunchArgument(
        "enable_tyre_3d_projection",
        default_value="true",
        description="segment_3d: launch tyre_3d_projection_node.",
    )
    tyre_3d_sync_slop_s_arg = DeclareLaunchArgument(
        "tyre_3d_sync_slop_s",
        default_value="2.0",
        description="tyre_3d_projection: max |seg−depth| stamp delta (s); latest mode warns above this.",
    )
    tyre_3d_depth_pairing_mode_arg = DeclareLaunchArgument(
        "tyre_3d_depth_pairing_mode",
        default_value="latest",
        description='tyre_3d_projection: "latest" (depth cache + ObjectsSegment) or "approximate" (message_filters).',
    )
    tyre_3d_tf_lookup_use_latest_arg = DeclareLaunchArgument(
        "tyre_3d_tf_lookup_use_latest",
        default_value="false",
        description="tyre_3d_projection: prefer latest TF (stub / extrapolation).",
    )
    minimal_perception_arg = DeclareLaunchArgument(
        "minimal_perception",
        default_value="false",
        description="segment_3d: skip heavy nodes (semantic fusion, markers, tire seg_processor, PCL) for Jetson GPU RAM.",
    )
    pcl_fallback_enabled_arg = DeclareLaunchArgument(
        "pcl_fallback_enabled",
        default_value="true",
        description="segment_3d: tire_detector_pcl + tire_merger. Set false with minimal_perception on 8GB Jetson.",
    )
    centroid_servo_enabled_arg = DeclareLaunchArgument(
        "centroid_servo_enabled",
        default_value="true",
        description="segment_3d: centroid_servo_node. Set false to save CPU/GPU.",
    )
    # Tire detection overrides (forwarded to segment_3d); see docs/TIRE_DETECTION_TROUBLESHOOTING.md
    prefer_tensorrt_inspection_arg = DeclareLaunchArgument(
        "prefer_tensorrt_inspection",
        default_value="auto",
        description="segment_3d: auto|true|false. auto=use engine if exists. Set false if invalid class indices.",
    )
    wheel_max_det_arg = DeclareLaunchArgument(
        "wheel_max_det",
        default_value="50",
        description="segment_3d: max detections. Lower (e.g. 50) reduces NMS time. Default 50.",
    )
    wheel_imgsz_arg = DeclareLaunchArgument(
        "wheel_imgsz",
        default_value="640",
        description="segment_3d: GPU tyre inference size. CPU ONNX uses tyre_onnx_imgsz (default 480).",
    )
    tyre_onnx_imgsz_arg = DeclareLaunchArgument(
        "tyre_onnx_imgsz",
        default_value="480",
        description="segment_3d ultralytics_node_cpu: square ONNX input (must match export). Ignores wheel_imgsz.",
    )
    wheel_confidence_arg = DeclareLaunchArgument(
        "wheel_confidence",
        default_value="0.5",
        description="segment_3d: confidence threshold. Raise to 0.6 to reduce false positives.",
    )
    model_load_delay_arg = DeclareLaunchArgument(
        "model_load_delay_s",
        default_value="1.0",
        description="segment_3d: seconds to delay before loading YOLO/TensorRT. 1.0 for 16 GB Jetson; increase if OOM.",
    )
    use_cpu_inference_arg = DeclareLaunchArgument(
        "use_cpu_inference",
        default_value="false",
        description="segment_3d: use CPU ONNX tire detection. Set true for CPU fallback. Default false = GPU TensorRT/PyTorch (16 GB Jetson).",
    )
    inference_interval_s_arg = DeclareLaunchArgument(
        "inference_interval_s",
        default_value="0.1",
        description="segment_3d: seconds between GPU tire inferences. 0.1 = 10 Hz for 16 GB Jetson.",
    )
    _fb_pt = os.path.join(workspace_root, "src", "Tyre_Inspection_Bot", "best_fallback.pt")
    _tp = os.path.join(workspace_root, "tyre_detection_project")
    _te = os.path.join(_tp, "best.engine")
    _tpt = os.path.join(_tp, "best.pt")
    _default_wheel_inspection = (
        _te if os.path.isfile(_te) else (_tpt if os.path.isfile(_tpt) else _fb_pt)
    )
    wheel_inspection_model_arg = DeclareLaunchArgument(
        "wheel_inspection_model",
        default_value=_default_wheel_inspection,
        description="segment_3d: tyre YOLO/TensorRT model path (.engine preferred).",
    )
    enable_semantic_segmentation_arg = DeclareLaunchArgument(
        "enable_semantic_segmentation",
        default_value="true",
        description="Aurora SDK: subscribe to semantic segmentation (device GPU). Set false to reduce memory.",
    )
    costmap_resolution_arg = DeclareLaunchArgument(
        "costmap_resolution",
        default_value="0.10",
        description="Nav2: override local/global costmap grid resolution in generated YAML.",
    )
    enable_cmd_vel_mux_arg = DeclareLaunchArgument(
        "enable_cmd_vel_mux",
        default_value="true",
        description="Nav2: centroid vs nav mux. Set false for direct cmd_vel (minimal demo).",
    )
    enable_depth_gate_arg = DeclareLaunchArgument(
        "enable_depth_gate",
        default_value="true",
        description="Nav2: gate motion on /stereo/navigation_permitted. Set false when bridge off.",
    )
    enable_vehicle_speed_filter_arg = DeclareLaunchArgument(
        "enable_vehicle_speed_filter",
        default_value="true",
        description="Nav2: vehicle speed filter mask node. Set false to save CPU.",
    )
    yolo_device_arg = DeclareLaunchArgument(
        "yolo_device",
        default_value="0",
        description="segment_3d ultralytics_tire: 0=cuda:0 or string cpu to avoid GPU OOM.",
    )
    depth_registered_publish_hz_arg = DeclareLaunchArgument(
        "depth_registered_publish_hz",
        default_value="0.0",
        description="segment_3d: max registered pointcloud rate (Hz); 0.0=full rate (must be float for depth_to_registered_pointcloud_node).",
    )

    # 0) Motor driver — subscribes to cmd_vel, forwards to ESP32; optionally publishes /wheel/odometry
    # (use_motor_driver:=false if no hardware; sim_no_move:=true uses stub_motor instead)
    motor_driver = Node(
        package="ugv_base_driver",
        executable="motor_driver_node",
        name="motor_driver",
        output="screen",
        parameters=[{
            "uart_port": LaunchConfiguration("uart_port"),
            "publish_wheel_odom": LaunchConfiguration("publish_wheel_odom"),
        }],
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration("use_motor_driver"), "' == 'true' and '",
                LaunchConfiguration("sim_no_move"), "' == 'false'"
            ])
        ),
    )
    stub_motor = Node(
        package="ugv_base_driver",
        executable="stub_motor_node",
        name="stub_motor",
        output="screen",
        condition=IfCondition(LaunchConfiguration("sim_no_move")),
    )
    aurora_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ugv_nav_share, "launch", "aurora_bringup.launch.py")
        ),
        launch_arguments={
            "ip_address": LaunchConfiguration("ip_address"),
            "use_bridge": LaunchConfiguration("use_bridge"),
            "enable_semantic_segmentation": LaunchConfiguration("enable_semantic_segmentation"),
        }.items(),
    )

    # 1) Reset Aurora map on startup (default: true) — each mission gets a fresh map
    # Aurora builds from the current scene (LiDAR, stereo depth, point cloud). Costmap and vehicle detection
    # then reflect what is actually in front of the robot; no stale/loaded map to ruin the area of interest.
    # slamware_ros_sdk_server_node subscribes to /slamware_ros_sdk_server_node/clear_map and calls requireMapReset()
    clear_map = ExecuteProcess(
        cmd=["ros2", "topic", "pub", "/slamware_ros_sdk_server_node/clear_map",
             "slamware_ros_sdk/msg/ClearMapRequest", "{}", "--once"],
        name="reset_aurora_map",
        output="screen",
        condition=IfCondition(LaunchConfiguration("reset_map_on_startup")),
    )
    clear_map_delayed = TimerAction(period=6.0, actions=[clear_map])

    # 2) Perception (YOLO + 3D boxes + depth pipeline) — after Aurora topics exist
    segment_3d_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(seg3d_share, "launch", "segment_3d.launch.py")
        ),
        launch_arguments={
            "use_bridge": LaunchConfiguration("use_bridge"),
            "prefer_tensorrt_inspection": LaunchConfiguration("prefer_tensorrt_inspection"),
            "wheel_max_det": LaunchConfiguration("wheel_max_det"),
            "wheel_imgsz": LaunchConfiguration("wheel_imgsz"),
            "tyre_onnx_imgsz": LaunchConfiguration("tyre_onnx_imgsz"),
            "wheel_inspection_model": LaunchConfiguration("wheel_inspection_model"),
            "wheel_confidence": LaunchConfiguration("wheel_confidence"),
            "model_load_delay_s": LaunchConfiguration("model_load_delay_s"),
            "inference_interval_s": LaunchConfiguration("inference_interval_s", default="0.1"),
            "enable_tyre_3d_projection": LaunchConfiguration("enable_tyre_3d_projection"),
            "tyre_3d_sync_slop_s": LaunchConfiguration("tyre_3d_sync_slop_s"),
            "tyre_3d_depth_pairing_mode": LaunchConfiguration("tyre_3d_depth_pairing_mode"),
            "tyre_3d_tf_lookup_use_latest": LaunchConfiguration("tyre_3d_tf_lookup_use_latest"),
            "use_cpu_inference": LaunchConfiguration("use_cpu_inference"),
            "sim_tyre_detections": LaunchConfiguration("sim_tyre_detections"),
            "use_yolo": PythonExpression([
                "'false' if '", LaunchConfiguration("sim_tyre_detections"),
                "' == 'true' else 'true'",
            ]),
            "pcl_fallback_enabled": PythonExpression([
                "'false' if '", LaunchConfiguration("sim_tyre_detections"),
                "' == 'true' else '", LaunchConfiguration("pcl_fallback_enabled"), "'",
            ]),
            "minimal_perception": LaunchConfiguration("minimal_perception"),
            "centroid_servo_enabled": LaunchConfiguration("centroid_servo_enabled"),
            "device": LaunchConfiguration("yolo_device"),
            "depth_registered_publish_hz": LaunchConfiguration("depth_registered_publish_hz"),
        }.items(),
    )
    segment_3d_delayed = TimerAction(period=8.0, actions=[segment_3d_launch])

    # 2b) EKF (when use_ekf) — fuse Aurora odom + IMU + wheel odom; must start before Nav2
    ekf_aurora_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ugv_nav_share, "launch", "ekf_aurora.launch.py")
        ),
        condition=IfCondition(LaunchConfiguration("use_ekf")),
    )
    ekf_aurora_delayed = TimerAction(period=5.0, actions=[ekf_aurora_launch])

    # 3) Nav2 + depth_gate + lifecycle — after Aurora map/odom/scan (and EKF if use_ekf)
    nav_aurora_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ugv_nav_share, "launch", "nav_aurora.launch.py")
        ),
        launch_arguments={
            "use_ekf": LaunchConfiguration("use_ekf"),
            "nav2_log_level": LaunchConfiguration("nav2_log_level", default="info"),
            "sim_tyre_detections": LaunchConfiguration("sim_tyre_detections"),
            "costmap_resolution": LaunchConfiguration("costmap_resolution"),
            "enable_cmd_vel_mux": LaunchConfiguration("enable_cmd_vel_mux"),
            "enable_depth_gate": LaunchConfiguration("enable_depth_gate"),
            "enable_vehicle_speed_filter": LaunchConfiguration("enable_vehicle_speed_filter"),
        }.items(),
    )
    nav_aurora_delayed = TimerAction(period=10.0, actions=[nav_aurora_launch])

    # 4) Inspection manager + photo capture — after Nav2 lifecycle
    # Nav2 nodes at 25s, nav_lifecycle_startup at 45s. Bringup can take 60s (TF
    # wait) + 30–60s (configure/activate). Inspection at 120s so Nav2 is almost
    # always ready; manager still waits for navigate_to_pose (nav2_wait_timeout).
    # When sim_tyre_detections:=true, use /sim/* topics so manager sees only
    # simulated detections (no competition from aurora_semantic_fusion / tire_merger).
    inspection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(inspection_share, "launch", "inspection_manager.launch.py")
        ),
        launch_arguments={
            "perception_only_mode": LaunchConfiguration("perception_only_mode"),
            "params_file": PythonExpression([
                "'", os.path.join(workspace_root, "PRODUCTION_CONFIG_SIM.yaml"), "' if '",
                LaunchConfiguration("sim_tyre_detections"),
                "' == 'true' else '", LaunchConfiguration("params_file"), "'",
            ]),
            "tire_detection_topic": PythonExpression([
                "'/sim/tire_bounding_boxes_merged' if '",
                LaunchConfiguration("sim_tyre_detections"),
                "' == 'true' else '/tire_bounding_boxes_merged'",
            ]),
            "vehicle_fallback_topic": PythonExpression([
                "'/sim/vehicle_bounding_boxes' if '",
                LaunchConfiguration("sim_tyre_detections"),
                "' == 'true' else '/aurora_semantic/vehicle_bounding_boxes'",
            ]),
            "require_nav_permitted": LaunchConfiguration("require_nav_permitted"),
            "use_tyre_3d_positions": LaunchConfiguration("use_tyre_3d_positions"),
            "require_detection_topic_at_startup": LaunchConfiguration(
                "require_detection_topic_at_startup"
            ),
            "demo_mode": LaunchConfiguration("demo_mode"),
            "demo_simulate_nav_success_topic": LaunchConfiguration("demo_simulate_nav_success_topic"),
            "use_tyre_geometry": LaunchConfiguration("use_tyre_geometry"),
            "use_batch_waypoints": LaunchConfiguration("use_batch_waypoints"),
        }.items(),
    )
    def _inspection_timer_action(context):
        raw = perform_substitutions(context, [LaunchConfiguration("inspection_delay_s")]).strip()
        try:
            delay = float(raw)
        except ValueError:
            delay = 90.0
        return [TimerAction(period=max(5.0, delay), actions=[inspection_launch])]

    inspection_delayed = OpaqueFunction(function=_inspection_timer_action)

    # Sim tyre detections: run verify_system.py --simulate --publish-objects-segment
    # Publishes to /sim/vehicle_bounding_boxes and /sim/tire_bounding_boxes_merged so
    # inspection_manager (remapped to those topics) sees only simulated detections.
    verify_system_script = os.path.join(workspace_root, "scripts", "verify_system.py")
    sim_tyre_node = ExecuteProcess(
        cmd=[
            "python3", "-u", verify_system_script,
            "--simulate", "--publish-objects-segment",
            "--topic-vehicle", "/sim/vehicle_bounding_boxes",
            "--topic-tire", "/sim/tire_bounding_boxes_merged",
            "--duration", "3600",
        ],
        name="verify_simulate_detections",
        output="screen",
        condition=IfCondition(LaunchConfiguration("sim_tyre_detections")),
    )
    sim_tyre_delayed = TimerAction(period=15.0, actions=[sim_tyre_node])

    # All nodes must use same RMW so TF, topics, and mission see each other.
    # startup.sh also sets this.
    set_rmw = SetEnvironmentVariable(name="RMW_IMPLEMENTATION", value="rmw_cyclonedds_cpp")

    return LaunchDescription([
        set_rmw,
        ip_address_arg,
        require_nav_permitted_arg,
        use_tyre_3d_positions_arg,
        inspection_delay_s_arg,
        require_detection_topic_at_startup_arg,
        demo_mode_arg,
        demo_simulate_nav_success_topic_arg,
        use_tyre_geometry_arg,
        use_batch_waypoints_arg,
        enable_tyre_3d_projection_arg,
        tyre_3d_sync_slop_s_arg,
        tyre_3d_depth_pairing_mode_arg,
        tyre_3d_tf_lookup_use_latest_arg,
        minimal_perception_arg,
        pcl_fallback_enabled_arg,
        centroid_servo_enabled_arg,
        params_file_arg,
        reset_map_arg,
        nav2_log_level_arg,
        prefer_tensorrt_inspection_arg,
        wheel_max_det_arg,
        wheel_imgsz_arg,
        tyre_onnx_imgsz_arg,
        wheel_confidence_arg,
        model_load_delay_arg,
        inference_interval_s_arg,
        wheel_inspection_model_arg,
        enable_semantic_segmentation_arg,
        costmap_resolution_arg,
        enable_cmd_vel_mux_arg,
        enable_depth_gate_arg,
        enable_vehicle_speed_filter_arg,
        yolo_device_arg,
        depth_registered_publish_hz_arg,
        use_cpu_inference_arg,
        use_bridge_arg,
        perception_only_arg,
        use_motor_driver_arg,
        publish_wheel_odom_arg,
        use_ekf_arg,
        uart_port_arg,
        sim_no_move_arg,
        sim_tyre_detections_arg,
        motor_driver,
        stub_motor,
        aurora_bringup,
        clear_map_delayed,
        segment_3d_delayed,
        ekf_aurora_delayed,
        nav_aurora_delayed,
        inspection_delayed,
        sim_tyre_delayed,
    ])
