# Full pipeline visualisation with NO physical motion: stub_motor eats /cmd_vel.
# Aurora semantic (vehicles) + tyre YOLO + tyre_3d_projection + Nav2 + RViz +
# inspection_manager + photo_capture_service (starts after inspection_delay_s, default 100 s).
# Nav2 publishes cmd_vel; cmd_vel_mux, depth_gate, vehicle_speed_filter are OFF to save CPU.
#
# Default `use_cpu_inference:=true` for stability on 8 GB Jetsons. CPU uses `ultralytics_node_cpu`
# with ONNX derived from `tyre_detection_project/best.pt` → `best.onnx` (not TensorRT `.engine`).
# For GPU + TensorRT / PyTorch: `use_cpu_inference:=false` (device string "0"; `wheel_inspection_model` may be .engine).
#
#   ros2 launch ugv_bringup demo_full_visualization.launch.py
#   ros2 launch ugv_bringup demo_full_visualization.launch.py use_cpu_inference:=false
#   ros2 launch ugv_bringup demo_full_visualization.launch.py wheel_imgsz:=640  # if ONNX exported at 640
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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.utilities import perform_substitutions
from launch_ros.actions import Node


def generate_launch_description():
    workspace = os.environ.get("UGV_WS", os.path.expanduser("~/ugv_ws"))
    _tp = os.path.join(workspace, "tyre_detection_project")
    _te = os.path.join(_tp, "best.engine")
    _tpt = os.path.join(_tp, "best.pt")
    _fb_pt = os.path.join(workspace, "src", "Tyre_Inspection_Bot", "best_fallback.pt")
    _default_wheel = _te if os.path.isfile(_te) else (_tpt if os.path.isfile(_tpt) else _fb_pt)

    ugv_nav_share = get_package_share_directory("ugv_nav")
    seg_share = get_package_share_directory("segmentation_3d")
    ugv_bringup_share = get_package_share_directory("ugv_bringup")
    inspection_share = get_package_share_directory("inspection_manager")
    segment_3d_launch_file = os.path.join(seg_share, "launch", "segment_3d.launch.py")
    nav_aurora = os.path.join(ugv_nav_share, "launch", "nav_aurora.launch.py")
    rviz_cfg = os.path.join(ugv_bringup_share, "config", "full_visualization.rviz")
    inspection_launch_file = os.path.join(inspection_share, "launch", "inspection_manager.launch.py")

    set_rmw = SetEnvironmentVariable(name="RMW_IMPLEMENTATION", value="rmw_cyclonedds_cpp")

    ip_address_arg = DeclareLaunchArgument("ip_address", default_value="192.168.11.1")
    use_bridge_arg = DeclareLaunchArgument("use_bridge", default_value="false")
    reset_map_arg = DeclareLaunchArgument(
        "reset_map_on_startup",
        default_value="true",
        description="Publish clear_map once ~6s after start",
    )
    wheel_inspection_model_arg = DeclareLaunchArgument(
        "wheel_inspection_model",
        default_value=_default_wheel,
        description=(
            "GPU path only: .engine or .pt passed to ultralytics_node. "
            "When use_cpu_inference is true, launch overrides to tyre_detection_project/best.pt so ONNX path is used."
        ),
    )
    use_cpu_inference_arg = DeclareLaunchArgument(
        "use_cpu_inference",
        default_value="true",
        description=(
            "Default true: ONNX CPU tyre node (stable on 8 GB). Set false for GPU; device is passed as string '0'."
        ),
    )
    wheel_imgsz_arg = DeclareLaunchArgument(
        "wheel_imgsz",
        default_value="480",
        description=(
            "Inference / ONNX input size (square). Must match exported ONNX/TensorRT (CPU ONNX default 480)."
        ),
    )
    prefer_tensorrt_arg = DeclareLaunchArgument(
        "prefer_tensorrt_inspection",
        default_value="true",
        description="Ignored when use_cpu_inference true (forced false). GPU: TensorRT when engine exists.",
    )
    inference_interval_s_arg = DeclareLaunchArgument(
        "inference_interval_s",
        default_value="0.5",
        description="Throttle tyre YOLO (~2 Hz default)",
    )
    depth_hz_arg = DeclareLaunchArgument(
        "depth_registered_publish_hz",
        default_value="2.0",
        description="Max registered point cloud rate (Hz)",
    )
    costmap_resolution_arg = DeclareLaunchArgument("costmap_resolution", default_value="0.10")
    costmap_local_update_frequency_arg = DeclareLaunchArgument(
        "costmap_local_update_frequency",
        default_value="2.0",
        description="Local costmap update rate (passed to nav_aurora YAML override)",
    )
    nav2_log_level_arg = DeclareLaunchArgument("nav2_log_level", default_value="info")

    params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(workspace, "PRODUCTION_CONFIG.yaml"),
        description="PRODUCTION_CONFIG for inspection_manager and photo_capture_service",
    )
    use_tyre_3d_positions_arg = DeclareLaunchArgument(
        "use_tyre_3d_positions",
        default_value="true",
        description="inspection_manager: plan goals from /tyre_3d_positions (tyre_3d_projection_node)",
    )
    require_nav_permitted_arg = DeclareLaunchArgument(
        "require_nav_permitted",
        default_value="false",
        description="Set false so goals are not gated on /stereo/navigation_permitted (depth gate off in this demo)",
    )
    require_detection_topic_at_startup_arg = DeclareLaunchArgument(
        "require_detection_topic_at_startup",
        default_value="false",
        description="Do not block startup on merged tire topic; segment_3d publishes /darknet_ros_3d/tire_bounding_boxes",
    )
    launch_visual_servo_arg = DeclareLaunchArgument(
        "launch_visual_servo",
        default_value="false",
        description="Omit visual_servo_align_server in the demo to save CPU",
    )
    inspection_delay_s_arg = DeclareLaunchArgument(
        "inspection_delay_s",
        default_value="100.0",
        description="Seconds after launch before starting inspection_manager (Nav2 lifecycle should be ready)",
    )

    stub_motor = Node(
        package="ugv_base_driver",
        executable="stub_motor_node",
        name="stub_motor",
        output="screen",
        parameters=[{"cmd_vel_topic": "/cmd_vel"}],
    )

    aurora_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ugv_nav_share, "launch", "aurora_bringup.launch.py")
        ),
        launch_arguments={
            "ip_address": LaunchConfiguration("ip_address"),
            "use_bridge": LaunchConfiguration("use_bridge"),
            "enable_semantic_segmentation": "true",
        }.items(),
    )

    clear_map = ExecuteProcess(
        cmd=[
            "ros2",
            "topic",
            "pub",
            "/slamware_ros_sdk_server_node/clear_map",
            "slamware_ros_sdk/msg/ClearMapRequest",
            "{}",
            "--once",
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("reset_map_on_startup")),
    )
    clear_map_delayed = TimerAction(period=6.0, actions=[clear_map])

    def _segment_3d_delayed(context):
        """Pass device as string ('0' or 'cpu'); CPU path forces .pt so ONNX is used (never .engine)."""
        use_cpu = perform_substitutions(context, [LaunchConfiguration("use_cpu_inference")]).strip().lower() in (
            "true",
            "1",
            "yes",
        )
        device_sub = TextSubstitution(text="cpu" if use_cpu else "0")
        if use_cpu:
            ptrt_sub = TextSubstitution(text="false")
            pt_path = os.path.join(workspace, "tyre_detection_project", "best.pt")
            if not os.path.isfile(pt_path):
                pt_path = _fb_pt
            wheel_model_sub = TextSubstitution(text=os.path.abspath(pt_path))
        else:
            ptrt_sub = LaunchConfiguration("prefer_tensorrt_inspection")
            wheel_model_sub = LaunchConfiguration("wheel_inspection_model")

        return [
            TimerAction(
                period=8.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(segment_3d_launch_file),
                        launch_arguments={
                            "use_bridge": LaunchConfiguration("use_bridge"),
                            "minimal_perception": "false",
                            "pcl_fallback_enabled": "false",
                            "centroid_servo_enabled": "false",
                            "enable_tyre_3d_projection": "true",
                            "use_yolo": "true",
                            "use_cpu_inference": LaunchConfiguration("use_cpu_inference"),
                            "device": device_sub,
                            "wheel_inspection_model": wheel_model_sub,
                            "wheel_imgsz": LaunchConfiguration("wheel_imgsz"),
                            "prefer_tensorrt_inspection": ptrt_sub,
                            "depth_registered_publish_hz": LaunchConfiguration("depth_registered_publish_hz"),
                            "inference_interval_s": LaunchConfiguration("inference_interval_s"),
                        }.items(),
                    )
                ],
            )
        ]

    segment_3d_delayed = OpaqueFunction(function=_segment_3d_delayed)

    nav_aurora_delayed = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav_aurora),
                launch_arguments={
                    "costmap_resolution": LaunchConfiguration("costmap_resolution"),
                    "costmap_local_update_frequency": LaunchConfiguration(
                        "costmap_local_update_frequency"
                    ),
                    "enable_cmd_vel_mux": "false",
                    "enable_depth_gate": "false",
                    "enable_vehicle_speed_filter": "false",
                    "nav2_log_level": LaunchConfiguration("nav2_log_level"),
                    "sim_tyre_detections": "false",
                }.items(),
            )
        ],
    )

    rviz = TimerAction(
        period=12.0,
        actions=[ExecuteProcess(cmd=["rviz2", "-d", rviz_cfg], output="screen")],
    )

    def _inspection_delayed(context):
        s = perform_substitutions(context, [LaunchConfiguration("inspection_delay_s")]).strip()
        try:
            delay = float(s)
        except ValueError:
            delay = 100.0
        return [
            TimerAction(
                period=delay,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(inspection_launch_file),
                        launch_arguments={
                            "params_file": LaunchConfiguration("params_file"),
                            "use_tyre_3d_positions": LaunchConfiguration("use_tyre_3d_positions"),
                            "require_nav_permitted": LaunchConfiguration("require_nav_permitted"),
                            "require_detection_topic_at_startup": LaunchConfiguration(
                                "require_detection_topic_at_startup"
                            ),
                            "tire_detection_topic": "/darknet_ros_3d/tire_bounding_boxes",
                            "vehicle_fallback_topic": "/aurora_semantic/vehicle_bounding_boxes",
                            "launch_visual_servo": LaunchConfiguration("launch_visual_servo"),
                            "perception_only_mode": "false",
                        }.items(),
                    )
                ],
            )
        ]

    inspection_delayed = OpaqueFunction(function=_inspection_delayed)

    return LaunchDescription(
        [
            set_rmw,
            ip_address_arg,
            use_bridge_arg,
            reset_map_arg,
            wheel_inspection_model_arg,
            use_cpu_inference_arg,
            wheel_imgsz_arg,
            prefer_tensorrt_arg,
            inference_interval_s_arg,
            depth_hz_arg,
            costmap_resolution_arg,
            costmap_local_update_frequency_arg,
            nav2_log_level_arg,
            params_file_arg,
            use_tyre_3d_positions_arg,
            require_nav_permitted_arg,
            require_detection_topic_at_startup_arg,
            launch_visual_servo_arg,
            inspection_delay_s_arg,
            stub_motor,
            aurora_bringup,
            clear_map_delayed,
            segment_3d_delayed,
            nav_aurora_delayed,
            inspection_delayed,
            rviz,
        ]
    )
