# Modular thesis demo: tyre YOLO + depth + tyre_3d_projection (no Nav2, semantic off).
# Default model: $UGV_WS/tyre_detection_project/best.pt; GPU unless use_cpu_inference:=true.
#
#   ros2 launch ugv_bringup demo_tyre_detection.launch.py
#   ros2 launch ugv_bringup demo_tyre_detection.launch.py use_cpu_inference:=true
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    workspace = os.environ.get("UGV_WS", os.path.expanduser("~/ugv_ws"))
    _tyre_pt = os.path.join(workspace, "tyre_detection_project", "best.pt")

    ugv_nav_share = get_package_share_directory("ugv_nav")
    seg_share = get_package_share_directory("segmentation_3d")
    ugv_bringup_share = get_package_share_directory("ugv_bringup")
    segment_3d = os.path.join(seg_share, "launch", "segment_3d.launch.py")
    rviz_cfg = os.path.join(ugv_bringup_share, "config", "tyre_detection.rviz")

    set_rmw = SetEnvironmentVariable(name="RMW_IMPLEMENTATION", value="rmw_cyclonedds_cpp")

    ip_address_arg = DeclareLaunchArgument("ip_address", default_value="192.168.11.1")
    use_bridge_arg = DeclareLaunchArgument("use_bridge", default_value="false")
    use_cpu_inference_arg = DeclareLaunchArgument(
        "use_cpu_inference",
        default_value="false",
        description="ONNX CPU tyre node (set true if GPU OOM).",
    )
    yolo_device_arg = DeclareLaunchArgument(
        "yolo_device",
        default_value="0",
        description="GPU id (e.g. 0) or cpu string for ultralytics when use_cpu_inference is false.",
    )
    wheel_imgsz_arg = DeclareLaunchArgument("wheel_imgsz", default_value="640")
    prefer_tensorrt_arg = DeclareLaunchArgument(
        "prefer_tensorrt_inspection",
        default_value="true",
        description="Use TensorRT engine when present (set false to force PyTorch).",
    )
    inference_interval_s_arg = DeclareLaunchArgument(
        "inference_interval_s",
        default_value="0.15",
        description="Throttle between tyre inferences (lower load on small Jetsons).",
    )
    wheel_inspection_model_arg = DeclareLaunchArgument(
        "wheel_inspection_model",
        default_value=_tyre_pt,
        description="Path to tyre .pt or .engine",
    )

    aurora_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ugv_nav_share, "launch", "aurora_bringup.launch.py")
        ),
        launch_arguments={
            "ip_address": LaunchConfiguration("ip_address"),
            "use_bridge": LaunchConfiguration("use_bridge"),
            "enable_semantic_segmentation": "false",
        }.items(),
    )

    segment_3d_delayed = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(segment_3d),
                launch_arguments={
                    "use_bridge": LaunchConfiguration("use_bridge"),
                    "minimal_perception": "true",
                    "pcl_fallback_enabled": "false",
                    "centroid_servo_enabled": "false",
                    "enable_tyre_3d_projection": "true",
                    "use_yolo": "true",
                    "use_cpu_inference": LaunchConfiguration("use_cpu_inference"),
                    "device": LaunchConfiguration("yolo_device"),
                    "wheel_inspection_model": LaunchConfiguration("wheel_inspection_model"),
                    "wheel_imgsz": LaunchConfiguration("wheel_imgsz"),
                    "prefer_tensorrt_inspection": LaunchConfiguration("prefer_tensorrt_inspection"),
                    "depth_registered_publish_hz": "2.0",
                    "inference_interval_s": LaunchConfiguration("inference_interval_s"),
                }.items(),
            )
        ],
    )

    rviz = TimerAction(
        period=12.0,
        actions=[ExecuteProcess(cmd=["rviz2", "-d", rviz_cfg], output="screen")],
    )

    return LaunchDescription(
        [
            set_rmw,
            ip_address_arg,
            use_bridge_arg,
            use_cpu_inference_arg,
            yolo_device_arg,
            wheel_imgsz_arg,
            prefer_tensorrt_arg,
            inference_interval_s_arg,
            wheel_inspection_model_arg,
            aurora_bringup,
            segment_3d_delayed,
            rviz,
        ]
    )
