# Minimal stack for 8GB Jetson: fewer GPU/CPU processes, YOLO @ 640 (match tyre .pt on CPU), coarse costmaps, tyre_3d goals.
# Skips semantic fusion, RViz vehicle markers, tire segmentation_processor, PCL fallback, centroid servo.
# Also disables Aurora semantic segmentation subscription, throttles depth pointcloud, optional Nav2 vel extras.
#
# Usage:
#   ros2 launch ugv_bringup minimal_tyre_inspection.launch.py
#   ros2 launch ugv_bringup minimal_tyre_inspection.launch.py sim_no_move:=true
import os

from ament_index_python.packages import get_package_share_directory

_TYRE_PT = os.path.join(
    os.environ.get("UGV_WS", os.path.join(os.path.expanduser("~"), "ugv_ws")),
    "tyre_detection_project",
    "best.pt",
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ugv_nav_share = get_package_share_directory("ugv_nav")
    full_bringup = os.path.join(ugv_nav_share, "launch", "full_bringup.launch.py")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "sim_no_move",
                default_value="false",
                description="Use stub_motor (no physical motion).",
            ),
            DeclareLaunchArgument(
                "reset_map_on_startup",
                default_value="false",
                description="Skip clear_map pub to reduce startup noise (optional).",
            ),
            DeclareLaunchArgument(
                "wheel_imgsz",
                default_value="640",
                description="YOLO inference size (must match TensorRT export, e.g. tyre_detection_project/best.engine).",
            ),
            DeclareLaunchArgument(
                "inference_interval_s",
                default_value="0.15",
                description="Throttle ultralytics tire (less GPU load).",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(full_bringup),
                launch_arguments={
                    "sim_no_move": LaunchConfiguration("sim_no_move"),
                    "reset_map_on_startup": LaunchConfiguration("reset_map_on_startup"),
                    "use_tyre_3d_positions": "true",
                    "require_nav_permitted": "false",
                    "enable_tyre_3d_projection": "true",
                    "require_detection_topic_at_startup": "false",
                    "minimal_perception": "true",
                    "pcl_fallback_enabled": "false",
                    "centroid_servo_enabled": "false",
                    "wheel_imgsz": LaunchConfiguration("wheel_imgsz"),
                    "inference_interval_s": LaunchConfiguration("inference_interval_s"),
                    "enable_semantic_segmentation": "false",
                    "costmap_resolution": "0.10",
                    "enable_cmd_vel_mux": "false",
                    "enable_depth_gate": "false",
                    "enable_vehicle_speed_filter": "false",
                    "yolo_device": "cpu",
                    "prefer_tensorrt_inspection": "false",
                    "depth_registered_publish_hz": "2.0",
                    "wheel_inspection_model": _TYRE_PT,
                }.items(),
            ),
        ]
    )
