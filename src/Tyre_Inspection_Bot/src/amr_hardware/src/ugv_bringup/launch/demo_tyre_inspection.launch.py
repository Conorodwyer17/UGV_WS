# Demo / thesis: full stack with direct tyre 3D goals (tyre_3d_projection_node + use_tyre_3d_positions).
# Usage:
#   ros2 launch ugv_bringup demo_tyre_inspection.launch.py
#   ros2 launch ugv_bringup demo_tyre_inspection.launch.py sim_no_move:=true
import os

from ament_index_python.packages import get_package_share_directory
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
                description="Use stub_motor (no motion) for TF / costmap checks.",
            ),
            DeclareLaunchArgument(
                "use_tyre_3d_positions",
                default_value="true",
                description="inspection_manager: prefer /tyre_3d_positions over vehicle boxes.",
            ),
            DeclareLaunchArgument(
                "require_nav_permitted",
                default_value="false",
                description="Set false if depth gate is not running.",
            ),
            DeclareLaunchArgument(
                "wheel_imgsz",
                default_value="480",
                description="YOLO inference size (match TensorRT engine; lower GPU memory than 640).",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(full_bringup),
                launch_arguments={
                    "sim_no_move": LaunchConfiguration("sim_no_move"),
                    "use_tyre_3d_positions": LaunchConfiguration("use_tyre_3d_positions"),
                    "require_nav_permitted": LaunchConfiguration("require_nav_permitted"),
                    "enable_tyre_3d_projection": "true",
                    "wheel_imgsz": LaunchConfiguration("wheel_imgsz"),
                }.items(),
            ),
        ]
    )
