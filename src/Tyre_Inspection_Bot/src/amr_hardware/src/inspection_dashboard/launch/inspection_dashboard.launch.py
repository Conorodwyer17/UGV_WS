from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("inspection_dashboard")
    config_file = os.path.join(pkg_share, "config", "dashboard.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value=config_file,
            description="Path to dashboard YAML config",
        ),
        Node(
            package="inspection_dashboard",
            executable="inspection_dashboard_node",
            name="inspection_dashboard",
            output="screen",
            parameters=[LaunchConfiguration("params_file")],
        ),
    ])
