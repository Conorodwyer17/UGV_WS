# Launch robot_localization EKF to fuse Aurora odom + IMU and publish /odometry/filtered + odom->base_link.
# Requires: sudo apt install ros-humble-robot-localization
# When using this launch, point Nav2 at filtered odom: use nav_aurora_ekf_overrides.yaml after nav_aurora.yaml
# (e.g. params_file: [nav_aurora.yaml, nav_aurora_ekf_overrides.yaml]).
# Note: Aurora SDK also publishes odom->base_link; for a single source either disable Aurora's TF if
# the SDK supports it, or run EKF for smoothing and use /odometry/filtered for Nav2 (TF may still
# come from Aurora; duplicate TF can cause issues - prefer configuring Aurora to not publish odom->base_link).

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("ugv_nav")
    params_file = os.path.join(pkg_share, "param", "ekf_aurora.yaml")

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[params_file, {"use_sim_time": False}],
    )

    return LaunchDescription([ekf_node])
