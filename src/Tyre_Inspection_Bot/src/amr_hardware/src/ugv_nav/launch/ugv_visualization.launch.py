# Launch RViz with UGV inspection stack visualization (map, camera, detections, path).
# Use the same RMW as the robot so topics are visible (e.g. run on VM/desktop on same network).
#
# Prerequisites (on robot/Jetson):
#   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#   ros2 launch ugv_nav aurora_bringup.launch.py
#   ros2 launch segmentation_3d segment_3d.launch.py
#   (optional) ros2 launch inspection_manager inspection_manager.launch.py
#
# Then on your VM or desktop (same network, same RMW):
#   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#   ros2 launch ugv_nav ugv_visualization.launch.py
#
# Run: ros2 launch ugv_nav ugv_visualization.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    ugv_nav_share = get_package_share_directory("ugv_nav")
    rviz_config = os.path.join(ugv_nav_share, "rviz", "ugv_visualization.rviz")

    # So RViz sees the same topics as the robot (Aurora, segment_3d, inspection_manager)
    set_rmw = SetEnvironmentVariable(
        name="RMW_IMPLEMENTATION",
        value="rmw_cyclonedds_cpp",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    return LaunchDescription([
        set_rmw,
        rviz_node,
    ])
