# Launch Aurora + preview for calibration.
# Use this when calibrating — brings up Aurora, bridge, and preview topics for rqt_image_view.

from launch import LaunchDescription
import os

# Reuse aurora_testing (Aurora + bridge) and add preview node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Aurora + bridge
    aurora_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory("ugv_nav"),
                "launch", "aurora_testing.launch.py"
            )
        ]),
    )

    # Preview node: publishes /aurora_calibration/preview_* for rqt_image_view
    preview_node = Node(
        package="ugv_nav",
        executable="aurora_calibration_preview",
        name="aurora_calibration_preview",
        output="screen",
    )

    return LaunchDescription([
        aurora_launch,
        preview_node,
    ])
