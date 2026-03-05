# Vision-only launch: Aurora + segment_3d. No Nav2, no motor, no inspection manager.
# Use for dry-run detection testing (e.g. scripts/start_vision_only.sh).
#
# Usage: ros2 launch ugv_nav vision_only.launch.py [ip_address:=192.168.11.1]

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ugv_nav_share = get_package_share_directory("ugv_nav")
    seg3d_share = get_package_share_directory("segmentation_3d")

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

    aurora_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ugv_nav_share, "launch", "aurora_bringup.launch.py")
        ),
        launch_arguments={
            "ip_address": LaunchConfiguration("ip_address"),
            "use_bridge": LaunchConfiguration("use_bridge"),
        }.items(),
    )

    segment_3d_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(seg3d_share, "launch", "segment_3d.launch.py")
        ),
        launch_arguments={
            "use_bridge": LaunchConfiguration("use_bridge"),
        }.items(),
    )
    segment_3d_delayed = TimerAction(period=8.0, actions=[segment_3d_launch])

    set_rmw = SetEnvironmentVariable(name="RMW_IMPLEMENTATION", value="rmw_cyclonedds_cpp")

    return LaunchDescription([
        set_rmw,
        ip_address_arg,
        use_bridge_arg,
        aurora_bringup,
        segment_3d_delayed,
    ])
