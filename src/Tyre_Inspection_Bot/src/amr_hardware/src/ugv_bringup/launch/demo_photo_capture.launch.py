# Modular thesis demo: Aurora left camera + photo_capture_service only (no Nav2, no perception stack).
# Save directory defaults under ~/ugv_ws/tire_inspection_photos (override with save_directory).
#
# Trigger capture (after camera is streaming):
#   ros2 service call /photo_capture_service/capture_photo std_srvs/srv/Trigger
# Or publish Bool on capture topic (see photo_capture_service params).
#
#   ros2 launch ugv_bringup demo_photo_capture.launch.py
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    workspace = os.environ.get("UGV_WS", os.path.expanduser("~/ugv_ws"))
    ugv_nav_share = get_package_share_directory("ugv_nav")

    set_rmw = SetEnvironmentVariable(name="RMW_IMPLEMENTATION", value="rmw_cyclonedds_cpp")

    ip_address_arg = DeclareLaunchArgument("ip_address", default_value="192.168.11.1")
    use_bridge_arg = DeclareLaunchArgument("use_bridge", default_value="false")
    save_directory_arg = DeclareLaunchArgument(
        "save_directory",
        default_value=os.path.join(workspace, "tire_inspection_photos"),
        description="Where images and manifest are written",
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

    photo_service = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="inspection_manager",
                executable="photo_capture_service",
                name="photo_capture_service",
                output="screen",
                parameters=[
                    {
                        "camera_topic": "/slamware_ros_sdk_server_node/left_image_raw",
                        "save_directory": LaunchConfiguration("save_directory"),
                        "image_format": "jpg",
                    }
                ],
            )
        ],
    )

    return LaunchDescription(
        [
            set_rmw,
            ip_address_arg,
            use_bridge_arg,
            save_directory_arg,
            aurora_bringup,
            photo_service,
        ]
    )
