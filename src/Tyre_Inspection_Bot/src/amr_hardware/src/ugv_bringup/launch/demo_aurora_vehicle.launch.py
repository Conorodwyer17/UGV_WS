# Modular thesis demo: Aurora semantic vehicle boxes + RViz (no Nav2, no tyre YOLO).
# Requires Aurora firmware with semantic_labels; shows /aurora_semantic/vehicle_markers.
#
#   ros2 launch ugv_bringup demo_aurora_vehicle.launch.py
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
from launch_ros.actions import Node
from launch.conditions import UnlessCondition


def generate_launch_description():
    ugv_nav_share = get_package_share_directory("ugv_nav")
    seg_share = get_package_share_directory("segmentation_3d")
    ugv_bringup_share = get_package_share_directory("ugv_bringup")
    intrinsics = os.path.join(seg_share, "config", "aurora_depth_intrinsics.yaml")
    vehicle_viz = os.path.join(seg_share, "launch", "vehicle_detection_visualize.launch.py")
    rviz_cfg = os.path.join(ugv_bringup_share, "config", "aurora_vehicle.rviz")

    set_rmw = SetEnvironmentVariable(name="RMW_IMPLEMENTATION", value="rmw_cyclonedds_cpp")

    ip_address_arg = DeclareLaunchArgument(
        "ip_address",
        default_value="192.168.11.1",
        description="SLAMTEC Aurora IP",
    )
    use_bridge_arg = DeclareLaunchArgument(
        "use_bridge",
        default_value="false",
        description="Legacy aurora_sdk_bridge (false = native depth)",
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

    aurora_depth_camera_info_node = Node(
        package="segmentation_3d",
        executable="aurora_depth_camera_info_node",
        name="aurora_depth_camera_info",
        output="screen",
        parameters=[
            {
                "intrinsics_file": intrinsics,
                "camera_info_topic": "/camera/depth/camera_info",
                "frame_id": "camera_depth_optical_frame",
            }
        ],
        condition=UnlessCondition(LaunchConfiguration("use_bridge")),
    )
    depth_to_registered = Node(
        package="segmentation_3d",
        executable="depth_to_registered_pointcloud_node",
        name="depth_to_registered_pointcloud",
        output="screen",
        parameters=[
            {
                "depth_topic": "/slamware_ros_sdk_server_node/depth_image_raw",
                "camera_info_topic": "/camera/depth/camera_info",
                "output_topic": "/segmentation_processor/registered_pointcloud",
                "output_frame_id": "camera_depth_optical_frame",
                "depth_points_topic": "/camera/depth/points",
                "publish_rate_hz": 0.0,
            }
        ],
        condition=UnlessCondition(LaunchConfiguration("use_bridge")),
    )
    depth_delayed = TimerAction(period=3.0, actions=[aurora_depth_camera_info_node, depth_to_registered])

    vehicle_viz_delayed = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(vehicle_viz),
                launch_arguments={}.items(),
            )
        ],
    )

    rviz = TimerAction(
        period=7.0,
        actions=[ExecuteProcess(cmd=["rviz2", "-d", rviz_cfg], output="screen")],
    )

    return LaunchDescription(
        [
            set_rmw,
            ip_address_arg,
            use_bridge_arg,
            aurora_bringup,
            depth_delayed,
            vehicle_viz_delayed,
            rviz,
        ]
    )
