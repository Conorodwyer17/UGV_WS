# Copyright 2024-2026 UGV Tire Inspection
# Launch SLAMTEC Aurora SDK node (slamware_ros_sdk) + aurora_sdk_bridge (depth pipeline).
# Uses robot_state_publisher + URDF for sensor transforms (fixes TF timestamp extrapolation errors).

import os
import sys

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ip_address_arg = DeclareLaunchArgument(
        "ip_address",
        default_value="192.168.11.1",
        description="IP address of the SLAMTEC Aurora device",
    )
    use_bridge_arg = DeclareLaunchArgument(
        "use_bridge",
        default_value="false",
        description=(
            "Legacy: enable aurora_sdk_bridge. Default false = Aurora 2.11 "
            "native depth (firmware 2.11+)."
        ),
    )
    stereo_camera_info_arg = DeclareLaunchArgument(
        "stereo_camera_info_enable",
        default_value="false",
        description=(
            "Enable Aurora factory camera_info (firmware 2.11+ may support). "
            "Try true after firmware upgrade."
        ),
    )
    enable_semantic_segmentation_arg = DeclareLaunchArgument(
        "enable_semantic_segmentation",
        default_value="true",
        description=(
            "Subscribe to Aurora semantic segmentation (uses device/GPU). "
            "Set false on Jetson OOM to free memory (vehicle boxes must use YOLO or other source)."
        ),
    )

    slamware_node = Node(
        package='slamware_ros_sdk',
        executable='slamware_ros_sdk_server_node',
        name='slamware_ros_sdk_server_node',
        output='both',
        parameters=[{
            "ip_address": LaunchConfiguration("ip_address"),
            "stereo_camera_info_enable": LaunchConfiguration(
                "stereo_camera_info_enable"
            ),
            "angle_compensate": True,
            "map_frame": "slamware_map",
            "robot_frame": "base_link",
            "odom_frame": "odom",
            "laser_frame": "laser",
            "imu_frame": "imu_link",
            "camera_left": "camera_left",
            "camera_right": "camera_right",
            "robot_pose_pub_period": 0.05,
            "scan_pub_period": 0.1,
            "map_pub_period": 0.2,
            "imu_raw_data_period": 0.005,
            "ladar_data_clockwise": True,
            "no_preview_image": False,
            "raw_image_on": True,
        }],
        remappings=[
            ('scan', 'scan'),
            ('odom', 'odom'),
            ('map', 'slamware_map'),
            ('map_metadata', 'map_metadata'),
        ],
    )

    # TF: map->slamware_map, slamware_map->odom with CURRENT timestamps (not stamp 0).
    # static_transform_publisher uses stamp 0, causing ~70s delay and costmap rejection.
    world_frame_tf_script = os.path.join(
        get_package_prefix("ugv_nav"),
        "lib",
        "ugv_nav",
        "world_frame_tf_publisher.py",
    )
    world_frame_tf = ExecuteProcess(
        cmd=[sys.executable, world_frame_tf_script],
        name="world_frame_tf_publisher",
        output="screen",
    )
    # Robot state publisher: URDF-based transforms for camera_left, camera_right, imu_link,
    # camera_depth_optical_frame, base_footprint. Ensures correct timestamps (fixes TF extrapolation).
    ugv_desc_share = get_package_share_directory("ugv_description")
    robot_description_path = os.path.join(ugv_desc_share, "urdf", "ugv_rover.urdf")
    with open(robot_description_path, "r") as f:
        robot_description = f.read()
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": False,
        }],
    )

    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("aurora_sdk_bridge"),
                    "launch",
                    "aurora_sdk_bridge.launch.py",
                )
            ]
        ),
        condition=IfCondition(LaunchConfiguration("use_bridge")),
    )

    # When use_bridge=false, aurora_sdk_bridge is not launched, so /stereo/navigation_permitted
    # is never published. depth_gate would block all cmd_vel. Publish True here so robot can move.
    nav_permitted_script = os.path.join(
        get_package_prefix("ugv_nav"),
        "lib",
        "ugv_nav",
        "navigation_permitted_publisher.py",
    )
    nav_permitted_pub = ExecuteProcess(
        cmd=[sys.executable, nav_permitted_script],
        name="navigation_permitted_publisher",
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("use_bridge")),
    )

    # Use CycloneDDS so diagnostics (and mission) in another terminal see the same topics/TF.
    set_rmw = SetEnvironmentVariable(name="RMW_IMPLEMENTATION", value="rmw_cyclonedds_cpp")

    return LaunchDescription([
        set_rmw,
        ip_address_arg,
        use_bridge_arg,
        stereo_camera_info_arg,
        enable_semantic_segmentation_arg,
        slamware_node,
        world_frame_tf,
        robot_state_publisher,
        bridge_launch,
        nav_permitted_pub,
    ])
