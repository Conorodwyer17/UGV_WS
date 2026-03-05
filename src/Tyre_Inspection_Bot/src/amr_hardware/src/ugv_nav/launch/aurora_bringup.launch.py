# Copyright 2024 UGV Tire Inspection
# Launch SLAMTEC Aurora SDK node (slamware_ros_sdk) + aurora_sdk_bridge (depth pipeline).

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

    # TF: map -> slamware_map (identity). Ensures "map" frame exists when Nav2 is not running,
    # so inspection_manager can transform detections to map.
    map2slamware = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_slamware_map",
        arguments=[
            "--x", "0",
            "--y", "0",
            "--z", "0",
            "--qx", "0",
            "--qy", "0",
            "--qz", "0",
            "--qw", "1",
            "--frame-id", "map",
            "--child-frame-id", "slamware_map",
        ],
    )
    # TF: slamware_map -> odom (Aurora provides odom relative to slamware_map).
    odom2map = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="odom2map",
        arguments=[
            "--x", "0",
            "--y", "0",
            "--z", "0",
            "--qx", "0",
            "--qy", "0",
            "--qz", "0",
            "--qw", "1",
            "--frame-id", "slamware_map",
            "--child-frame-id", "odom",
        ],
    )
    # Camera and IMU frames (adjust if your URDF differs)
    leftcam2base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="leftcam2base",
        arguments=[
            "--x", "0.0418",
            "--y", "0.03",
            "--z", "0",
            "--qx", "-0.5",
            "--qy", "0.5",
            "--qz", "-0.5",
            "--qw", "0.5",
            "--frame-id", "base_link",
            "--child-frame-id", "camera_left",
        ],
    )
    rightcam2leftcam = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="rightcam2Leftcam",
        arguments=[
            "--x", "0.06",
            "--y", "0",
            "--z", "0",
            "--qx", "0",
            "--qy", "0",
            "--qz", "0",
            "--qw", "1",
            "--frame-id", "camera_left",
            "--child-frame-id", "camera_right",
        ],
    )
    imu2leftcam = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="imu2Leftcam",
        arguments=[
            "--x", "0.03",
            "--y", "0",
            "--z", "0",
            "--qx", "0",
            "--qy", "0",
            "--qz", "-0.7071068",
            "--qw", "0.7071068",
            "--frame-id", "camera_left",
            "--child-frame-id", "imu_link",
        ],
    )
    # Aurora depth image uses camera_depth_optical_frame; align with camera_left
    depth_optical2left = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="depth_optical2left",
        arguments=[
            "--x", "0",
            "--y", "0",
            "--z", "0",
            "--qx", "0",
            "--qy", "0",
            "--qz", "0",
            "--qw", "1",
            "--frame-id", "camera_left",
            "--child-frame-id", "camera_depth_optical_frame",
        ],
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
        slamware_node,
        map2slamware,
        odom2map,
        leftcam2base,
        rightcam2leftcam,
        imu2leftcam,
        depth_optical2left,
        bridge_launch,
        nav_permitted_pub,
    ])
