# Aurora testing launch — NO robot motion.
# Brings up Aurora SDK + TF + Aurora SDK Bridge (depth pipeline).
# Does NOT launch Nav2, motor driver, or any cmd_vel consumer.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    ip_address_arg = DeclareLaunchArgument(
        "ip_address",
        default_value="192.168.11.1",
        description="IP address of the SLAMTEC Aurora device",
    )
    use_bridge_arg = DeclareLaunchArgument(
        "use_bridge",
        default_value="true",
        description="Enable aurora_sdk_bridge (depth pipeline)",
    )

    # === Aurora SDK (raw_image_on=true required for bridge) ===
    slamware_node = Node(
        package="slamware_ros_sdk",
        executable="slamware_ros_sdk_server_node",
        name="slamware_ros_sdk_server_node",
        output="both",
        parameters=[{
            "ip_address": LaunchConfiguration("ip_address"),
            "stereo_camera_info_enable": False,  # Classic Aurora: use bridge
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
            ("scan", "scan"),
            ("odom", "odom"),
            ("map", "slamware_map"),
            ("map_metadata", "map_metadata"),
        ],
    )

    # === TF: Aurora frame tree (new-style arguments) ===
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

    # Aurora SDK Bridge (depth, point cloud, navigation_permitted)
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

    return LaunchDescription([
        ip_address_arg,
        use_bridge_arg,
        slamware_node,
        odom2map,
        leftcam2base,
        rightcam2leftcam,
        imu2leftcam,
        depth_optical2left,
        bridge_launch,
    ])
