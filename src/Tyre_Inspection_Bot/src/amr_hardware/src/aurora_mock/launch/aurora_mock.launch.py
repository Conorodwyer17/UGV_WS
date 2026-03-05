#!/usr/bin/env python3
"""
Launch Aurora mock node for simulation without hardware.
Publishes synthetic odom, scan, map, images to /slamware_ros_sdk_server_node/*.
Use with vehicle_inspection_sim.launch.py use_mock:=true.
When use_sim_time:=true, launches clock_publisher first (use_sim_time=false) then aurora_mock.
"""
import os
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_prefix = get_package_prefix("aurora_mock")
    aurora_mock_script = os.path.join(pkg_prefix, "bin", "aurora_mock_node")
    depth_generator_script = os.path.join(pkg_prefix, "bin", "depth_generator_node")
    synthetic_vehicle_script = os.path.join(pkg_prefix, "bin", "synthetic_vehicle_publisher")

    return LaunchDescription([
        DeclareLaunchArgument(
            "publish_synthetic_vehicle",
            default_value="true",
            description="When true, publish static vehicle box for full mission simulation",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation time (true when mock provides /clock)",
        ),
        DeclareLaunchArgument(
            "synthetic_vehicle_depth",
            default_value="false",
            description="When true, fill depth image center with valid depth for segment_3d + simulated_detection.",
        ),
        DeclareLaunchArgument(
            "vehicle_count",
            default_value="1",
            description="Number of synthetic vehicles to publish (for multi-vehicle testing).",
        ),
        DeclareLaunchArgument(
            "vehicle_spacing_m",
            default_value="2.0",
            description="Spacing between vehicles along x-axis when vehicle_count > 1.",
        ),
        DeclareLaunchArgument(
            "use_stress_test",
            default_value="false",
            description="When true, enable noise, latency, jitter, dropout for robustness testing.",
        ),
        # Clock publisher: source of /clock; must run with use_sim_time=false
        ExecuteProcess(
            cmd=[os.path.join(pkg_prefix, "bin", "clock_publisher")],
            name="clock_publisher",
            output="screen",
            additional_env={"ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID", "")},
            condition=IfCondition(LaunchConfiguration("use_sim_time")),
        ),
        # Aurora mock: use ExecuteProcess with conditional use_sim_time and synthetic_vehicle_depth
        ExecuteProcess(
            cmd=[
                aurora_mock_script, "--ros-args",
                "-p", PythonExpression(["'use_sim_time:=' + '", LaunchConfiguration("use_sim_time"), "'"]),
                "-p", PythonExpression(["'synthetic_vehicle_depth:=' + '", LaunchConfiguration("synthetic_vehicle_depth", default="false"), "'"]),
                "-p", PythonExpression(["'enable_odom_noise:=' + ('true' if '", LaunchConfiguration("use_stress_test", default="false"), "' == 'true' else 'false')"]),
                "-p", PythonExpression(["'enable_scan_noise:=' + ('true' if '", LaunchConfiguration("use_stress_test", default="false"), "' == 'true' else 'false')"]),
                "-p", PythonExpression(["'enable_depth_noise:=' + ('true' if '", LaunchConfiguration("use_stress_test", default="false"), "' == 'true' else 'false')"]),
                "-p", PythonExpression(["'message_dropout_prob:=' + ('0.01' if '", LaunchConfiguration("use_stress_test", default="false"), "' == 'true' else '0.0')"]),
                "-p", PythonExpression(["'enable_timing_realism:=' + ('true' if '", LaunchConfiguration("use_stress_test", default="false"), "' == 'true' else 'false')"]),
                "-p", PythonExpression(["'odom_latency_ms:=' + ('15.0' if '", LaunchConfiguration("use_stress_test", default="false"), "' == 'true' else '0.0')"]),
                "-p", PythonExpression(["'scan_latency_ms:=' + ('10.0' if '", LaunchConfiguration("use_stress_test", default="false"), "' == 'true' else '0.0')"]),
                "-p", PythonExpression(["'depth_latency_ms:=' + ('15.0' if '", LaunchConfiguration("use_stress_test", default="false"), "' == 'true' else '0.0')"]),
                "-p", PythonExpression(["'image_latency_ms:=' + ('15.0' if '", LaunchConfiguration("use_stress_test", default="false"), "' == 'true' else '0.0')"]),
            ],
            name="aurora_mock_node",
            output="screen",
            additional_env={"ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID", "")},
            condition=IfCondition(LaunchConfiguration("use_sim_time")),
        ),
        ExecuteProcess(
            cmd=[
                aurora_mock_script, "--ros-args",
                "-p", PythonExpression(["'enable_odom_noise:=' + ('true' if '", LaunchConfiguration("use_stress_test", default="false"), "' == 'true' else 'false')"]),
                "-p", PythonExpression(["'enable_scan_noise:=' + ('true' if '", LaunchConfiguration("use_stress_test", default="false"), "' == 'true' else 'false')"]),
                "-p", PythonExpression(["'enable_depth_noise:=' + ('true' if '", LaunchConfiguration("use_stress_test", default="false"), "' == 'true' else 'false')"]),
                "-p", PythonExpression(["'message_dropout_prob:=' + ('0.01' if '", LaunchConfiguration("use_stress_test", default="false"), "' == 'true' else '0.0')"]),
                "-p", PythonExpression(["'enable_timing_realism:=' + ('true' if '", LaunchConfiguration("use_stress_test", default="false"), "' == 'true' else 'false')"]),
                "-p", PythonExpression(["'odom_latency_ms:=' + ('15.0' if '", LaunchConfiguration("use_stress_test", default="false"), "' == 'true' else '0.0')"]),
                "-p", PythonExpression(["'scan_latency_ms:=' + ('10.0' if '", LaunchConfiguration("use_stress_test", default="false"), "' == 'true' else '0.0')"]),
                "-p", PythonExpression(["'depth_latency_ms:=' + ('15.0' if '", LaunchConfiguration("use_stress_test", default="false"), "' == 'true' else '0.0')"]),
                "-p", PythonExpression(["'image_latency_ms:=' + ('15.0' if '", LaunchConfiguration("use_stress_test", default="false"), "' == 'true' else '0.0')"]),
            ],
            name="aurora_mock_node",
            output="screen",
            additional_env={"ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID", "")},
            condition=UnlessCondition(LaunchConfiguration("use_sim_time")),
        ),
        # Depth generator: heavy depth/point_cloud in separate process when synthetic_vehicle_depth=true
        ExecuteProcess(
            cmd=[
                depth_generator_script, "--ros-args",
                "-p", PythonExpression(["'use_sim_time:=' + '", LaunchConfiguration("use_sim_time"), "'"]),
            ],
            name="depth_generator_node",
            output="screen",
            additional_env={"ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID", "")},
            condition=IfCondition(LaunchConfiguration("synthetic_vehicle_depth", default="false")),
        ),
        # TF: map -> slamware_map (identity); Nav2 expects map frame
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_slamware_map",
            arguments=["--x", "0", "--y", "0", "--z", "0",
                       "--yaw", "0", "--pitch", "0", "--roll", "0",
                       "--frame-id", "map", "--child-frame-id", "slamware_map"],
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ),
        # TF: slamware_map -> odom (identity; mock publishes odom->base_link)
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="slamware_to_odom",
            arguments=["--x", "0", "--y", "0", "--z", "0",
                       "--yaw", "0", "--pitch", "0", "--roll", "0",
                       "--frame-id", "slamware_map", "--child-frame-id", "odom"],
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ),
        # TF: base_link -> camera_left (Aurora camera pose)
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_camera_left",
            arguments=["--x", "0.0418", "--y", "0.03", "--z", "0",
                       "--yaw", "0", "--pitch", "0", "--roll", "0",
                       "--frame-id", "base_link", "--child-frame-id", "camera_left"],
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ),
        # TF: camera_left -> camera_depth_optical_frame
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="camera_left_to_depth_optical",
            arguments=["--x", "0", "--y", "0", "--z", "0",
                       "--yaw", "0", "--pitch", "0", "--roll", "0",
                       "--frame-id", "camera_left", "--child-frame-id", "camera_depth_optical_frame"],
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ),
        # TF: base_link -> laser (scan frame for costmap)
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_laser",
            arguments=["--x", "0", "--y", "0", "--z", "0",
                       "--yaw", "0", "--pitch", "0", "--roll", "0",
                       "--frame-id", "base_link", "--child-frame-id", "laser"],
            parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        ),
        # Synthetic vehicle box(es) for full mission simulation (inspection_manager + tire_detector_pcl)
        ExecuteProcess(
            cmd=[
                synthetic_vehicle_script,
                "--ros-args",
                "-p", "output_topic:=/aurora_semantic/vehicle_bounding_boxes",
                "-p", "vehicle_x_m:=2.0",
                "-p", "vehicle_y_m:=0.0",
                "-p", "vehicle_z_m:=0.3",
                "-p", "vehicle_extent_m:=1.5",
                "-p", "publish_rate_hz:=5.0",
                "-p", PythonExpression(["'vehicle_count:=' + '", LaunchConfiguration("vehicle_count", default="1"), "'"]),
                "-p", PythonExpression(["'vehicle_spacing_m:=' + '", LaunchConfiguration("vehicle_spacing_m", default="2.0"), "'"]),
                "-p", PythonExpression(["'use_sim_time:=' + '", LaunchConfiguration("use_sim_time"), "'"]),
            ],
            name="synthetic_vehicle_publisher",
            output="screen",
            condition=IfCondition(LaunchConfiguration("publish_synthetic_vehicle", default="true")),
        ),
    ])
