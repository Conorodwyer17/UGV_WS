#!/usr/bin/env python3
"""
Vehicle inspection simulation launch.
Supports rosbag replay (use_bag:=true) or Aurora mock (use_mock:=true) for offline testing.
- use_bag:=true: plays recorded bag with --clock, launches Nav2 + segment_3d + inspection_manager with use_sim_time.
- use_mock:=true: launches aurora_mock (synthetic odom, scan, map, images) + Nav2 + segment_3d + inspection_manager.
- use_bag and use_mock false: placeholder; start Aurora + Nav2 separately.
"""
import os
import sys

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory, get_package_prefix


def generate_launch_description():
    use_bag = LaunchConfiguration("use_bag", default="false")
    use_mock = LaunchConfiguration("use_mock", default="false")
    sim_share = get_package_share_directory("sim")
    default_bag = os.path.join(sim_share, "bags", "aurora_sample")

    # Sim mode: either bag replay or mock
    use_sim = PythonExpression(["'", use_bag, "' == 'true' or '", use_mock, "' == 'true'"])
    use_sim_time = PythonExpression(["'", use_bag, "' == 'true' or '", use_mock, "' == 'true'"])  # Both bag and mock use /clock

    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument("use_bag", default_value="false"))
    ld.add_action(DeclareLaunchArgument("use_mock", default_value="false"))
    ld.add_action(
        DeclareLaunchArgument(
            "bag_path",
            default_value=default_bag,
            description="Path to rosbag directory (when use_bag:=true)",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_tight_goal_tolerance",
            default_value="false",
            description="If true, use xy_goal_tolerance 0.1 and max_vel_x 0.18 (Phase 3 config) for Nav2.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_ekf",
            default_value="false",
            description="If true, use /odometry/filtered (EKF fusion). Requires ekf_aurora + wheel odom; not used in mock.",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "vehicle_count",
            default_value="1",
            description="Number of synthetic vehicles when use_mock:=true (for multi-vehicle testing).",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "use_stress_test",
            default_value="false",
            description="When true with use_mock, enable noise, latency, jitter, and dropout for robustness testing.",
        )
    )

    # When use_bag: play bag with --clock for use_sim_time
    bag_play = ExecuteProcess(
        cmd=["ros2", "bag", "play", LaunchConfiguration("bag_path"), "--clock"],
        name="bag_play",
        output="screen",
        condition=IfCondition(use_bag),
    )
    ld.add_action(bag_play)

    # When use_mock: clock publisher + Aurora mock + navigation_permitted (for depth_gate)
    aurora_mock_share = get_package_share_directory("aurora_mock")
    aurora_mock_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(aurora_mock_share, "launch", "aurora_mock.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "synthetic_vehicle_depth": "true",  # Valid depth for simulated_detection + segment_3d
            "vehicle_count": LaunchConfiguration("vehicle_count", default="1"),
            "use_stress_test": LaunchConfiguration("use_stress_test", default="false"),
        }.items(),
        condition=IfCondition(use_mock),
    )
    ld.add_action(aurora_mock_launch)

    nav_permitted_script = os.path.join(
        get_package_prefix("ugv_nav"), "lib", "ugv_nav", "navigation_permitted_publisher.py"
    )
    nav_permitted = ExecuteProcess(
        cmd=[sys.executable, nav_permitted_script, "--ros-args", "-p", "use_sim_time:=true"],
        name="navigation_permitted_publisher",
        output="screen",
        condition=IfCondition(use_mock),
    )
    ld.add_action(nav_permitted)

    ugv_nav_share = get_package_share_directory("ugv_nav")
    seg3d_share = get_package_share_directory("segmentation_3d")
    inspection_share = get_package_share_directory("inspection_manager")

    # Nav2 (odom, scan, map, TF from bag or mock)
    nav_aurora = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ugv_nav_share, "launch", "nav_aurora.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "use_tight_goal_tolerance": LaunchConfiguration("use_tight_goal_tolerance", default="false"),
            "use_ekf": LaunchConfiguration("use_ekf", default="false"),
            "external_tf": use_mock,  # aurora_mock provides map->slamware_map, base_footprint; skip duplicates
            "nav2_log_level": PythonExpression(["'debug' if '", use_mock, "' == 'true' else 'info'"]),
        }.items(),
        condition=IfCondition(use_sim),
    )
    # When use_mock: 5s delay so mock has time to publish TF, map, odom before Nav2; else 5s.
    nav_delayed_mock = TimerAction(period=5.0, actions=[nav_aurora], condition=IfCondition(use_mock))
    nav_delayed_other = TimerAction(period=5.0, actions=[nav_aurora], condition=UnlessCondition(use_mock))
    ld.add_action(nav_delayed_mock)
    ld.add_action(nav_delayed_other)

    # segment_3d (needs left_image_raw, depth, semantic from bag or mock)
    # use_synthetic_vehicle:=true when use_mock: skip aurora_semantic_fusion, use synthetic vehicle from aurora_mock
    segment_3d = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(seg3d_share, "launch", "segment_3d.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "pcl_fallback_enabled": "true",
            "centroid_servo_enabled": "true",
            "use_synthetic_vehicle": use_mock,
            "use_yolo": PythonExpression(["'false' if '", use_mock, "' == 'true' else 'true'"]),
            "use_simulated_detection": use_mock,  # ObjectsSegment from ground truth when mock
            "miss_probability": PythonExpression(["'0.1' if '", LaunchConfiguration("use_stress_test", default="false"), "' == 'true' else '0.0'"]),
        }.items(),
        condition=IfCondition(use_sim),
    )
    seg_delayed = TimerAction(period=8.0, actions=[segment_3d])
    ld.add_action(seg_delayed)

    # inspection_manager
    inspection = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(inspection_share, "launch", "inspection_manager.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "vehicle_detection_topic": "/darknet_ros_3d/vehicle_bounding_boxes",
            "tire_detection_topic": "/tire_bounding_boxes_merged",
            "vehicle_fallback_topic": "/aurora_semantic/vehicle_bounding_boxes",
            "sensor_health_timeout": PythonExpression(["'5.0' if '", use_mock, "' == 'true' else '30.0'"]),
        }.items(),
        condition=IfCondition(use_sim),
    )
    inspection_delayed = TimerAction(period=50.0, actions=[inspection])
    ld.add_action(inspection_delayed)

    ld.add_action(
        LogInfo(
            msg=[
                "Sim: use_bag=", use_bag, ", use_mock=", use_mock,
                ". use_bag: replay bag. use_mock: synthetic Aurora. Record: ./sim/record_aurora_mission.sh",
            ]
        )
    )

    return ld
