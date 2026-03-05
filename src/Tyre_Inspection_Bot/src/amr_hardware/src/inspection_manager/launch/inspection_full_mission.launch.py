# Single entrypoint for full autonomous tyre inspection mission.
# With launch_aurora_nav:=false (default): segment_3d + inspection_manager only.
#   Prerequisites: Aurora running (map, odom, scan, images), Nav2 running (navigate_to_pose).
# With launch_aurora_nav:=true: Aurora + Nav2 + segment_3d + inspection_manager (full stack).
# Usage: ros2 launch inspection_manager inspection_full_mission.launch.py
#   [launch_aurora_nav:=true]  # optional: bring Aurora + Nav2
# Mission auto-starts when TF and Nav2 are ready (start_mission_on_ready: true). Or publish to /inspection_manager/start_mission (std_msgs/Bool) when start_mission_on_ready: false.

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    seg3d_share = get_package_share_directory("segmentation_3d")
    inspection_share = get_package_share_directory("inspection_manager")
    ugv_nav_share = get_package_share_directory("ugv_nav")

    set_rmw = SetEnvironmentVariable(name="RMW_IMPLEMENTATION", value="rmw_cyclonedds_cpp")

    launch_aurora_nav_arg = DeclareLaunchArgument(
        "launch_aurora_nav",
        default_value="false",
        description="If true, launch Aurora + Nav2. If false, expect Aurora and Nav2 running separately.",
    )

    launch_segment_3d_arg = DeclareLaunchArgument(
        "launch_segment_3d",
        default_value="true",
        description="Launch detection stack (segment_3d). Set false if already running.",
    )

    perception_only_arg = DeclareLaunchArgument(
        "perception_only_mode",
        default_value="false",
        description="Disable all movement for perception validation.",
    )

    pcl_fallback_arg = DeclareLaunchArgument(
        "pcl_fallback_enabled",
        default_value="true",
        description="If true, use tire_merger (YOLO + PCL fallback); tire_detection_topic=/tire_bounding_boxes_merged.",
    )

    # Tire detection topic: merged when PCL fallback enabled, else YOLO directly
    tire_detection_topic = PythonExpression([
        "'/tire_bounding_boxes_merged' if '", LaunchConfiguration("pcl_fallback_enabled"),
        "' == 'true' else '/darknet_ros_3d/tire_bounding_boxes'"
    ])

    # Shared inspection launch
    inspection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(inspection_share, "launch", "inspection_manager.launch.py")
        ),
        launch_arguments={
            "vehicle_detection_topic": LaunchConfiguration("vehicle_detection_topic", default="/darknet_ros_3d/vehicle_bounding_boxes"),
            "tire_detection_topic": tire_detection_topic,
            "vehicle_fallback_topic": LaunchConfiguration("vehicle_fallback_topic", default="/aurora_semantic/vehicle_bounding_boxes"),
            "perception_only_mode": LaunchConfiguration("perception_only_mode"),
        }.items(),
    )

    # Standalone path (launch_aurora_nav:=false): segment_3d at 5s, inspection at 8s
    segment_3d_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(seg3d_share, "launch", "segment_3d.launch.py")),
        launch_arguments={
            "pcl_fallback_enabled": LaunchConfiguration("pcl_fallback_enabled"),
            "centroid_servo_enabled": LaunchConfiguration("centroid_servo_enabled", default="true"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("launch_segment_3d")),
    )
    standalone_group = GroupAction(
        condition=UnlessCondition(LaunchConfiguration("launch_aurora_nav")),
        actions=[
            TimerAction(period=5.0, actions=[segment_3d_launch]),
            TimerAction(period=8.0, actions=[inspection_launch]),
        ],
    )

    # Full stack path (launch_aurora_nav:=true): aurora, segment_3d at 8s, nav at 10s, inspection at 120s
    aurora_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ugv_nav_share, "launch", "aurora_bringup.launch.py")
        ),
        launch_arguments={"ip_address": LaunchConfiguration("ip_address", default="192.168.11.1")}.items(),
    )
    nav_aurora_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ugv_nav_share, "launch", "nav_aurora.launch.py"))
    )
    segment_3d_launch_full = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(seg3d_share, "launch", "segment_3d.launch.py")),
        launch_arguments={
            "pcl_fallback_enabled": LaunchConfiguration("pcl_fallback_enabled"),
            "centroid_servo_enabled": LaunchConfiguration("centroid_servo_enabled", default="true"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("launch_segment_3d")),
    )
    full_stack_group = GroupAction(
        condition=IfCondition(LaunchConfiguration("launch_aurora_nav")),
        actions=[
            aurora_bringup,
            TimerAction(period=8.0, actions=[segment_3d_launch_full]),
            TimerAction(period=10.0, actions=[nav_aurora_launch]),
            TimerAction(period=120.0, actions=[inspection_launch]),
        ],
    )

    return LaunchDescription([
        set_rmw,
        launch_aurora_nav_arg,
        DeclareLaunchArgument("ip_address", default_value="192.168.11.1", description="Aurora device IP (when launch_aurora_nav:=true)"),
        perception_only_arg,
        DeclareLaunchArgument(
            "vehicle_detection_topic",
            default_value="/darknet_ros_3d/vehicle_bounding_boxes",
        ),
        pcl_fallback_arg,
        DeclareLaunchArgument(
            "centroid_servo_enabled",
            default_value="true",
            description="If true, run centroid_servo_node for image-based fine positioning.",
        ),
        DeclareLaunchArgument(
            "vehicle_fallback_topic",
            default_value="/aurora_semantic/vehicle_bounding_boxes",
        ),
        launch_segment_3d_arg,
        standalone_group,
        full_stack_group,
    ])
