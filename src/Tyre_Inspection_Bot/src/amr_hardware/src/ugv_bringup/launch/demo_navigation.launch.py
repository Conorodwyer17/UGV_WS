# Modular thesis demo: Aurora + depth registered cloud + Nav2 (no YOLO, semantic off).
# Uses cmd_vel_mux + depth_gate by default; vehicle_speed_filter off (no vehicle boxes without fusion).
# Place a 2D goal in RViz or run teleop in another terminal.
#
#   ros2 launch ugv_bringup demo_navigation.launch.py
#   ros2 launch ugv_bringup demo_navigation.launch.py sim_no_move:=true
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
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    ugv_nav_share = get_package_share_directory("ugv_nav")
    seg_share = get_package_share_directory("segmentation_3d")
    ugv_bringup_share = get_package_share_directory("ugv_bringup")
    segment_3d = os.path.join(seg_share, "launch", "segment_3d.launch.py")
    nav_aurora = os.path.join(ugv_nav_share, "launch", "nav_aurora.launch.py")
    rviz_cfg = os.path.join(ugv_bringup_share, "config", "navigation.rviz")

    set_rmw = SetEnvironmentVariable(name="RMW_IMPLEMENTATION", value="rmw_cyclonedds_cpp")

    ip_address_arg = DeclareLaunchArgument("ip_address", default_value="192.168.11.1")
    use_bridge_arg = DeclareLaunchArgument("use_bridge", default_value="false")
    sim_no_move_arg = DeclareLaunchArgument(
        "sim_no_move",
        default_value="false",
        description="Use stub_motor (no motion)",
    )
    use_motor_driver_arg = DeclareLaunchArgument(
        "use_motor_driver",
        default_value="true",
        description="Real motor driver (false only with sim_no_move)",
    )
    uart_port_arg = DeclareLaunchArgument("uart_port", default_value="/dev/ttyTHS1")
    publish_wheel_odom_arg = DeclareLaunchArgument("publish_wheel_odom", default_value="true")
    costmap_resolution_arg = DeclareLaunchArgument("costmap_resolution", default_value="0.10")
    reset_map_arg = DeclareLaunchArgument(
        "reset_map_on_startup",
        default_value="true",
        description="Publish clear_map once after startup",
    )
    enable_cmd_vel_mux_arg = DeclareLaunchArgument("enable_cmd_vel_mux", default_value="true")
    enable_depth_gate_arg = DeclareLaunchArgument("enable_depth_gate", default_value="true")
    enable_vehicle_speed_filter_arg = DeclareLaunchArgument(
        "enable_vehicle_speed_filter",
        default_value="false",
        description="Off when Aurora semantic fusion is not running",
    )
    nav2_log_level_arg = DeclareLaunchArgument("nav2_log_level", default_value="info")

    motor_driver = Node(
        package="ugv_base_driver",
        executable="motor_driver_node",
        name="motor_driver",
        output="screen",
        parameters=[
            {
                "uart_port": LaunchConfiguration("uart_port"),
                "publish_wheel_odom": LaunchConfiguration("publish_wheel_odom"),
            }
        ],
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration("use_motor_driver"),
                    "' == 'true' and '",
                    LaunchConfiguration("sim_no_move"),
                    "' == 'false'",
                ]
            )
        ),
    )
    stub_motor = Node(
        package="ugv_base_driver",
        executable="stub_motor_node",
        name="stub_motor",
        output="screen",
        condition=IfCondition(LaunchConfiguration("sim_no_move")),
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

    clear_map = ExecuteProcess(
        cmd=[
            "ros2",
            "topic",
            "pub",
            "/slamware_ros_sdk_server_node/clear_map",
            "slamware_ros_sdk/msg/ClearMapRequest",
            "{}",
            "--once",
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("reset_map_on_startup")),
    )
    clear_map_delayed = TimerAction(period=6.0, actions=[clear_map])

    # Depth-only segment_3d: no YOLO, no tyre projection (minimal_perception skips heavy nodes).
    segment_3d_delayed = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(segment_3d),
                launch_arguments={
                    "use_bridge": LaunchConfiguration("use_bridge"),
                    "minimal_perception": "true",
                    "use_yolo": "false",
                    "enable_tyre_3d_projection": "false",
                    "pcl_fallback_enabled": "false",
                    "centroid_servo_enabled": "false",
                    "depth_registered_publish_hz": "2.0",
                }.items(),
            )
        ],
    )

    nav_aurora_delayed = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav_aurora),
                launch_arguments={
                    "costmap_resolution": LaunchConfiguration("costmap_resolution"),
                    "enable_cmd_vel_mux": LaunchConfiguration("enable_cmd_vel_mux"),
                    "enable_depth_gate": LaunchConfiguration("enable_depth_gate"),
                    "enable_vehicle_speed_filter": LaunchConfiguration("enable_vehicle_speed_filter"),
                    "nav2_log_level": LaunchConfiguration("nav2_log_level"),
                }.items(),
            )
        ],
    )

    rviz = TimerAction(
        period=14.0,
        actions=[ExecuteProcess(cmd=["rviz2", "-d", rviz_cfg], output="screen")],
    )

    return LaunchDescription(
        [
            set_rmw,
            ip_address_arg,
            use_bridge_arg,
            sim_no_move_arg,
            use_motor_driver_arg,
            uart_port_arg,
            publish_wheel_odom_arg,
            costmap_resolution_arg,
            reset_map_arg,
            enable_cmd_vel_mux_arg,
            enable_depth_gate_arg,
            enable_vehicle_speed_filter_arg,
            nav2_log_level_arg,
            motor_driver,
            stub_motor,
            aurora_bringup,
            clear_map_delayed,
            segment_3d_delayed,
            nav_aurora_delayed,
            rviz,
        ]
    )
