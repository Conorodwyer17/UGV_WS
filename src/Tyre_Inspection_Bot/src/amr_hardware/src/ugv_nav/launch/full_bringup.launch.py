# Full stack bringup: Aurora, Nav2, perception (segment_3d), inspection manager, motor driver.
# Starts everything in order with delays so dependencies are up before clients.
# Inspection manager is delayed until after Nav2 lifecycle (120s in nav_aurora) so the
# NavigateToPose action server is available when the mission tries to rotate/drive.
# Usage: ros2 launch ugv_nav full_bringup.launch.py
#   [ip_address:=192.168.11.1] [config_file:=...]
#   [use_motor_driver:=true] [uart_port:=/dev/ttyTHS1]

import os
from ament_index_python.packages import get_package_prefix, get_package_share_directory
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
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    ugv_nav_share = get_package_share_directory("ugv_nav")
    seg3d_share = get_package_share_directory("segmentation_3d")
    inspection_share = get_package_share_directory("inspection_manager")

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
    config_file_arg = DeclareLaunchArgument(
        "config_file",
        default_value="",
        description=(
            "Path to PRODUCTION_CONFIG.yaml. If empty, uses workspace "
            "PRODUCTION_CONFIG.yaml when present."
        ),
    )
    dry_run_arg = DeclareLaunchArgument(
        "dry_run",
        default_value="false",
        description="Inspection manager dry_run: log goals, no Nav2 send.",
    )
    use_motor_driver_arg = DeclareLaunchArgument(
        "use_motor_driver",
        default_value="true",
        description="Launch motor driver (ugv_base_driver) to send cmd_vel to base",
    )
    uart_port_arg = DeclareLaunchArgument(
        "uart_port",
        default_value="/dev/ttyTHS1",
        description="UART port for motor driver (e.g. /dev/ttyTHS1 on Jetson)",
    )

    # 0) Motor driver — subscribes to cmd_vel, forwards to ESP32
    # (use_motor_driver:=false if no hardware)
    motor_driver_script = os.path.join(
        get_package_prefix("ugv_base_driver"), "lib", "ugv_base_driver", "motor_driver_node"
    )
    motor_driver = ExecuteProcess(
        cmd=[motor_driver_script],
        name="motor_driver",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_motor_driver")),
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

    # 2) Perception (YOLO + 3D boxes + depth pipeline) — after Aurora topics exist
    segment_3d_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(seg3d_share, "launch", "segment_3d.launch.py")
        ),
        launch_arguments={"use_bridge": LaunchConfiguration("use_bridge")}.items(),
    )
    segment_3d_delayed = TimerAction(period=8.0, actions=[segment_3d_launch])

    # 3) Nav2 + depth_gate + lifecycle — after Aurora map/odom/scan
    nav_aurora_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ugv_nav_share, "launch", "nav_aurora.launch.py")
        ),
    )
    nav_aurora_delayed = TimerAction(period=10.0, actions=[nav_aurora_launch])

    # 4) Inspection manager + photo capture — after Nav2 lifecycle
    # Nav2 nodes at 25s, nav_lifecycle_startup at 45s. Bringup can take 60s (TF
    # wait) + 30–60s (configure/activate). Inspection at 120s so Nav2 is almost
    # always ready; manager still waits for navigate_to_pose (nav2_wait_timeout).
    inspection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(inspection_share, "launch", "inspection_manager.launch.py")
        ),
        launch_arguments={
            "config_file": LaunchConfiguration("config_file"),
            "dry_run": LaunchConfiguration("dry_run"),
        }.items(),
    )
    inspection_delayed = TimerAction(period=120.0, actions=[inspection_launch])

    # All nodes must use same RMW so TF, topics, and mission see each other.
    # startup.sh also sets this.
    set_rmw = SetEnvironmentVariable(name="RMW_IMPLEMENTATION", value="rmw_cyclonedds_cpp")

    return LaunchDescription([
        set_rmw,
        ip_address_arg,
        use_bridge_arg,
        config_file_arg,
        dry_run_arg,
        use_motor_driver_arg,
        uart_port_arg,
        motor_driver,
        aurora_bringup,
        segment_3d_delayed,
        nav_aurora_delayed,
        inspection_delayed,
    ])
