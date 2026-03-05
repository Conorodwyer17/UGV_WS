# Launch Nav2 with Aurora: odom, scan, and map from Aurora SDK.
# No SLAM, no AMCL - Aurora provides localization and map.

import os
import re
import sys
import tempfile

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, SetRemap

# Regex to match any absolute path ending with ugv_nav/behavior_trees/ (works for any workspace location)
_BT_PATH_PATTERN = re.compile(r'/[^\s]+/ugv_nav/behavior_trees/')


def _substitute_bt_paths(content: str, ugv_nav_dir: str) -> str:
    """Replace hardcoded behaviour tree paths with package-relative paths."""
    bt_prefix = os.path.join(ugv_nav_dir, 'behavior_trees')
    return _BT_PATH_PATTERN.sub(bt_prefix + '/', content)


def _create_nav_with_substituted_params(context):
    """Create temp param files with substituted BT paths and return nav TimerActions."""
    ugv_nav_dir = get_package_share_directory('ugv_nav')
    param_dir = os.path.join(ugv_nav_dir, 'param')
    param_basenames = [
        'nav_aurora.yaml',
        'nav_aurora_ekf.yaml',
        'nav_aurora_tight_full.yaml',
        'nav_aurora_tight_ekf.yaml',
        'nav_aurora_sim.yaml',
    ]
    temp_files = {}
    for basename in param_basenames:
        src_path = os.path.join(param_dir, basename)
        with open(src_path, 'r') as f:
            content = f.read()
        substituted = _substitute_bt_paths(content, ugv_nav_dir)
        fd, tmp_path = tempfile.mkstemp(suffix='.yaml', prefix='nav_aurora_')
        os.close(fd)
        with open(tmp_path, 'w') as f:
            f.write(substituted)
        temp_files[basename] = tmp_path

    sim_tyre = context.perform_substitution(
        LaunchConfiguration('sim_tyre_detections', default='false')
    )
    use_tight = context.perform_substitution(
        LaunchConfiguration('use_tight_goal_tolerance', default='false')
    )
    use_ekf = context.perform_substitution(
        LaunchConfiguration('use_ekf', default='false')
    )

    if sim_tyre == 'true':
        params_file = temp_files['nav_aurora_sim.yaml']
    elif use_tight == 'true' and use_ekf == 'true':
        params_file = temp_files['nav_aurora_tight_ekf.yaml']
    elif use_tight == 'true':
        params_file = temp_files['nav_aurora_tight_full.yaml']
    elif use_ekf == 'true':
        params_file = temp_files['nav_aurora_ekf.yaml']
    else:
        params_file = temp_files['nav_aurora.yaml']

    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')
    nav_group = GroupAction(
        actions=[
            SetRemap(src='/map', dst='/slamware_ros_sdk_server_node/map'),
            SetRemap(src='cmd_vel', dst='cmd_vel_nav_source'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'navigation_launch.py')
                ),
                launch_arguments={
                    'namespace': '',
                    'use_sim_time': LaunchConfiguration('use_sim_time', default='false'),
                    'autostart': 'false',
                    'params_file': params_file,
                    'use_composition': 'False',
                    'use_respawn': 'False',
                    'container_name': 'nav2_container',
                    'log_level': LaunchConfiguration('nav2_log_level', default='info'),
                }.items(),
            ),
        ]
    )
    return [
        TimerAction(
            period=5.0,
            actions=[nav_group],
            condition=IfCondition(LaunchConfiguration('external_tf', default='false')),
        ),
        TimerAction(
            period=15.0,
            actions=[nav_group],
            condition=UnlessCondition(LaunchConfiguration('external_tf', default='false')),
        ),
    ]


def generate_launch_description():
    # Force Fast DDS to use UDP only (avoids SHM port conflicts when multiple ROS processes run).
    # For more reliable Nav2 lifecycle bringup, use Cyclone in *all* terminals:
    #   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp  (install: ros-humble-rmw-cyclonedds-cpp)
    set_fastdds_udp = SetEnvironmentVariable(name='FASTDDS_BUILTIN_TRANSPORTS', value='UDPv4')

    # TF: Nav2 uses global_frame "map"; Aurora SDK uses "slamware_map".
    # Publish map->slamware_map (identity) so map frame exists.
    # Aurora must be running (slamware_map->odom->base_link).
    # When external_tf:=true (use_mock), aurora_mock provides these; skip to avoid duplicate TF.
    map_to_slamware_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_slamware_map',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--yaw', '0',
            '--pitch', '0',
            '--roll', '0',
            '--frame-id', 'map',
            '--child-frame-id', 'slamware_map',
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time', default='false')}],
        condition=UnlessCondition(LaunchConfiguration('external_tf', default='false')),
    )
    # Nav2 expects base_footprint; Aurora/URDF use base_link. Always needed (aurora_mock doesn't provide it).
    base_footprint_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_base_footprint',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--yaw', '0',
            '--pitch', '0',
            '--roll', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'base_footprint',
        ],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time', default='false')}],
    )

    # Delay Nav2 bringup so TF and services are ready (Aurora must publish map, odom, scan first).
    # When external_tf (use_mock): 5s so costmap builds TF buffer; else 15s for real Aurora.
    ugv_nav_prefix = get_package_prefix('ugv_nav')
    startup_script = os.path.join(ugv_nav_prefix, 'lib', 'ugv_nav', 'nav_lifecycle_startup.py')
    depth_gate_script = os.path.join(ugv_nav_prefix, 'lib', 'ugv_nav', 'depth_gate_node.py')
    cmd_vel_mux_script = os.path.join(ugv_nav_prefix, 'lib', 'ugv_nav', 'cmd_vel_mux_node.py')
    lifecycle_startup = ExecuteProcess(
        cmd=[
            sys.executable, startup_script,
            '--ros-args', '-p', PythonExpression(["'use_sim_time:=' + '", LaunchConfiguration('use_sim_time', default='false'), "'"]),
        ],
        name='nav_lifecycle_startup',
        output='screen',
    )
    # When external_tf (e.g. use_mock): give controller_server more time to init (45s); else 35s
    lifecycle_startup_delayed_ext = TimerAction(
        period=45.0, actions=[lifecycle_startup],
        condition=IfCondition(LaunchConfiguration('external_tf', default='false')),
    )
    lifecycle_startup_delayed_other = TimerAction(
        period=35.0, actions=[lifecycle_startup],
        condition=UnlessCondition(LaunchConfiguration('external_tf', default='false')),
    )
    ld = LaunchDescription()
    ld.add_action(set_fastdds_udp)
    ld.add_action(
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Launch RViz',
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'aurora_ip',
            default_value='192.168.11.1',
            description='Aurora IP (for reference)',
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time (true for rosbag replay)',
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'use_tight_goal_tolerance',
            default_value='false',
            description='If true, use xy_goal_tolerance 0.1 and max_vel_x 0.18 (nav_aurora_tight.yaml)',
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'use_ekf',
            default_value='false',
            description='If true, use /odometry/filtered (EKF fusion); requires ekf_aurora.launch + wheel odom',
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'external_tf',
            default_value='false',
            description='If true, map->slamware_map and base_footprint provided externally (e.g. aurora_mock); skip publishing',
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'nav2_log_level',
            default_value='info',
            description='Log level for Nav2 nodes (e.g. debug for controller_server lifecycle stall diagnosis)',
        )
    )
    # cmd_vel mux: nav vs centroid_servo -> cmd_vel_nav (depth_gate subscribes to cmd_vel_nav)
    cmd_vel_mux = ExecuteProcess(
        cmd=[sys.executable, cmd_vel_mux_script],
        name='cmd_vel_mux',
        output='screen',
    )
    # Depth gate: forwards cmd_vel_nav -> cmd_vel when /stereo/navigation_permitted True
    depth_gate = ExecuteProcess(
        cmd=[sys.executable, depth_gate_script],
        name='depth_gate',
        output='screen',
    )

    # Vehicle speed filter: publishes dynamic OccupancyGrid mask from vehicle boxes.
    # Slows robot to 50% when within 1.5 m of detected vehicles (Nav2 SpeedFilter).
    # Use ExecuteProcess to avoid libexec lookup issues on some platforms (Jetson).
    vehicle_speed_filter_script = os.path.join(ugv_nav_prefix, 'lib', 'ugv_nav', 'vehicle_speed_filter_node.py')
    vehicle_speed_filter = ExecuteProcess(
        cmd=[
            sys.executable, vehicle_speed_filter_script,
            '--ros-args',
            '-p', 'vehicle_boxes_topic:=/aurora_semantic/vehicle_bounding_boxes',
            '-p', 'filter_mask_topic:=/inspection/speed_filter_mask',
            '-p', 'filter_info_topic:=/local_costmap/costmap_filter_info',
            '-p', 'mask_frame_id:=map',
            '-p', 'vehicle_buffer_m:=1.5',
            '-p', 'speed_percent_in_zone:=50',
        ],
        name='vehicle_speed_filter',
        output='screen',
    )

    ld.add_action(map_to_slamware_tf)
    ld.add_action(base_footprint_tf)
    ld.add_action(cmd_vel_mux)
    ld.add_action(depth_gate)
    ld.add_action(vehicle_speed_filter)
    ld.add_action(OpaqueFunction(function=_create_nav_with_substituted_params))
    ld.add_action(lifecycle_startup_delayed_ext)
    ld.add_action(lifecycle_startup_delayed_other)
    return ld
