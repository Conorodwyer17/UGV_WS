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
from launch.utilities import perform_substitutions
from launch_ros.actions import Node, SetRemap

# Regex to match any absolute path ending with ugv_nav/behavior_trees/ (works for any workspace location)
_BT_PATH_PATTERN = re.compile(r'/[^\s]+/ugv_nav/behavior_trees/')


def _substitute_bt_paths(content: str, ugv_nav_dir: str) -> str:
    """Replace hardcoded behaviour tree paths with package-relative paths."""
    bt_prefix = os.path.join(ugv_nav_dir, 'behavior_trees')
    return _BT_PATH_PATTERN.sub(bt_prefix + '/', content)


def _create_nav_with_substituted_params(context):
    """Create temp param files with substituted BT paths, costmap resolution; return nav TimerActions."""
    ugv_nav_dir = get_package_share_directory('ugv_nav')
    param_dir = os.path.join(ugv_nav_dir, 'param')
    param_basenames = [
        'nav_aurora.yaml',
        'nav_aurora_ekf.yaml',
        'nav_aurora_tight_full.yaml',
        'nav_aurora_tight_ekf.yaml',
        'nav_aurora_sim.yaml',
    ]
    res = perform_substitutions(
        context, [LaunchConfiguration('costmap_resolution', default='0.10')]
    ).strip()
    local_cmap_hz = perform_substitutions(
        context, [LaunchConfiguration('costmap_local_update_frequency', default='')]
    ).strip()
    temp_files = {}
    for basename in param_basenames:
        src_path = os.path.join(param_dir, basename)
        with open(src_path, 'r') as f:
            content = f.read()
        substituted = _substitute_bt_paths(content, ugv_nav_dir)
        # Force 2D costmap grid resolution (local/global); does not change z_resolution or mask_resolution.
        substituted = re.sub(
            r'^(\s*)resolution:\s*[\d.]+(\s*)$',
            rf'\1resolution: {res}\2',
            substituted,
            flags=re.MULTILINE,
        )
        substituted = substituted.replace("resolution: 0.05", f"resolution: {res}")
        # Optional: lower local costmap CPU (default YAML uses 5.0 Hz for local layer)
        if local_cmap_hz:
            substituted = substituted.replace(
                '      update_frequency: 5.0',
                f'      update_frequency: {local_cmap_hz}',
                1,
            )
        fd, tmp_path = tempfile.mkstemp(suffix='.yaml', prefix='nav_aurora_')
        os.close(fd)
        with open(tmp_path, 'w') as f:
            f.write(substituted)
        temp_files[basename] = tmp_path

    sim_tyre = perform_substitutions(
        context, [LaunchConfiguration('sim_tyre_detections', default='false')]
    )
    use_tight = perform_substitutions(
        context, [LaunchConfiguration('use_tight_goal_tolerance', default='false')]
    )
    use_ekf = perform_substitutions(
        context, [LaunchConfiguration('use_ekf', default='false')]
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

    enable_mux = perform_substitutions(
        context, [LaunchConfiguration('enable_cmd_vel_mux', default='true')]
    ).lower() in ('true', '1', 'yes')
    enable_dgate = perform_substitutions(
        context, [LaunchConfiguration('enable_depth_gate', default='true')]
    ).lower() in ('true', '1', 'yes')
    if not enable_mux and not enable_dgate:
        cmd_vel_dst = 'cmd_vel'
    elif not enable_mux and enable_dgate:
        cmd_vel_dst = 'cmd_vel_nav'
    else:
        cmd_vel_dst = 'cmd_vel_nav_source'

    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')
    nav_group = GroupAction(
        actions=[
            SetRemap(src='/map', dst='/slamware_ros_sdk_server_node/map'),
            SetRemap(src='cmd_vel', dst=cmd_vel_dst),
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


def _vel_bridge_actions(context):
    """cmd_vel_mux / depth_gate / vehicle_speed_filter — optional for minimal Jetson demos."""
    ugv_nav_prefix = get_package_prefix('ugv_nav')
    cmd_vel_mux_script = os.path.join(ugv_nav_prefix, 'lib', 'ugv_nav', 'cmd_vel_mux_node.py')
    depth_gate_script = os.path.join(ugv_nav_prefix, 'lib', 'ugv_nav', 'depth_gate_node.py')
    vehicle_speed_filter_script = os.path.join(
        ugv_nav_prefix, 'lib', 'ugv_nav', 'vehicle_speed_filter_node.py'
    )
    mux_on = perform_substitutions(
        context, [LaunchConfiguration('enable_cmd_vel_mux', default='true')]
    ).lower() in ('true', '1', 'yes')
    dgate_on = perform_substitutions(
        context, [LaunchConfiguration('enable_depth_gate', default='true')]
    ).lower() in ('true', '1', 'yes')
    vsf_on = perform_substitutions(
        context, [LaunchConfiguration('enable_vehicle_speed_filter', default='true')]
    ).lower() in ('true', '1', 'yes')
    actions = []
    if mux_on:
        mux_out = 'cmd_vel_nav' if dgate_on else 'cmd_vel'
        actions.append(
            ExecuteProcess(
                cmd=[
                    sys.executable,
                    cmd_vel_mux_script,
                    '--ros-args',
                    '-p',
                    f'cmd_vel_out_topic:={mux_out}',
                ],
                name='cmd_vel_mux',
                output='screen',
            )
        )
    if dgate_on:
        actions.append(
            ExecuteProcess(
                cmd=[sys.executable, depth_gate_script],
                name='depth_gate',
                output='screen',
            )
        )
    if vsf_on:
        actions.append(
            ExecuteProcess(
                cmd=[
                    sys.executable,
                    vehicle_speed_filter_script,
                    '--ros-args',
                    '-p',
                    'vehicle_boxes_topic:=/aurora_semantic/vehicle_bounding_boxes',
                    '-p',
                    'filter_mask_topic:=/inspection/speed_filter_mask',
                    '-p',
                    'filter_info_topic:=/local_costmap/costmap_filter_info',
                    '-p',
                    'mask_frame_id:=slamware_map',
                    '-p',
                    'vehicle_buffer_m:=1.5',
                    '-p',
                    'speed_percent_in_zone:=50',
                ],
                name='vehicle_speed_filter',
                output='screen',
            )
        )
    return actions


def generate_launch_description():
    # Force Fast DDS to use UDP only (avoids SHM port conflicts when multiple ROS processes run).
    # For more reliable Nav2 lifecycle bringup, use Cyclone in *all* terminals:
    #   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp  (install: ros-humble-rmw-cyclonedds-cpp)
    set_fastdds_udp = SetEnvironmentVariable(name='FASTDDS_BUILTIN_TRANSPORTS', value='UDPv4')

    # map->slamware_map, slamware_map->odom: from aurora_bringup world_frame_tf_publisher (current timestamps).
    # When external_tf:=true (use_mock), aurora_mock provides these; nav_aurora does not duplicate.
    # base_footprint: from URDF (base_link->base_footprint via robot_state_publisher in aurora_bringup).

    # Delay Nav2 bringup so TF and services are ready (Aurora must publish map, odom, scan first).
    # When external_tf (use_mock): 5s so costmap builds TF buffer; else 15s for real Aurora.
    ugv_nav_prefix = get_package_prefix('ugv_nav')
    startup_script = os.path.join(ugv_nav_prefix, 'lib', 'ugv_nav', 'nav_lifecycle_startup.py')
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
    ld.add_action(
        DeclareLaunchArgument(
            'costmap_resolution',
            default_value='0.10',
            description='Override Nav2 local/global costmap grid resolution in generated param YAML (Jetson RAM).',
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'costmap_local_update_frequency',
            default_value='',
            description=(
                'If non-empty, replace local costmap update_frequency (default 5.0 Hz in YAML), e.g. 2.0 for demos.'
            ),
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'enable_cmd_vel_mux',
            default_value='true',
            description='Nav2 vs centroid servo mux. Set false for minimal demo (direct cmd_vel chain).',
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'enable_depth_gate',
            default_value='true',
            description='Gate cmd_vel on /stereo/navigation_permitted. Set false for bench (bridge off).',
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            'enable_vehicle_speed_filter',
            default_value='true',
            description='Publish speed filter mask from vehicle boxes. Set false to save CPU.',
        )
    )
    ld.add_action(OpaqueFunction(function=_vel_bridge_actions))
    ld.add_action(OpaqueFunction(function=_create_nav_with_substituted_params))
    ld.add_action(lifecycle_startup_delayed_ext)
    ld.add_action(lifecycle_startup_delayed_other)
    return ld
