# Launch only Aurora semantic vehicle detection + marker visualization.
# Use when Aurora (slamware_ros_sdk_server_node) is already running and publishing
# semantic_labels + depth_image_raw. Subscribes to those, publishes
# /aurora_semantic/vehicle_bounding_boxes and /aurora_semantic/vehicle_markers (RViz).
# If semantic_labels is not published (e.g. device not providing it), set fallback_vehicle_boxes_topic
# to use YOLO 3D boxes (e.g. run segment_3d and pass /darknet_ros_3d/vehicle_bounding_boxes).
#
# Usage (Aurora already running):
#   ros2 launch segmentation_3d vehicle_detection_visualize.launch.py
# With YOLO fallback when semantic unavailable:
#   ros2 launch segmentation_3d vehicle_detection_visualize.launch.py fallback_vehicle_boxes_topic:=/darknet_ros_3d/vehicle_bounding_boxes
#
# In RViz: Add display -> By topic -> /aurora_semantic/vehicle_markers (MarkerArray).
# Frame: slamware_map.
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory, get_package_prefix
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('segmentation_3d')
    pkg_prefix = get_package_prefix('segmentation_3d')
    intrinsics_path = os.path.join(pkg_share, 'config', 'aurora_depth_intrinsics.yaml')
    marker_script = os.path.join(pkg_prefix, 'lib', 'segmentation_3d', 'vehicle_boxes_marker_node')

    fallback_arg = DeclareLaunchArgument(
        'fallback_vehicle_boxes_topic',
        default_value='',
        description='If semantic_labels not received after 10s, use vehicle boxes from this topic. Empty = Aurora-only (default).',
    )

    aurora_semantic_fusion_node = Node(
        package='segmentation_3d',
        executable='aurora_semantic_fusion_node',
        name='aurora_semantic_fusion',
        output='screen',
        parameters=[{
            'semantic_topic': '/slamware_ros_sdk_server_node/semantic_labels',
            'depth_topic': '/slamware_ros_sdk_server_node/depth_image_raw',
            'output_topic': '/aurora_semantic/vehicle_bounding_boxes',
            'intrinsics_file': intrinsics_path,
            'fallback_vehicle_boxes_topic': LaunchConfiguration('fallback_vehicle_boxes_topic'),
        }],
    )

    # Run marker script with system python3 (Node looks for executable in pkg lib dir)
    vehicle_boxes_marker_node = ExecuteProcess(
        cmd=[FindExecutable(name='python3'), marker_script],
        name='vehicle_boxes_marker',
        output='screen',
        shell=False,
    )

    return LaunchDescription([
        fallback_arg,
        aurora_semantic_fusion_node,
        vehicle_boxes_marker_node,
    ])
