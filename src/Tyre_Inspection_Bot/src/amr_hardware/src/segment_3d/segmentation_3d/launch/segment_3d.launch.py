# segment_3d launch — Aurora 2.11 native depth + YOLO tire detection
# Aurora native depth: /slamware_ros_sdk_server_node/depth_image_raw (224x416)
# aurora_depth_camera_info publishes CameraInfo matching 416x224.
# depth_to_registered_pointcloud -> /segmentation_processor/registered_pointcloud, /camera/depth/points.
# Bridge deprecated: use_bridge=false is default; bridge opt-in for legacy firmware 1.2.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('segmentation_3d')
    config_depth = os.path.join(pkg_share, 'config', 'config.yaml')
    intrinsics_path = os.path.join(pkg_share, 'config', 'aurora_depth_intrinsics.yaml')

    use_bridge_arg = DeclareLaunchArgument(
        "use_bridge",
        default_value="false",
        description="Legacy: if true, use aurora_sdk_bridge for depth. Default false = Aurora 2.11 native.",
    )

    camera_rgb_topic_arg = DeclareLaunchArgument(
        'camera_rgb_topic',
        default_value='/slamware_ros_sdk_server_node/left_image_raw',
        description='Camera RGB image topic',
    )
    fallback_vehicle_boxes_arg = DeclareLaunchArgument(
        'fallback_vehicle_boxes_topic',
        default_value='/darknet_ros_3d/bounding_boxes',
        description='If semantic_labels not received after 10s, use vehicle boxes from this topic (YOLO 3D). Empty to disable.',
    )

    # Semantic fusion for vehicle detection (Aurora 2.11) — same intrinsics as depth pipeline.
    # When Aurora does not publish semantic_labels, fallback to YOLO 3D boxes from segmentation_processor.
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
            'semantic_stale_s': 2.0,
            'depth_stale_s': 2.0,
        }],
    )

    ultralytics_node = Node(
        package='segmentation_3d',
        executable='ultralytics_node',
        name='ultralytics_segmentation',
        output='screen',
        parameters=[{'camera_rgb_topic': LaunchConfiguration('camera_rgb_topic')}],
    )

    # Aurora 2.11: CameraInfo for depth 416x224 (matches depth_image_raw)
    aurora_depth_camera_info_node = Node(
        package='segmentation_3d',
        executable='aurora_depth_camera_info_node',
        name='aurora_depth_camera_info',
        output='screen',
        parameters=[{
            'intrinsics_file': intrinsics_path,
            'camera_info_topic': '/camera/depth/camera_info',
            'frame_id': 'camera_depth_optical_frame',
        }],
        condition=UnlessCondition(LaunchConfiguration("use_bridge")),
    )

    # Aurora native (default): depth from device
    depth_to_pointcloud_aurora = Node(
        package='segmentation_3d',
        executable='depth_to_registered_pointcloud_node',
        name='depth_to_registered_pointcloud',
        output='screen',
        parameters=[{
            'depth_topic': '/slamware_ros_sdk_server_node/depth_image_raw',
            'camera_info_topic': '/camera/depth/camera_info',
            'output_topic': '/segmentation_processor/registered_pointcloud',
            'output_frame_id': 'camera_depth_optical_frame',
            'depth_points_topic': '/camera/depth/points',
        }],
        condition=UnlessCondition(LaunchConfiguration("use_bridge")),
    )
    # Bridge (legacy firmware 1.2)
    depth_to_pointcloud_bridge = Node(
        package='segmentation_3d',
        executable='depth_to_registered_pointcloud_node',
        name='depth_to_registered_pointcloud',
        output='screen',
        parameters=[{
            'depth_topic': '/camera/depth/image',
            'camera_info_topic': '/camera/depth/camera_info',
            'output_topic': '/segmentation_processor/registered_pointcloud',
            'output_frame_id': 'camera_depth_optical_frame',
            'depth_points_topic': '/camera/depth/points',
        }],
        condition=IfCondition(LaunchConfiguration("use_bridge")),
    )

    segmentation_processor_node = Node(
        package='segmentation_3d',
        executable='segmentation_processor_node',
        name='segmentation_processor_node',
        output='screen',
        parameters=[config_depth],
    )

    delayed_depth_pipeline = TimerAction(
        period=3.0,
        actions=[depth_to_pointcloud_aurora, depth_to_pointcloud_bridge, segmentation_processor_node],
    )

    return LaunchDescription([
        use_bridge_arg,
        camera_rgb_topic_arg,
        fallback_vehicle_boxes_arg,
        aurora_semantic_fusion_node,
        ultralytics_node,
        aurora_depth_camera_info_node,
        delayed_depth_pipeline,
    ])

