# segment_3d launch — Aurora 2.11 native depth + dual YOLO streams (vehicle+tire)
# Aurora native depth: /slamware_ros_sdk_server_node/depth_image_raw (224x416)
# aurora_depth_camera_info publishes CameraInfo matching 416x224.
# depth_to_registered_pointcloud -> /segmentation_processor/registered_pointcloud, /camera/depth/points.
# Bridge deprecated: use_bridge=false is default; bridge opt-in for legacy firmware 1.2.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def _create_tire_nodes(context, wheel_model_path):
    """Create ultralytics_tire_node (GPU) or ultralytics_tire_cpu (CPU) based on use_cpu_inference."""
    use_cpu = perform_substitutions(context, [LaunchConfiguration('use_cpu_inference', default='false')])
    use_cpu = str(use_cpu).strip().lower() in ('true', '1', 'yes')

    if use_cpu:
        onnx_path = wheel_model_path.replace('.pt', '.onnx')
        workspace = os.environ.get("UGV_WS", os.path.expanduser("~/ugv_ws"))
        if not os.path.isfile(os.path.expanduser(onnx_path)):
            onnx_path = os.path.join(workspace, "src", "Tyre_Inspection_Bot", "best_fallback.onnx")
        return [
            Node(
                package='segmentation_3d',
                executable='ultralytics_node_cpu',
                name='ultralytics_tire',
                output='screen',
                parameters=[{
                    'use_sim_time': LaunchConfiguration('use_sim_time', default='false'),
                    'model_path': onnx_path,
                    'camera_rgb_topic': LaunchConfiguration('camera_rgb_topic'),
                    'objects_segment_topic': '/ultralytics_tire/segmentation/objects_segment',
                    'imgsz': 224,  # CPU: 224 for ~5 Hz; wheel_imgsz is for GPU
                    'conf_threshold': LaunchConfiguration('wheel_confidence'),
                    'inference_interval_s': 0.2,
                    'interested_class_names': ['wheel'],
                }],
                condition=IfCondition(LaunchConfiguration('use_yolo', default='true')),
            )
        ]

    prefer_trt_val = perform_substitutions(context, [LaunchConfiguration('prefer_tensorrt_inspection')])
    prefer_trt_str = str(prefer_trt_val).strip().lower() if prefer_trt_val else 'auto'
    return [
        Node(
            package='segmentation_3d',
            executable='ultralytics_node',
            name='ultralytics_tire',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time', default='false'),
                'camera_rgb_topic': LaunchConfiguration('camera_rgb_topic'),
                'inspection_model': wheel_model_path,
                'use_vehicle_yolo': LaunchConfiguration('use_vehicle_yolo', default='false'),
                'log_raw_detections': True,
                'confidence': LaunchConfiguration('wheel_confidence'),
                'device': LaunchConfiguration('device'),
                'half': LaunchConfiguration('half'),
                'fixed_mode': 'inspection',
                'prefer_tensorrt_inspection': prefer_trt_str,
                'subscribe_mode_topic': False,
                'objects_segment_topic': '/ultralytics_tire/segmentation/objects_segment',
                'segmentation_image_topic': '/ultralytics_tire/segmentation/image',
                'inference_interval_s': LaunchConfiguration('inference_interval_s', default='0.1'),
                'interested_class_names': ['wheel'],
                'max_det': LaunchConfiguration('wheel_max_det', default='50'),
                'imgsz': LaunchConfiguration('wheel_imgsz', default='640'),
                'model_load_delay_s': LaunchConfiguration('model_load_delay_s', default='1.0'),
            }],
            condition=IfCondition(LaunchConfiguration('use_yolo', default='true')),
        )
    ]


def generate_launch_description():
    pkg_share = get_package_share_directory('segmentation_3d')
    config_depth = os.path.join(pkg_share, 'config', 'config.yaml')
    intrinsics_path = os.path.join(pkg_share, 'config', 'aurora_depth_intrinsics.yaml')

    use_bridge_arg = DeclareLaunchArgument(
        "use_bridge",
        default_value="false",
        description="Legacy: if true, use aurora_sdk_bridge for depth. Default false = Aurora 2.11 native.",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time (true for rosbag replay with --clock).",
    )

    camera_rgb_topic_arg = DeclareLaunchArgument(
        'camera_rgb_topic',
        default_value='/slamware_ros_sdk_server_node/left_image_raw',
        description='Camera RGB image topic',
    )
    fallback_vehicle_boxes_arg = DeclareLaunchArgument(
        'fallback_vehicle_boxes_topic',
        default_value='',  # Aurora-only by default (perfection / single source); set to /darknet_ros_3d/vehicle_bounding_boxes for YOLO fallback if semantic missing
        description='If semantic_labels not received after 10s, use vehicle boxes from this topic (YOLO 3D). Empty = Aurora-only.',
    )
    log_raw_detections_arg = DeclareLaunchArgument(
        'log_raw_detections',
        default_value='false',
        description='If true, ultralytics_node logs raw detections (class_id, name, confidence, bbox) at 2 Hz for perception validation.',
    )
    confidence_arg = DeclareLaunchArgument(
        'confidence',
        default_value='0.25',
        description='YOLO confidence threshold (e.g. 0.1 to see weak detections).',
    )
    wheel_confidence_arg = DeclareLaunchArgument(
        'wheel_confidence',
        default_value='0.5',
        description='YOLO wheel-model confidence threshold. 0.5 ensures tires detected when visible; raise to 0.65-0.8 to reduce false positives.',
    )
    wheel_max_det_arg = DeclareLaunchArgument(
        'wheel_max_det',
        default_value='50',
        description='Max detections per image for tire model; lower reduces NMS time (avoids "NMS time limit exceeded"). Default 50.',
    )
    wheel_imgsz_arg = DeclareLaunchArgument(
        'wheel_imgsz',
        default_value='640',
        description='Inference input size for tire model; 480 or 416 for faster inference on Jetson. Default 640.',
    )
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='0',
        description='Inference device: 0 (GPU) or cpu when no CUDA.',
    )
    half_arg = DeclareLaunchArgument(
        'half',
        default_value='true',
        description='Use FP16 when GPU; set false for CPU.',
    )
    pcl_fallback_arg = DeclareLaunchArgument(
        'pcl_fallback_enabled',
        default_value='true',
        description='If true, run tire_detector_pcl + tire_merger for point-cloud tire fallback when YOLO misses tires.',
    )
    centroid_servo_arg = DeclareLaunchArgument(
        'centroid_servo_enabled',
        default_value='true',
        description='If true, run centroid_servo_node for image-based fine positioning when proximity gate passed.',
    )
    use_synthetic_vehicle_arg = DeclareLaunchArgument(
        'use_synthetic_vehicle',
        default_value='false',
        description='When true (use_mock sim), skip aurora_semantic_fusion; use synthetic_vehicle_publisher from aurora_mock.',
    )
    use_yolo_arg = DeclareLaunchArgument(
        'use_yolo',
        default_value='true',
        description='When false (e.g. use_mock), skip ultralytics nodes to avoid slow CPU inference; tire goals from vehicle geometry.',
    )
    use_simulated_detection_arg = DeclareLaunchArgument(
        'use_simulated_detection',
        default_value='false',
        description='When true (use_mock sim), run simulated_detection_node to publish ObjectsSegment from ground-truth vehicle boxes.',
    )
    sim_tyre_detections_arg = DeclareLaunchArgument(
        'sim_tyre_detections',
        default_value='false',
        description='When true (full_bringup sim), skip ultralytics_tire and tire_merger; verify_system.py publishes to /sim/* topics.',
    )
    miss_probability_arg = DeclareLaunchArgument(
        'miss_probability',
        default_value='0.0',
        description='Probability (0-1) that simulated_detection skips a tire (tests PCL fallback, return-later). Use with use_stress_test.',
    )
    # TensorRT: enable by default when best_fallback.engine exists (PERFORMANCE_TUNING, acc-qcar2 pattern)
    _workspace = os.environ.get("UGV_WS", os.path.expanduser("~/ugv_ws"))
    _engine_path = os.path.join(_workspace, "src", "Tyre_Inspection_Bot", "best_fallback.engine")
    _prefer_tensorrt_default = os.path.isfile(_engine_path)
    prefer_tensorrt_inspection_arg = DeclareLaunchArgument(
        'prefer_tensorrt_inspection',
        default_value=str(_prefer_tensorrt_default).lower(),
        description='Use TensorRT engine for wheel detection. Set false if you see "invalid class index" (28,37,39,45,47) - engine may need re-export from best_fallback.pt.',
    )
    use_vehicle_yolo_arg = DeclareLaunchArgument(
        'use_vehicle_yolo',
        default_value='false',
        description='Enable YOLO vehicle detection. Aurora is primary for vehicle boxes; set true only for YOLO fallback. Default false saves GPU/CPU.',
    )
    use_cpu_inference_arg = DeclareLaunchArgument(
        'use_cpu_inference',
        default_value='false',
        description='Use CPU ONNX tire detection. Set true for fallback. Default false = GPU (16 GB Jetson).',
    )
    inference_interval_s_arg = DeclareLaunchArgument(
        'inference_interval_s',
        default_value='0.1',
        description='Seconds between GPU tire inferences. 0.1 = 10 Hz for 16 GB Jetson; increase if model cannot keep up.',
    )
    model_load_delay_arg = DeclareLaunchArgument(
        'model_load_delay_s',
        default_value='1.0',
        description='Seconds to delay before loading YOLO/TensorRT model. 1.0 for 16 GB Jetson; increase if OOM persists.',
    )

    # Semantic fusion for vehicle detection (Aurora 2.11) — same intrinsics as depth pipeline.
    # Default: Aurora-only (fallback_vehicle_boxes_topic=""). Set fallback to /darknet_ros_3d/vehicle_bounding_boxes for YOLO fallback if semantic missing.
    aurora_semantic_fusion_node = Node(
        package='segmentation_3d',
        executable='aurora_semantic_fusion_node',
        name='aurora_semantic_fusion',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time', default='false'),
            'semantic_topic': '/slamware_ros_sdk_server_node/semantic_labels',
            'depth_topic': '/slamware_ros_sdk_server_node/depth_image_raw',
            'output_topic': '/aurora_semantic/vehicle_bounding_boxes',
            'intrinsics_file': intrinsics_path,
            'fallback_vehicle_boxes_topic': LaunchConfiguration('fallback_vehicle_boxes_topic'),
            'semantic_stale_s': 2.0,
            'depth_stale_s': 2.0,
        }],
        condition=UnlessCondition(LaunchConfiguration('use_synthetic_vehicle', default='false')),
    )

    # Wheel detection: canonical model best_fallback.pt (wheel class)
    _bot_dir = os.path.join(_workspace, "src", "Tyre_Inspection_Bot")
    wheel_model_path = os.path.join(_bot_dir, "best_fallback.pt")
    # Vehicle boxes from Aurora by default; enable use_vehicle_yolo for YOLO fallback only.
    _use_vehicle_yolo = LaunchConfiguration('use_vehicle_yolo', default='false')
    # ultralytics_tire_node uses OpaqueFunction to resolve prefer_tensorrt_inspection at launch time and convert
    # to string (avoids InvalidParameterTypeException when launch passes bool from prefer_tensorrt_inspection:=false)
    ultralytics_tire_action = OpaqueFunction(
        function=_create_tire_nodes,
        args=[wheel_model_path],
    )
    ultralytics_vehicle_node = Node(
        package='segmentation_3d',
        executable='ultralytics_node',
        name='ultralytics_vehicle',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time', default='false'),
            'camera_rgb_topic': LaunchConfiguration('camera_rgb_topic'),
            'inspection_model': wheel_model_path,
            'log_raw_detections': LaunchConfiguration('log_raw_detections'),
            'confidence': LaunchConfiguration('confidence'),
            'device': LaunchConfiguration('device'),
            'half': LaunchConfiguration('half'),
            'fixed_mode': 'navigation',
            'subscribe_mode_topic': False,
            'objects_segment_topic': '/ultralytics_vehicle/segmentation/objects_segment',
            'segmentation_image_topic': '/ultralytics_vehicle/segmentation/image',
            'inference_interval_s': 2.0,  # Throttle: reduce CPU load so Nav2 controller keeps up
        }],
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('use_yolo', default='true'),
                "' == 'true' and '", _use_vehicle_yolo, "' == 'true'"
            ])
        ),
    )
    simulated_detection_node = Node(
        package='segmentation_3d',
        executable='simulated_detection_node',
        name='simulated_detection',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time', default='false'),
            'vehicle_boxes_topic': '/aurora_semantic/vehicle_bounding_boxes',
            'output_topic': '/ultralytics_tire/segmentation/objects_segment',
            'miss_probability': LaunchConfiguration('miss_probability', default='0.0'),
        }],
        condition=IfCondition(LaunchConfiguration('use_simulated_detection', default='false')),
    )

    # Aurora 2.11: CameraInfo for depth 416x224 (matches depth_image_raw)
    aurora_depth_camera_info_node = Node(
        package='segmentation_3d',
        executable='aurora_depth_camera_info_node',
        name='aurora_depth_camera_info',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time', default='false'),
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
            'use_sim_time': LaunchConfiguration('use_sim_time', default='false'),
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
            'use_sim_time': LaunchConfiguration('use_sim_time', default='false'),
            'depth_topic': '/camera/depth/image',
            'camera_info_topic': '/camera/depth/camera_info',
            'output_topic': '/segmentation_processor/registered_pointcloud',
            'output_frame_id': 'camera_depth_optical_frame',
            'depth_points_topic': '/camera/depth/points',
        }],
        condition=IfCondition(LaunchConfiguration("use_bridge")),
    )

    segmentation_processor_vehicle_node = Node(
        package='segmentation_3d',
        executable='segmentation_processor_node',
        name='segmentation_processor_vehicle',
        output='screen',
        parameters=[
            config_depth,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time', default='false'),
                'input_segment_topic': '/ultralytics_vehicle/segmentation/objects_segment',
                'output_bbx3d_topic': '/darknet_ros_3d/vehicle_bounding_boxes',
                'interested_classes': ['car', 'truck', 'bus'],
                'minimum_probability': 0.3,
                'working_frame': 'slamware_map',
            },
        ],
        condition=IfCondition(_use_vehicle_yolo),
    )
    segmentation_processor_tire_node = Node(
        package='segmentation_3d',
        executable='segmentation_processor_node',
        name='segmentation_processor_tire',
        output='screen',
        parameters=[
            config_depth,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time', default='false'),
                'input_segment_topic': '/ultralytics_tire/segmentation/objects_segment',
                'output_bbx3d_topic': '/darknet_ros_3d/tire_bounding_boxes',
                # best_fallback.pt outputs "wheel" (id=22); see docs/BEST_FALLBACK_MODEL_CLASSES.md
                'interested_classes': ['wheel', 'tire', 'tyre', 'car_tire', 'car_tyre', 'car-tire'],
                # Align with PRODUCTION_CONFIG min_tire_probability 0.35; 0.8 was filtering valid wheels
                'minimum_probability': 0.35,
                'min_valid_points_wheel': 1,
                'wheel_cluster_tolerance': 0.6,
                'working_frame': 'slamware_map',
            },
        ],
    )

    # Point-cloud tire fallback: when YOLO misses tires, use PCL clustering in vehicle ROI
    config_pcl = os.path.join(pkg_share, 'config', 'tire_detector_pcl.yaml')
    tire_detector_pcl_node = Node(
        package='segmentation_3d',
        executable='tire_detector_pcl_node',
        name='tire_detector_pcl',
        output='screen',
        parameters=[config_pcl, {'use_sim_time': LaunchConfiguration('use_sim_time', default='false')}],
        condition=IfCondition(LaunchConfiguration('pcl_fallback_enabled')),
    )
    tire_merger_node = Node(
        package='segmentation_3d',
        executable='tire_merger_node',
        name='tire_merger',
        output='screen',
        parameters=[config_pcl, {'use_sim_time': LaunchConfiguration('use_sim_time', default='false')}],
        condition=IfCondition(LaunchConfiguration('pcl_fallback_enabled')),
    )

    # Converts BoundingBoxes3d -> MarkerArray so RViz can show vehicle bounding boxes on the map
    config_centroid = os.path.join(pkg_share, 'config', 'centroid_servo.yaml')
    centroid_servo_node = Node(
        package='segmentation_3d',
        executable='centroid_servo_node',
        name='centroid_servo',
        output='screen',
        parameters=[config_centroid, {'use_sim_time': LaunchConfiguration('use_sim_time', default='false')}],
        condition=IfCondition(LaunchConfiguration('centroid_servo_enabled')),
    )
    vehicle_boxes_marker_node = Node(
        package='segmentation_3d',
        executable='vehicle_boxes_marker_node',
        name='vehicle_boxes_marker',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time', default='false'),
            'input_topic': '/aurora_semantic/vehicle_bounding_boxes',
            'marker_topic': '/aurora_semantic/vehicle_markers',
        }],
    )

    # Vehicle processor only when use_vehicle_yolo; tire processor and PCL always in pipeline.
    delayed_depth_pipeline = TimerAction(
        period=3.0,
        actions=[
            depth_to_pointcloud_aurora,
            depth_to_pointcloud_bridge,
            GroupAction(
                condition=IfCondition(_use_vehicle_yolo),
                actions=[segmentation_processor_vehicle_node],
            ),
            segmentation_processor_tire_node,
            tire_detector_pcl_node,
            tire_merger_node,
        ],
    )

    return LaunchDescription([
        use_bridge_arg,
        use_sim_time_arg,
        camera_rgb_topic_arg,
        fallback_vehicle_boxes_arg,
        log_raw_detections_arg,
        confidence_arg,
        wheel_confidence_arg,
        wheel_max_det_arg,
        wheel_imgsz_arg,
        device_arg,
        half_arg,
        pcl_fallback_arg,
        centroid_servo_arg,
        use_synthetic_vehicle_arg,
        use_yolo_arg,
        use_simulated_detection_arg,
        sim_tyre_detections_arg,
        miss_probability_arg,
        prefer_tensorrt_inspection_arg,
        use_vehicle_yolo_arg,
        use_cpu_inference_arg,
        inference_interval_s_arg,
        model_load_delay_arg,
        aurora_semantic_fusion_node,
        vehicle_boxes_marker_node,
        ultralytics_vehicle_node,
        ultralytics_tire_action,
        simulated_detection_node,
        centroid_servo_node,
        aurora_depth_camera_info_node,
        delayed_depth_pipeline,
    ])

