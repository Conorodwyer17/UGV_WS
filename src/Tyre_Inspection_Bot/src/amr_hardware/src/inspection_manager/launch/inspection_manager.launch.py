import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    vehicle_detection_topic = LaunchConfiguration("vehicle_detection_topic")
    tire_detection_topic = LaunchConfiguration("tire_detection_topic")
    vehicle_fallback_topic = LaunchConfiguration("vehicle_fallback_topic")
    image_topic = LaunchConfiguration("image_topic")
    params_file = LaunchConfiguration("params_file")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    use_navigation_action = LaunchConfiguration("use_navigation_action")
    launch_visual_servo = LaunchConfiguration("launch_visual_servo")
    perception_only_mode = LaunchConfiguration("perception_only_mode")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "vehicle_detection_topic",
                default_value="/darknet_ros_3d/vehicle_bounding_boxes",
                description="Live vehicle 3D boxes topic",
            ),
            DeclareLaunchArgument(
                "tire_detection_topic",
                default_value="/darknet_ros_3d/tire_bounding_boxes",
                description="Live tire 3D boxes topic",
            ),
            DeclareLaunchArgument(
                "vehicle_fallback_topic",
                default_value="/aurora_semantic/vehicle_bounding_boxes",
                description="Fallback vehicle boxes (e.g. Aurora semantic); empty to disable",
            ),
            DeclareLaunchArgument(
                "image_topic",
                default_value="/slamware_ros_sdk_server_node/left_image_raw",
                description="Live camera topic used for photo capture",
            ),
            DeclareLaunchArgument(
                "camera_info_topic",
                default_value="/camera/depth/camera_info",
                description="Camera info topic for metadata",
            ),
            DeclareLaunchArgument(
                "use_navigation_action",
                default_value="true",
                description="Use real NavigateToPose action (disable for integration simulation).",
            ),
            DeclareLaunchArgument(
                "launch_visual_servo",
                default_value="true",
                description="Launch built-in visual_servo align action server.",
            ),
            DeclareLaunchArgument(
                "perception_only_mode",
                default_value="false",
                description="Disable all movement (Nav2, visual_servo, cmd_vel). For perception validation only.",
            ),
            DeclareLaunchArgument(
                "save_directory",
                default_value="~/ugv_ws/tire_inspection_photos",
                description="Directory to save captured tire inspection photos",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(os.path.expanduser("~/ugv_ws"), "PRODUCTION_CONFIG.yaml"),
                description="Path to PRODUCTION_CONFIG for inspection_manager and photo_capture_service",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation time (true for rosbag replay with --clock).",
            ),
            DeclareLaunchArgument(
                "sensor_health_timeout",
                default_value="30.0",
                description="Seconds to wait for sensor health in INIT before proceeding to SEARCH_VEHICLE.",
            ),
            DeclareLaunchArgument(
                "require_nav_permitted",
                default_value="true",
                description="Gate goals on /stereo/navigation_permitted. Set false for headless runs without depth gate.",
            ),
            Node(
                package="inspection_manager",
                executable="photo_capture_service",
                name="photo_capture_service",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time", default="false"),
                        "camera_topic": image_topic,
                        "save_directory": LaunchConfiguration("save_directory"),
                    },
                ],
            ),
            Node(
                package="inspection_manager",
                executable="inspection_manager_node",
                name="inspection_manager",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "use_sim_time": LaunchConfiguration("use_sim_time", default="false"),
                        "detection_topic": tire_detection_topic,
                        "vehicle_detection_topic": vehicle_detection_topic,
                        "vehicle_boxes_topic": vehicle_fallback_topic,
                        "sensor_health_timeout": LaunchConfiguration("sensor_health_timeout", default="30.0"),
                        "require_nav_permitted": LaunchConfiguration("require_nav_permitted", default="true"),
                    },
                ],
            ),
            Node(
                package="inspection_manager",
                executable="visual_servo_align_server",
                name="visual_servo_align_server",
                output="screen",
                parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time", default="false")}],
                condition=IfCondition(
                    PythonExpression(
                        ["'", launch_visual_servo, "' == 'true' and '", perception_only_mode, "' != 'true'"]
                    )
                ),
            ),
        ]
    )
