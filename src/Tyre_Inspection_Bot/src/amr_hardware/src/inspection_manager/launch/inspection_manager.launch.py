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
                default_value=os.path.join(
                    os.environ.get("UGV_WS", os.path.expanduser("~/ugv_ws")),
                    "PRODUCTION_CONFIG.yaml",
                ),
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
            DeclareLaunchArgument(
                "use_tyre_3d_positions",
                default_value="false",
                description="When true, mission can navigate from /tyre_3d_positions (tyre_3d_projection_node) instead of vehicle boxes.",
            ),
            DeclareLaunchArgument(
                "require_detection_topic_at_startup",
                default_value="true",
                description="If false, startup does not wait for tire merge topic (use with tyre_3d only / minimal perception).",
            ),
            DeclareLaunchArgument(
                "demo_mode",
                default_value="false",
                description="Bypass photo distance gates (stub motor / no-motion thesis demo).",
            ),
            DeclareLaunchArgument(
                "demo_simulate_nav_success_topic",
                default_value="",
                description="Subscribe to std_msgs/Empty on this topic → synthetic Nav2 arrival in INSPECT_TIRE when demo_mode (e.g. /navigation_success).",
            ),
            DeclareLaunchArgument(
                "use_tyre_geometry",
                default_value="true",
                description="Infer vehicle axes and visit order from /tyre_3d_positions (stable) instead of jittery vehicle box when possible.",
            ),
            DeclareLaunchArgument(
                "use_batch_waypoints",
                default_value="false",
                description="Use Nav2 FollowWaypoints with full map-frame approach list (perimeter+standoff per tyre).",
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
                        "use_tyre_3d_positions": LaunchConfiguration(
                            "use_tyre_3d_positions", default="false"
                        ),
                        "require_detection_topic_at_startup": LaunchConfiguration(
                            "require_detection_topic_at_startup", default="true"
                        ),
                        "demo_mode": LaunchConfiguration("demo_mode", default="false"),
                        "demo_simulate_nav_success_topic": LaunchConfiguration(
                            "demo_simulate_nav_success_topic", default=""
                        ),
                        "use_tyre_geometry": LaunchConfiguration("use_tyre_geometry", default="true"),
                        "use_batch_waypoints": LaunchConfiguration(
                            "use_batch_waypoints", default="false"
                        ),
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
