from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    vehicle_detection_topic = LaunchConfiguration("vehicle_detection_topic")
    tire_detection_topic = LaunchConfiguration("tire_detection_topic")
    image_topic = LaunchConfiguration("image_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    use_navigation_action = LaunchConfiguration("use_navigation_action")
    launch_visual_servo = LaunchConfiguration("launch_visual_servo")
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
            Node(
                package="inspection_manager",
                executable="photo_capture_api",
                name="photo_capture_api",
                output="screen",
                parameters=[
                    {
                        "image_topic": image_topic,
                        "camera_info_topic": camera_info_topic,
                    }
                ],
            ),
            Node(
                package="inspection_manager",
                executable="manager_node",
                name="inspection_manager",
                output="screen",
                parameters=[
                    {
                        "vehicle_detection_topic": vehicle_detection_topic,
                        "tire_detection_topic": tire_detection_topic,
                        "use_navigation_action": use_navigation_action,
                    }
                ],
            ),
            Node(
                package="inspection_manager",
                executable="visual_servo_align_server",
                name="visual_servo_align_server",
                output="screen",
                condition=IfCondition(launch_visual_servo),
            ),
        ]
    )
