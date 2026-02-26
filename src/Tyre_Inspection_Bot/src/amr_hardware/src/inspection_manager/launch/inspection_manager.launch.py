import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _inspection_manager_nodes(context):
    """Build inspection_manager and photo_capture_service nodes; load PRODUCTION_CONFIG if path given."""
    config_file = context.perform_substitution(LaunchConfiguration("config_file", default=""))
    # If config_file empty, use workspace PRODUCTION_CONFIG.yaml (Aurora 2.11 semantic)
    if not config_file or not os.path.isfile(config_file):
        for prefix in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep):
            if prefix:
                ws_root = os.path.dirname(prefix)
                default_cfg = os.path.join(ws_root, "PRODUCTION_CONFIG.yaml")
                if os.path.isfile(default_cfg):
                    config_file = default_cfg
                    break
                # When running from install space, config is often in source root (parent of install)
                if os.path.basename(ws_root) == "install":
                    parent_root = os.path.dirname(ws_root)
                    default_cfg = os.path.join(parent_root, "PRODUCTION_CONFIG.yaml")
                    if os.path.isfile(default_cfg):
                        config_file = default_cfg
                        break
    params = []
    if config_file and os.path.isfile(config_file):
        params.append(config_file)
    params.append({
        "vehicle_labels": LaunchConfiguration("vehicle_labels"),
        "tire_label": LaunchConfiguration("tire_label"),
        "detection_topic": LaunchConfiguration("detection_topic"),
        "vehicle_boxes_topic": LaunchConfiguration("vehicle_boxes_topic"),
        "use_dynamic_detection": LaunchConfiguration("use_dynamic_detection"),
        "approach_offset": LaunchConfiguration("approach_offset"),
        "tire_offset": LaunchConfiguration("tire_offset"),
        "dry_run": LaunchConfiguration("dry_run"),
    })
    return [
        Node(
            package="inspection_manager",
            executable="inspection_manager_node",
            name="inspection_manager",
            output="screen",
            parameters=params,
        ),
        Node(
            package="inspection_manager",
            executable="photo_capture_service",
            name="photo_capture_service",
            output="screen",
            parameters=params if (config_file and os.path.isfile(config_file)) else [params[-1]],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value="",
            description="Path to PRODUCTION_CONFIG.yaml (optional); params from file are overridden by launch args.",
        ),
        DeclareLaunchArgument(
            "vehicle_labels",
            default_value="car,truck,bus",
            description="Comma-separated vehicle class names",
        ),
        DeclareLaunchArgument(
            "tire_label",
            default_value="car-tire",
            description="Tire class name (must match model output and segment_3d interested_classes)",
        ),
        DeclareLaunchArgument(
            "detection_topic",
            default_value="/darknet_ros_3d/bounding_boxes",
            description="3D bounding boxes topic (tires + vehicles when vehicle_boxes_topic empty)",
        ),
        DeclareLaunchArgument(
            "vehicle_boxes_topic",
            default_value="/aurora_semantic/vehicle_bounding_boxes",
            description="Semantic vehicle boxes (Aurora 2.11). When set, vehicles from this topic; tires from detection_topic. Empty string = use YOLO vehicles from detection_topic.",
        ),
        DeclareLaunchArgument(
            "use_dynamic_detection",
            default_value="true",
            description="Use dynamic detection from bounding boxes",
        ),
        DeclareLaunchArgument(
            "approach_offset",
            default_value="0.5",
            description="Offset when approaching vehicle (meters)",
        ),
        DeclareLaunchArgument(
            "tire_offset",
            default_value="0.4",
            description="Offset when approaching tire (meters)",
        ),
        DeclareLaunchArgument(
            "dry_run",
            default_value="false",
            description="If true, log Nav2 goals but do not send; for validation without motion.",
        ),
        OpaqueFunction(function=_inspection_manager_nodes),
    ])
