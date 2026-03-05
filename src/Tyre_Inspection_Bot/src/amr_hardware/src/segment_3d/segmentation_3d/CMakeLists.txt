cmake_minimum_required(VERSION 3.8)
project(segmentation_3d)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gb_visual_detection_3d_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
# pcl_ros is deprecated in ROS 2 Humble, pcl_conversions provides what we need
# find_package(pcl_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(segmentation_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL REQUIRED)

include_directories(
  include/${PROJECT_NAME}/
)

# Declare a C++ executable
add_executable(segmentation_processor_node
  src/segmentation_processor.cpp
  src/point_processor.cpp
)

ament_target_dependencies(segmentation_processor_node
  gb_visual_detection_3d_msgs
  pcl_conversions
  # pcl_ros removed - not needed, pcl_conversions sufficient
  rclcpp
  segmentation_msgs
  sensor_msgs
  std_msgs
  tf2_geometry_msgs
  tf2_ros
  message_filters
  visualization_msgs
  tf2_sensor_msgs
  PCL
)

# Install targets
install(TARGETS segmentation_processor_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python executables
install(PROGRAMS
  scripts/ultralytics_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME ultralytics_node
)
install(PROGRAMS
  scripts/ultralytics_node_cpu.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME ultralytics_node_cpu
)
install(PROGRAMS
  scripts/aurora_camera_info_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME aurora_camera_info_node
)
install(PROGRAMS
  scripts/aurora_depth_camera_info_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME aurora_depth_camera_info_node
)
install(PROGRAMS
  scripts/aurora_semantic_fusion_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME aurora_semantic_fusion_node
)
install(PROGRAMS
  scripts/depth_to_registered_pointcloud_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME depth_to_registered_pointcloud_node
)
install(PROGRAMS
  scripts/vehicle_boxes_marker_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME vehicle_boxes_marker_node
)
install(PROGRAMS
  scripts/print_inspection_model_classes.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME print_inspection_model_classes
)
install(PROGRAMS
  scripts/tire_detector_pcl_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME tire_detector_pcl_node
)
install(PROGRAMS
  scripts/tire_merger_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME tire_merger_node
)
install(PROGRAMS
  scripts/centroid_servo_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME centroid_servo_node
)
install(PROGRAMS
  scripts/simulated_detection_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME simulated_detection_node
)

# Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  PATTERN "*.launch" EXCLUDE
)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
