cmake_minimum_required(VERSION 3.8)
project(inspection_waypoint_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(photo_capture_waypoint_task SHARED src/photo_capture_waypoint_task.cpp)
target_include_directories(photo_capture_waypoint_task PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(photo_capture_waypoint_task
  rclcpp rclcpp_lifecycle nav2_core nav2_util pluginlib std_msgs geometry_msgs)
pluginlib_export_plugin_description_file(nav2_waypoint_follower plugins.xml)

install(TARGETS photo_capture_waypoint_task
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(FILES plugins.xml DESTINATION share/${PROJECT_NAME})

ament_export_dependencies(rclcpp rclcpp_lifecycle nav2_core nav2_util pluginlib std_msgs geometry_msgs)
ament_export_libraries(photo_capture_waypoint_task)
ament_package()
