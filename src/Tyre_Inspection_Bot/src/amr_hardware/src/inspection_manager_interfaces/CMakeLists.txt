cmake_minimum_required(VERSION 3.8)
project(inspection_manager_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MissionStatus.msg"
  "srv/StartMission.srv"
  "srv/CapturePhoto.srv"
  "action/ApproachTire.action"
  "action/AlignTire.action"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
