cmake_minimum_required(VERSION 3.8)
project(quest_teleop_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/QuestButtonState.msg"
  "msg/QuestControllerState.msg"
  "msg/CartesianTeleopCommand.msg"
  "msg/ControllerStatus.msg"
  "msg/GripperStatus.msg"
  "msg/HomingStatus.msg"
  "msg/CbfStatus.msg"
  "srv/CalibrateGripper.srv"
  DEPENDENCIES std_msgs geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
