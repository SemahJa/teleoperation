cmake_minimum_required(VERSION 3.8)
project(quest_teleop)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(quest_teleop_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nlohmann_json REQUIRED)

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS quest_teleop/pose_mapper.py DESTINATION lib/${PROJECT_NAME} RENAME pose_mapper)
install(PROGRAMS quest_teleop/mock_measured_pose.py DESTINATION lib/${PROJECT_NAME} RENAME mock_measured_pose)

add_executable(quest_usb_bridge src/quest_usb_bridge.cpp)
ament_target_dependencies(quest_usb_bridge rclcpp quest_teleop_interfaces)
target_link_libraries(quest_usb_bridge nlohmann_json::nlohmann_json)

install(TARGETS quest_usb_bridge DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_pose_mapper test/test_pose_mapper.py)
  ament_add_pytest_test(test_bridge_mapper_integration test/test_bridge_mapper_integration.py TIMEOUT 60)
  ament_add_pytest_test(test_configuration test/test_configuration.py)
endif()

ament_package()
