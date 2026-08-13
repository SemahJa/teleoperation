cmake_minimum_required(VERSION 3.8)
project(panda_bringup)
find_package(ament_cmake REQUIRED)
find_package(Franka REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(franka_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(cartesian_impedance_controller src/cartesian_impedance_controller.cpp)
ament_target_dependencies(cartesian_impedance_controller Franka franka_msgs geometry_msgs rclcpp sensor_msgs)
target_link_libraries(cartesian_impedance_controller Eigen3::Eigen)

install(TARGETS cartesian_impedance_controller
	DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
ament_package()
