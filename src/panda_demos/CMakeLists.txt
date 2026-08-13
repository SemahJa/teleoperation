cmake_minimum_required(VERSION 3.8)
project(panda_demos)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)

add_executable(cartesian_circle_publisher src/cartesian_circle_publisher.cpp)
ament_target_dependencies(cartesian_circle_publisher geometry_msgs rclcpp visualization_msgs)
install(TARGETS cartesian_circle_publisher DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_package()
