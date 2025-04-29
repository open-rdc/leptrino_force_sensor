cmake_minimum_required(VERSION 3.5)
project(leptrino_force_sensor)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(leptrino_force_sensor_node
  src/leptrino_force_sensor_node.cpp
)

ament_target_dependencies(leptrino_force_sensor_node
  rclcpp
  geometry_msgs
)

install(TARGETS
  leptrino_force_sensor_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
