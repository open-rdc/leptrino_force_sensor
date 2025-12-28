cmake_minimum_required(VERSION 3.5)
project(leptrino_force_sensor)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(leptrino_force_sensor_node
  src/leptrino_force_sensor_node.cpp
)

ament_target_dependencies(leptrino_force_sensor_node
  rclcpp
  geometry_msgs
  std_srvs
)

install(TARGETS
  leptrino_force_sensor_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
