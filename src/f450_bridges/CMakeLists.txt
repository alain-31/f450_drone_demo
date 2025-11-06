cmake_minimum_required(VERSION 3.8)
project(f450_bridges)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(path_node src/path_node.cpp)
ament_target_dependencies(path_node rclcpp geometry_msgs nav_msgs tf2 tf2_ros)

install(TARGETS path_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
