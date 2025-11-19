cmake_minimum_required(VERSION 3.5)
project(f450_drone_bridge)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)


# Include directories
include_directories(
  include
)

# ============================================
# Executables
# ============================================

# Path node
add_executable(path_node 
  src/path_node.cpp
)
ament_target_dependencies(path_node 
  rclcpp 
  geometry_msgs 
  nav_msgs 
  tf2_ros
)

# Attitude to TF node
add_executable(attitude_to_tf_node
  src/attitude_to_tf.cpp
)
ament_target_dependencies(attitude_to_tf_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
)

# Cardinal markers node
add_executable(cardinal_markers_node
  src/cardinal_markers.cpp
)
ament_target_dependencies(cardinal_markers_node
  rclcpp
  visualization_msgs
)

# roll correction for attitude display plugin bug 
add_executable(imu_roll_corrector_node
 src/imu_roll_corrector.cpp
 )

 ament_target_dependencies(imu_roll_corrector_node
  rclcpp
  sensor_msgs
  tf2)


# ============================================
# Installation
# ============================================

# Install executables
install(TARGETS 
  path_node
  attitude_to_tf_node
  cardinal_markers_node
  imu_roll_corrector_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY
  include/
  DESTINATION include
)

# Install launch files
install(DIRECTORY 
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py"
)

ament_package()