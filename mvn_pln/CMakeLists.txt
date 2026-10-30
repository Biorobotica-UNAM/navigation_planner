cmake_minimum_required(VERSION 3.8)
project(mvn_pln)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# =====================
# Compile options
# =====================
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =====================
# Dependencies
# =====================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(actionlib_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(rcl_interfaces REQUIRED)

# =====================
# C++ Node
# =====================
add_executable(mvn_pln_node
  src/mvn_pln_node.cpp
)

ament_target_dependencies(mvn_pln_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_srvs
  action_msgs
  actionlib_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  rcl_interfaces
)

# =====================
# Install
# =====================
install(TARGETS mvn_pln_node
  DESTINATION lib/${PROJECT_NAME}
)

# =====================
# Ament Package
# =====================
ament_package()
