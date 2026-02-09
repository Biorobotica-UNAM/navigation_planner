cmake_minimum_required(VERSION 3.8)
project(augment_gridmap_online)

# Compile options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =====================
# Dependencies
# =====================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rcl_interfaces REQUIRED)

# =====================
# C++ Node
# =====================
add_executable(augment_gridmap_online_node
  src/augment_gridmap_online_node.cpp
)

ament_target_dependencies(augment_gridmap_online_node
  rclcpp
  geometry_msgs
  nav_msgs
  visualization_msgs
  std_srvs
  rcl_interfaces
)

# =====================
# Install
# =====================
install(TARGETS augment_gridmap_online_node
  DESTINATION lib/${PROJECT_NAME}
)

# =====================
# Ament package
# =====================
ament_package()
