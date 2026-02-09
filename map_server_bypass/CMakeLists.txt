cmake_minimum_required(VERSION 3.8)
project(map_server_bypass)

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
find_package(nav_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)

# =====================
# C++ Node
# =====================
add_executable(map_server_bypass_node
  src/map_server_bypass_node.cpp
)

ament_target_dependencies(map_server_bypass_node
  rclcpp
  nav_msgs
  rcl_interfaces
)

# =====================
# Install
# =====================
install(TARGETS map_server_bypass_node
  DESTINATION lib/${PROJECT_NAME}
)

# =====================
# Ament Package
# =====================
ament_package()
