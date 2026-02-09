cmake_minimum_required(VERSION 3.8)
project(path_planner)

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
find_package(rcl_interfaces REQUIRED)

# =====================
# C++ Library
# =====================
add_library(path_planner_lib
  src/PathPlanner.cpp
)

target_include_directories(path_planner_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(path_planner_lib
  rclcpp
  nav_msgs
  geometry_msgs
)

# =====================
# C++ Node
# =====================
add_executable(path_planner_node
  src/path_planner_node.cpp
)

target_link_libraries(path
