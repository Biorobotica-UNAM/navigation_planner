cmake_minimum_required(VERSION 3.8)
project(path_planner)

# =====================
# Compile options
# =====================
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# =====================
# Dependencies
# =====================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# =====================
# C++ Library
# =====================
add_library(path_planner_lib
  src/PathPlanner.cpp
)

target_include_directories(path_planner_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
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

target_link_libraries(path_planner_node
  path_planner_lib
)

ament_target_dependencies(path_planner_node
  rclcpp
  nav_msgs
  geometry_msgs
)

# =====================
# Install
# =====================
install(
  TARGETS
    path_planner_lib
    path_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(path_planner_lib)
ament_export_dependencies(
  rclcpp
  nav_msgs
  geometry_msgs
)

ament_package()
