cmake_minimum_required(VERSION 3.8)
project(simple_move)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "CMake build type" FORCE)
endif()

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
find_package(path_planner REQUIRED)
find_package(action_msgs REQUIRED)
find_package(actionlib_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(rcl_interfaces REQUIRED)

# =====================
# C++ Node
# =====================
add_executable(simple_move_node
  src/simple_move_node.cpp
)
target_include_directories(simple_move_node PRIVATE include)

ament_target_dependencies(simple_move_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  path_planner
  action_msgs
  actionlib_msgs
  tf2
  tf2_ros
  rcl_interfaces
)

# =====================
# Install Targets
# =====================
install(TARGETS simple_move_node
  DESTINATION lib/${PROJECT_NAME}
)

# =====================
# Ament Package
# =====================
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_tracking_reference test/test_tracking_reference.cpp)
  target_include_directories(test_tracking_reference PRIVATE include)
endif()

ament_package()
