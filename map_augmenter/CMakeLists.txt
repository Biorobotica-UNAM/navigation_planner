cmake_minimum_required(VERSION 3.8)
project(map_augmenter)  # <- pon aquí el nombre real de tu paquete si es otro

# =====================
# Compile options
# =====================
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ===== Dependencies =====
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)

find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(Eigen3 REQUIRED)

# ===== Executables =====

# 1) Map augmenter + merger
add_executable(map_augmenter_node
  src/map_augmenter_node.cpp
)
ament_target_dependencies(map_augmenter_node
  rclcpp
  rcl_interfaces
  nav_msgs
  geometry_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
)
target_link_libraries(map_augmenter_node Eigen3::Eigen)

# 2) Obstacle detector
add_executable(obstacle_detector_node
  src/obstacle_detector_node.cpp
)
ament_target_dependencies(obstacle_detector_node
  rclcpp
  rcl_interfaces
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
)
target_link_libraries(obstacle_detector_node Eigen3::Eigen)

# ===== Install =====
install(TARGETS
  map_augmenter_node
  obstacle_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
