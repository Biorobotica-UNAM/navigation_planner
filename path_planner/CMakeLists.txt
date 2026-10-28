cmake_minimum_required(VERSION 3.8)
project(path_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pumas_interfaces REQUIRED)

add_library(path_planner_lib src/PathPlanner.cpp)
target_include_directories(path_planner_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(path_planner_lib nav_msgs geometry_msgs)

add_executable(path_planner_node src/path_planner_node.cpp)
target_link_libraries(path_planner_node path_planner_lib)
ament_target_dependencies(path_planner_node
  rclcpp rcl_interfaces nav_msgs geometry_msgs pumas_interfaces)

install(TARGETS path_planner_lib
  EXPORT export_path_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS path_planner_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_path_planner HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(path_planner_lib)
ament_export_dependencies(nav_msgs geometry_msgs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_path_planner test/test_path_planner.cpp)
  target_link_libraries(test_path_planner path_planner_lib)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME test_path_planner_node
    COMMAND "${Python3_EXECUTABLE}" "${CMAKE_CURRENT_SOURCE_DIR}/test/test_node.py"
      "$<TARGET_FILE:path_planner_node>")
  set_tests_properties(test_path_planner_node PROPERTIES TIMEOUT 60 RUN_SERIAL TRUE)
endif()

ament_package()
