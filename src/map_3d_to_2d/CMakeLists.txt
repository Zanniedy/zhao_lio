cmake_minimum_required(VERSION 3.8)
project(map_3d_to_2d)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
if(NOT DEFINED yaml-cpp_DIR AND EXISTS /usr/lib/x86_64-linux-gnu/cmake/yaml-cpp)
  set(yaml-cpp_DIR /usr/lib/x86_64-linux-gnu/cmake/yaml-cpp)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

# PCL (IMPORTANT)
find_package(PCL REQUIRED)

# -------------------------------
# Include directories
# -------------------------------
include_directories(include ${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# -------------------------------
# Executable
# -------------------------------
add_executable(mapper src/mapper.cpp)

ament_target_dependencies(
  mapper
  rclcpp
  sensor_msgs
  nav_msgs
  pcl_conversions
  pcl_ros
  tf2
  tf2_ros
  std_msgs geometry_msgs visualization_msgs)

# Link PCL explicitly
target_link_libraries(mapper ${PCL_LIBRARIES})

set(CMAKE_AUTOMOC ON)
add_library(map_3d_to_2d_panel SHARED
  include/map_3d_to_2d/rviz_panel.hpp
  src/rviz_panel.cpp)
target_include_directories(map_3d_to_2d_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(map_3d_to_2d_panel Qt5::Widgets)
ament_target_dependencies(map_3d_to_2d_panel
  pluginlib rclcpp rviz_common std_msgs geometry_msgs)

pluginlib_export_plugin_description_file(rviz_common map_3d_to_2d_plugin.xml)

# -------------------------------
# Install
# -------------------------------
install(TARGETS mapper DESTINATION lib/${PROJECT_NAME})
install(TARGETS map_3d_to_2d_panel
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})
install(FILES map_3d_to_2d_plugin.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights comment the
  # line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo) comment the line
  # when this package is in a git repo and when a copyright and license is added
  # to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
