cmake_minimum_required(VERSION 3.16)
project(zhao_lio_ros2 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# A conda yaml-cpp on CMAKE_PREFIX_PATH links rosbag2 against the wrong ABI; prefer the system one ROS uses.
if(NOT DEFINED yaml-cpp_DIR AND EXISTS /usr/lib/x86_64-linux-gnu/cmake/yaml-cpp)
  set(yaml-cpp_DIR /usr/lib/x86_64-linux-gnu/cmake/yaml-cpp)
endif()

find_package(ament_cmake REQUIRED)
find_package(zhao_lio_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(livox_ros_driver2 QUIET)

set(ROS_DEPS rclcpp sensor_msgs nav_msgs geometry_msgs tf2_ros)
if(livox_ros_driver2_FOUND)
  message(STATUS "livox_ros_driver2 found: CustomMsg input enabled")
  list(APPEND ROS_DEPS livox_ros_driver2)
  add_compile_definitions(HAVE_LIVOX_DRIVER)
else()
  message(WARNING "livox_ros_driver2 not found: only PointCloud2 input is available")
endif()

add_executable(zhao_lio_node src/zhao_lio_node.cpp)
target_include_directories(zhao_lio_node PRIVATE include)
target_link_libraries(zhao_lio_node zhao_lio_core::zhao_lio_core)
ament_target_dependencies(zhao_lio_node ${ROS_DEPS})

find_package(rosbag2_cpp REQUIRED)
add_executable(zhao_lio_replay src/replay.cpp)
target_include_directories(zhao_lio_replay PRIVATE include)
target_link_libraries(zhao_lio_replay zhao_lio_core::zhao_lio_core)
ament_target_dependencies(zhao_lio_replay ${ROS_DEPS} rosbag2_cpp)


install(TARGETS zhao_lio_node zhao_lio_replay DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
ament_package()
