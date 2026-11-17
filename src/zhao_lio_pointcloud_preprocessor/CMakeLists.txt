cmake_minimum_required(VERSION 3.16)
project(zhao_lio_pointcloud_preprocessor LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_path(NANOFLANN_INCLUDE_DIR nanoflann.hpp REQUIRED)

add_library(zhao_lio_pointcloud_optimizer src/optimizer.cpp src/dynamic_filter.cpp src/cluster_tracker.cpp)
target_include_directories(zhao_lio_pointcloud_optimizer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(zhao_lio_pointcloud_optimizer PRIVATE ${NANOFLANN_INCLUDE_DIR})
target_link_libraries(zhao_lio_pointcloud_optimizer Eigen3::Eigen)
ament_target_dependencies(zhao_lio_pointcloud_optimizer sensor_msgs)

add_executable(zhao_lio_pointcloud_preprocessor_node src/node.cpp)
target_link_libraries(zhao_lio_pointcloud_preprocessor_node zhao_lio_pointcloud_optimizer)
ament_target_dependencies(zhao_lio_pointcloud_preprocessor_node rclcpp sensor_msgs std_msgs tf2_ros visualization_msgs)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS zhao_lio_pointcloud_optimizer zhao_lio_pointcloud_preprocessor_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_optimizer test/test_optimizer.cpp)
  target_link_libraries(test_optimizer zhao_lio_pointcloud_optimizer)
  ament_target_dependencies(test_optimizer sensor_msgs)
  ament_add_gtest(test_dynamic_filter test/test_dynamic_filter.cpp)
  target_link_libraries(test_dynamic_filter zhao_lio_pointcloud_optimizer)
  ament_target_dependencies(test_dynamic_filter sensor_msgs)
  ament_add_gtest(test_cluster_tracker test/test_cluster_tracker.cpp)
  target_link_libraries(test_cluster_tracker zhao_lio_pointcloud_optimizer)
endif()

ament_export_include_directories(include)
ament_export_dependencies(sensor_msgs Eigen3)
ament_export_libraries(zhao_lio_pointcloud_optimizer)
ament_package()
