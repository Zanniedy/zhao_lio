cmake_minimum_required(VERSION 3.16)
project(dynablox_ros2 LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(glog REQUIRED)
find_package(Protobuf REQUIRED)
find_package(PCL REQUIRED COMPONENTS common kdtree search segmentation)
find_package(Threads REQUIRED)

set(VOXBLOX ${CMAKE_CURRENT_SOURCE_DIR}/third_party/voxblox/voxblox)
set(DYNABLOX ${CMAKE_CURRENT_SOURCE_DIR}/third_party/dynablox/dynablox)
set(PROTO_SOURCES ${CMAKE_CURRENT_BINARY_DIR}/voxblox/Block.pb.cc ${CMAKE_CURRENT_BINARY_DIR}/voxblox/Layer.pb.cc)
add_custom_command(OUTPUT ${PROTO_SOURCES} ${CMAKE_CURRENT_BINARY_DIR}/voxblox/Block.pb.h ${CMAKE_CURRENT_BINARY_DIR}/voxblox/Layer.pb.h
  COMMAND protobuf::protoc --cpp_out=${CMAKE_CURRENT_BINARY_DIR} -I${VOXBLOX}/proto
    ${VOXBLOX}/proto/voxblox/Block.proto ${VOXBLOX}/proto/voxblox/Layer.proto
  DEPENDS ${VOXBLOX}/proto/voxblox/Block.proto ${VOXBLOX}/proto/voxblox/Layer.proto)
add_library(dynablox_voxblox STATIC ${PROTO_SOURCES}
  ${VOXBLOX}/src/core/block.cc
  ${VOXBLOX}/src/integrator/integrator_utils.cc
  ${VOXBLOX}/src/integrator/tsdf_integrator.cc
  ${VOXBLOX}/src/utils/timing.cc
  ${VOXBLOX}/src/utils/neighbor_tools.cc
  ${VOXBLOX}/src/utils/protobuf_utils.cc
  ${VOXBLOX}/src/utils/voxel_utils.cc)
target_include_directories(dynablox_voxblox PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/compat ${VOXBLOX}/include ${CMAKE_CURRENT_BINARY_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/minkindr/minkindr/include)
target_link_libraries(dynablox_voxblox PUBLIC Eigen3::Eigen glog::glog protobuf::libprotobuf Threads::Threads)
add_library(dynablox_official STATIC
  ${DYNABLOX}/src/processing/clustering.cpp
  ${DYNABLOX}/src/processing/tracking.cpp
  ${DYNABLOX}/src/processing/ever_free_integrator.cpp)
target_include_directories(dynablox_official PUBLIC compat include ${DYNABLOX}/include ${PCL_INCLUDE_DIRS})
target_link_libraries(dynablox_official PUBLIC dynablox_voxblox ${PCL_LIBRARIES})
add_library(dynablox_adapter STATIC src/adapter.cpp)
target_link_libraries(dynablox_adapter PUBLIC dynablox_official)
add_executable(dynablox_node src/node.cpp)
target_link_libraries(dynablox_node dynablox_adapter)
ament_target_dependencies(dynablox_node rclcpp sensor_msgs std_msgs tf2_ros visualization_msgs)
install(TARGETS dynablox_node RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})
install(FILES README.md SOURCE.md SOURCE.sha256 DESTINATION share/${PROJECT_NAME})
install(FILES third_party/dynablox/LICENSE DESTINATION share/${PROJECT_NAME}/licenses/dynablox)
install(FILES third_party/voxblox/LICENSE DESTINATION share/${PROJECT_NAME}/licenses/voxblox)
install(FILES third_party/minkindr/LICENSE DESTINATION share/${PROJECT_NAME}/licenses/minkindr)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_dynablox test/test_dynablox.cpp)
  target_link_libraries(test_dynablox dynablox_adapter)
endif()
ament_package()
