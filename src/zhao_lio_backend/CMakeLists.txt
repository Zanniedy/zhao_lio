cmake_minimum_required(VERSION 3.16)
project(zhao_lio_backend LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(zhao_lio_core REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(TBB REQUIRED)

file(GLOB BACKEND_SOURCES CONFIGURE_DEPENDS src/*.cpp)
add_library(zhao_lio_backend SHARED ${BACKEND_SOURCES})
target_include_directories(zhao_lio_backend PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(zhao_lio_backend PUBLIC zhao_lio_core::zhao_lio_core Eigen3::Eigen TBB::tbb)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  file(GLOB TEST_SOURCES CONFIGURE_DEPENDS test/test_*.cpp)
  foreach(test_src ${TEST_SOURCES})
    get_filename_component(test_name ${test_src} NAME_WE)
    ament_add_gtest(backend_${test_name} ${test_src} TIMEOUT 300)
    target_link_libraries(backend_${test_name} zhao_lio_backend)
  endforeach()
endif()

install(DIRECTORY include/ DESTINATION include)
install(TARGETS zhao_lio_backend EXPORT zhao_lio_backendTargets
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)
ament_export_targets(zhao_lio_backendTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(zhao_lio_core Eigen3 TBB)
ament_package()
