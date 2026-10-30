cmake_minimum_required(VERSION 3.16)
project(zhao_lio_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(TBB REQUIRED)

file(GLOB_RECURSE CORE_SOURCES CONFIGURE_DEPENDS src/*.cpp)
add_library(zhao_lio_core SHARED ${CORE_SOURCES})
target_include_directories(zhao_lio_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(zhao_lio_core PUBLIC Eigen3::Eigen TBB::tbb)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  file(GLOB TEST_SOURCES CONFIGURE_DEPENDS test/test_*.cpp)
  foreach(test_src ${TEST_SOURCES})
    get_filename_component(test_name ${test_src} NAME_WE)
    ament_add_gtest(${test_name} ${test_src} TIMEOUT 300)
    target_link_libraries(${test_name} zhao_lio_core)
  endforeach()
endif()

file(GLOB BENCH_SOURCES CONFIGURE_DEPENDS bench/*.cpp)
foreach(bench_src ${BENCH_SOURCES})
  get_filename_component(bench_name ${bench_src} NAME_WE)
  add_executable(${bench_name} ${bench_src})
  target_link_libraries(${bench_name} zhao_lio_core)
endforeach()

install(DIRECTORY include/ third_party/ DESTINATION include)
install(TARGETS zhao_lio_core EXPORT zhao_lio_coreTargets
  LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin
  INCLUDES DESTINATION include)
ament_export_targets(zhao_lio_coreTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 TBB)
ament_package()
