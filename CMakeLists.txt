cmake_minimum_required(VERSION 3.22)

project(gnss_imu_fusion VERSION 0.1.0 LANGUAGES CXX)

include(cmake/common.cmake)

find_package(Threads REQUIRED)

add_subdirectory(src)

# check.hpp, the tiny test helper the other repositories' tests use too.
add_library(gnss_imu_fusion_test_support INTERFACE)
target_include_directories(gnss_imu_fusion_test_support
                           INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/test/cpp)

option(GNSS_IMU_FUSION_BUILD_TESTS "Build the gnss-imu-fusion unit tests"
       ${PROJECT_IS_TOP_LEVEL})
if(GNSS_IMU_FUSION_BUILD_TESTS)
  enable_testing()
  add_subdirectory(test/cpp)
endif()

# Simulation helpers and the Gazebo accuracy overlay plugin.
add_subdirectory(sim)
