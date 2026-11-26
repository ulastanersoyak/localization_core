cmake_minimum_required(VERSION 3.22)

project(localization_core VERSION 0.1.0 LANGUAGES CXX)

include(cmake/common.cmake)

add_subdirectory(src)
option(LOCALIZATION_CORE_BUILD_TESTS "Build the localization_core unit tests"
       ${PROJECT_IS_TOP_LEVEL})
if(LOCALIZATION_CORE_BUILD_TESTS)
  enable_testing()
  add_subdirectory(test/cpp)
endif()

# Header only helpers the projects' simulation programs share.
add_subdirectory(sim)
