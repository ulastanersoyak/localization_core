# Every library and header only part (matrix, kalman_filter, queue, ...)
# lives under includes/.
add_library(localization_core_headers INTERFACE)
target_include_directories(localization_core_headers
                           INTERFACE ${PROJECT_SOURCE_DIR}/includes)
target_compile_features(localization_core_headers INTERFACE cxx_std_20)

# The serial port based parts need Boost.Asio, everything else builds without
# it so the tests can run on any machine. Prefer the BoostConfig.cmake shipped
# by Boost itself, fall back to CMake's FindBoost module for header only
# installs. Users check `if(TARGET gps)` / `if(TARGET mti_630)`.
find_package(Boost 1.70 CONFIG QUIET)
if(NOT Boost_FOUND)
  find_package(Boost 1.70 MODULE QUIET)
endif()

add_subdirectory(time_and_location)
add_subdirectory(rmc)
add_subdirectory(utm)

if(Boost_FOUND)
  add_subdirectory(utility)
  add_subdirectory(gps)
else()
  message(STATUS "localization_core: Boost not found, skipping the serial port, gps and mti_630 libraries")
endif()

add_subdirectory(mti_630)
