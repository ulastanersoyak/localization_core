# GeographicLib does the conversions (Fedora: GeographicLib-devel, Ubuntu:
# libgeographiclib-dev). Ubuntu ships a find module, Fedora a config file.
list(APPEND CMAKE_MODULE_PATH /usr/share/cmake/geographiclib)
find_package(GeographicLib REQUIRED)

add_library(utm STATIC utm.cpp)
target_include_directories(utm SYSTEM PUBLIC ${GeographicLib_INCLUDE_DIRS})
target_link_libraries(utm PUBLIC localization_core_headers
                              ${GeographicLib_LIBRARIES})
