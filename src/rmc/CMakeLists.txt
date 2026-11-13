add_library(rmc STATIC rmc.cpp)
target_link_libraries(rmc PUBLIC localization_core_headers time_and_location)
