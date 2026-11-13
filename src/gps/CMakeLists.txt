add_library(gps STATIC gps.cpp)
target_link_libraries(gps PUBLIC localization_core_headers utm rmc serial_port)
