add_library(gps STATIC gps.cpp)
target_link_libraries(gps PUBLIC gnss_imu_fusion_headers utm rmc serial_port)
