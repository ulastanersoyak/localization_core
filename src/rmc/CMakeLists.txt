add_library(rmc STATIC rmc.cpp)
target_link_libraries(rmc PUBLIC gnss_imu_fusion_headers time_and_location)
