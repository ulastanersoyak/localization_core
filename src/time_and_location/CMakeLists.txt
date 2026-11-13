add_library(time_and_location STATIC utc.cpp parser.cpp coordinate.cpp date.cpp)
target_link_libraries(time_and_location PUBLIC gnss_imu_fusion_headers)
