add_library(utm STATIC utm.cpp)
target_link_libraries(utm PUBLIC gnss_imu_fusion_headers rmc time_and_location)
