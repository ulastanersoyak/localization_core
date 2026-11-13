add_library(serial_port STATIC serial_port.cpp)
target_link_libraries(serial_port PUBLIC gnss_imu_fusion_headers Boost::headers
                                         Threads::Threads)
