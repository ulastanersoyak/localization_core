add_library(serial_port STATIC serial_port.cpp)
target_link_libraries(serial_port PUBLIC localization_core_headers Boost::headers
                                         Threads::Threads)
