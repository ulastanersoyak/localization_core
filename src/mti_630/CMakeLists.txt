add_library(xbus STATIC data_packet_parser.cpp xbus_packet.cpp)
target_link_libraries(xbus PUBLIC gnss_imu_fusion_headers time_and_location)

if(TARGET serial_port)
  add_library(mti_630 STATIC mti_630.cpp)
  target_link_libraries(mti_630 PUBLIC gnss_imu_fusion_headers xbus serial_port)
endif()
