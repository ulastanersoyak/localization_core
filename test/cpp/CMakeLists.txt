add_executable(tests test.cpp)
target_link_libraries(tests PRIVATE gnss_imu_fusion_test_support utm xbus rmc
                                    time_and_location)
add_test(NAME tests COMMAND tests)

# The GPS class needs Boost.Asio, see src/CMakeLists.txt.
if(TARGET gps)
  target_link_libraries(tests PRIVATE gps)
  target_compile_definitions(tests PRIVATE HAVE_GPS)
endif()
