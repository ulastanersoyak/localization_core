add_executable(tests test.cpp)
# check.hpp's directory (test/cpp) has to come after includes/: the test
# headers mirror the library paths (gps/gps.hpp includes the real gps/gps.hpp).
target_link_libraries(tests PRIVATE utm xbus rmc time_and_location
                                    localization_core_test_support)
add_test(NAME tests COMMAND tests)

# The GPS class needs Boost.Asio, see src/CMakeLists.txt.
if(TARGET gps)
  target_link_libraries(tests PRIVATE gps)
  target_compile_definitions(tests PRIVATE HAVE_GPS)
endif()
