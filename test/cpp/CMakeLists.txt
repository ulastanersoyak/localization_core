add_executable(tests test.cpp)
target_include_directories(tests PRIVATE ${CMAKE_SOURCE_DIR}/includes ${CMAKE_SOURCE_DIR}/sim/src ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(tests kalman utm xbus rmc time_and_location)
add_test(NAME tests COMMAND tests)

# The GPS class needs Boost.Asio, see src/CMakeLists.txt.
if(TARGET gps)
  target_link_libraries(tests gps)
  target_compile_definitions(tests PRIVATE HAVE_GPS)
endif()
