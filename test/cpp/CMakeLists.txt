# GoogleTest: each TEST is its own ctest test (gtest_discover_tests).
find_package(GTest REQUIRED)
include(GoogleTest)
add_executable(tests utm_test.cpp kalman_test.cpp)
target_link_libraries(tests PRIVATE utm localization_core_headers GTest::gtest_main)
gtest_discover_tests(tests)
