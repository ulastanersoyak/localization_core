add_executable(tests test.cpp)
# check.hpp's directory (test/cpp) has to come after includes/: the test
# headers mirror the library paths (utm/utm.hpp, kalman/...).
target_link_libraries(tests PRIVATE utm localization_core_test_support)
add_test(NAME tests COMMAND tests)
