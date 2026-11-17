# The header only parts (kalman_filter, the measurement types, queue) live
# under includes/; their linear algebra is Eigen's.
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
add_library(localization_core_headers INTERFACE)
target_include_directories(localization_core_headers
                           INTERFACE ${PROJECT_SOURCE_DIR}/includes)
target_compile_features(localization_core_headers INTERFACE cxx_std_20)
target_link_libraries(localization_core_headers INTERFACE Eigen3::Eigen)

add_subdirectory(utm)
