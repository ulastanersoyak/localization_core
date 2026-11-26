# Header only helpers the projects' simulation programs share: ros_sim.hpp
# (tf2 lookups in the enu frame, pose messages; needs ROS 2, the programs
# link rclcpp, tf2_ros and tf2_eigen themselves) and imu_errors.hpp (the AHRS
# error model, plain C++).
add_library(localization_core_sim INTERFACE)
target_include_directories(localization_core_sim
                           INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(localization_core_sim INTERFACE localization_core_headers)
