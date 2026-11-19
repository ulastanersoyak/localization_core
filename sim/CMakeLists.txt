# Header only helpers the projects' simulation programs share (ros_sim.hpp:
# tf2 lookups in the enu frame, pose messages). Only usable where ROS 2 is
# sourced; the programs link rclcpp, tf2_ros and tf2_eigen themselves.
add_library(localization_core_sim INTERFACE)
target_include_directories(localization_core_sim
                           INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/src)
