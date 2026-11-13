# Header only helpers for the Gazebo bridges (local_frame, error_stats).
add_library(gnss_imu_fusion_sim INTERFACE)
target_include_directories(gnss_imu_fusion_sim
                           INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/src)

# GUI overlay showing the estimate accuracy on the Gazebo screen. It is built
# into <build>/gz_gui_plugins, point GZ_GUI_PLUGIN_PATH there. The shared GUI
# layout is sim/gui.config.
find_package(gz-transport13 QUIET)
find_package(gz-msgs10 QUIET)
find_package(gz-gui8 QUIET)
find_package(Qt5 COMPONENTS Core Quick QUIET)
if(gz-transport13_FOUND AND gz-msgs10_FOUND AND gz-gui8_FOUND AND Qt5_FOUND)
  qt5_add_resources(accuracy_overlay_rcc gui_plugin/AccuracyOverlay.qrc)
  add_library(AccuracyOverlay SHARED gui_plugin/AccuracyOverlay.cc
                                     gui_plugin/AccuracyOverlay.hh
                                     ${accuracy_overlay_rcc})
  set_target_properties(AccuracyOverlay PROPERTIES
    AUTOMOC ON
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/gz_gui_plugins)
  # moc generated code trips this one.
  target_compile_options(AccuracyOverlay PRIVATE -Wno-useless-cast)
  target_link_libraries(AccuracyOverlay PRIVATE gz-gui8::gz-gui8
                        gz-transport13::core gz-msgs10::core Qt5::Core
                        Qt5::Quick)
endif()
