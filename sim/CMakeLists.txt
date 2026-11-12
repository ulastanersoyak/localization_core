add_executable(positioning_sim src/positioning_sim.cpp)
target_link_libraries(positioning_sim kalman utm xbus rmc time_and_location
                      gz-transport13::core gz-msgs10::core)
target_include_directories(positioning_sim PRIVATE ${CMAKE_SOURCE_DIR}/includes)

add_executable(pole_tracking_sim src/pole_tracking_sim.cpp)
target_link_libraries(pole_tracking_sim xbus gz-transport13::core
                      gz-msgs10::core)
target_include_directories(pole_tracking_sim PRIVATE ${CMAKE_SOURCE_DIR}/includes)

# GUI overlay showing the prediction accuracy on the Gazebo screen.
find_package(gz-gui8 QUIET)
find_package(Qt5 COMPONENTS Core Quick QUIET)
if(gz-gui8_FOUND AND Qt5_FOUND)
  qt5_add_resources(accuracy_overlay_rcc gui_plugin/AccuracyOverlay.qrc)
  add_library(AccuracyOverlay SHARED gui_plugin/AccuracyOverlay.cc
                                     gui_plugin/AccuracyOverlay.hh
                                     ${accuracy_overlay_rcc})
  set_target_properties(AccuracyOverlay PROPERTIES AUTOMOC ON)
  # moc generated code trips this one.
  target_compile_options(AccuracyOverlay PRIVATE -Wno-useless-cast)
  target_link_libraries(AccuracyOverlay gz-gui8::gz-gui8 gz-transport13::core
                        gz-msgs10::core Qt5::Core Qt5::Quick)
endif()
