# gnss-imu-fusion

the common parts of two positioning projects for a raspberry pi 4 carrying a
u-blox max-m10s gnss receiver and an xsens mti-630 imu, and the glue between
them. the projects live in their own repositories and pull this one in as a
git submodule:

| repository | what |
|---|---|
| `gnss-imu-prediction` | gps + imu kalman filter that estimates the position and predicts where the vehicle will be one second later, with the gazebo "ghost" simulation and the device program |
| `drone-pole-localization` | locates a drone from a pole carrying a uwb anchor and a pan-tilt camera with a yolo detector, optionally fused with the drone's imu and gps |

## what is in here

```
max-m10s ── nmea (9600 baud) ──▶ rmc/gga parser ──▶ utm ──┐
                                                          ├──▶ queue ──▶ (the projects' filters)
mti-630 ─── xbus (115200 baud) ─▶ mtdata2 parser ─────────┘
```

| target | path | what |
|---|---|---|
| `time_and_location` | `includes/time_and_location`, `src/time_and_location` | nmea field parsers: utc, date, coordinates |
| `rmc` | `includes/rmc`, `src/rmc` | checksum verified `$..rmc` sentences |
| `utm` | `includes/utm`, `src/utm` | lat/lon ⇄ utm, grid convergence |
| `xbus` | `includes/mti_630`, `src/mti_630` | xbus framing and mtdata2 parsing, acceleration rotated into east/north/up |
| `serial_port` | `includes/utility`, `src/utility` | boost.asio serial port (needs boost) |
| `gps` | `includes/gps`, `src/gps` | max-m10s reader: rmc + gga of the same epoch → `gps_fix` (needs boost) |
| `mti_630` | `includes/mti_630`, `src/mti_630` | mti-630 reader (needs boost) |
| `gnss_imu_fusion_headers` | `includes/kalman`, `includes/utility` | header only: fixed size `matrix`, generic `kalman_filter`, `gps_fix`/`sensor_event`, thread safe `queue` |
| `gnss_imu_fusion_sim` | `sim/src` | simulation helpers: gazebo ⇄ lat/lon local frame, error statistics |
| `AccuracyOverlay` | `sim/gui_plugin` | gazebo gui plugin drawing the live accuracy, built into `<build>/gz_gui_plugins` |
| `gnss_imu_fusion_test_support` | `test/cpp/check.hpp` | the tiny check helpers the tests use |

also here: `sim/gui.config` (the gazebo gui layout both simulations use),
`cmake/common.cmake` (c++ standard and warnings for all three repositories),
`test/python/` (serial capture script and a recorded nmea log), `docs/`
(datasheets and papers, `uart.sh` for enabling the uarts on the pi).

## building and testing

needs cmake ≥ 3.22 and a c++20 compiler. boost (asio, headers only) is only
needed for the serial port parts; without it everything else still builds.

```
cmake -S . -B build
cmake --build build
ctest --test-dir build
```

## using it from another repository

```
git submodule add ../gnss-imu-fusion external/gnss-imu-fusion
```

```cmake
project(my_project LANGUAGES CXX)
include(external/gnss-imu-fusion/cmake/common.cmake)
add_subdirectory(external/gnss-imu-fusion)

target_link_libraries(my_target PRIVATE utm xbus gnss_imu_fusion_headers)
if(TARGET gps) # only with boost
  target_link_libraries(my_device_program PRIVATE gps mti_630)
endif()
```

the unit tests here are only built when this is the top level project
(`-DGNSS_IMU_FUSION_BUILD_TESTS=ON` forces them).

to work on all three at once, clone them side by side:

```
~/gnss-imu-fusion             this repository
~/gnss-imu-prediction         external/gnss-imu-fusion → ../gnss-imu-fusion
~/drone-pole-localization     external/gnss-imu-fusion → ../gnss-imu-fusion
```

a change here reaches a project after committing it here and moving that
project's submodule: `git -C external/gnss-imu-fusion pull` then committing
`external/gnss-imu-fusion` in the project.

## history

this repository started as a 2d gps + imu position predictor for a wheeled
robot, grew into a 3d drone filter and then a pole based drone tracker. the
full history up to the split is kept here (branch `pre-split`); the two
projects were split off with the history of their own files.
