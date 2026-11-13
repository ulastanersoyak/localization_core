# localization_core

the common parts of two positioning projects for a raspberry pi 4 carrying a
u-blox max-m10s gnss receiver and an xsens mti-630 imu, and the glue between
them. the projects live in their own repositories and pull this one in as a
git submodule:

| repository | what |
|---|---|
| [gnss_imu_prediction](https://github.com/ulastanersoyak/gnss_imu_prediction) | gps + imu kalman filter that estimates the position and predicts where the vehicle will be one second later, with the gazebo "ghost" simulation and the device program |
| [drone_localization](https://github.com/ulastanersoyak/drone_localization) | locates a drone in 3d from external sensors; currently a pole with a uwb anchor and a pan-tilt camera running a yolo detector, optionally fused with the drone's imu and gps |

guides: [getting started](https://ulastanersoyak.github.io/localization_core/getting-started.html) ·
[how it works](https://ulastanersoyak.github.io/localization_core/how-it-works.html)

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
| `localization_core_headers` | `includes/kalman`, `includes/utility` | header only: fixed size `matrix`, generic `kalman_filter`, `gps_fix`/`sensor_event`, thread safe `queue` |
| `localization_core_sim` | `sim/src` | simulation helpers: gazebo ⇄ lat/lon local frame, error statistics |
| `AccuracyOverlay` | `sim/gui_plugin` | gazebo gui plugin drawing the live accuracy, built into `<build>/gz_gui_plugins` |
| `localization_core_test_support` | `test/cpp/check.hpp` | the tiny check helpers the tests use |

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
git submodule add ../localization_core external/localization_core
```

```cmake
project(my_project LANGUAGES CXX)
include(external/localization_core/cmake/common.cmake)
add_subdirectory(external/localization_core)

target_link_libraries(my_target PRIVATE utm xbus localization_core_headers)
if(TARGET gps) # only with boost
  target_link_libraries(my_device_program PRIVATE gps mti_630)
endif()
```

the unit tests here are only built when this is the top level project
(`-DLOCALIZATION_CORE_BUILD_TESTS=ON` forces them).

to work on all three at once, clone them side by side:

```
~/localization_core             this repository
~/gnss_imu_prediction         external/localization_core → ../localization_core
~/drone_localization     external/localization_core → ../localization_core
```

a change here reaches a project after committing it here and moving that
project's submodule: `git -C external/localization_core pull` then committing
`external/localization_core` in the project.

## history

this repository started as a 2d gps + imu position predictor for a wheeled
robot, grew into a 3d drone filter and then a drone localization project. the
full history up to the split is kept here (branch `pre-split`); the two
projects were split off with the history of their own files.
