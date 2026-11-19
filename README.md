# localization_core

the common parts of two positioning projects that run in simulation
(cosys-airsim on unreal engine 5.2, ros 2 jazzy), and the simulator side they
share. the projects live in their own repositories and pull this one in as a
git submodule:

| repository | what |
|---|---|
| [gnss_imu_prediction](https://github.com/ulastanersoyak/gnss_imu_prediction) | gps + imu kalman filter that estimates the position and predicts where the vehicle will be one second later, with the "ghost" simulation |
| [drone_localization](https://github.com/ulastanersoyak/drone_localization) | locates a drone in 3d from external sensors; currently a pole with a uwb anchor and a pan-tilt camera running a yolo detector, optionally fused with the drone's imu and gps |

guides: [getting started](https://ulastanersoyak.github.io/localization_core/getting-started.html) ·
[how it works](https://ulastanersoyak.github.io/localization_core/how-it-works.html)

## what is in here

| target | path | what |
|---|---|---|
| `utm` | `includes/utm`, `src/utm` | lat/lon ⇄ utm and grid convergence, converted by geographiclib |
| `localization_core_headers` | `includes/kalman`, `includes/utility` | header only: the generic `kalman_filter` (eigen), the measurement types `gps_fix` and `imu_sample`, thread safe `queue` |
| `localization_core_sim` | `sim/src` | header only, for the simulation programs (ros 2): tf2 lookups in the `enu` frame and pose messages |

also here: `airsim/` (the simulator side, see `airsim/README.md`),
`cmake/common.cmake` (c++ standard and warnings for all three repositories),
`sim/tools/` (the antwerp building tiles), `docs/` (the guides and two utm
papers).

## building and testing

needs cmake ≥ 3.22, a c++20 compiler, eigen, geographiclib and googletest
(the tests):

```
sudo dnf install cmake gcc-c++ eigen3-devel GeographicLib-devel gtest-devel      # fedora
sudo apt install cmake g++ libeigen3-dev libgeographiclib-dev libgtest-dev      # ubuntu
cmake -S . -B build
cmake --build build
ctest --test-dir build
```

the `ros2` distrobox that `airsim/setup.sh` uses has all of them.

## using it from another repository

```
git submodule add ../localization_core external/localization_core
```

```cmake
project(my_project LANGUAGES CXX)
include(external/localization_core/cmake/common.cmake)
add_subdirectory(external/localization_core)

target_link_libraries(my_target PRIVATE utm localization_core_headers)
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
robot on a raspberry pi 4 (u-blox max-m10s, xsens mti-630), grew into a 3d
drone filter and then a drone localization project, and is now simulation
only. the full history is kept here; the two projects were split off with the
history of their own files.
