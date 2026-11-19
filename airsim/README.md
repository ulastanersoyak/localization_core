# airsim

The simulator side of gnss_imu_prediction and drone_localization:
[Cosys-AirSim](https://github.com/Cosys-Lab/Cosys-AirSim) (submodule
`../external/Cosys-AirSim`, branch 5.2.1) on Unreal Engine 5.2, over ROS 2
Jazzy. What lives here is glue; Cosys-AirSim's own documentation is at
<https://cosys-lab.github.io/Cosys-AirSim/>.

## Setup and running

Needs Unreal Engine 5.2 in `~/UnrealEngine/5.2` (`UE_ROOT`) and a `ros2`
distrobox (Ubuntu 24.04, ROS 2 Jazzy).

```sh
git submodule update --init --depth 1 external/Cosys-AirSim
airsim/setup.sh              # once: patch + build Cosys-AirSim, ROS 2 workspace, city tiles, maps
airsim/run.sh drone          # Antwerp with a drone: W A S D, Space / Shift, Q / E in the window
airsim/run.sh free           # Antwerp, free camera
airsim/run.sh --map Field --settings airsim/settings/positioning_field.json --rain 0.5
```

The projects run their simulations through `run.sh` (`sim/run.sh` in
gnss_imu_prediction, `sim/run_pole.sh` in drone_localization). Weather on any
run: `--rain`, `--snow`, `--fog`, `--dust` (0..1), `--wind SPEED[@FROM_DEG]`;
`--offscreen` (projects: `--no-gui`) for no window.

## How a run works

`run.sh` starts Unreal on the host, then one `ros2 launch` in the box:
`launch/airsim.launch.py` (AirSim's ROS 2 node, the `/cmd_vel` relay, the
weather, the `enu` frame, the recorder), which the projects' launch files
include. The program's exit ends the launch; `run.sh` then stops Unreal.

* **Frames.** AirSim's node publishes TF `world` → `drone/odom_local` →
  `drone` (x north, y west, z up). The launch file adds `enu` → `world`
  (east, north, up, on the ground at the settings' `OriginGeopoint`); the
  programs work in `enu` through tf2, and the truth is the TF `enu` →
  `drone`.
* **Time.** AirSim's stamps are on its own clock, which trails the wall
  clock by a few seconds: never mix them with `time.time()` or `now()`.
* **Options** of the projects' scripts (`--duration 60`) become ROS
  parameters (`duration:=60`, `params.sh`). The programs define theirs with
  generate_parameter_library; `launch/ros_params.py` gives the doubles
  among them a decimal point, which its typed declarations need.
* **Flying.** `autopilot.py` flies the projects' figure eight over
  `/cmd_vel` (the launch file's `autopilot:=true`), on the true pose; the
  programs only measure and estimate. `keyboard.sh` replaces it for manual
  flying.
* **Scoring.** Each project run is recorded (rosbag2, `/tf` included) and
  scored at the end by `evaluate.py` with evo; evo's tools plot the same
  recording (in the box, ROS 2 sourced):

```sh
airsim/.venv/bin/python airsim/evaluate.py RUN_DIR "fused=/pole/fused"
airsim/.venv/bin/evo_traj bag2 RUN_DIR /pole/fused --ref /tf:enu.drone --plot
```

## Files

| file | |
|---|---|
| `setup.sh` | applies `patches/`, builds AirLib with Unreal's toolchain, builds the ROS 2 node in the `ros2` distrobox, creates `.venv` (AirSim client, pyproj, evo), city tiles, Unreal project and maps (delete `unreal/Content/Maps` to regenerate them) |
| `run.sh` | Unreal with a map and settings, then one ROS 2 launch file (`--launch`, default `launch/airsim.launch.py`; `LOCSIM_ROS_SETUP` sources more workspaces) |
| `launch/airsim.launch.py` | AirSim's ROS 2 node, the `/cmd_vel` relay, weather, the `enu` frame, recording (`record:=DIR`), the autopilot (`autopilot:=true`), rviz2 (`rviz:=true`) |
| `launch/ros_params.py` | the projects' `params:="..."` → ROS arguments, doubles made doubles |
| `autopilot.py` | flies the figure eight (speed and height swinging) over `/cmd_vel` |
| `evaluate.py` | scores a recording against the truth (evo) |
| `params.sh` | `--name value` → ROS parameters, for the projects' run scripts |
| `keyboard.sh` | fly from a terminal with ROS 2's `teleop_twist_keyboard` (`/cmd_vel`) |
| `weather.py` | rain, snow, fog, dust, wind through AirSim's API |
| `settings/` | AirSim settings: `antwerp_drone`, `antwerp_free`, `positioning_*`, `pole_*` (the pole camera) |
| `maps/build_maps.py` | generates `/Game/Maps/Field` and `/Game/Maps/Antwerp` |
| `patches/` | our fixes to Cosys-AirSim: building with Unreal 5.2's toolchain, the ROS node's frame conversions, camera poses and the external camera's gimbal, and skipping the empty images AirSim sometimes returns (the node crashed on them) |
| `unreal/` | the Unreal project; `Plugins/AirSim` links into the submodule; `Source/LocalizationSim/DroneKeyboard` flies the drone with W A S D in its window (`-DroneKeyboard`) |

Also in localization_core: `sim/src/ros_sim.hpp` (tf2 lookups and pose
messages the programs share), `sim/tools/locsim_city.py` (Antwerp's open 3D
data as `.glb` tiles, with PROJ). City data: "Antwerpen in 3D", Stad
Antwerpen, under the Flemish open data licence (free reuse with
attribution).
