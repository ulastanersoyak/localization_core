#!/usr/bin/env bash
# One time setup of the AirSim simulation (and again after updating the
# submodule). Steps, each skipped when already done:
#
#   1. external/Cosys-AirSim (submodule, branch 5.2.1) checked out, our
#      patches applied (airsim/patches, see airsim/README.md)
#   2. its dependencies (rpclib 2.3.0, Eigen 3.4.0, the versions its own
#      setup.sh pins) and AirLib, built with Unreal's toolchain (its build.sh)
#   3. its ROS 2 workspace, in the "ros2" distrobox (ROS 2 Jazzy), plus the
#      ROS packages it and airsim/launch use (rebuilt incrementally each time)
#   4. the Antwerp building tiles (sim/tools/locsim_city.py, open data)
#   5. the Unreal host project (airsim/unreal) and its maps (Field, Antwerp)
#      (airsim/maps/build_maps.py, Unreal's glTF importer)
#
# Needs: Unreal Engine 5.2 ($UE_ROOT, default ~/UnrealEngine/5.2), a "ros2"
# distrobox with ROS 2 Jazzy ($LOCSIM_ROS_BOX), git, cmake, wget, unzip.
set -euo pipefail

here=$(cd "$(dirname "$0")" && pwd)
core=$(cd "$here/.." && pwd)
airsim=$core/external/Cosys-AirSim
ue_root=${UE_ROOT:-$HOME/UnrealEngine/5.2}
box=${LOCSIM_ROS_BOX:-ros2}
step () { echo; echo "=== $*"; }

step "1. Cosys-AirSim submodule and patches"
git -C "$core" submodule update --init --depth 1 external/Cosys-AirSim
for patch in "$here"/patches/*.patch; do
  if git -C "$airsim" apply --reverse --check "$patch" 2>/dev/null; then
    echo "already applied: $(basename "$patch")"
  else
    git -C "$airsim" apply "$patch"
    echo "applied: $(basename "$patch")"
  fi
done

step "2. AirLib (rpclib, Eigen, then build.sh with Unreal's toolchain)"
cd "$airsim"
if [[ ! -d external/rpclib/rpclib-2.3.0 ]]; then
  mkdir -p external/rpclib
  wget -q https://github.com/rpclib/rpclib/archive/v2.3.0.zip
  unzip -q v2.3.0.zip -d external/rpclib && rm v2.3.0.zip
fi
if [[ ! -d AirLib/deps/eigen3/Eigen ]]; then
  wget -q -O eigen3.zip https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.zip
  unzip -q eigen3.zip -d temp_eigen && mkdir -p AirLib/deps/eigen3
  mv temp_eigen/eigen*/Eigen AirLib/deps/eigen3 && rm -rf temp_eigen eigen3.zip
fi
if [[ ! -f AirLib/lib/libAirLib.a ]]; then
  # CMake 4 refuses rpclib's old cmake_minimum_required without this.
  CMAKE_POLICY_VERSION_MINIMUM=3.5 ./build.sh --ue-root "$ue_root"
else
  echo "AirLib already built"
fi

step "3. ROS 2 workspace (in the $box distrobox)"
distrobox enter "$box" -- bash -c "
  set -e
  sudo apt-get update -qq
  sudo apt-get install -y -qq ros-jazzy-mavros-msgs ros-jazzy-geographic-msgs \
    ros-jazzy-topic-tools ros-jazzy-teleop-twist-keyboard python3-venv \
    libgeographiclib-dev libeigen3-dev
  . /opt/ros/jazzy/setup.bash
  cd '$airsim/ros2'
  # AirSim's Python client (weather.py, drone_localization's detectors and
  # dataset recorder), pyproj (the city tiles) and evo (evaluate.py), next
  # to the system ROS 2 Python packages.
  if [[ ! -x '$here/.venv/bin/python' ]]; then
    python3 -m venv --system-site-packages '$here/.venv'
    '$here/.venv/bin/pip' install -q rpc-msgpack
    '$here/.venv/bin/pip' install -q -e '$airsim/PythonClient'
    '$here/.venv/bin/pip' install -q pyproj
  fi
  '$here/.venv/bin/python' -c 'import evo' 2>/dev/null || '$here/.venv/bin/pip' install -q evo
  # Incremental: quick when nothing changed, and picks up new patches.
  colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_POLICY_VERSION_MINIMUM=3.5"

step "4. Antwerp building tiles"
city=$here/data/antwerp
if [[ ! -f $city/city.json ]]; then
  # (pyproj is in the AirSim environment of the ros2 box)
  distrobox enter "$box" -- "$here/.venv/bin/python" "$core/sim/tools/locsim_city.py" antwerp \
    --settings "$here/settings/antwerp_drone.json" --radius 300 --out "$city"
else
  echo "tiles already prepared in $city"
fi

step "5. Unreal project and its maps"
"$ue_root/Engine/Build/BatchFiles/Linux/Build.sh" LocalizationSimEditor Linux Development \
  -Project="$here/unreal/LocalizationSim.uproject" -WaitMutex
if [[ ! -f $here/unreal/Content/Maps/Antwerp.umap || ! -f $here/unreal/Content/Maps/Field.umap ]]; then
  LOCSIM_CITY_DIR=$city "$ue_root/Engine/Binaries/Linux/UnrealEditor-Cmd" \
    "$here/unreal/LocalizationSim.uproject" -run=pythonscript \
    -script="$here/maps/build_maps.py" -unattended -nullrhi -nosplash
else
  echo "map already built (delete airsim/unreal/Content/Maps to rebuild)"
fi

echo
echo "done: airsim/run.sh drone (with airsim/keyboard.sh) or airsim/run.sh free"
