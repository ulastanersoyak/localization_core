#!/usr/bin/env bash
# Runs the pole tracking simulation: Gazebo, the YOLO detector and the bridge
# that fuses the sensors and scores them against the ground truth. Run inside
# the gazebo distrobox, after sim/detector/setup.sh and sim/detector/train.sh.
#
#   sim/run_pole.sh [--headless] [pole_tracking_sim options, see --help]
set -euo pipefail

root=$(cd "$(dirname "$0")/.." && pwd)
build=$root/build-sim

headless=false
args=()
for arg in "$@"; do
  if [[ $arg == --headless ]]; then headless=true; else args+=("$arg"); fi
done

if [[ ! -f $build/drone.pt ]]; then
  echo "no detector weights, run sim/detector/setup.sh and sim/detector/train.sh" >&2
  exit 1
fi

cmake -S "$root" -B "$build" -DCMAKE_BUILD_TYPE=Release >/dev/null
cmake --build "$build" -j --target pole_tracking_sim AccuracyOverlay
export GZ_GUI_PLUGIN_PATH="$build/sim${GZ_GUI_PLUGIN_PATH:+:$GZ_GUI_PLUGIN_PATH}"
. "$root/sim/detector/env.sh"

# The camera needs a display even without the Gazebo window: headless EGL
# rendering does not work in the distrobox.
if $headless; then
  gz sim -s -r "$root/sim/worlds/pole_tracking.sdf" &>"$build/gz.log" &
else
  # The shared GUI layout plus a panel with what YOLO sees, drawn by detect.py.
  # Its topic picker can switch it to the raw /pole/camera feed.
  gui_config=$build/pole_gui.config
  cat "$root/sim/gui.config" - >"$gui_config" <<'XML'

<plugin filename="ImageDisplay" name="YOLO detections">
  <gz-gui>
    <title>YOLO detections</title>
    <property type="string" key="state">docked</property>
  </gz-gui>
  <topic>/pole/camera/annotated</topic>
  <topic_picker>true</topic_picker>
</plugin>
XML
  gz sim -r --gui-config "$gui_config" \
    "$root/sim/worlds/pole_tracking.sdf" &>"$build/gz.log" &
fi
gz_pid=$!
python "$root/sim/detector/detect.py" --weights "$build/drone.pt" \
  --imgsz "${DETECTOR_IMGSZ:-1280}" &>"$build/detector.log" &
detector_pid=$!
cleanup() {
  kill "$detector_pid" 2>/dev/null || true
  pkill -P "$gz_pid" 2>/dev/null || true
  kill "$gz_pid" 2>/dev/null || true
  wait 2>/dev/null || true
}
trap cleanup EXIT

"$build/sim/pole_tracking_sim" "${args[@]}"
