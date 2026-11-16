#!/usr/bin/env bash
# Runs Unreal with Cosys-AirSim on the host, then one ROS 2 launch file in the
# "ros2" distrobox: by default launch/airsim.launch.py (AirSim's ROS 2 node,
# the /cmd_vel relay, weather); the projects pass their own, which includes
# that one and adds their programs.
#
#   airsim/run.sh drone            Antwerp, a quadrotor you fly in the Unreal
#                                  window: W A S D, Space / Shift, Q / E
#                                  (unreal/Source/LocalizationSim/DroneKeyboard)
#   airsim/run.sh free             Antwerp, no vehicle, a free camera
#                                  (arrow keys / W A S D, Page Up / Page Down)
#   airsim/run.sh --map Field|Antwerp --settings FILE   anything else
#
#   --offscreen                    no window
#   --speed M                      keyboard flying speed in drone mode, m/s (3)
#   --rain I --snow I --fog I --dust I --leaves I (0..1), --wind S[@FROM_DEG]
#                                  weather (passed to the launch file)
#   --launch FILE                  another launch file (it gets
#                                  $LOCSIM_AIRSIM_DIR = this folder)
#   NAME:=VALUE                    more arguments for the launch file
#
# The run ends when Unreal or the launch file ends (a program's --duration,
# Ctrl-C, closing the window); the other is then stopped. Unreal runs from
# $UE_ROOT (default ~/UnrealEngine/5.2), ROS 2 in $LOCSIM_ROS_BOX (default
# ros2); $LOCSIM_ROS_SETUP lists more setup.bash files (colcon workspaces,
# separated by spaces) to source before the launch. First time:
# airsim/setup.sh.
set -euo pipefail

here=$(cd "$(dirname "$0")" && pwd)
airsim=$here/../external/Cosys-AirSim
ue_root=${UE_ROOT:-$HOME/UnrealEngine/5.2}
box=${LOCSIM_ROS_BOX:-ros2}
map=Antwerp
settings=$here/settings/antwerp_drone.json
launch=$here/launch/airsim.launch.py
unreal_args=()
launch_args=()
while [[ $# -gt 0 ]]; do
  case $1 in
    drone) map=Antwerp; settings=$here/settings/antwerp_drone.json; unreal_args+=(-DroneKeyboard); shift ;;
    free) map=Antwerp; settings=$here/settings/antwerp_free.json; launch_args+=(vehicle:=false); shift ;;
    --map) map=$2; shift 2 ;;
    --settings) settings=$(realpath "$2"); shift 2 ;;
    --launch) launch=$(realpath "$2"); shift 2 ;;
    --offscreen) unreal_args+=(-RenderOffscreen); shift ;;
    --speed) unreal_args+=("-DroneKeyboardSpeed=$2"); shift 2 ;;
    --rain|--snow|--fog|--dust|--leaves|--wind) launch_args+=("${1#--}:=$2"); shift 2 ;;
    *:=*) launch_args+=("$1"); shift ;;
    *) echo "unknown option $1 (see the top of $0)" >&2; exit 2 ;;
  esac
done
if [[ ! -f $here/unreal/Content/Maps/$map.umap ]]; then
  echo "map $map is not built yet, run airsim/setup.sh" >&2
  exit 1
fi
port_open () { (exec 3<>/dev/tcp/127.0.0.1/41451) 2>/dev/null; }
# A simulator that is still shutting down (the previous run) gets 30 s.
for _ in $(seq 30); do port_open || break; sleep 1; done
if port_open; then
  echo "something already listens on AirSim's port 41451 (another simulator running?)" >&2
  exit 1
fi

unreal= ros=
cleanup () {
  # The launch runs inside the distrobox, not as our child: stop it by its
  # command line (it then stops everything it started).
  pkill -INT -f "ros2 launch $launch" 2>/dev/null || true
  [[ -n $unreal ]] && kill "$unreal" 2>/dev/null || true
  for _ in 1 2 3 4 5 6 7 8 9 10; do  # up to 10 s to stop cleanly
    if pgrep -f "ros2 launch $launch" >/dev/null || { [[ -n $unreal ]] && kill -0 "$unreal" 2>/dev/null; }; then
      sleep 1
    else
      break
    fi
  done
  [[ -n $unreal ]] && kill -9 "$unreal" 2>/dev/null || true
  pkill -9 -f "ros2 launch $launch" 2>/dev/null || true
  # Return only once the port is free, so a next run can start right away.
  for _ in $(seq 20); do port_open || break; sleep 1; done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
trap 'exit 129' HUP

mkdir -p "$here/unreal/Saved"
"$ue_root/Engine/Binaries/Linux/UnrealEditor" "$here/unreal/LocalizationSim.uproject" "/Game/Maps/$map" \
  -game -windowed -ResX=1600 -ResY=900 ${unreal_args[@]+"${unreal_args[@]}"} -settings="$settings" -log \
  >"$here/unreal/Saved/run.log" 2>&1 &
unreal=$!
echo "Unreal starting: map $map, settings $(basename "$settings")"
until port_open; do
  kill -0 "$unreal" 2>/dev/null || { echo "Unreal exited, see $here/unreal/Saved/Logs/LocalizationSim.log" >&2; exit 1; }
  sleep 1
done

echo "ros2 launch $(basename "$launch") ${launch_args[*]}"
setups=". /opt/ros/jazzy/setup.bash && . '$airsim/ros2/install/setup.bash'"
for setup in ${LOCSIM_ROS_SETUP:-}; do setups+=" && . '$setup'"; done
distrobox enter "$box" -- bash -c "$setups
  export LOCSIM_AIRSIM_DIR='$here'
  exec ros2 launch '$launch' ${launch_args[*]@Q}" &
ros=$!
wait -n "$unreal" "$ros"
