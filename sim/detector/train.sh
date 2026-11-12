#!/usr/bin/env bash
# Generates a labelled image set from simulated flights and trains the drone
# detector. The labels come from the true pose, so nothing is labelled by hand.
# Run inside the gazebo distrobox after sim/detector/setup.sh.
#
#   sim/detector/train.sh [epochs]
set -euo pipefail

root=$(cd "$(dirname "$0")/../.." && pwd)
build=$root/build-sim
data=$build/yolo-data
epochs=${1:-60}
. "$root/sim/detector/env.sh"

cmake -S "$root" -B "$build" -DCMAKE_BUILD_TYPE=Release >/dev/null
cmake --build "$build" -j --target pole_tracking_sim
rm -rf "$data"

# name split radius altitude swing
flights=("a train 8 8 3" "b train 6 6 2" "c train 10 10 4" "d val 9 7 3")
pids=()
for flight in "${flights[@]}"; do
  read -r name split radius altitude swing <<<"$flight"
  (
    export GZ_PARTITION=yolo_$name
    gz sim -s -r "$root/sim/worlds/pole_tracking.sdf" &>"$build/gz_$name.log" &
    gz_pid=$!
    trap 'pkill -P $gz_pid 2>/dev/null || true; kill $gz_pid 2>/dev/null || true' EXIT
    python "$root/sim/detector/make_dataset.py" --out "$data/$split" \
      --prefix "$name" --seconds 95 &
    dataset_pid=$!
    # --fixed-camera: make_dataset.py labels frames with the rest pose.
    "$build/sim/pole_tracking_sim" --no-follow --fixed-camera --duration 95 --seed 7 \
      --radius "$radius" --altitude "$altitude" --altitude-swing "$swing" >/dev/null
    wait "$dataset_pid"
  ) &
  pids+=($!)
done
for pid in "${pids[@]}"; do wait "$pid"; done

cat >"$data/drone.yaml" <<YAML
path: $data
train: train/images
val: val/images
names:
  0: drone
YAML

cd "$build"
yolo detect train model=yolo11n.pt data="$data/drone.yaml" epochs="$epochs" \
  imgsz=1280 batch=6 patience=20 mosaic=0.5 fliplr=0.5 flipud=0 degrees=0 \
  project="$build/yolo-runs" name=drone exist_ok=True verbose=False
cp "$build/yolo-runs/drone/weights/best.pt" "$build/drone.pt"
echo "weights: $build/drone.pt"
