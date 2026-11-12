#!/usr/bin/env bash
# Installs YOLO into build-sim/yolo-venv. Run once, inside the gazebo distrobox.
set -euo pipefail
root=$(cd "$(dirname "$0")/../.." && pwd)
sudo apt-get install -y -q python3-venv python3-pip
# The system site packages provide Gazebo's Python bindings.
python3 -m venv --system-site-packages "$root/build-sim/yolo-venv"
. "$root/build-sim/yolo-venv/bin/activate"
pip install --upgrade pip
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128
pip install ultralytics
