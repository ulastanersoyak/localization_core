# Sourced by the scripts here: enters the virtual environment and, when the
# host has an NVIDIA driver, lets torch in the distrobox reach it.
_detector_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
. "$_detector_root/build-sim/yolo-venv/bin/activate"
if [[ -e /run/host/usr/lib64/libcuda.so.1 ]]; then
  _libs=$_detector_root/build-sim/host-nvidia-libs
  mkdir -p "$_libs"
  for _lib in /run/host/usr/lib64/libcuda.so* /run/host/usr/lib64/libnvidia-*.so*; do
    ln -sf "$_lib" "$_libs/$(basename "$_lib")"
  done
  export LD_LIBRARY_PATH=$_libs${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
fi
