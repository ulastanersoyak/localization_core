# Sourced by the projects' run scripts: turns program options written as
# --name value (or a bare --flag) into ROS parameters name:=value (flag:=true),
# dashes becoming underscores. A value may not start with "--".
#
#   to_ros_params --duration 60 --gps-noise 2 --manual   ->  ros_params=(duration:=60 gps_noise:=2 manual:=true)
to_ros_params () {
  ros_params=()
  while [[ $# -gt 0 ]]; do
    if [[ $1 != --?* ]]; then
      echo "unexpected argument $1 (options are --name value)" >&2
      return 2
    fi
    local name=${1#--}
    name=${name//-/_}
    if [[ $# -gt 1 && $2 != --* ]]; then
      ros_params+=("$name:=$2"); shift 2
    else
      ros_params+=("$name:=true"); shift
    fi
  done
}
