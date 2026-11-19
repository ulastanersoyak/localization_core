"""Turns the projects' `params:="duration:=60 gps_noise:=2"` launch argument
into ROS arguments for a program whose parameters are defined with
generate_parameter_library (a YAML file next to its source).

generate_parameter_library declares each parameter with a fixed type, and
ROS reads `duration:=60` as an integer, which a double parameter refuses.
The definition says which parameters are doubles, so their values get a
decimal point here. Names the definition does not know are passed on
unchanged (another node may take them: all of them go to the autopilot too).

    from ros_params import ros_arguments
    Node(..., ros_arguments=ros_arguments("duration:=60", "sim/src/positioning_sim_parameters.yaml"))
"""
import shlex

import yaml


def double_names(definition):
    with open(definition) as f:
        spec = yaml.safe_load(f)
    (parameters,) = spec.values()  # one namespace per file
    return {name for name, p in parameters.items() if p.get("type") == "double"}


def ros_arguments(text, definition=None):
    doubles = double_names(definition) if definition else set()
    args = []
    for item in shlex.split(text):
        name, sep, value = item.partition(":=")
        if not sep:
            raise ValueError(f"parameter {item!r} is not NAME:=VALUE")
        if name in doubles:
            try:
                value = repr(float(value))
            except ValueError:
                raise ValueError(f"parameter {name} needs a number, got {value!r}") from None
        args += ["-p", f"{name}:={value}"]
    return args
