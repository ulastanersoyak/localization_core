"""The ROS 2 side of the simulator: Cosys-AirSim's ROS 2 node, the /cmd_vel
relay and the weather. Unreal itself runs on the host (airsim/run.sh starts
it, then this file in the ros2 distrobox). The projects' launch files include
this one and add their programs.

    ros2 launch airsim.launch.py [vehicle:=false] [rain:=0.5] [fog:=0.2] [wind:=8@270] ...

Arguments:
    vehicle     true: start AirSim's ROS node and the relay (false for the
                free camera, which has no vehicle)
    rain snow fog dust leaves   weather intensity 0..1 (empty: none)
    wind        SPEED[@FROM_DEG], m/s, the direction it blows from
    spawn_height  metres from the ground up to AirSim's "world" frame, the
                PlayerStart the vehicle spawns at (1.5 in our maps)
    record      a rosbag2 directory to record into (empty: no recording);
                evaluate.py scores the programs' estimates from it
    record_topics  topics to record besides /tf and /tf_static (spaces)
    autopilot   true: fly the figure eight (autopilot.py) over /cmd_vel
    autopilot_params  its ROS parameters, NAME:=VALUE separated by spaces
                (names it does not declare are ignored, so the projects pass
                all of their program's parameters)

Frames: AirSim's ROS node publishes TF world -> drone/odom_local -> drone,
world being x north, y west, z up at the PlayerStart. This file adds the
static transform enu -> world (a quarter turn about z, raised by
spawn_height), so "enu" is east, north, up on the ground at the
OriginGeopoint: the frame the programs and rviz2 work in, and the truth is
the TF enu -> drone.
"""
import math
import os
import shlex

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # airsim/
WEATHER = ["rain", "snow", "fog", "dust", "leaves", "wind"]


def weather(context):
    args = []
    for name in WEATHER:
        value = LaunchConfiguration(name).perform(context)
        if value:
            args += ["--" + name, value]
    if not args:
        return []
    # A few seconds after AirSim answers (run.sh starts this file then), so
    # its weather actor is up.
    return [TimerAction(period=5.0, actions=[ExecuteProcess(
        cmd=[os.path.join(HERE, ".venv", "bin", "python"), os.path.join(HERE, "weather.py"), *args],
        name="weather", output="screen")])]


def recorder(context):
    path = LaunchConfiguration("record").perform(context)
    if not path:
        return []
    topics = shlex.split(LaunchConfiguration("record_topics").perform(context))
    return [ExecuteProcess(cmd=["ros2", "bag", "record", "--output", path, "/tf", "/tf_static", *topics],
                           name="record", output={"stdout": "log", "stderr": "log"})]


def autopilot(context):
    if LaunchConfiguration("autopilot").perform(context) != "true":
        return []
    overrides = [a for p in shlex.split(LaunchConfiguration("autopilot_params").perform(context)) for a in ("-p", p)]
    return [Node(executable=os.path.join(HERE, "autopilot.py"), name="autopilot", ros_arguments=overrides,
                 output={"stdout": "log", "stderr": "log"})]


def stop_without_airsim(event, context):
    # AirSim's node (from its own launch file) died: without it the programs
    # would wait for data forever.
    if event.action.process_details["name"].startswith("airsim_node"):
        return [EmitEvent(event=Shutdown(reason="AirSim's ROS node exited"))]
    return None


def generate_launch_description():
    vehicle = IfCondition(LaunchConfiguration("vehicle"))
    return LaunchDescription([
        DeclareLaunchArgument("vehicle", default_value="true"),
        *[DeclareLaunchArgument(name, default_value="") for name in WEATHER],
        DeclareLaunchArgument("spawn_height", default_value="1.5"),
        DeclareLaunchArgument("record", default_value=""),
        DeclareLaunchArgument("record_topics", default_value=""),
        DeclareLaunchArgument("autopilot", default_value="false"),
        DeclareLaunchArgument("autopilot_params", default_value=""),

        # enu -> world (see the docstring): world's x axis (north) is enu's y.
        Node(package="tf2_ros", executable="static_transform_publisher", name="enu_frame",
             arguments=["--z", LaunchConfiguration("spawn_height"), "--yaw", str(math.pi / 2),
                        "--frame-id", "enu", "--child-frame-id", "world"],
             output={"stdout": "log", "stderr": "log"}),

        # Cosys-AirSim's node; enable_api_control arms the drone so it obeys
        # velocity commands (its launch file defaults to false).
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([FindPackageShare("airsim_ros_pkgs"), "/launch/airsim_node.launch.py"]),
            launch_arguments={"enable_api_control": "true", "output": "log"}.items(),
            condition=vehicle),

        # /cmd_vel (geometry_msgs/Twist, from teleop_twist_keyboard or the
        # projects' programs) -> AirSim's VelCmd. --wait-for-start: learn the
        # input type from the live topic; reliable QoS, which the node's
        # subscription requires.
        Node(package="topic_tools", executable="transform", name="cmd_vel_relay",
             arguments=["/cmd_vel", "/airsim_node/drone/vel_cmd_body_frame", "airsim_interfaces/msg/VelCmd",
                        "airsim_interfaces.msg.VelCmd(twist=m)", "--import", "airsim_interfaces",
                        "--wait-for-start", "--qos-reliability", "reliable"],
             # its output to the ROS log only: topic_tools 1.3.4 prints a
             # traceback when stopped (it shuts rclpy down a second time)
             output={"stdout": "log", "stderr": "log"},
             condition=vehicle),

        OpaqueFunction(function=weather),
        OpaqueFunction(function=recorder),
        OpaqueFunction(function=autopilot),
        RegisterEventHandler(OnProcessExit(on_exit=stop_without_airsim)),
    ])
