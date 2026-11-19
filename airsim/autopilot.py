#!/usr/bin/env python3
"""Flies AirSim's drone in a figure eight over /cmd_vel, for the projects'
simulations: the programs only measure and estimate, this node flies.

    ros2 run ... autopilot.py --ros-args -p radius:=8.0 [-p start_topic:=/pole/detections]

It sits on the ground until it starts (start_delay seconds after the truth
appears, or after the first message on start_topic, e.g. the first drone
detection, since YOLO takes seconds to load), climbs to `altitude`, then
flies full circles alternating left and right, so the track is a figure
eight. The speed swings between 1 and 4 m/s (period 23 s) and the height by
`altitude_swing` around `altitude` (period 17 s), so the estimators see
acceleration on every axis.

The autopilot flies on the truth (TF enu -> drone, from AirSim's odometry);
the estimators never see it. Times come from the TF stamps, AirSim's clock,
which is not the wall clock. AirSim's ROS node turns /cmd_vel (forward,
left, up and yaw rate in the body frame) into velocity commands through the
relay in launch/airsim.launch.py; Cosys-AirSim's own controller does the
rest.

Why not AirSim's moveOnPathAsync: it flies a path at one fixed speed, and it
would take control through the RPC API next to the ROS node's velocity
commands. The /cmd_vel route keeps the varying speed and lets
teleop_twist_keyboard (keyboard.sh) replace this node for manual flying.
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.time import Time
from rosidl_runtime_py.utilities import get_message
from tf2_ros import Buffer, TransformException, TransformListener

SPEED_MEAN = 2.5      # m/s
SPEED_SWING = 1.5     # m/s
SPEED_PERIOD = 23.0   # s
HEIGHT_PERIOD = 17.0  # s
MAX_CLIMB = 2.0       # m/s
RATE = 20.0           # commands per second


class Autopilot(Node):
    def __init__(self):
        super().__init__("autopilot")
        self.radius = self.number("radius", 12.0, "figure-eight loop radius, metres")
        self.altitude = self.number("altitude", 8.0, "cruise height above the ground, metres")
        self.swing = self.number("altitude_swing", 3.0, "climb and descend this far around it, metres")
        self.start_delay = self.number("start_delay", 3.0, "seconds on the ground after the start (first GPS fix)")
        self.start_topic = self.declare_parameter(
            "start_topic", "", ParameterDescriptor(description="wait for the first message on this topic")).value

        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.start_sub = None
        self.started = not self.start_topic  # a start topic: wait for it
        self.t0 = None                       # start time, AirSim's clock
        self.airborne = False
        self.direction = 1.0
        self.turned = 0.0
        self.last_yaw = None
        self.create_timer(1.0 / RATE, self.step)

    def number(self, name, default, description):
        # dynamic typing: `-p radius:=8` arrives as an integer
        descriptor = ParameterDescriptor(description=description, dynamic_typing=True)
        return float(self.declare_parameter(name, default, descriptor).value)

    def subscribe_start(self):
        # The topic's type is only known once someone publishes it.
        for name, types in self.get_topic_names_and_types():
            if name == self.start_topic:
                self.start_sub = self.create_subscription(get_message(types[0]), name, self.on_start, 10)
                self.get_logger().info(f"waiting for the first message on {name}")

    def on_start(self, _msg):
        if not self.started:
            self.get_logger().info("start signal received")
        self.started = True

    def step(self):
        if not self.started and self.start_sub is None:
            self.subscribe_start()
        try:
            truth = self.tf.lookup_transform("enu", "drone", Time())
        except TransformException:
            return
        t = Time.from_msg(truth.header.stamp).nanoseconds * 1e-9
        q = truth.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        height = truth.transform.translation.z
        if self.last_yaw is not None and self.airborne:
            self.turned += math.remainder(yaw - self.last_yaw, 2 * math.pi)
        self.last_yaw = yaw
        if self.started and self.t0 is None:
            self.t0 = t
        cmd = Twist()
        ft = -1.0 if self.t0 is None else t - self.t0 - self.start_delay  # flight time
        if ft > 0:
            rate = 2 * math.pi / HEIGHT_PERIOD
            target = self.altitude + self.swing * math.sin(rate * ft)
            feed_forward = self.swing * rate * math.cos(rate * ft) if self.airborne else 0.0
            cmd.linear.z = max(-MAX_CLIMB, min(MAX_CLIMB, feed_forward + target - height))
            self.airborne = self.airborne or height > self.altitude / 2
            if self.airborne:
                speed = SPEED_MEAN + SPEED_SWING * math.sin(2 * math.pi * ft / SPEED_PERIOD)
                if abs(self.turned) >= 2 * math.pi:  # a full circle: turn the other way
                    self.direction = -self.direction
                    self.turned = 0.0
                cmd.linear.x = speed
                cmd.angular.z = self.direction * speed / self.radius
        self.cmd.publish(cmd)


def main():
    rclpy.init()
    node = Autopilot()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        try:
            node.cmd.publish(Twist())  # hover
        except Exception:  # noqa: BLE001 - the context may be gone at shutdown
            pass


if __name__ == "__main__":
    main()
