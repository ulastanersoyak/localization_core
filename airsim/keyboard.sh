#!/usr/bin/env bash
# Flies the AirSim drone from a terminal with ROS 2's teleop_twist_keyboard:
# it publishes /cmd_vel, which airsim/run.sh relays to AirSim (keys, shown by
# the tool: i forward, , back, j / l turn, J / L sideways, t / b up / down,
# k stop, q / z faster / slower). In the Unreal window of `airsim/run.sh
# drone` you can also fly directly: W A S D, Space / Shift, Q / E.
exec distrobox enter "${LOCSIM_ROS_BOX:-ros2}" -- bash -c \
  '. /opt/ros/jazzy/setup.bash && ros2 run teleop_twist_keyboard teleop_twist_keyboard'
