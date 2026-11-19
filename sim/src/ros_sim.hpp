#ifndef ROS_SIM_HPP
#define ROS_SIM_HPP

// Small helpers the projects' simulation programs (ROS 2 nodes for
// Cosys-AirSim, airsim/README.md) share: TF lookups into the "enu" frame and
// pose messages. (Their parameters come from generate_parameter_library.)
//
// Frames: AirSim's ROS node publishes TF world -> drone/odom_local -> drone
// (x north, y west, z up); airsim/launch/airsim.launch.py adds enu -> world.
// The programs work in enu (east, north, up, on the ground at AirSim's
// OriginGeopoint), so the truth is the TF enu -> drone and everything else
// AirSim publishes is turned into enu with tf2 rather than by hand.

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>
#include <map>
#include <mutex>
#include <optional>
#include <string>

namespace ros_sim
{
inline const std::string world_frame = "enu";

// A pose in the enu frame, for the estimates the programs publish (and
// evaluate.py scores from a recording).
inline geometry_msgs::msg::PoseStamped
pose_msg (const rclcpp::Time &stamp, const Eigen::Vector3d &position,
          const Eigen::Quaterniond &orientation = Eigen::Quaterniond::Identity ())
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = world_frame;
  msg.pose.position.x = position.x ();
  msg.pose.position.y = position.y ();
  msg.pose.position.z = position.z ();
  msg.pose.orientation = tf2::toMsg (orientation);
  return msg;
}

// A level orientation along the horizontal part of a velocity, for the
// estimates' pose messages: rviz2 draws them as arrows pointing where the
// drone is going. Identity (east) when it is slower than 0.3 m/s.
inline Eigen::Quaterniond
heading_of (const Eigen::Vector3d &velocity)
{
  if (std::hypot (velocity.x (), velocity.y ()) < 0.3)
    {
      return Eigen::Quaterniond::Identity ();
    }
  return Eigen::Quaterniond (
      Eigen::AngleAxisd (std::atan2 (velocity.y (), velocity.x ()), Eigen::Vector3d::UnitZ ()));
}

// The TF tree, queried in the enu frame. Thread safe (tf2's buffer is).
class frames
{
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  std::mutex mutex_{};
  std::map<std::string, Eigen::Isometry3d> fixed_{};

public:
  explicit frames (rclcpp::Node &node)
      : buffer_ (node.get_clock ()), listener_ (buffer_)
  {
  }

  // enu <- frame at the time t (interpolated between the transforms tf2
  // has), if tf2 has it.
  std::optional<Eigen::Isometry3d>
  lookup (const std::string &frame, const rclcpp::Time &t)
  {
    try
      {
        return tf2::transformToEigen (buffer_.lookupTransform (world_frame, frame, t));
      }
    catch (const tf2::TransformException &)
      {
        return std::nullopt;
      }
  }

  // enu <- frame for a frame that does not move against enu (AirSim's world
  // and odometry frames): looked up once, then remembered.
  std::optional<Eigen::Isometry3d>
  fixed (const std::string &frame)
  {
    std::lock_guard lock (mutex_);
    if (const auto it = fixed_.find (frame); it != fixed_.end ())
      {
        return it->second;
      }
    try
      {
        const auto t = tf2::transformToEigen (
            buffer_.lookupTransform (world_frame, frame, tf2::TimePointZero));
        fixed_.emplace (frame, t);
        return t;
      }
    catch (const tf2::TransformException &)
      {
        return std::nullopt;
      }
  }

  // Adds a transform that is not on /tf (a camera pose AirSim publishes
  // with each image), so lookup() interpolates it like any other.
  void
  add (const geometry_msgs::msg::TransformStamped &transform)
  {
    buffer_.setTransform (transform, "ros_sim", false);
  }
};
} // namespace ros_sim

#endif
