#ifndef MEASUREMENTS_HPP
#define MEASUREMENTS_HPP

#include "utm/utm.hpp"

#include <Eigen/Geometry>

#include <optional>

inline constexpr double standard_gravity_mps2 = 9.80665;

// A GPS fix as a receiver reports it.
struct gps_fix
{
  double time_s{ 0 };
  utm position{};
  std::optional<double> altitude_m{};
  double speed_mps{ 0 };
  std::optional<double> course_deg{}; // true course, clockwise from north
  std::optional<double> hdop{};
};

// An IMU sample as an AHRS reports it: specific force in the sensor frame
// and the sensor's orientation.
struct imu_sample
{
  double time_s{ 0 };
  // Includes gravity's reaction: at rest it points up [m/s^2].
  Eigen::Vector3d specific_force{ Eigen::Vector3d::Zero () };
  // Rotates sensor frame vectors into east/north/up.
  Eigen::Quaterniond orientation{ Eigen::Quaterniond::Identity () };

  // Acceleration in east/north/up with gravity removed. Any error in gravity
  // shows up as a constant vertical acceleration bias.
  [[nodiscard]] Eigen::Vector3d
  acceleration_enu (double gravity_mps2 = standard_gravity_mps2) const
  {
    return orientation * specific_force - Eigen::Vector3d (0, 0, gravity_mps2);
  }
};

#endif // !MEASUREMENTS_HPP
