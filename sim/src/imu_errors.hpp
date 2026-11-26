#ifndef IMU_ERRORS_HPP
#define IMU_ERRORS_HPP

// The errors a real AHRS has and AirSim's IMU lacks, for the projects'
// simulation programs. AirSim gives white noise and the exact orientation; a
// real AHRS also has an accelerometer bias (fixed at startup here) and a
// slowly wandering orientation error (first order Gauss-Markov). A tilt
// error leaks gravity into the horizontal acceleration.

#include "kalman/measurements.hpp"

#include <Eigen/Geometry>

#include <cmath>
#include <optional>
#include <random>

namespace ros_sim
{
class imu_errors
{
  std::mt19937 rng_;
  std::normal_distribution<double> noise_{ 0.0, 1.0 };
  Eigen::Vector3d bias_{};
  Eigen::Vector3d sigma_{};                                    // roll, pitch, heading [rad]
  double tau_s_;
  Eigen::Vector3d orientation_error_{ Eigen::Vector3d::Zero () }; // rotation vector, enu [rad]
  std::optional<double> last_t_{};

public:
  // accel_bias: sigma of the bias per axis [m/s^2]; tilt, heading: sigma of
  // the orientation error [rad]; tau_s: how slowly that error wanders.
  imu_errors (double accel_bias, double tilt, double heading, double tau_s, unsigned seed)
      : rng_ (seed), sigma_ (tilt, tilt, heading), tau_s_ (tau_s)
  {
    for (auto &b : bias_)
      {
        b = accel_bias * noise_ (rng_);
      }
  }

  [[nodiscard]] imu_sample
  apply (imu_sample sample)
  {
    // The first sample starts from the steady state distribution.
    const double phi = last_t_ ? std::exp (-(sample.time_s - *last_t_) / tau_s_) : 0.0;
    last_t_ = sample.time_s;
    for (int i = 0; i < 3; ++i)
      {
        orientation_error_[i]
            = (phi * orientation_error_[i]) + (sigma_[i] * std::sqrt (1 - (phi * phi)) * noise_ (rng_));
      }

    sample.specific_force += bias_;
    // The orientation maps sensor to local, so the local frame error goes on
    // the left.
    const double angle = orientation_error_.norm ();
    if (angle > 1e-12)
      {
        sample.orientation
            = Eigen::Quaterniond (Eigen::AngleAxisd (angle, orientation_error_ / angle)) * sample.orientation;
      }
    return sample;
  }
};
} // namespace ros_sim

#endif // !IMU_ERRORS_HPP
