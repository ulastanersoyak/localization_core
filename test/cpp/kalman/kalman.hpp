#ifndef KALMAN_TEST_HPP
#define KALMAN_TEST_HPP

#include "check.hpp"
#include "kalman/kalman_filter.hpp"
#include "kalman/measurements.hpp"

#include <cmath>
#include <numbers>

void
kalman_filter_test ()
{
  // One state measured twice with equal noise: the estimate is the mean and
  // the variance halves.
  kalman_filter<1> filter{ kalman_filter<1>::vector{ 0.0 },
                           kalman_filter<1>::matrix{ 1e6 } };
  const Eigen::Matrix<double, 1, 1> H{ 1.0 };
  const Eigen::Matrix<double, 1, 1> R{ 4.0 };
  filter.update (Eigen::Matrix<double, 1, 1>{ 10.0 }, H, R);
  filter.update (Eigen::Matrix<double, 1, 1>{ 12.0 }, H, R);
  check_near (filter.state () (0), 11.0, 1e-3, "mean of two measurements");
  check_near (filter.covariance () (0, 0), 2.0, 1e-3, "variance halves");

  // NIS: an innovation of one standard deviation scores 1.
  const double sigma = std::sqrt (filter.covariance () (0, 0) + R (0, 0));
  check_near (filter.nis (Eigen::Matrix<double, 1, 1>{ filter.state () (0) + sigma }, H, R),
              1.0, 1e-9, "nis of one sigma");

  // Constant velocity: position follows the velocity.
  kalman_filter<2> moving{ { 0.0, 3.0 }, kalman_filter<2>::matrix::Identity () };
  kalman_filter<2>::matrix F;
  F << 1, 0.5, 0, 1;
  moving.predict (F, kalman_filter<2>::matrix::Zero ());
  check_near (moving.state () (0), 1.5, 1e-12, "predicted position");
}

void
imu_sample_test ()
{
  // Level and at rest: no acceleration once gravity is removed.
  imu_sample level{};
  level.specific_force = { 0, 0, standard_gravity_mps2 };
  check (level.acceleration_enu ().norm () < 1e-12, "at rest");

  // Yawed 90 degrees counterclockwise: forward (sensor x) is north.
  imu_sample turned{};
  turned.specific_force = { 2, 0, standard_gravity_mps2 };
  turned.orientation = Eigen::AngleAxisd (std::numbers::pi / 2,
                                          Eigen::Vector3d::UnitZ ());
  const auto a = turned.acceleration_enu ();
  check_near (a.x (), 0, 1e-12, "turned east");
  check_near (a.y (), 2, 1e-12, "turned north");
  check_near (a.z (), 0, 1e-12, "turned up");
}

#endif // !KALMAN_TEST_HPP
