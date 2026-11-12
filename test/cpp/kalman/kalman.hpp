#ifndef KALMAN_TEST_HPP
#define KALMAN_TEST_HPP

#include "check.hpp"
#include "kalman/position_estimator.hpp"

#include <cstdint>
#include <random>

namespace kalman_test
{
constexpr double start_easting = 477638.0;
constexpr double start_northing = 4425188.0;

gps_fix
make_fix (double easting, double northing, int second, double speed_mps = 0,
          std::optional<double> course = std::nullopt)
{
  const utc time{ 7, static_cast<std::uint8_t> (second / 60),
                  static_cast<std::uint8_t> (second % 60) };
  return gps_fix{ .position = utm (easting, northing, 36, 'S', time),
                  .speed_mps = speed_mps,
                  .course_deg = course,
                  .hdop = 1.0 };
}
} // namespace kalman_test

void
matrix_test ()
{
  const matrix<3, 3> a{ 4, 1, 2, //
                        1, 3, 0, //
                        2, 0, 5 };
  const auto product = a * inverse (a);
  for (std::size_t r = 0; r < 3; ++r)
    {
      for (std::size_t c = 0; c < 3; ++c)
        {
          check_near (product (r, c), r == c ? 1.0 : 0.0, 1e-12,
                      "matrix inverse");
        }
    }
  check_throws<std::domain_error> (
      [] { (void)inverse (matrix<2, 2>{ 1, 2, 2, 4 }); }, "singular matrix");
}

void
estimator_gps_only_test ()
{
  using namespace kalman_test;

  // 10 m/s due east, noisy fixes, no course information: the filter has to
  // work out the velocity from the positions alone.
  std::mt19937 rng (42);
  std::normal_distribution<double> noise (0.0, 2.0);
  position_estimator estimator{};
  constexpr double speed = 10.0;

  check (!estimator.estimate (), "no estimate before the first fix");
  for (int t = 0; t <= 40; ++t)
    {
      const auto result = estimator.add_gps_fix (
          make_fix (start_easting + speed * t + noise (rng),
                    start_northing + noise (rng), t, speed));
      check (result
                 == (t == 0 ? position_estimator::fix_result::initialized
                            : position_estimator::fix_result::accepted),
             "fix accepted at t=" + std::to_string (t));
    }

  const auto est = estimator.estimate ();
  check (est.has_value (), "estimate available");
  // Without course/speed measurements the velocity is only as good as the
  // differenced positions, check against the filter's own 3 sigma bound.
  check (est->sigma_velocity_east_mps < 2.5, "velocity uncertainty bounded");
  check_near (est->velocity_east_mps, speed, 3 * est->sigma_velocity_east_mps,
              "gps only east velocity");
  check_near (est->velocity_north_mps, 0.0, 3 * est->sigma_velocity_north_mps,
              "gps only north velocity");
  check_near (est->position.easting (), start_easting + speed * 40, 2.0,
              "gps only easting");
  check_near (est->position.northing (), start_northing, 2.0,
              "gps only northing");
  check (est->sigma_east_m < 2.0, "filter is more certain than one fix");

  const auto ahead = estimator.predict_ahead (2.0);
  check_near (ahead->position.easting () - est->position.easting (),
              2 * est->velocity_east_mps, 1e-6, "prediction ahead");

  // A 200 m jump is an outlier and must not move the estimate.
  const auto result = estimator.add_gps_fix (
      make_fix (start_easting + speed * 41 + 200, start_northing, 41, speed));
  check (result == position_estimator::fix_result::rejected,
         "outlier rejected");
  check_near (estimator.estimate ()->position.easting (),
              start_easting + speed * 41, 3.0, "outlier ignored");

  // A real jump (persistent) eventually restarts the filter.
  auto last = position_estimator::fix_result::rejected;
  for (int t = 42; t < 50 && last == position_estimator::fix_result::rejected;
       ++t)
    {
      last = estimator.add_gps_fix (
          make_fix (start_easting + 5000 + speed * t, start_northing, t));
    }
  check (last == position_estimator::fix_result::reset,
         "persistent jump resets the filter");
}

void
estimator_course_test ()
{
  using namespace kalman_test;

  // With speed and course the velocity is known after one update.
  position_estimator estimator{};
  (void)estimator.add_gps_fix (
      make_fix (start_easting, start_northing, 0, 5.0, 0.0));
  (void)estimator.add_gps_fix (
      make_fix (start_easting, start_northing + 5, 1, 5.0, 0.0));
  const auto est = estimator.estimate ();
  // Course is relative to true north, the grid is rotated by the convergence
  // (about -0.17 degrees here), so a little velocity leaks into east.
  check_near (est->velocity_north_mps, 5.0, 0.2, "course north velocity");
  check_near (est->velocity_east_mps, 0.0, 0.1, "course east velocity");
  check (est->sigma_velocity_north_mps < 0.3, "course velocity is certain");
}

void
estimator_imu_test ()
{
  using namespace kalman_test;

  position_estimator estimator{};
  Xbus sample{};
  sample.sampleTimeFineAvailable = true;
  sample.freeAccAvailable = true;
  sample.freeAcc = { 1.0, 0.0, 0.0 };

  check (!estimator.add_imu_sample (sample),
         "imu ignored before the first fix");
  (void)estimator.add_gps_fix (make_fix (start_easting, start_northing, 0));
  const double sigma_before = estimator.estimate ()->sigma_east_m;

  // 1 s at 100 Hz of 1 m/s^2 eastward acceleration.
  for (std::uint32_t i = 1; i <= 100; ++i)
    {
      sample.sampleTimeFine = i * 100;
      check (estimator.add_imu_sample (sample), "imu sample used");
    }

  const auto est = estimator.estimate ();
  check_near (est->age_s, 1.0, 1e-9, "imu prediction time");
  check_near (est->velocity_east_mps, 1.0, 0.02, "imu integrated velocity");
  check_near (est->position.easting () - start_easting, 0.5, 0.02,
              "imu integrated position");
  check (est->sigma_east_m > sigma_before, "uncertainty grows without gps");

  // Prediction keeps accelerating: x + v t + a t^2 / 2.
  const auto ahead = estimator.predict_ahead (1.0);
  check_near (ahead->position.easting () - est->position.easting (),
              est->velocity_east_mps + 0.5, 0.02,
              "prediction uses the measured acceleration");

  // The next fix is accepted and pulls the estimate towards it.
  const auto result = estimator.add_gps_fix (
      make_fix (start_easting + 0.5, start_northing, 1, 1.0, 90.0));
  check (result == position_estimator::fix_result::accepted,
         "fix after imu prediction");
  check_near (estimator.estimate ()->age_s, 0.0, 1e-9, "age reset by fix");
}

void
estimator_altitude_test ()
{
  using namespace kalman_test;

  // Climbing at 2 m/s with noisy altitude and horizontal fixes: the climb
  // rate is never measured, it has to come out of the altitudes.
  std::mt19937 rng (7);
  std::normal_distribution<double> noise (0.0, 2.0);
  std::normal_distribution<double> alt_noise (0.0, 3.4);
  position_estimator estimator{};
  constexpr double climb = 2.0;
  constexpr double start_altitude = 894.0;
  const auto fix_at = [&] (int t, double altitude)
    {
      auto fix = make_fix (start_easting + noise (rng),
                           start_northing + noise (rng), t);
      fix.altitude_m = altitude;
      return fix;
    };

  for (int t = 0; t <= 40; ++t)
    {
      (void)estimator.add_gps_fix (
          fix_at (t, start_altitude + climb * t + alt_noise (rng)));
    }
  auto est = estimator.estimate ();
  check_near (est->altitude_m, start_altitude + climb * 40,
              3 * est->sigma_up_m, "altitude tracked");
  check (est->sigma_up_m < 3.4, "filter is more certain than one altitude");
  check_near (est->velocity_up_mps, climb, 3 * est->sigma_velocity_up_mps,
              "climb rate from the altitudes");

  const auto ahead = estimator.predict_ahead (2.0);
  check_near (ahead->altitude_m - est->altitude_m, 2 * est->velocity_up_mps,
              1e-6, "altitude prediction");

  // A 200 m altitude spike alone is an outlier.
  const auto result
      = estimator.add_gps_fix (fix_at (41, start_altitude + climb * 41 + 200));
  check (result == position_estimator::fix_result::rejected,
         "altitude outlier rejected");

  // Going straight up has a low ground speed, which zeroes the horizontal
  // velocity but must not zero the climb rate.
  position_estimator straight_up{};
  for (int t = 0; t <= 20; ++t)
    {
      auto fix = make_fix (start_easting, start_northing, t);
      fix.altitude_m = start_altitude + climb * t;
      (void)straight_up.add_gps_fix (fix);
    }
  check_near (straight_up.estimate ()->velocity_up_mps, climb, 0.1,
              "low ground speed keeps the climb rate");
  check_near (straight_up.estimate ()->velocity_east_mps, 0.0, 0.1,
              "low ground speed zeroes the horizontal velocity");

  // The filter starts without altitude and takes the first one it gets.
  position_estimator late{};
  (void)late.add_gps_fix (make_fix (start_easting, start_northing, 0));
  check (late.estimate ()->sigma_up_m > 1000, "altitude unknown at first");
  auto first = make_fix (start_easting, start_northing, 1);
  first.altitude_m = start_altitude;
  check (late.add_gps_fix (first) == position_estimator::fix_result::accepted,
         "first altitude accepted");
  check_near (late.estimate ()->altitude_m, start_altitude, 0.1,
              "first altitude adopted");
  check (late.estimate ()->sigma_up_m < 10, "altitude known after a fix");
}

void
estimator_vertical_imu_test ()
{
  using namespace kalman_test;

  // 1 s at 100 Hz of 2 m/s^2 upward, from raw acceleration and orientation:
  // the configured gravity has to be the one removed.
  estimator_config config{};
  config.gravity_mps2 = 9.8;
  position_estimator estimator{ config };
  auto fix = make_fix (start_easting, start_northing, 0);
  fix.altitude_m = 100.0;
  (void)estimator.add_gps_fix (fix);

  Xbus sample{};
  sample.accAvailable = true;
  sample.acc = { 0.0, 0.0, 9.8 + 2.0 };
  sample.quaternionAvailable = true;
  sample.quat = { 1.0, 0.0, 0.0, 0.0 };
  sample.sampleTimeFineAvailable = true;
  for (std::uint32_t i = 1; i <= 100; ++i)
    {
      sample.sampleTimeFine = i * 100;
      (void)estimator.add_imu_sample (sample);
    }

  const auto est = estimator.estimate ();
  check_near (est->velocity_up_mps, 2.0, 0.03, "imu integrated climb rate");
  check_near (est->altitude_m - 100.0, 1.0, 0.03, "imu integrated altitude");
  check_near (est->velocity_east_mps, 0.0, 1e-9, "no horizontal leakage");

  const auto ahead = estimator.predict_ahead (1.0);
  check_near (ahead->altitude_m - est->altitude_m, est->velocity_up_mps + 1.0,
              0.03, "prediction uses the vertical acceleration");
}

#endif // !KALMAN_TEST_HPP
