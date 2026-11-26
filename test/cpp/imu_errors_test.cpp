#include <gtest/gtest.h>

#include "imu_errors.hpp"

TEST (imu_errors, model)
{
  // Without errors the sample passes unchanged.
  ros_sim::imu_errors none{ 0.0, 0.0, 0.0, 30.0, 1 };
  imu_sample sample{};
  sample.specific_force = { 1.0, 2.0, 9.8 };
  const auto same = none.apply (sample);
  ASSERT_TRUE (same.specific_force.isApprox (sample.specific_force)) << "no bias";
  ASSERT_TRUE (same.orientation.isApprox (sample.orientation)) << "no orientation error";

  // The bias is fixed: two samples get the same offset. The orientation
  // error wanders slowly: 10 ms apart it hardly changes.
  ros_sim::imu_errors errors{ 0.05, 0.01, 0.02, 30.0, 1 };
  sample.time_s = 0.0;
  const auto a = errors.apply (sample);
  sample.time_s = 0.01;
  const auto b = errors.apply (sample);
  ASSERT_TRUE ((a.specific_force - sample.specific_force).norm () > 0) << "biased";
  ASSERT_TRUE (a.specific_force.isApprox (b.specific_force)) << "fixed bias";
  ASSERT_TRUE (!a.orientation.isApprox (sample.orientation)) << "orientation error";
  ASSERT_LT (a.orientation.angularDistance (b.orientation), 1e-3) << "slowly wandering";
}
