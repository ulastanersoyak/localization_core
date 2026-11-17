#include "geo/utm.hpp"
#include "kalman/kalman.hpp"

#include <exception>
#include <iostream>

int
main ()
{
  int failures = 0;
  const auto run = [&failures] (const char *name, void (*test) ())
    {
      try
        {
          test ();
          std::cout << "[ OK ] " << name << "\n";
        }
      catch (const std::exception &e)
        {
          ++failures;
          std::cout << "[FAIL] " << name << ": " << e.what () << "\n";
        }
    };

  run ("utm_conversion", utm_conversion_test);
  run ("kalman_filter", kalman_filter_test);
  run ("imu_sample", imu_sample_test);
  return failures == 0 ? 0 : 1;
}
