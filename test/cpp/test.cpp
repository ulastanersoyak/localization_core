#ifdef HAVE_GPS
#include "gps/gps.hpp"
#endif
#include "geo/utm.hpp"
#include "kalman/kalman.hpp"
#include "mti_630/xbus.hpp"
#include "parser/parser.hpp"
#include "parser/rmc.hpp"
#include "pole/pole.hpp"

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

  run ("parse_coordinate", parse_coordinate_test);
  run ("parse_coordinate_edge", parse_coordinate_edge_test);
  run ("parse_date_and_utc", parse_date_and_utc_test);
  run ("parse_utc", parse_utc_test);
  run ("rmc_construction", rmc_construction_test);
  run ("rmc_sentence", rmc_sentence_test);
  run ("utm_conversion", utm_conversion_test);
  run ("utm_direction", utm_direction_test);
  run ("xbus_parse", xbus_parse_test);
  run ("xbus_framing", xbus_framing_test);
  run ("matrix", matrix_test);
  run ("estimator_gps_only", estimator_gps_only_test);
  run ("estimator_course", estimator_course_test);
  run ("estimator_imu", estimator_imu_test);
  run ("estimator_altitude", estimator_altitude_test);
  run ("estimator_vertical_imu", estimator_vertical_imu_test);
  run ("pole_estimator", pole_estimator_test);
#ifdef HAVE_GPS
  run ("gps_altitude_pairing", gps_altitude_pairing_test);
#endif
  return failures == 0 ? 0 : 1;
}
