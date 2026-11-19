#include <gtest/gtest.h>

#include "utm/utm.hpp"

#include <array>
#include <cstdint>
#include <stdexcept>
#include <string>

TEST (utm, conversion)
{
  struct reference
  {
    double lat;
    double lon;
    std::uint32_t zone;
    char band;
    double easting;
    double northing;
  };

  // Reference values computed with pyproj (PROJ) 3.8.
  // clang-format off
  const std::array<reference, 6> references{ {
    {  39.97656,   32.73812, 36, 'S', 477638.347, 4425188.446 }, // Ankara
    { -33.8568,   151.2153,  56, 'H', 334900.570, 6252288.753 }, // Sydney
    {  40.6892,   -74.0445,  18, 'T', 580735.871, 4504695.165 }, // New York
    {  60.0,        5.0,     32, 'V', 276979.926, 6658157.202 }, // Norway
    {  -0.5,      -78.5,     17, 'M', 778265.778, 9944681.960 }, // Ecuador
    {  78.0,       15.0,     33, 'X', 500000.000, 8658369.586 }, // Svalbard
  } };
  // clang-format on

  for (const auto &ref : references)
    {
      const std::string name
          = std::to_string (ref.lat) + "," + std::to_string (ref.lon);
      const utm u (ref.lat, ref.lon);
      ASSERT_TRUE (u.zone_number () == ref.zone) << "utm zone " + name;
      ASSERT_TRUE (u.zone_char () == ref.band) << "utm band " + name;
      ASSERT_NEAR (u.easting (), ref.easting, 0.01) << "utm easting " + name;
      ASSERT_NEAR (u.northing (), ref.northing, 0.01)
          << "utm northing " + name;

      const auto [lat, lon] = u.to_latlon ();
      ASSERT_NEAR (
          utm (u.easting (), u.northing (), u.zone_number (), u.zone_char ())
              .convergence (),
          u.convergence (), 1e-12)
          << "grid convergence " + name;
      ASSERT_NEAR (lat, ref.lat, 1e-7) << "inverse latitude " + name;
      ASSERT_NEAR (lon, ref.lon, 1e-7) << "inverse longitude " + name;
    }

  // Grid convergence, also from pyproj.
  ASSERT_NEAR (utm (39.97656, 32.73812).convergence () * 180
                   / 3.14159265358979,
               -0.168252, 1e-5)
      << "convergence west of meridian";
  ASSERT_NEAR (utm (-33.8568, 151.2153).convergence () * 180
                   / 3.14159265358979,
               0.994515, 1e-5)
      << "convergence southern hemisphere";

  ASSERT_THROW (([] { const utm u (85.0, 0.0); }) (), std::out_of_range)
      << "latitude outside utm";
}
