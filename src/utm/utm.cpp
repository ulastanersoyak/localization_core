#include "utm/utm.hpp"

#include <GeographicLib/UTMUPS.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <stdexcept>
#include <string_view>

namespace
{
constexpr double DEG_TO_RAD{ std::numbers::pi / 180 };

// Latitude band letter, C to X in 8 degree steps from -80 (X spans 12).
char
band_letter (double latitude)
{
  constexpr std::string_view letters{ "CDEFGHJKLMNPQRSTUVWX" };
  const auto band = static_cast<std::size_t> (std::floor ((latitude + 80) / 8));
  return letters[std::min (band, letters.size () - 1)];
}
} // namespace

utm::utm (double latitude, double longitude)
{
  if (latitude < -80 || latitude > 84 || longitude < -180 || longitude > 180)
    {
      throw std::out_of_range ("latitude/longitude outside of utm coverage");
    }

  // The standard zone, Norway and Svalbard exceptions included.
  int zone{ 0 };
  bool north{ true };
  double gamma{ 0 };
  double scale{ 0 };
  GeographicLib::UTMUPS::Forward (latitude, longitude, zone, north, easting_,
                                  northing_, gamma, scale,
                                  GeographicLib::UTMUPS::UTM);
  zone_number_ = static_cast<std::uint32_t> (zone);
  zone_char_ = band_letter (latitude);
  convergence_ = gamma * DEG_TO_RAD;
}

utm::utm (double easting, double northing, std::uint32_t zone_number,
          char zone_char)
    : easting_ (easting), northing_ (northing), zone_number_ (zone_number),
      zone_char_ (zone_char)
{
  double latitude{ 0 };
  double longitude{ 0 };
  double gamma{ 0 };
  double scale{ 0 };
  GeographicLib::UTMUPS::Reverse (static_cast<int> (zone_number_),
                                  !is_southern (), easting_, northing_,
                                  latitude, longitude, gamma, scale);
  convergence_ = gamma * DEG_TO_RAD;
}

std::pair<double, double>
utm::to_latlon () const
{
  double latitude{ 0 };
  double longitude{ 0 };
  GeographicLib::UTMUPS::Reverse (static_cast<int> (zone_number_),
                                  !is_southern (), easting_, northing_,
                                  latitude, longitude);
  return { latitude, longitude };
}
