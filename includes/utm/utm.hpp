#ifndef UTM_HPP
#define UTM_HPP

#include <cstdint>
#include <utility>

// A position in the UTM grid (WGS84). The conversions are GeographicLib's.
class utm
{
  double easting_{ 0 };
  double northing_{ 0 };
  std::uint32_t zone_number_{ 0 };
  char zone_char_{ '_' };
  // Angle from true north to grid north (radians, clockwise positive).
  double convergence_{ 0 };

public:
  utm () = default;
  // Throws std::out_of_range outside the UTM latitude band [-80, 84].
  utm (double latitude_deg, double longitude_deg);
  utm (double easting, double northing, std::uint32_t zone_number,
       char zone_char);

  [[nodiscard]] double
  easting () const noexcept
  {
    return easting_;
  }
  [[nodiscard]] double
  northing () const noexcept
  {
    return northing_;
  }
  [[nodiscard]] std::uint32_t
  zone_number () const noexcept
  {
    return zone_number_;
  }
  [[nodiscard]] char
  zone_char () const noexcept
  {
    return zone_char_;
  }
  [[nodiscard]] double
  convergence () const noexcept
  {
    return convergence_;
  }
  [[nodiscard]] bool
  is_southern () const noexcept
  {
    return zone_char_ < 'N';
  }

  // {latitude, longitude} in decimal degrees.
  [[nodiscard]] std::pair<double, double> to_latlon () const;

  void
  update_easting (double distance) noexcept
  {
    easting_ += distance;
  }
  void
  update_northing (double distance) noexcept
  {
    northing_ += distance;
  }
};

#endif // !UTM_HPP
