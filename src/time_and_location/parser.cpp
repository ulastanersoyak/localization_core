#include "time_and_location/parser.hpp"
#include "time_and_location/utc.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <stdexcept>
#include <string>

namespace
{
int
parse_int (std::string_view token)
{
  int value{ 0 };
  const char *end = token.data () + token.size ();
  const auto [ptr, ec] = std::from_chars (token.data (), end, value);
  if (token.empty () || ec != std::errc{} || ptr != end)
    {
      throw std::invalid_argument ("not an integer: " + std::string (token));
    }
  return value;
}

double
parse_double (std::string_view token)
{
  double value{ 0 };
  const char *end = token.data () + token.size ();
  const auto [ptr, ec] = std::from_chars (token.data (), end, value);
  if (token.empty () || ec != std::errc{} || ptr != end)
    {
      throw std::invalid_argument ("not a number: " + std::string (token));
    }
  return value;
}

std::uint8_t
parse_two_digits (std::string_view token, std::size_t pos, int max)
{
  const int value = parse_int (token.substr (pos, 2));
  if (value < 0 || value > max)
    {
      throw std::invalid_argument ("field out of range: "
                                   + std::string (token));
    }
  return static_cast<std::uint8_t> (value);
}
} // namespace

[[nodiscard]] utc
parse_utc (std::string_view token)
{
  if (token.size () < 6 || (token.size () > 6 && token[6] != '.'))
    {
      throw std::invalid_argument ("malformed utc: " + std::string (token));
    }

  std::uint16_t millisecond{ 0 };
  if (token.size () > 7)
    {
      const double fraction = parse_double (token.substr (6));
      millisecond = static_cast<std::uint16_t> (
          std::lround (std::min (fraction, 0.999) * 1000.0));
    }

  return utc{ parse_two_digits (token, 0, 23), parse_two_digits (token, 2, 59),
              parse_two_digits (token, 4, 60), millisecond };
}

[[nodiscard]] date
parse_date (std::string_view token)
{
  if (token.size () != 6)
    {
      throw std::invalid_argument ("malformed date: " + std::string (token));
    }
  const date dt{
    parse_two_digits (token, 0, 31),
    parse_two_digits (token, 2, 12),
    parse_two_digits (token, 4, 99),
  };
  if (dt.dd == 0 || dt.mm == 0)
    {
      throw std::invalid_argument ("malformed date: " + std::string (token));
    }
  return dt;
}

[[nodiscard]] std::variant<utc, date>
parse_date_and_utc (std::string_view token)
{
  if (token.size () == 6)
    {
      return parse_date (token);
    }
  return parse_utc (token);
}

[[nodiscard]] coordinate
parse_coordinate (std::string_view token)
{
  // 3958.59177,N - 03244.28792,E
  if (token.size () < 2)
    {
      throw std::invalid_argument ("malformed coordinate: "
                                   + std::string (token));
    }

  const char indicator = token.back ();
  const bool is_latitude = (indicator == 'N' || indicator == 'S');
  if (!is_latitude && indicator != 'E' && indicator != 'W')
    {
      throw std::invalid_argument ("bad coordinate indicator: "
                                   + std::string (token));
    }

  auto numeric_part = token.substr (0, token.size () - 1);
  if (!numeric_part.empty () && numeric_part.back () == ',')
    {
      numeric_part.remove_suffix (1);
    }

  // The minutes are always the two digits before the decimal point, the
  // degrees are whatever precedes them (2 digits for lat, 3 for lon).
  const auto dot = numeric_part.find ('.');
  const auto minute_start
      = (dot == std::string_view::npos ? numeric_part.size () : dot) - 2;
  if (dot < 3)
    {
      throw std::invalid_argument ("malformed coordinate: "
                                   + std::string (token));
    }
  if (dot == std::string_view::npos && numeric_part.size () < 3)
    {
      throw std::invalid_argument ("malformed coordinate: "
                                   + std::string (token));
    }

  const int degree = parse_int (numeric_part.substr (0, minute_start));
  const double minute = parse_double (numeric_part.substr (minute_start));
  if (degree > (is_latitude ? 90 : 180) || minute < 0.0 || minute >= 60.0)
    {
      throw std::invalid_argument ("coordinate out of range: "
                                   + std::string (token));
    }

  return {
    .degree = static_cast<std::uint8_t> (degree),
    .minute = minute,
    .indicator = indicator,
  };
}
