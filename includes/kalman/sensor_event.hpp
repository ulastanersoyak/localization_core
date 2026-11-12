#ifndef SENSOR_EVENT_HPP
#define SENSOR_EVENT_HPP

#include "mti_630/data_packet_parser.hpp"
#include "utm/utm.hpp"

#include <optional>
#include <variant>

struct gps_fix
{
  utm position{};
  std::optional<double> altitude_m{}; // from the GGA of the same epoch
  double speed_mps{ 0 };
  std::optional<double> course_deg{}; // true course, clockwise from north
  std::optional<double> hdop{};       // from the latest GGA sentence
};

// Pushed by a finite source (e.g. a replay file) when it runs out of data.
struct end_of_stream
{
};

using sensor_event = std::variant<gps_fix, Xbus, end_of_stream>;

#endif // !SENSOR_EVENT_HPP
