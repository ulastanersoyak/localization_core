#ifndef DATA_PACKET_PARSER_HPP
#define DATA_PACKET_PARSER_HPP

#include "time_and_location/utc.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <vector>

// Local earth-fixed frame an MTData2 item is expressed in (bits 2-3 of the
// data id). The MTi default is ENU.
enum class xbus_frame : std::uint8_t
{
  ENU = 0x0,
  NED = 0x4,
  NWU = 0x8
};

struct Xbus
{
  std::array<double, 3> euler{}; // roll, pitch, yaw [deg]
  bool eulerAvailable = false;

  std::array<double, 4> quat{}; // w, x, y, z (sensor to local frame)
  bool quaternionAvailable = false;
  xbus_frame quatFrame{ xbus_frame::ENU };

  std::array<double, 3> acc{}; // sensor frame, includes gravity [m/s^2]
  bool accAvailable = false;

  std::array<double, 3> freeAcc{}; // local frame, gravity removed [m/s^2]
  bool freeAccAvailable = false;
  xbus_frame freeAccFrame{ xbus_frame::ENU };

  std::array<double, 3> rot{}; // sensor frame [rad/s]
  bool rotAvailable = false;

  std::array<double, 2> latlon{};
  bool latlonAvailable = false;

  double altitude{ 0.0 };
  bool altitudeAvailable = false;

  std::array<double, 3> vel{};
  bool velocityAvailable = false;

  std::array<double, 3> mag{};
  bool magAvailable = false;

  uint16_t packetCounter{ 0 };
  bool packetCounterAvailable = false;

  uint32_t sampleTimeFine{ 0 }; // 10 kHz ticks
  bool sampleTimeFineAvailable = false;

  utc time{ 0, 0, 0 };
  bool utcTimeAvailable = false;

  uint32_t statusWord{ 0 };
  bool statusWordAvailable = false;

  static constexpr double rad2deg = 57.295779513082320876798154814105;
  static constexpr double sampleTimeFineHz = 10000.0;
  static constexpr double standardGravity = 9.80665; // [m/s^2]

  static double
  asinClamped (double x)
  {
    return std::asin (std::clamp (x, -1.0, 1.0));
  }

  void
  convertQuatToEuler ()
  {
    if (!quaternionAvailable)
      {
        return;
      }

    const auto &[q0, q1, q2, q3] = quat;
    const double sqw = q0 * q0;
    const double dphi = 2.0 * (sqw + q3 * q3) - 1.0;
    const double dpsi = 2.0 * (sqw + q1 * q1) - 1.0;

    euler[0] = std::atan2 (2.0 * (q2 * q3 + q0 * q1), dphi) * rad2deg;
    euler[1] = -asinClamped (2.0 * (q1 * q3 - q0 * q2)) * rad2deg;
    euler[2] = std::atan2 (2.0 * (q1 * q2 + q0 * q3), dpsi) * rad2deg;

    eulerAvailable = true;
  }

  // Acceleration {east, north, up} in the local frame with gravity removed,
  // from free acceleration if present, otherwise by rotating the raw
  // acceleration with the orientation quaternion and subtracting gravity.
  // Any error in gravity shows up as a constant vertical acceleration bias,
  // so pass the local value if it is known (Gazebo defaults to 9.8).
  [[nodiscard]] std::optional<std::array<double, 3>>
  accelerationENU (double gravity = standardGravity) const;

  // The horizontal part of accelerationENU(), which does not depend on
  // gravity.
  [[nodiscard]] std::optional<std::array<double, 2>>
  horizontalAccelerationEN () const;
};

class DataPacketParser
{
public:
  // Parses a complete MTData2 message (preamble to checksum). Returns false
  // if it is not a well formed MTData2 message.
  static bool parseDataPacket (const std::vector<uint8_t> &packet,
                               Xbus &xbusData);
};

#endif // DATA_PACKET_PARSER_HPP
