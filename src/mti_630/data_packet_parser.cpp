#include "mti_630/data_packet_parser.hpp"

#include <bit>
#include <cstddef>
#include <cstdint>

namespace
{
constexpr std::uint8_t MID_MTDATA2 = 0x36;

std::uint16_t
be_u16 (const std::uint8_t *p)
{
  return static_cast<std::uint16_t> ((p[0] << 8) | p[1]);
}

std::uint32_t
be_u32 (const std::uint8_t *p)
{
  return (static_cast<std::uint32_t> (p[0]) << 24)
         | (static_cast<std::uint32_t> (p[1]) << 16)
         | (static_cast<std::uint32_t> (p[2]) << 8) | p[3];
}

std::uint64_t
be_u64 (const std::uint8_t *p)
{
  return (static_cast<std::uint64_t> (be_u32 (p)) << 32) | be_u32 (p + 4);
}

// Size in bytes of one value for the precision encoded in the low two bits of
// the data id: float32, fp12.20, fp16.32, float64.
std::size_t
real_size (std::uint16_t dataId)
{
  constexpr std::array<std::size_t, 4> sizes{ 4, 4, 6, 8 };
  return sizes[dataId & 0x3];
}

double
read_real (const std::uint8_t *p, std::uint16_t dataId)
{
  switch (dataId & 0x3)
    {
    case 0:
      return std::bit_cast<float> (be_u32 (p));
    case 1: // fp12.20, signed
      return static_cast<std::int32_t> (be_u32 (p)) / 1048576.0;
    case 2: // fp16.32: 32 bit fraction followed by a signed 16 bit integer
      {
        const auto fraction = static_cast<std::int64_t> (be_u32 (p));
        const auto integer = static_cast<std::int16_t> (be_u16 (p + 4));
        return static_cast<double> (static_cast<std::int64_t> (integer)
                                        * 4294967296LL
                                    + fraction)
               / 4294967296.0;
      }
    default:
      return std::bit_cast<double> (be_u64 (p));
    }
}

template <std::size_t N>
bool
read_reals (const std::uint8_t *p, std::size_t len, std::uint16_t dataId,
            std::array<double, N> &out)
{
  const auto size = real_size (dataId);
  if (len != N * size)
    {
      return false;
    }
  for (std::size_t i = 0; i < N; ++i)
    {
      out[i] = read_real (p + i * size, dataId);
    }
  return true;
}

void
parse_item (Xbus &xbus, std::uint16_t dataId, const std::uint8_t *p,
            std::size_t len)
{
  const auto frame = static_cast<xbus_frame> (dataId & 0x000C);
  switch (dataId & 0xFFF0)
    {
    case 0x1010: // UtcTime
      if (len == 12)
        {
          const auto nanosec = be_u32 (p);
          xbus.time = utc (p[8], p[9], p[10],
                           static_cast<std::uint16_t> (nanosec / 1000000));
          xbus.utcTimeAvailable = true;
        }
      break;
    case 0x1020: // PacketCounter
      if (len == 2)
        {
          xbus.packetCounter = be_u16 (p);
          xbus.packetCounterAvailable = true;
        }
      break;
    case 0x1060: // SampleTimeFine
      if (len == 4)
        {
          xbus.sampleTimeFine = be_u32 (p);
          xbus.sampleTimeFineAvailable = true;
        }
      break;
    case 0x2010: // Quaternion
      xbus.quaternionAvailable = read_reals (p, len, dataId, xbus.quat);
      xbus.quatFrame = frame;
      xbus.convertQuatToEuler ();
      break;
    case 0x2030: // EulerAngles
      xbus.eulerAvailable = read_reals (p, len, dataId, xbus.euler);
      break;
    case 0x4020: // Acceleration
      xbus.accAvailable = read_reals (p, len, dataId, xbus.acc);
      break;
    case 0x4030: // FreeAcceleration
      xbus.freeAccAvailable = read_reals (p, len, dataId, xbus.freeAcc);
      xbus.freeAccFrame = frame;
      break;
    case 0x8020: // RateOfTurn
      xbus.rotAvailable = read_reals (p, len, dataId, xbus.rot);
      break;
    case 0x5040: // LatLon
      xbus.latlonAvailable = read_reals (p, len, dataId, xbus.latlon);
      break;
    case 0x5020: // AltitudeEllipsoid
      {
        std::array<double, 1> altitude{};
        xbus.altitudeAvailable = read_reals (p, len, dataId, altitude);
        xbus.altitude = altitude[0];
        break;
      }
    case 0xC020: // MagneticField
      xbus.magAvailable = read_reals (p, len, dataId, xbus.mag);
      break;
    case 0xD010: // VelocityXYZ
      xbus.velocityAvailable = read_reals (p, len, dataId, xbus.vel);
      break;
    case 0xE020: // StatusWord
      if (len == 4)
        {
          xbus.statusWord = be_u32 (p);
          xbus.statusWordAvailable = true;
        }
      break;
    default: // Unknown items are skipped using their length field.
      break;
    }
}

// Maps a vector from the given local frame to {east, north, up}.
std::array<double, 3>
to_east_north_up (double x, double y, double z, xbus_frame frame)
{
  switch (frame)
    {
    case xbus_frame::NED:
      return { y, x, -z };
    case xbus_frame::NWU:
      return { -y, x, z };
    case xbus_frame::ENU:
    default:
      return { x, y, z };
    }
}
} // namespace

std::optional<std::array<double, 3>>
Xbus::accelerationENU (double gravity) const
{
  if (freeAccAvailable)
    {
      return to_east_north_up (freeAcc[0], freeAcc[1], freeAcc[2],
                               freeAccFrame);
    }
  if (accAvailable && quaternionAvailable)
    {
      // v' = v + 2w (u x v) + 2 u x (u x v), u = (x, y, z)
      const auto &[w, x, y, z] = quat;
      const auto &[vx, vy, vz] = acc;
      const double tx = 2 * (y * vz - z * vy);
      const double ty = 2 * (z * vx - x * vz);
      const double tz = 2 * (x * vy - y * vx);
      const double rx = vx + w * tx + (y * tz - z * ty);
      const double ry = vy + w * ty + (z * tx - x * tz);
      const double rz = vz + w * tz + (x * ty - y * tx);
      // At rest the accelerometer measures gravity's reaction, pointing up.
      auto enu = to_east_north_up (rx, ry, rz, quatFrame);
      enu[2] -= gravity;
      return enu;
    }
  return std::nullopt;
}

std::optional<std::array<double, 2>>
Xbus::horizontalAccelerationEN () const
{
  const auto enu = accelerationENU ();
  if (!enu)
    {
      return std::nullopt;
    }
  return std::array<double, 2>{ (*enu)[0], (*enu)[1] };
}

bool
DataPacketParser::parseDataPacket (const std::vector<uint8_t> &packet,
                                   Xbus &xbusData)
{
  // FA FF 36 LEN <items> CS   or   FA FF 36 FF LENH LENL <items> CS
  if (packet.size () < 5 || packet[2] != MID_MTDATA2)
    {
      return false;
    }

  const bool extended = packet[3] == 0xFF;
  const std::size_t start = extended ? 6 : 4;
  if (packet.size () < start + 1)
    {
      return false;
    }
  const std::size_t length = extended ? be_u16 (&packet[4]) : packet[3];
  if (start + length + 1 != packet.size ())
    {
      return false;
    }

  const std::size_t end = start + length;
  std::size_t offset = start;
  while (offset + 3 <= end)
    {
      const auto dataId = be_u16 (&packet[offset]);
      const std::size_t itemLength = packet[offset + 2];
      offset += 3;
      if (offset + itemLength > end)
        {
          return false;
        }
      parse_item (xbusData, dataId, &packet[offset], itemLength);
      offset += itemLength;
    }
  return offset == end;
}
