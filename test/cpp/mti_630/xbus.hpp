#ifndef XBUS_TEST_HPP
#define XBUS_TEST_HPP

#include "check.hpp"
#include "mti_630/data_packet_parser.hpp"
#include "mti_630/xbus_packet.hpp"

#include <bit>
#include <cstdint>
#include <vector>

namespace xbus_test
{
void
append_u16 (std::vector<std::uint8_t> &out, std::uint16_t v)
{
  out.push_back (static_cast<std::uint8_t> (v >> 8));
  out.push_back (static_cast<std::uint8_t> (v));
}

void
append_u32 (std::vector<std::uint8_t> &out, std::uint32_t v)
{
  append_u16 (out, static_cast<std::uint16_t> (v >> 16));
  append_u16 (out, static_cast<std::uint16_t> (v));
}

void
append_item (std::vector<std::uint8_t> &out, std::uint16_t id,
             const std::vector<std::uint8_t> &data)
{
  append_u16 (out, id);
  out.push_back (static_cast<std::uint8_t> (data.size ()));
  out.insert (out.end (), data.begin (), data.end ());
}

std::vector<std::uint8_t>
floats (std::initializer_list<float> values)
{
  std::vector<std::uint8_t> out;
  for (const float v : values)
    {
      append_u32 (out, std::bit_cast<std::uint32_t> (v));
    }
  return out;
}

std::vector<std::uint8_t>
fp1632 (std::initializer_list<double> values)
{
  std::vector<std::uint8_t> out;
  for (const double v : values)
    {
      const auto fixed = static_cast<std::int64_t> (v * 4294967296.0);
      append_u32 (out, static_cast<std::uint32_t> (fixed & 0xFFFFFFFF));
      append_u16 (out, static_cast<std::uint16_t> (fixed >> 32));
    }
  return out;
}

std::vector<std::uint8_t>
mtdata2 (const std::vector<std::uint8_t> &payload)
{
  std::vector<std::uint8_t> packet{ 0xFA, 0xFF, 0x36 };
  if (payload.size () < 0xFF)
    {
      packet.push_back (static_cast<std::uint8_t> (payload.size ()));
    }
  else
    {
      packet.push_back (0xFF);
      append_u16 (packet, static_cast<std::uint16_t> (payload.size ()));
    }
  packet.insert (packet.end (), payload.begin (), payload.end ());
  std::uint8_t sum = 0;
  for (std::size_t i = 1; i < packet.size (); ++i)
    {
      sum = static_cast<std::uint8_t> (sum + packet[i]);
    }
  packet.push_back (static_cast<std::uint8_t> (0x100 - sum));
  return packet;
}

// Feeds bytes and returns every complete packet with a valid checksum, the
// same way mti_630::work does.
std::vector<std::vector<std::uint8_t>>
frame (const std::vector<std::uint8_t> &bytes)
{
  std::vector<std::vector<std::uint8_t>> packets;
  XbusPacket packet;
  for (const auto byte : bytes)
    {
      packet.feedByte (byte);
      while (packet.isPacketComplete ())
        {
          if (packet.validateChecksum ())
            {
              packets.push_back (packet.getRawData ());
              packet.reset ();
            }
          else
            {
              packet.resync ();
            }
        }
    }
  return packets;
}
} // namespace xbus_test

void
xbus_parse_test ()
{
  using namespace xbus_test;

  std::vector<std::uint8_t> payload;
  std::vector<std::uint8_t> tick;
  append_u32 (tick, 123456);
  append_item (payload, 0x1060, tick);
  append_item (payload, 0x4030, floats ({ 1.5F, -2.25F, 0.5F }));
  append_item (payload, 0x8020, floats ({ 0.1F, 0.2F, 0.3F }));
  append_item (payload, 0x2010, floats ({ 1.0F, 0.0F, 0.0F, 0.0F }));
  append_item (payload, 0x5042, fp1632 ({ -33.5, 151.25 }));
  append_item (payload, 0x0810, { 1, 2, 3 }); // unknown, must be skipped
  const auto packet = mtdata2 (payload);

  Xbus xbus{};
  check (DataPacketParser::parseDataPacket (packet, xbus), "xbus parse");
  check (xbus.sampleTimeFineAvailable && xbus.sampleTimeFine == 123456,
         "xbus sample time fine");
  check (xbus.freeAccAvailable, "xbus free acceleration available");
  check_near (xbus.freeAcc[0], 1.5, 1e-6, "xbus free acc x");
  check_near (xbus.freeAcc[1], -2.25, 1e-6, "xbus free acc y");
  check_near (xbus.freeAcc[2], 0.5, 1e-6, "xbus free acc z");
  check_near (xbus.rot[0], 0.1, 1e-6, "xbus rate of turn x (axis order)");
  check_near (xbus.rot[2], 0.3, 1e-6, "xbus rate of turn z (axis order)");
  check (xbus.quaternionAvailable && xbus.eulerAvailable, "xbus quaternion");
  check_near (xbus.latlon[0], -33.5, 1e-9, "xbus negative fp16.32");
  check_near (xbus.latlon[1], 151.25, 1e-9, "xbus fp16.32");

  const auto acc = xbus.horizontalAccelerationEN ();
  check (acc.has_value (), "xbus horizontal acceleration");
  check_near ((*acc)[0], 1.5, 1e-6, "xbus acceleration east");
  check_near ((*acc)[1], -2.25, 1e-6, "xbus acceleration north");

  // Raw acceleration rotated by a 90 degree yaw: sensor x points north.
  Xbus rotated{};
  rotated.accAvailable = true;
  rotated.acc = { 2.0, 0.0, 9.81 };
  rotated.quaternionAvailable = true;
  rotated.quat = { std::sqrt (0.5), 0.0, 0.0, std::sqrt (0.5) };
  const auto rotated_acc = rotated.horizontalAccelerationEN ();
  check (rotated_acc.has_value (), "rotated acceleration");
  check_near ((*rotated_acc)[0], 0.0, 1e-9, "rotated acceleration east");
  check_near ((*rotated_acc)[1], 2.0, 1e-9, "rotated acceleration north");

  const auto free_enu = xbus.accelerationENU ();
  check (free_enu.has_value (), "xbus acceleration enu");
  check_near ((*free_enu)[2], 0.5, 1e-6, "free acceleration up as given");

  // Climbing at 2 m/s^2 while level.
  Xbus climbing{};
  climbing.accAvailable = true;
  climbing.acc = { 0.0, 0.0, Xbus::standardGravity + 2.0 };
  climbing.quaternionAvailable = true;
  climbing.quat = { 1.0, 0.0, 0.0, 0.0 };
  const auto climbing_acc = climbing.accelerationENU ();
  check (climbing_acc.has_value (), "climbing acceleration");
  check_near ((*climbing_acc)[2], 2.0, 1e-9, "climbing acceleration up");

  // At rest rolled 90 degrees: gravity is on the sensor y axis and must
  // still be removed from up, not leak into the horizontal.
  Xbus rolled{};
  rolled.accAvailable = true;
  rolled.acc = { 0.0, 9.8, 0.0 };
  rolled.quaternionAvailable = true;
  rolled.quat = { std::sqrt (0.5), std::sqrt (0.5), 0.0, 0.0 };
  const auto rolled_acc = rolled.accelerationENU (9.8);
  check (rolled_acc.has_value (), "rolled acceleration");
  check_near ((*rolled_acc)[0], 0.0, 1e-9, "rolled acceleration east");
  check_near ((*rolled_acc)[1], 0.0, 1e-9, "rolled acceleration north");
  check_near ((*rolled_acc)[2], 0.0, 1e-9, "rolled acceleration up");

  // NED: down is positive, so the at-rest reading is -g on z.
  Xbus ned{};
  ned.accAvailable = true;
  ned.acc = { 1.0, 0.5, -Xbus::standardGravity };
  ned.quaternionAvailable = true;
  ned.quat = { 1.0, 0.0, 0.0, 0.0 };
  ned.quatFrame = xbus_frame::NED;
  const auto ned_acc = ned.accelerationENU ();
  check (ned_acc.has_value (), "ned acceleration");
  check_near ((*ned_acc)[0], 0.5, 1e-9, "ned acceleration east");
  check_near ((*ned_acc)[1], 1.0, 1e-9, "ned acceleration north");
  check_near ((*ned_acc)[2], 0.0, 1e-9, "ned acceleration up");

  Xbus ned_free{};
  ned_free.freeAccAvailable = true;
  ned_free.freeAcc = { 0.0, 0.0, 1.5 }; // sinking
  ned_free.freeAccFrame = xbus_frame::NED;
  check_near ((*ned_free.accelerationENU ())[2], -1.5, 1e-9,
              "ned free acceleration up");

  // Corrupted length must not read past the end.
  auto truncated = packet;
  truncated[3] = static_cast<std::uint8_t> (truncated[3] + 10);
  Xbus ignored{};
  check (!DataPacketParser::parseDataPacket (truncated, ignored),
         "xbus bad length rejected");
}

void
xbus_framing_test ()
{
  using namespace xbus_test;

  std::vector<std::uint8_t> payload;
  append_item (payload, 0x4030, floats ({ 1.0F, 2.0F, 3.0F }));
  const auto good = mtdata2 (payload);

  auto bad = good;
  bad[6] ^= 0x55; // corrupt data, checksum no longer matches

  // Extended length packet (payload > 254 bytes).
  std::vector<std::uint8_t> big_payload;
  append_item (big_payload, 0x0810, std::vector<std::uint8_t> (200, 0xFA));
  append_item (big_payload, 0x0820, std::vector<std::uint8_t> (200, 0xFF));
  append_item (big_payload, 0x4030, floats ({ 4.0F, 5.0F, 6.0F }));
  const auto extended = mtdata2 (big_payload);

  std::vector<std::uint8_t> stream{ 0x00, 0xFA, 0x12, 0xFA }; // noise
  stream.insert (stream.end (), bad.begin (), bad.end ());
  stream.insert (stream.end (), good.begin (), good.end ());
  stream.insert (stream.end (), extended.begin (), extended.end ());
  stream.insert (stream.end (), good.begin (), good.end ());

  const auto packets = frame (stream);
  check (packets.size () == 3,
         "xbus framing packet count: " + std::to_string (packets.size ()));
  check (packets[0] == good && packets[1] == extended && packets[2] == good,
         "xbus framing content");

  Xbus xbus{};
  check (DataPacketParser::parseDataPacket (packets[1], xbus),
         "xbus extended parse");
  check_near (xbus.freeAcc[2], 6.0, 1e-6, "xbus extended payload");
}

#endif // !XBUS_TEST_HPP
