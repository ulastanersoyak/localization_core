#ifndef GPS_HPP
#define GPS_HPP

#include "kalman/sensor_event.hpp"
#include "utility/queue.hpp"
#include "utility/serial_port.hpp"

#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <stop_token>
#include <string>
#include <string_view>

class gps
{
  std::unique_ptr<serial_port> port_{};
  std::ifstream replay_{};
  std::string pending_{};
  std::optional<double> hdop_{};
  // Latest GGA, and an RMC fix waiting for the GGA of its epoch.
  std::string gga_time_{};
  std::optional<double> gga_altitude_m_{};
  std::optional<gps_fix> pending_fix_{};
  std::string pending_time_{};

  // Next line from the port or the replay file. nullopt when stopped or when
  // the replay file is exhausted.
  std::optional<std::string> next_line (const std::stop_token &stop);

public:
  // Live receiver. Throws boost::system::system_error if the port cannot be
  // opened.
  gps (std::string_view port_name, std::uint32_t baud_rate,
       boost::asio::serial_port_base::parity::type parity,
       boost::asio::serial_port_base::stop_bits::type stop_bits,
       boost::asio::serial_port_base::character_size char_size);

  // Replays a recorded NMEA log, one sentence per line (anything before '$'
  // is ignored). Throws std::runtime_error if the file cannot be opened.
  explicit gps (const std::filesystem::path &replay_file);

  // Feeds one NMEA sentence. Altitude and HDOP are only in GGA, which
  // receivers send just after the RMC of the same epoch, so a valid RMC is
  // held until its GGA arrives and that sentence returns the fix. An altitude
  // from the previous epoch would be a second stale on a climbing vehicle. If
  // the GGA came first, or no GGA was ever seen, the RMC returns the fix.
  [[nodiscard]] std::optional<gps_fix>
  process_sentence (std::string_view sentence);

  // Pushes fixes until stopped. A replay pushes end_of_stream when done.
  void work (const std::stop_token &stop, queue<sensor_event> &events);
};

#endif // GPS_HPP
