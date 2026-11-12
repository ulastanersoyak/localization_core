#include "gps/gps.hpp"
#include "kalman/position_estimator.hpp"
#include "kalman/sensor_event.hpp"
#include "mti_630/mti_630.hpp"
#include "utility/queue.hpp"

#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <memory>
#include <optional>
#include <thread>

namespace
{
volatile std::sig_atomic_t stop_requested = 0;

void
handle_signal (int /*signal*/)
{
  stop_requested = 1;
}

template <typename... Ts> struct overloaded : Ts...
{
  using Ts::operator()...;
};

constexpr double prediction_horizon_s = 1.0;
constexpr std::size_t imu_print_divider = 10; // 100 Hz IMU -> 10 Hz output

void
print_header ()
{
  std::cout << "source,fix_utc,age_s,lat,lon,alt,easting,northing,vel_east,"
               "vel_north,vel_up,sigma_east,sigma_north,sigma_up,gps_easting,"
               "gps_northing,gps_alt,ahead_easting,ahead_northing,ahead_alt,"
               "status\n";
}

void
print_row (std::string_view source, const position_estimator &estimator,
           const gps_fix *fix, std::string_view status)
{
  const auto est = estimator.estimate ();
  const auto ahead = estimator.predict_ahead (prediction_horizon_s);
  if (!est || !ahead)
    {
      return;
    }
  const auto [lat, lon] = est->position.to_latlon ();

  std::cout << std::fixed << source << ',' << est->position.time () << ','
            << std::setprecision (2) << est->age_s << ','
            << std::setprecision (8) << lat << ',' << lon << ','
            << std::setprecision (3) << est->altitude_m << ','
            << est->position.easting () << ',' << est->position.northing ()
            << ',' << est->velocity_east_mps << ',' << est->velocity_north_mps
            << ',' << est->velocity_up_mps << ',' << est->sigma_east_m << ','
            << est->sigma_north_m << ',' << est->sigma_up_m << ',';
  if (fix)
    {
      std::cout << fix->position.easting () << ',' << fix->position.northing ()
                << ',';
      if (fix->altitude_m)
        {
          std::cout << *fix->altitude_m;
        }
    }
  else
    {
      std::cout << ",,";
    }
  std::cout << ',' << ahead->position.easting () << ','
            << ahead->position.northing () << ',' << ahead->altitude_m << ','
            << status << '\n';
}

std::string_view
to_string (position_estimator::fix_result result)
{
  switch (result)
    {
    case position_estimator::fix_result::initialized:
      return "initialized";
    case position_estimator::fix_result::accepted:
      return "accepted";
    case position_estimator::fix_result::rejected:
      return "rejected";
    case position_estimator::fix_result::reset:
      return "reset";
    }
  return "unknown";
}
} // namespace

int
main (int argc, char *argv[])
{
  // clang-format off
  // header,  utc,   fix, latitude  , longitude   ,knot ,,date  ,mode, checksum
  // $GNRMC,072336.00,A,3958.59380,N,03244.28403,E,0.053,,260724,,,A,V*18
  // clang-format on

  if (argc > 2)
    {
      std::cerr << "usage: " << argv[0] << " [nmea_replay_file]\n";
      return 2;
    }
  const bool replay = argc == 2;

  std::signal (SIGINT, handle_signal);
  std::signal (SIGTERM, handle_signal);

  queue<sensor_event> events{};
  std::unique_ptr<gps> gps_{};
  std::unique_ptr<mti_630> mti{};

  try
    {
      if (replay)
        {
          gps_ = std::make_unique<gps> (argv[1]);
        }
      else
        {
          //           GPIO 14-15 | 14->tx 15->rx | 1hz
          gps_ = std::make_unique<gps> (
              "/dev/ttyAMA0", 9600,
              boost::asio::serial_port_base::parity::none,
              boost::asio::serial_port_base::stop_bits::one,
              boost::asio::serial_port_base::character_size (8));
        }
    }
  catch (const std::exception &e)
    {
      std::cerr << "cannot open gps: " << e.what () << "\n";
      return 1;
    }

  // A replayed GPS log has no matching IMU data, so the IMU is live only.
  if (!replay)
    {
      try
        {
          //           GPIO 8-9 | 9->tx 8->rx | 100 hz
          mti = std::make_unique<mti_630> (
              "/dev/ttyAMA4", 115200,
              boost::asio::serial_port_base::parity::none,
              boost::asio::serial_port_base::stop_bits::one,
              boost::asio::serial_port_base::character_size (8));
        }
      catch (const std::exception &e)
        {
          std::cerr << "cannot open imu (" << e.what ()
                    << "), running gps only\n";
        }
    }

  // Declared after the devices so they are stopped and joined first.
  std::jthread gps_thread ([&] (const std::stop_token &stop)
                             { gps_->work (stop, events); });
  std::jthread mti_thread{};
  if (mti)
    {
      mti_thread = std::jthread ([&] (const std::stop_token &stop)
                                   { mti->work (stop, events); });
    }

  position_estimator estimator{};
  std::size_t imu_samples{ 0 };
  print_header ();

  while (stop_requested == 0)
    {
      auto event = events.pop_for (std::chrono::milliseconds (200));
      if (!event)
        {
          continue;
        }
      if (std::holds_alternative<end_of_stream> (*event))
        {
          break;
        }

      std::visit (
          overloaded{
              [&] (const gps_fix &fix)
                {
                  const auto result = estimator.add_gps_fix (fix);
                  print_row ("gps", estimator, &fix, to_string (result));
                },
              [&] (const Xbus &sample)
                {
                  if (estimator.add_imu_sample (sample)
                      && ++imu_samples % imu_print_divider == 0)
                    {
                      print_row ("imu", estimator, nullptr, "predicted");
                    }
                },
              [] (const end_of_stream &) {},
          },
          *event);
    }

  std::cout.flush ();
  return 0;
}
