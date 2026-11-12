#ifndef GPS_TEST_HPP
#define GPS_TEST_HPP

#include "check.hpp"
#include "gps/gps.hpp"

#include <filesystem>

void
gps_altitude_pairing_test ()
{
  gps receiver{ std::filesystem::path{ "/dev/null" } };

  // Before any GGA was seen there is nothing to wait for.
  const auto first = receiver.process_sentence (
      "$GNRMC,072223.00,A,3958.59211,N,03244.28744,E,0.139,,260724,,,A,V*19");
  check (first.has_value (), "rmc without a gga stream is not held");
  check (!first->altitude_m, "no altitude without a gga");
  check (!receiver.process_sentence ("$GNGGA,072223.00,3958.59211,N,"
                                     "03244.28744,E,1,05,2.30,894.2,M,36.2,M,,"
                                     "*45"),
         "gga without a waiting rmc gives no fix");

  // The usual order: the RMC waits for the GGA of its own epoch.
  check (!receiver.process_sentence (
             "$GNRMC,072224.00,A,3958.59177,N,03244.28699,E,0.384,237.98,"
             "260724,,,A,V*01"),
         "rmc held for its gga");
  const auto paired = receiver.process_sentence (
      "$GNGGA,072224.00,3958.59177,N,03244.28699,E,1,05,2.30,894.0,M,36.2,M,,"
      "*42");
  check (paired.has_value (), "gga completes the fix");
  check_near (*paired->altitude_m, 894.0, 1e-9, "altitude of the same epoch");
  check_near (*paired->hdop, 2.30, 1e-9, "hdop of the same epoch");

  // The other order: the GGA of this epoch is already there.
  (void)receiver.process_sentence ("$GNGGA,072225.00,3958.59210,N,"
                                   "03244.28733,E,1,05,2.30,893.6,M,36.2,M,,"
                                   "*41");
  const auto gga_first = receiver.process_sentence (
      "$GNRMC,072225.00,A,3958.59210,N,03244.28733,E,0.056,,260724,,,A,V*16");
  check (gga_first.has_value (), "rmc after its gga is not held");
  check_near (*gga_first->altitude_m, 893.6, 1e-9, "gga first altitude");
}

#endif // !GPS_TEST_HPP
