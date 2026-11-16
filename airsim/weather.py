#!/usr/bin/env python3
"""Sets AirSim's weather and wind (Cosys-AirSim's own API, nothing else).

    weather.py --rain 0.6 --fog 0.2 --wind 6@270

Options are AirSim's weather parameters, each 0..1 (WeatherParameter in its
Python client): --rain, --snow, --fog, --dust, --leaves, --road-wetness;
--wind SPEED[@FROM_DEG] in m/s, the direction it blows from (clockwise from
north, 225 = south-west), sent as AirSim's NED wind vector. airsim/run.sh
calls this once the simulator is up; it can also be run while flying.
"""
import argparse
import math

import cosysairsim as airsim


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    for name in ("rain", "snow", "fog", "dust", "leaves", "road-wetness"):
        ap.add_argument("--" + name, type=float, metavar="0..1")
    ap.add_argument("--wind", metavar="SPEED[@FROM_DEG]")
    a = ap.parse_args()

    client = airsim.VehicleClient()
    client.confirmConnection()
    params = {
        "rain": airsim.WeatherParameter.Rain,
        "snow": airsim.WeatherParameter.Snow,
        "fog": airsim.WeatherParameter.Fog,
        "dust": airsim.WeatherParameter.Dust,
        "leaves": airsim.WeatherParameter.MapleLeaf,
        "road_wetness": airsim.WeatherParameter.Roadwetness,
    }
    values = {k: getattr(a, k) for k in params if getattr(a, k) is not None}
    if values:
        client.simEnableWeather(True)
        for key, value in values.items():
            client.simSetWeatherParameter(params[key], max(0.0, min(1.0, value)))
            print(f"weather: {key} {value}")
    if a.wind:
        speed, _, came_from = a.wind.partition("@")
        speed, came_from = float(speed), math.radians(float(came_from or 225))
        # Blows towards from + 180 deg; NED: x north, y east.
        north, east = -speed * math.cos(came_from), -speed * math.sin(came_from)
        client.simSetWind(airsim.Vector3r(north, east, 0))
        print(f"wind: {speed} m/s from {math.degrees(came_from):.0f} deg")


if __name__ == "__main__":
    main()
