#!/usr/bin/env python3
"""Unit tests of sim/tools/locsim_city.py: the .glb reader/writer and the
tile selection (the coordinate conversion is PROJ's).

Run with the AirSim environment, which has pyproj:
    airsim/.venv/bin/python test/python/test_city.py
"""
import math
import os
import pathlib
import sys
import tempfile
import unittest
from array import array

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..", "sim", "tools"))
import locsim_city as city  # noqa: E402


class Glb(unittest.TestCase):
    def test_round_trip(self):
        positions = array("f", [0, 0, 0, 1, 0, 0, 0, 1, -2])
        indices = array("I", [0, 1, 2])
        with tempfile.TemporaryDirectory() as d:
            path = pathlib.Path(d) / "t.glb"
            city.write_glb(path, positions, indices, {"k": 1})
            prims = list(city.read_glb(path.read_bytes()))
        self.assertEqual(len(prims), 1)
        self.assertEqual(list(prims[0][0]), list(positions))
        self.assertEqual(prims[0][1], [0, 1, 2])


class Tiles(unittest.TestCase):
    def test_distance(self):
        square = {"geometry": {"type": "Polygon", "coordinates": [[[4.0, 51.0], [4.01, 51.0], [4.01, 51.01], [4.0, 51.01], [4.0, 51.0]]]}}
        self.assertEqual(city.tile_distance_m(square, 51.005, 4.005), 0.0)
        self.assertAlmostEqual(city.tile_distance_m(square, 51.005, 4.02),
                               0.01 * 111320 * math.cos(math.radians(51.005)), delta=1)


if __name__ == "__main__":
    unittest.main()
