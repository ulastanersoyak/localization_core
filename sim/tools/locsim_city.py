#!/usr/bin/env python3
"""Prepares open 3D city data as scenery for the AirSim maps.

Downloads the building tiles around a point, converts them from the city's
national grid to a local ENU frame (metres from that point, ground at z = 0)
and writes one .glb per tile, which airsim/maps/build_maps.py imports into
Unreal with its glTF importer:

    locsim_city.py antwerp --center 51.2192 4.4015 --radius 300 \\
        --out airsim/data/antwerp

The point must be the OriginGeopoint of the AirSim settings (--settings FILE
reads it from there), so the buildings and AirSim's GPS agree.

The coordinate conversion is PROJ's (pyproj): the city's grid to WGS84 with
the transformation EPSG recommends, then to the local frame with PROJ's
topocentric conversion. Downloads are cached in the output folder, so running
it again (another radius, another centre) is quick. Needs pyproj (installed
in localization_core/airsim/.venv by airsim/setup.sh).

Sources are a small table (SOURCES); Antwerp is the first. Another city
needs an index of tiles, a URL pattern and its grid's EPSG code.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import pathlib
import struct
import sys
import urllib.request
from array import array
from dataclasses import dataclass

from pyproj import Transformer

# ---------------------------------------------------------------------------
# Sources


@dataclass(frozen=True)
class Source:
    name: str
    index_url: str  # GeoJSON (CRS84) of tiles with a property naming the tile
    tile_property: str
    tile_url: str  # format string with {tile}
    crs: str  # the grid of the files, as PROJ knows it
    attribution: str
    licence: str
    # glTF axes of the source files: the grid's (east, north, height) are
    # glTF (x, -z, y), as FME writes them.


SOURCES = {
    "antwerp": Source(
        name="antwerp",
        index_url="https://s3-ant1.antwerpen.be/prd-3d/3d.geojson",
        tile_property="SECCODE",
        tile_url="https://s3-ant1.antwerpen.be/prd-open-data-datatank/3D/glb/seccode_{tile}.glb",
        crs="EPSG:31370",  # Belgian Lambert 72; heights in TAW
        attribution="Stad Antwerpen, Antwerpen in 3D",
        licence="Modellicentie Gratis Hergebruik (Flemish Open Data Licence)",
    ),
}

# ---------------------------------------------------------------------------
# glb


def read_glb(data: bytes):
    """Yields (positions, indices) per triangle primitive, raw file coords.

    positions: array('f') x, y, z; indices: list of ints. Node transforms
    other than none are not expected in city tiles and are refused.
    """
    magic, version, _ = struct.unpack_from("<III", data, 0)
    if magic != 0x46546C67 or version != 2:
        raise ValueError("not a glTF 2.0 binary file")
    pos, doc, bin_chunk = 12, None, b""
    while pos + 8 <= len(data):
        length, kind = struct.unpack_from("<II", data, pos)
        chunk = data[pos + 8 : pos + 8 + length]
        if kind == 0x4E4F534A:
            doc = json.loads(chunk)
        elif kind == 0x004E4942:
            bin_chunk = chunk
        pos += 8 + ((length + 3) & ~3)
    if doc is None:
        raise ValueError("glb without json")
    for node in doc.get("nodes", []):
        if any(k in node for k in ("matrix", "rotation", "scale", "translation")):
            raise ValueError("node transforms are not supported for city tiles")

    def accessor(i):
        a = doc["accessors"][i]
        bv = doc["bufferViews"][a["bufferView"]]
        offset = bv.get("byteOffset", 0) + a.get("byteOffset", 0)
        fmt = {5121: "B", 5123: "H", 5125: "I", 5126: "f"}[a["componentType"]]
        width = {"SCALAR": 1, "VEC3": 3}[a["type"]]
        size = struct.calcsize(fmt)
        stride = bv.get("byteStride", size * width)
        if stride == size * width:
            out = array(fmt)
            out.frombytes(bin_chunk[offset : offset + a["count"] * stride])
            return out
        out = array(fmt)
        for k in range(a["count"]):
            out.extend(struct.unpack_from("<" + fmt * width, bin_chunk, offset + k * stride))
        return out

    for mesh in doc.get("meshes", []):
        for prim in mesh.get("primitives", []):
            if prim.get("mode", 4) != 4 or "POSITION" not in prim.get("attributes", {}):
                continue
            positions = accessor(prim["attributes"]["POSITION"])
            if "indices" in prim:
                indices = accessor(prim["indices"]).tolist()
            else:
                indices = list(range(len(positions) // 3))
            yield positions, indices


def write_glb(path: pathlib.Path, positions: array, indices: array, extras: dict):
    """One mesh, one triangle primitive, uint32 indices."""
    pos_bytes = positions.tobytes()
    idx_bytes = indices.tobytes()
    xs, ys, zs = positions[0::3], positions[1::3], positions[2::3]
    doc = {
        "asset": {"version": "2.0", "generator": "locsim_city.py", "extras": extras},
        "scene": 0,
        "scenes": [{"nodes": [0]}],
        "nodes": [{"mesh": 0, "name": path.stem}],
        "meshes": [{"primitives": [{"attributes": {"POSITION": 0}, "indices": 1, "mode": 4}]}],
        "accessors": [
            {
                "bufferView": 0,
                "componentType": 5126,
                "count": len(positions) // 3,
                "type": "VEC3",
                "min": [min(xs), min(ys), min(zs)],
                "max": [max(xs), max(ys), max(zs)],
            },
            {"bufferView": 1, "componentType": 5125, "count": len(indices), "type": "SCALAR"},
        ],
        "bufferViews": [
            {"buffer": 0, "byteOffset": 0, "byteLength": len(pos_bytes), "target": 34962},
            {"buffer": 0, "byteOffset": len(pos_bytes), "byteLength": len(idx_bytes), "target": 34963},
        ],
        "buffers": [{"byteLength": len(pos_bytes) + len(idx_bytes)}],
    }
    text = json.dumps(doc, separators=(",", ":")).encode()
    text += b" " * (-len(text) % 4)
    body = pos_bytes + idx_bytes
    body += b"\0" * (-len(body) % 4)
    with open(path, "wb") as f:
        f.write(struct.pack("<III", 0x46546C67, 2, 12 + 8 + len(text) + 8 + len(body)))
        f.write(struct.pack("<II", len(text), 0x4E4F534A) + text)
        f.write(struct.pack("<II", len(body), 0x004E4942) + body)


# ---------------------------------------------------------------------------
# Tile selection


def _ring_points(geometry):
    coords = geometry["coordinates"]
    polys = coords if geometry["type"] == "MultiPolygon" else [coords]
    for poly in polys:
        for ring in poly:
            yield ring


def _inside(lon, lat, ring):
    hit = False
    for (x1, y1), (x2, y2) in zip(ring, ring[1:] + ring[:1]):
        if (y1 > lat) != (y2 > lat) and lon < x1 + (lat - y1) * (x2 - x1) / (y2 - y1):
            hit = not hit
    return hit


def tile_distance_m(feature, lat0, lon0):
    """Distance from the point to the tile polygon (0 inside), flat earth."""
    kx = 111320 * math.cos(math.radians(lat0))
    ky = 110540
    best = math.inf
    inside = False
    for ring in _ring_points(feature["geometry"]):
        inside = inside or _inside(lon0, lat0, [tuple(p[:2]) for p in ring])
        pts = [((p[0] - lon0) * kx, (p[1] - lat0) * ky) for p in ring]
        for (x1, y1), (x2, y2) in zip(pts, pts[1:] + pts[:1]):
            dx, dy = x2 - x1, y2 - y1
            t = max(0.0, min(1.0, -(x1 * dx + y1 * dy) / (dx * dx + dy * dy or 1)))
            best = min(best, math.hypot(x1 + t * dx, y1 + t * dy))
    return 0.0 if inside else best


def fetch(url: str, path: pathlib.Path) -> bytes:
    if path.exists() and path.stat().st_size > 0:
        return path.read_bytes()
    print(f"  downloading {url}", file=sys.stderr)
    req = urllib.request.Request(url, headers={"User-Agent": "locsim_city/1"})
    with urllib.request.urlopen(req, timeout=120) as r:
        data = r.read()
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(path.suffix + ".part")
    tmp.write_bytes(data)
    tmp.replace(path)
    return data


# ---------------------------------------------------------------------------


def prepare(source: Source, lat0, lon0, radius, out: pathlib.Path, ground=None, ground_radius=150.0):
    cache = out / "cache"
    index = json.loads(fetch(source.index_url, cache / "index.geojson"))
    tiles = sorted(
        (f["properties"][source.tile_property], d)
        for f in index["features"]
        if (d := tile_distance_m(f, lat0, lon0)) <= radius
    )
    if not tiles:
        raise SystemExit("no tiles within the radius; is the centre inside the city?")
    print(f"{len(tiles)} tiles within {radius:.0f} m", file=sys.stderr)

    # The centre in the grid, for cropping.
    e0, n0 = Transformer.from_crs("EPSG:4326", source.crs, always_xy=True).transform(lon0, lat0)

    raw = []  # (tile, [(positions, indices)])
    for tile, _ in tiles:
        data = fetch(source.tile_url.format(tile=tile), cache / f"{tile}.glb")
        raw.append((tile, list(read_glb(data))))

    if ground is None:
        bases = []
        for _, prims in raw:
            for positions, _ in prims:
                cx = (min(positions[0::3]) + max(positions[0::3])) / 2
                cn = -(min(positions[2::3]) + max(positions[2::3])) / 2
                if math.hypot(cx - e0, cn - n0) < ground_radius:
                    bases.append(min(positions[1::3]))
        if not bases:
            raise SystemExit("no buildings near the centre to estimate the ground height, pass --ground")
        bases.sort()
        ground = bases[len(bases) // 2]
    print(f"ground height {ground:.2f} m (grid heights)", file=sys.stderr)

    # Grid -> WGS84 (EPSG's recommended transformation), then -> ENU at the
    # centre (PROJ's topocentric conversion). The grid's heights (above sea
    # level) stand in for ellipsoidal heights on both sides, so the ground
    # height maps to z = 0: only height differences matter here.
    to_lonlat = Transformer.from_crs(source.crs, "EPSG:4326", always_xy=True)
    to_enu = Transformer.from_pipeline(
        "+proj=pipeline +step +proj=unitconvert +xy_in=deg +xy_out=rad "
        "+step +proj=cart +ellps=WGS84 "
        f"+step +proj=topocentric +ellps=WGS84 +lon_0={lon0} +lat_0={lat0} +h_0={ground}")

    out.mkdir(parents=True, exist_ok=True)
    files, total_tris, total_buildings = [], 0, 0
    for tile, prims in raw:
        positions_out, indices_out = array("f"), array("I")
        for positions, indices in prims:
            count = len(positions) // 3
            cx = (min(positions[0::3]) + max(positions[0::3])) / 2
            cn = -(min(positions[2::3]) + max(positions[2::3])) / 2
            if math.hypot(cx - e0, cn - n0) > radius:
                continue
            base = len(positions_out) // 3
            # source glTF: x = easting, y = height, z = -northing
            lon, lat = to_lonlat.transform(positions[0::3], [-v for v in positions[2::3]])
            e, n, u = to_enu.transform(lon, lat, list(positions[1::3]))
            for i in range(count):
                positions_out.extend((e[i], u[i], -n[i]))  # glTF axes
            indices_out.extend(base + k for k in indices)
            total_buildings += 1
        if not indices_out:
            continue
        name = f"{tile}.glb"
        write_glb(
            out / name,
            positions_out,
            indices_out,
            {
                "source": source.name,
                "tile": tile,
                "attribution": source.attribution,
                "licence": source.licence,
                "origin": [lat0, lon0],
                "ground_height_grid_m": ground,
            },
        )
        files.append(name)
        total_tris += len(indices_out) // 3
    manifest = {
        "source": source.name,
        "attribution": source.attribution,
        "licence": source.licence,
        "center": [lat0, lon0],
        "radius_m": radius,
        "ground_height_grid_m": ground,
        "files": files,
        "buildings": total_buildings,
        "triangles": total_tris,
    }
    (out / "city.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return manifest


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("source", choices=sorted(SOURCES))
    ap.add_argument("--center", nargs=2, type=float, metavar=("LAT", "LON"), help="scenario origin (WGS84 degrees)")
    ap.add_argument("--settings", type=pathlib.Path, help="take the centre from an AirSim settings.json (OriginGeopoint)")
    ap.add_argument("--radius", type=float, default=350.0, help="metres around the centre (default 350)")
    ap.add_argument("--out", type=pathlib.Path, required=True, help="output folder (tiles are cached in OUT/cache)")
    ap.add_argument("--ground", type=float, help="ground height in the grid's heights (default: median building base)")
    args = ap.parse_args(argv)

    if args.settings:
        origin = json.load(open(args.settings))["OriginGeopoint"]
        lat0, lon0 = origin["Latitude"], origin["Longitude"]
    elif args.center:
        lat0, lon0 = args.center
    else:
        ap.error("give --center LAT LON or --settings FILE")

    m = prepare(SOURCES[args.source], lat0, lon0, args.radius, args.out, args.ground)
    print(f"{m['buildings']} buildings, {m['triangles']} triangles in {len(m['files'])} files", file=sys.stderr)
    print(f"attribution: {m['attribution']} ({m['licence']})", file=sys.stderr)

if __name__ == "__main__":
    main()
