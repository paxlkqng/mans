#!/usr/bin/env python3
"""Fetch a small real MOLA MEGDR topography window around Jezero Crater.

The input is the authoritative PDS3 MOLA 128-pixel/degree tile. The script
uses HTTP range requests, so it does not download the 124 MiB source tile.
Output is MANS terrain JSON with full source metadata and no interpolation.
"""

from __future__ import annotations

import argparse
import json
import struct
from datetime import datetime, timezone
from urllib.request import Request, urlopen

PRODUCT_URL = (
    "https://pds-geosciences.wustl.edu/mgs/mgs-m-mola-5-megdr-l3-v1/"
    "mgsl_300x/meg128/megt44n000hb.img"
)
LABEL_URL = PRODUCT_URL[:-3] + "lbl"
PIXELS_PER_DEGREE = 128
TILE_NORTH_DEG = 44.0
TILE_WEST_DEG = 0.0
LINE_SAMPLES = 11520
RECORD_BYTES = LINE_SAMPLES * 2
MARS_RADIUS_M = 3_396_000.0


def range_get(url: str, start: int, end: int) -> bytes:
    request = Request(url, headers={"Range": f"bytes={start}-{end}"})
    with urlopen(request, timeout=60) as response:
        data = response.read()
    expected = end - start + 1
    if len(data) != expected:
        raise RuntimeError(f"range response has {len(data)} bytes, expected {expected}")
    return data


def sample_window(latitude: float, longitude: float, size: int) -> list[list[int]]:
    if not 0.0 <= longitude < 90.0 or not 0.0 < latitude <= 44.0:
        raise ValueError("window center is outside MEGT44N000HB (0–44°N, 0–90°E)")
    center_row = round((TILE_NORTH_DEG - latitude) * PIXELS_PER_DEGREE)
    center_col = round((longitude - TILE_WEST_DEG) * PIXELS_PER_DEGREE)
    first_row, first_col = center_row - size // 2, center_col - size // 2
    if first_row < 0 or first_col < 0 or first_col + size > LINE_SAMPLES:
        raise ValueError("requested window reaches outside source tile")
    rows: list[list[int]] = []
    for row in range(first_row, first_row + size):
        offset = row * RECORD_BYTES + first_col * 2
        raw = range_get(PRODUCT_URL, offset, offset + size * 2 - 1)
        values = list(struct.unpack(f">{size}h", raw))
        # PDS label documents -32768 as a missing-value sentinel for MOLA maps.
        if any(value == -32768 for value in values):
            raise ValueError(f"MOLA window has missing sample in source row {row}")
        rows.append(values)
    return rows


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--latitude", type=float, default=18.444, help="planetocentric degrees north")
    parser.add_argument("--longitude", type=float, default=77.450, help="east-positive degrees")
    parser.add_argument("--size", type=int, default=32, help="square output size, in MOLA samples")
    parser.add_argument("--output", default="data/real/jezero-mola-128.json")
    args = parser.parse_args()
    if args.size < 2 or args.size > 256:
        raise ValueError("size must be in 2..256")

    heights = sample_window(args.latitude, args.longitude, args.size)
    lat_step = 1.0 / PIXELS_PER_DEGREE
    cell_size_m = MARS_RADIUS_M * lat_step * 3.141592653589793 / 180.0
    payload = {
        "schemaVersion": 1,
        "size": args.size,
        "cellSize": cell_size_m,
        "heights": [value for row in heights for value in row],
        "source": "NASA MGS MOLA MEGDR topography, MEGT44N000HB",
        "provenance": "Observed MOLA median topography; not interpolated by MANS.",
        "metadata": {
            "datasetName": "Mars Global Surveyor MOLA Mission Experiment Gridded Data Record",
            "productId": "MEGT44N000HB.IMG",
            "sourceUrl": PRODUCT_URL,
            "labelUrl": LABEL_URL,
            "instrument": "Mars Orbiter Laser Altimeter (MOLA)",
            "coordinateReferenceSystem": "IAU2000, planetocentric, east-positive, Simple Cylindrical",
            "verticalDatum": "MOLA topography: planetary radius minus GMM3 areoid",
            "resolution": "1/128 degree per sample; approximately 463 m at the equator",
            "acquisition": "1999-02 through 2001-06; product created 2003-03-21",
            "centerLatitudeDeg": args.latitude,
            "centerLongitudeDeg": args.longitude,
            "processing": "HTTP byte-range extraction of signed 16-bit PDS3 samples; no resampling or gap filling",
            "generatedAt": datetime.now(timezone.utc).isoformat(),
        },
    }
    if args.output == "-":
        print(json.dumps(payload, indent=2))
        return
    with open(args.output, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2)
        handle.write("\n")
    print(f"wrote {args.output}: {args.size}×{args.size} observed MOLA samples")


if __name__ == "__main__":
    main()
