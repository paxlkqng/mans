#!/usr/bin/env python3
"""Crop an authoritative Mars DTM GeoTIFF into the MANS 65x65 local grid.
The center coordinate must be supplied in the GeoTIFF's own projected CRS.
"""
import argparse
import json
from pathlib import Path
import numpy as np
import rasterio
from rasterio.enums import Resampling
from rasterio.transform import from_bounds
from rasterio.warp import reproject

parser = argparse.ArgumentParser()
parser.add_argument('input', type=Path, help='USGS HiRISE/CTX DTM GeoTIFF')
parser.add_argument('--center-x', type=float, required=True, help='center easting in source CRS metres')
parser.add_argument('--center-y', type=float, required=True, help='center northing in source CRS metres')
parser.add_argument('--output', type=Path, default=Path('public/data/terrain.json'))
args = parser.parse_args()
size, cell = 65, 12
half = (size - 1) * cell / 2
bounds = (args.center_x-half, args.center_y-half, args.center_x+half, args.center_y+half)
with rasterio.open(args.input) as src:
    if src.crs is None:
        raise SystemExit('Source GeoTIFF lacks CRS. Inspect PDS label and georeference before use.')
    if src.res[0] > cell or abs(src.res[1]) > cell:
        raise SystemExit(f'Source resolution {src.res} is coarser than 12 m; cannot imply 12 m detail.')
    if (src.crs.linear_units or '').lower() not in {'metre', 'meter', 'metres', 'meters'}:
        raise SystemExit('Source CRS must use metres. Reproject with planetary CRS first.')
    dest = np.full((size, size), np.nan, dtype=np.float32)
    transform = from_bounds(*bounds, size, size)
    reproject(source=rasterio.band(src, 1), destination=dest, src_transform=src.transform, src_crs=src.crs,
              src_nodata=src.nodata, dst_transform=transform, dst_crs=src.crs, dst_nodata=np.nan,
              resampling=Resampling.bilinear)
    if not np.isfinite(dest).all():
        raise SystemExit('Selected crop has missing pixels. Choose a fully covered local region.')
    # Raster rows run north to south; MANS +z runs south to north.
    values = np.flipud(dest).reshape(-1).astype(float).tolist()
    product = {'size': size, 'cellSize': cell, 'heights': values, 'source': f'{args.input.name} · {src.crs.to_string()} · center ({args.center_x}, {args.center_y})',
               'provenance': 'observed', 'boundsSourceCRS': bounds, 'verticalUnits': src.units[0] or 'unknown',
               'note': 'Resampled from observed DTM; slopes and routes are derived, not NASA-approved.'}
args.output.parent.mkdir(parents=True, exist_ok=True)
args.output.write_text(json.dumps(product, separators=(',', ':')))
print(f'Wrote {args.output} with {len(values)} finite samples')
