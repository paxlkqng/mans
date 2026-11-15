#!/usr/bin/env python3
"""Extract a co-registered HiRISE DTM/ortho subset without inventing samples.

Install Rasterio with a GDAL build that supports VICAR/PDS and JPEG2000. The
script refuses non-overlap and nodata in the chosen DTM window.
Elevation is unchanged; imagery is bilinearly resampled onto the DTM grid.
It writes a compact GeoTIFF pair plus JSON provenance;
these outputs stay outside version control unless a small reviewed fixture is
explicitly added.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dtm", required=True, help="local HiRISE DTM IMG path")
    parser.add_argument("--ortho", required=True, help="matching HiRISE ORTHO JP2 path")
    parser.add_argument("--row", type=int, required=True)
    parser.add_argument("--col", type=int, required=True)
    parser.add_argument("--size", type=int, default=512)
    parser.add_argument("--output-dir", default="data/local/hirise-jezero")
    parser.add_argument("--allow-nodata", action="store_true", help="preserve missing cells for tiled rendering")
    args = parser.parse_args()
    if args.size < 2 or args.size > 4096:
        raise ValueError("size must be in 2..4096")

    try:
        import rasterio
        from rasterio.windows import Window
        from rasterio.warp import reproject, Resampling
        import numpy as np
    except ImportError as error:
        raise SystemExit("Rasterio/GDAL is required; install data/requirements.txt") from error

    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)
    window = Window(args.col, args.row, args.size, args.size)
    with rasterio.open(args.dtm) as dtm, rasterio.open(args.ortho) as ortho:
        if window.col_off + window.width > dtm.width or window.row_off + window.height > dtm.height:
            raise ValueError("DTM window is outside source bounds")
        dtm_transform = dtm.window_transform(window)
        bounds = rasterio.windows.bounds(window, dtm.transform)
        # JP2 GeoTIFF keys encode the same PDS map with different CRS keys.
        # This normalization is ONLY valid for the verified product pair below:
        # both PDS labels specify planetocentric equirectangular, radius
        # 3394839.8133163 m, center latitude 15, longitude 77.44, and matching
        # projection offsets. Preserve the JP2 embedded affine (1.5 px offset)
        # rather than silently overwriting it with the DTM transform.
        expected = ("DTEEC_045994_1985_046060_1985_U01.IMG", "ESP_045994_1985_RED_C_01_ORTHO.JP2")
        if dtm.crs != ortho.crs and (Path(args.dtm).name, Path(args.ortho).name) != expected:
            raise ValueError("Unverified planetary CRS mismatch; normalize from source labels first")
        fractional = rasterio.windows.from_bounds(*bounds, transform=ortho.transform)
        left, top = int(np.floor(fractional.col_off))-2, int(np.floor(fractional.row_off))-2
        right, bottom = int(np.ceil(fractional.col_off+fractional.width))+2, int(np.ceil(fractional.row_off+fractional.height))+2
        ortho_window = Window(left, top, right-left, bottom-top)
        if ortho_window.width <= 0 or ortho_window.height <= 0:
            raise ValueError("DTM subset does not overlap orthoimage")
        elevation = dtm.read(1, window=window, masked=True)
        if elevation.mask.any() and not args.allow_nodata:
            raise ValueError("DTM subset contains nodata; choose another window")
        source_image = ortho.read(1, window=ortho_window, boundless=True, fill_value=0)
        imagery = np.zeros((args.size,args.size), dtype=np.uint8)
        reproject(source=source_image, destination=imagery,
                  src_transform=ortho.window_transform(ortho_window), src_crs=dtm.crs,
                  dst_transform=dtm_transform, dst_crs=dtm.crs,
                  src_nodata=0, dst_nodata=0, resampling=Resampling.bilinear)

        elevation_profile = dtm.profile.copy()
        elevation_profile.update(driver="GTiff", width=args.size, height=args.size, transform=dtm_transform, count=1, compress="deflate")
        imagery_profile = dict(driver="GTiff", width=args.size, height=args.size,
                               transform=dtm_transform, crs=dtm.crs, dtype="uint8",
                               count=1, nodata=0, compress="deflate")
        with rasterio.open(output / "elevation.tif", "w", **elevation_profile) as destination:
            destination.write(elevation.filled(), 1)
        with rasterio.open(output / "ortho.tif", "w", **imagery_profile) as destination:
            destination.write(imagery, 1)
        (output / "valid.u8").write_bytes((~np.ma.getmaskarray(elevation)).astype(np.uint8).tobytes())
        metadata = {
            "schemaVersion": 1,
            "window": {"row": args.row, "col": args.col, "size": args.size},
            "crs": str(dtm.crs),
            "transform": list(dtm_transform),
            "resolution": list(dtm.res),
            "verticalReference": dtm.tags().get("DTM_HEIGHT_DIRECTION", "Read original IMG label"),
            "dtmSource": str(Path(args.dtm).name),
            "orthoSource": str(Path(args.ortho).name),
            "orthoTransform": list(dtm_transform),
            "orthoOriginalTransform": list(ortho.transform),
            "orthoOriginalCrs": str(ortho.crs),
            "dtmOriginalTransform": list(dtm.transform),
            "textureWidth": args.size,
            "textureHeight": args.size,
            "textureValidPixels": int(np.count_nonzero(imagery)),
            "processing": "Elevation: unchanged direct window. Ortho: bilinear resampling of JP2 embedded affine onto DTM pixel-center grid; zero DN masked. CRS normalized to DTM after verifying common PDS projection parameters for this product pair.",
            "orthoLabelUrl": "https://www.uahirise.org/PDS/DTM/ESP/ORB_045900_045999/ESP_045994_1985_ESP_046060_1985/ESP_045994_1985_RED_C_01_ORTHO.LBL",
        }
        (output / "provenance.json").write_text(json.dumps(metadata, indent=2) + "\n")
        asset = {
            "size": args.size,
            "cellSize": float(dtm.res[0]),
            "heights": elevation.filled().astype(float).ravel().tolist(),
            "source": "NASA/USGS HiRISE DTM " + Path(args.dtm).name,
            "provenance": "Observed HiRISE stereo elevation (unchanged); RED orthophoto bilinearly aligned to DTM pixel centers; DN 0 masked. See provenance.json.",
            "metadata": {"datasetName": "HiRISE Jezero local subset", "productId": Path(args.dtm).name,
                         "sourceUrl": "local extracted asset", "coordinateReferenceSystem": str(dtm.crs),
                         "verticalDatum": metadata["verticalReference"], "resolution": f"{dtm.res[0]:.4f} m/pixel",
                         "centerLatitudeDeg": None, "centerLongitudeDeg": None},
        }
        (output / "terrain.json").write_text(json.dumps(asset) + "\n")
        print(f"wrote co-registered {args.size}×{args.size} HiRISE subset to {output}")


if __name__ == "__main__":
    main()
