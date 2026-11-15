# NASA terrain preparation

`fetch_mola_jezero.py` downloads a compact, observed MOLA topography window around Jezero from the NASA PDS Geosciences Node. It uses range requests against `MEGT44N000HB.IMG`, so the 124 MiB PDS source raster remains remote.

```sh
python3 tools/nasa-data/fetch_mola_jezero.py --size 32
```

The output is `data/real/jezero-mola-128.json`. It includes product ID, source URLs, coordinate system, vertical datum, native sampling and the precise extraction method. MOLA samples are approximately 463 m apart at the equator and are unsuitable for astronaut-scale hazard certification. The script rejects windows with missing MOLA samples instead of filling them.

For high-resolution planning, provide a co-registered HiRISE DTM and orthoimage to `data/prepare_terrain.py`; retain the original product metadata in the generated record.

## HiRISE Jezero local terrain

`hirise-jezero-045994-046060.json` records a verified co-registered Jezero stereo pair: `DTEEC_045994_1985_046060_1985_U01.IMG` and `ESP_045994_1985_RED_C_01_ORTHO.JP2`. The DTM is about 381 MB and the 1 m orthorectified image is about 34 MB, so neither is committed.

Extract a DTM window and align its matching orthophoto with:

```sh
python3 tools/nasa-data/extract_hirise_subset.py \
  --dtm /path/to/DTEEC_045994_1985_046060_1985_U01.IMG \
  --ortho /path/to/ESP_045994_1985_RED_C_01_ORTHO.JP2 \
  --row 4000 --col 3000 --size 512
```

The DTM window is unchanged. The original JP2 embedded affine differs from the DTM by 1.5 pixels; image pixels are bilinearly resampled onto the DTM sample-center grid. The earlier extraction rounded this window and assigned the DTM transform without resampling; regenerate older assets before using them as textures.

The two original PDS labels specify the same planetocentric equirectangular projection (sphere radius 3394839.8133163 m, center latitude 15°, longitude 77.44°E). GDAL interprets the JP2 CRS keys differently; for this verified product pair only, extraction normalizes the CRS to the DTM definition while retaining the source embedded affine for sampling. Other unverified CRS mismatches are rejected. Original and resulting transforms, the label URL, and resampling method are saved in `provenance.json`.

No elevation is interpolated. Image DN 0 is masked as missing; saturation values are preserved. The GPU uses the RED-band grayscale display DN values, not a calibrated reflectance visualization. Linear texture filtering and modest directional lighting are visual operations. Generated assets live in ignored `data/local/`.

Verify actual asset alignment and unchanged elevations against independent source-pixel calculations:

```sh
.venv/bin/python tools/nasa-data/test_texture_alignment.py
```
