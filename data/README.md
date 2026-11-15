# Mars terrain data pipeline

The default browser demo uses procedural **simulated** terrain. To replace it with observed topography, download a Jezero DTM GeoTIFF from the [USGS Mars 2020 Terrain Relative Navigation HiRISE DTM mosaic](https://astrogeology.usgs.gov/search/map/mars_2020_terrain_relative_navigation_hirise_dtm_mosaic). The full image is large; keep it outside Git. A [MOLA MEGDR](https://pds-geosciences.wustl.edu/missions/mgs/megdr.html) crop is useful for regional context but too coarse for a 12 m local route grid.

The USGS mosaic uses a Mars equirectangular projected coordinate system. Read the GeoTIFF CRS, units, pixel size, no-data value, vertical datum, and coverage with `gdalinfo` or Rasterio before choosing a crop. The browser's displayed latitude/longitude is an approximate local planetocentric, east-positive conversion centered at 18.444°N, 77.450°E. For a scientifically aligned crop, compute the desired center easting/northing in the source **Mars CRS**, then pass those source-CRS coordinates to the script. Do not use an Earth EPSG projection.

```bash
python3 -m venv .venv
. .venv/bin/activate
pip install -r data/requirements.txt
python data/prepare_terrain.py /path/to/JEZ_hirise_DTM.tif --center-x X_METRES --center-y Y_METRES
npm run dev
```

The script rejects an unknown CRS, non-metre units, coarse resolution, and missing pixels. It resamples a 768 × 768 m area to the 65 × 65 grid and writes `public/data/terrain.json`. The app loads that file automatically. The generated JSON includes the source filename and CRS; retain the original PDS/USGS metadata alongside any exported artifact. The JSON is derived from an observed DTM, while route costs, hazard classifications, all sensor readings, weather, and communication geometry in the browser remain simulated.

## Other data sources

- [MEDA calibrated environmental bundle](https://pdsatmospheres.nmsu.edu/data_and_services/atmospheres_data/PERSEVERANCE/meda.html): temperature, pressure, wind, and dust-related instrument data; **not ingested** in this prototype.
- [NAIF SPICE archives](https://naif.jpl.nasa.gov/naif/data_pds_archived.html): orbital geometry kernels; **not ingested**. Link availability and single-range observation in the demo are scenario simulation.
- [USGS Mars coordinate convention](https://planetarynames.wr.usgs.gov/Page/MARS/system): planetocentric latitude, 0–360° east longitude for modern maps. Always check each source product's label, especially older west-positive products.

The PDS and USGS products are public scientific data. Keep attribution and product identifiers when redistributing derived crops; verify the specific product's use notice and metadata. This pipeline does not certify terrain safety or positional accuracy.
