# Implementation plan and status

1. **MVP — complete:** React/Vite/TypeScript mission console, procedural test terrain, 3D map, start/destination selection, Rust WebAssembly A*, slope-aware route comparison, elevation profile.
2. **Navigation simulation — complete as a concept demo:** first-person camera, route corridor, playback, IMU acceleration/angular rate simulation, intermittent visual fixes, weak one-dimensional radio ranging, uncertainty display, failure scenarios.
3. **Planetary data integration — partial:** documented USGS DTM crop script and automatic local JSON loading. The default distribution has no observed DTM. MEDA and SPICE formats/access are documented but their observations and geometry are not ingested. Weather and link geometry remain simulated.
4. **Next engineering work:** ingest and validate a small Jezero HiRISE crop with product metadata; add geodetic CRS transforms based on the source PDS label; propagate full state covariance and map-registration uncertainty; derive actual orbiter line-of-sight from SPICE kernels; incorporate timestamped MEDA observations.

The prototype intentionally makes no safety or positional-accuracy claim.
