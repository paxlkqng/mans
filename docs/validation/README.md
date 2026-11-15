# Independent A* route verification

The existing A* solver and native route statistics were checked against a test-only
Dijkstra implementation. The reference independently enumerates mesh facets,
checks valid samples and hard constraints, and recomputes surface distance,
elevation gain/loss, maximum segment slope, and objective cost. It does not call
production neighbor, cost, heuristic, or statistics helpers. Production algorithms
and UI were unchanged.

## Numerical results

All eight real-data comparisons passed on the existing nine-tile HiRISE navigation
grid (1534 × 1534 samples, 1.0096633268711 m spacing). Four fixed endpoint pairs
cover column seams, row seams, a four-tile junction, and a regional crossing.
Dijkstra searched the complete supplied graph without spatial cropping or
route-based pruning. The evaluation took approximately 3.6 seconds in release mode on the
development machine, including loading and report generation.

| Pair | Shortest surface distance (m) | Lower-risk surface distance (m) | Largest absolute A* − Dijkstra objective difference |
| --- | ---: | ---: | ---: |
| Column seam | 35.718551 | 35.718551 | 0 |
| Row seam | 35.712845 | 35.712845 | 0 |
| Four-tile junction | 49.986750 | 49.986750 | 0 |
| Regional crossing | 1464.714790 | 1493.883472 | 0 |

All independently reconstructed edges obeyed valid-data and slope constraints.
Native distance, horizontal distance, elevation gain/loss, and maximum slope
agreed with independently reconstructed statistics within 1e-7. Objective-cost
tolerance was `1e-7 + 1e-9 * abs(Dijkstra cost)`. Shortest surface distance never
exceeded the lower-risk route's distance within tolerance. Lower-risk routes
minimized their own objective; maximum slope is not guaranteed to decrease.

Five additional automated tests cover flat terrain, both diagonal directions,
steep barriers, unreachable destinations, NoData, blocked corner cutting,
invalid endpoints, isolated samples, analytic mesh geometry, and 96 fixed-seed
terrain fixtures. Tile boundary integration is tested with the real-data pairs.

The [machine-readable report](hirise-route-verification.json) contains projected
and raster endpoints, both policy objectives, Dijkstra costs, route metrics,
pass/fail results, search counts, CRS, and SHA-256 hashes of source elevation
assets and masks. Local NASA assets must already be installed to reproduce it.

## Model and reproduction

Both policies use the same 23° hard segment-slope limit, measured elevations,
NoData restrictions, facet-support and diagonal corner constraints. Per-edge
objective is `surface_distance * (1 + slope_weight * (maximum_slope_deg / 12)^2)
+ elevation_weight * (gain + loss)`. Shortest weights are `(0, 0)`; lower-risk
weights are `(2.5, 1)`. Elevations along the alternate diagonal include the actual
mesh's shared-edge midpoint, rather than assuming a straight endpoint chord.

```sh
cargo test -p mans-nav --test independent_verification
cargo test --release -p mans-desktop route_verification::hirise_reference_report -- --ignored --nocapture
cargo fmt --check
cargo test --workspace
cargo build -p mans-desktop
```

These are bounded numerical checks of a discrete terrain model, not an exhaustive
proof or validation of real astronaut safety. No hazards beyond the represented
valid-data and slope constraints were validated. The 23° limit is an analyst
assumption, not a certified traversability threshold.
