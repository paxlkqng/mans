//! One full-resolution navigation grid, independent of render residency and LOD.
use crate::{terrain_picking::TerrainHit, terrain_tiles::Manifest};
use mans_nav::{astar_with_policy, Point, RoutePolicy};
use serde::Deserialize;
use std::{
    path::Path,
    sync::{
        atomic::{AtomicU64, Ordering},
        mpsc, Arc,
    },
};

pub struct NavigationGrid {
    pub manifest: Manifest,
    pub size: usize,
    pub heights: Vec<f64>,
    pub left: f64,
    pub top: f64,
}
#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct TileGrid {
    size: usize,
    cell_size: f64,
    heights: Vec<f64>,
}
impl NavigationGrid {
    pub fn load(path: &Path) -> Result<Self, String> {
        let m = Manifest::read(path)?;
        let size = m.count * m.stride + 1;
        let mut heights = vec![f64::NAN; size * size];
        let mut seen = vec![false; size * size];
        let root = path.parent().ok_or("Manifest has no parent")?;
        for tile in &m.tiles {
            if tile.status == "missing" || tile.valid_samples == 0 {
                continue;
            }
            let folder = root.join(&tile.path);
            let file = folder.join("terrain.json");
            let g: TileGrid = serde_json::from_str(
                &std::fs::read_to_string(&file).map_err(|e| format!("{}: {e}", file.display()))?,
            )
            .map_err(|e| e.to_string())?;
            let mask = std::fs::read(folder.join("valid.u8")).map_err(|e| e.to_string())?;
            if g.size != m.size
                || g.heights.len() != m.size * m.size
                || mask.len() != g.heights.len()
                || (g.cell_size - m.cell_size).abs() > 1e-8
            {
                return Err(format!("{}: invalid navigation grid/mask", tile.id));
            }
            for row in 0..m.size {
                for col in 0..m.size {
                    let local = row * m.size + col;
                    let global = (tile.z * m.stride + row) * size + tile.x * m.stride + col;
                    let h = if mask[local] != 0
                        && g.heights[local].is_finite()
                        && g.heights[local] > -1e10
                    {
                        g.heights[local]
                    } else {
                        f64::NAN
                    };
                    if seen[global] {
                        if heights[global].is_finite()
                            && h.is_finite()
                            && (heights[global] - h).abs() > 1e-6
                        {
                            return Err(format!("{}: shared edge elevation mismatch", tile.id));
                        }
                        if !h.is_finite() {
                            heights[global] = f64::NAN;
                        }
                    } else {
                        heights[global] = h;
                        seen[global] = true;
                    }
                }
            }
        }
        let left = m
            .tiles
            .iter()
            .map(|t| t.bounds[0])
            .fold(f64::INFINITY, f64::min);
        let top = m
            .tiles
            .iter()
            .map(|t| t.bounds[3])
            .fold(f64::NEG_INFINITY, f64::max);
        tracing::info!(
            size,
            samples = heights.len(),
            "full-resolution navigation grid loaded once"
        );
        Ok(Self {
            manifest: m,
            size,
            heights,
            left,
            top,
        })
    }
    pub fn point(&self, e: f64, n: f64) -> Result<Point, String> {
        let x = (e - self.left) / self.manifest.cell_size;
        let z = (self.top - n) / self.manifest.cell_size;
        if !x.is_finite()
            || !z.is_finite()
            || x < -0.001
            || z < -0.001
            || x > self.size as f64 - 1.0 + 0.001
            || z > self.size as f64 - 1.0 + 0.001
        {
            return Err("Endpoint outside navigation coverage".into());
        }
        let p = Point {
            x: x.round().clamp(0.0, (self.size - 1) as f64) as usize,
            z: z.round().clamp(0.0, (self.size - 1) as f64) as usize,
        };
        if !self.heights[p.z * self.size + p.x].is_finite() {
            return Err("Endpoint snaps to NoData; select another measured sample".into());
        }
        Ok(p)
    }
    pub fn tile_local(&self, p: Point) -> (usize, usize, usize) {
        let tx = (p.x / self.manifest.stride).min(self.manifest.count - 1);
        let tz = (p.z / self.manifest.stride).min(self.manifest.count - 1);
        let index = self
            .manifest
            .tiles
            .iter()
            .position(|t| t.x == tx && t.z == tz)
            .unwrap();
        (
            index,
            p.z - tz * self.manifest.stride,
            p.x - tx * self.manifest.stride,
        )
    }
    pub fn hit(&self, p: Point) -> TerrainHit {
        let (tile, row, column) = self.tile_local(p);
        let easting = self.left + p.x as f64 * self.manifest.cell_size;
        let northing = self.top - p.z as f64 * self.manifest.cell_size;
        let elevation = self.heights[p.z * self.size + p.x];
        TerrainHit {
            tile: self.manifest.tiles[tile].id.clone(),
            row: row as f64,
            column: column as f64,
            easting,
            northing,
            elevation,
            world: glam::Vec3::new(
                (easting - self.manifest.origin[0]) as f32,
                (elevation - self.manifest.origin[2]) as f32,
                (self.manifest.origin[1] - northing) as f32,
            ),
            geographic: crate::terrain_picking::geographic(&self.manifest.crs, easting, northing),
            coarse: false,
        }
    }
    pub fn route(
        &self,
        start: [f64; 2],
        goal: [f64; 2],
        policy: RoutePolicy,
        cancelled: impl FnMut() -> bool,
    ) -> Result<NavigationRoute, String> {
        let a = self.point(start[0], start[1])?;
        let b = self.point(goal[0], goal[1])?;
        let path = astar_with_policy(
            &self.heights,
            self.size,
            self.manifest.cell_size,
            a,
            b,
            None,
            policy,
            cancelled,
        );
        if path.is_empty() {
            return Err("No route under this slope policy and valid-data mask. No fallback path was generated.".into());
        }
        let mut samples = vec![];
        for (index, p) in path.iter().enumerate() {
            if index > 0 {
                let q = path[index - 1];
                if (p.x as isize - q.x as isize) * (p.z as isize - q.z as isize) > 0 {
                    let h = (self.heights[q.z * self.size + p.x]
                        + self.heights[p.z * self.size + q.x])
                        * 0.5;
                    samples.push(RouteSample {
                        x: (p.x + q.x) as f64 * 0.5,
                        z: (p.z + q.z) as f64 * 0.5,
                        elevation: h,
                        distance: 0.0,
                    });
                }
            }
            samples.push(RouteSample {
                x: p.x as f64,
                z: p.z as f64,
                elevation: self.heights[p.z * self.size + p.x],
                distance: 0.0,
            });
        }
        let mut length = 0.0;
        let mut horizontal = 0.0;
        let mut gain = 0.0;
        let mut loss = 0.0;
        let mut max_slope = 0.0_f64;
        let mut segments = vec![];
        let world = |p: &RouteSample| {
            [
                (self.left + p.x * self.manifest.cell_size - self.manifest.origin[0]) as f32,
                (p.elevation - self.manifest.origin[2]) as f32,
                (self.manifest.origin[1] - self.top + p.z * self.manifest.cell_size) as f32,
            ]
        };
        for i in 1..samples.len() {
            let a = samples[i - 1];
            let b = samples[i];
            let ds = (a.x - b.x).hypot(a.z - b.z) * self.manifest.cell_size;
            let dh = b.elevation - a.elevation;
            length += ds.hypot(dh);
            horizontal += ds;
            gain += dh.max(0.0);
            loss += (-dh).max(0.0);
            max_slope = max_slope.max((dh.abs() / ds).atan().to_degrees());
            samples[i].distance = length;
            let mid = Point {
                x: ((a.x + b.x) * 0.5).floor() as usize,
                z: ((a.z + b.z) * 0.5).floor() as usize,
            };
            let (tile, _, _) = self.tile_local(mid);
            segments.push(RouteSegment {
                tile,
                a: world(&a),
                b: world(&b),
            });
        }
        Ok(NavigationRoute {
            start: self.hit(a),
            goal: self.hit(b),
            samples,
            segments,
            length,
            horizontal,
            gain,
            loss,
            max_slope,
            cells: path.len(),
            policy,
        })
    }
}
#[derive(Clone, Copy)]
pub struct RouteSample {
    pub x: f64,
    pub z: f64,
    pub elevation: f64,
    pub distance: f64,
}
pub struct RouteSegment {
    pub tile: usize,
    pub a: [f32; 3],
    pub b: [f32; 3],
}
pub struct NavigationRoute {
    pub start: TerrainHit,
    pub goal: TerrainHit,
    pub samples: Vec<RouteSample>,
    pub segments: Vec<RouteSegment>,
    pub length: f64,
    pub horizontal: f64,
    pub gain: f64,
    pub loss: f64,
    pub max_slope: f64,
    pub cells: usize,
    pub policy: RoutePolicy,
}
pub struct RouteComparison {
    pub shortest: NavigationRoute,
    pub lower_risk: NavigationRoute,
}
impl RouteComparison {
    pub fn identical(&self) -> bool {
        self.shortest.samples.len() == self.lower_risk.samples.len()
            && self
                .shortest
                .samples
                .iter()
                .zip(&self.lower_risk.samples)
                .all(|(a, b)| a.x == b.x && a.z == b.z)
    }
    pub fn additional_distance(&self) -> f64 {
        (self.lower_risk.length - self.shortest.length).max(0.0)
    }
}
impl NavigationGrid {
    pub fn compare(
        &self,
        start: [f64; 2],
        goal: [f64; 2],
        policy: RoutePolicy,
        mut cancelled: impl FnMut() -> bool,
    ) -> Result<RouteComparison, String> {
        let shortest = self.route(
            start,
            goal,
            RoutePolicy {
                max_slope_deg: policy.max_slope_deg,
                slope_weight: 0.0,
                elevation_weight: 0.0,
            },
            &mut cancelled,
        )?;
        if cancelled() {
            return Err("Route comparison cancelled".into());
        }
        let lower_risk = self.route(start, goal, policy, &mut cancelled)?;
        Ok(RouteComparison {
            shortest,
            lower_risk,
        })
    }
}
pub struct Request {
    pub revision: u64,
    pub start: [f64; 2],
    pub goal: [f64; 2],
    pub policy: RoutePolicy,
}
pub struct RouteWorker {
    tx: mpsc::SyncSender<Request>,
    pub results: mpsc::Receiver<(u64, Result<RouteComparison, String>)>,
    pub revision: Arc<AtomicU64>,
}
impl RouteWorker {
    pub fn new(path: std::path::PathBuf) -> Result<Self, String> {
        let (tx, rx) = mpsc::sync_channel::<Request>(1);
        let (out, results) = mpsc::channel();
        let revision = Arc::new(AtomicU64::new(0));
        let current = revision.clone();
        std::thread::Builder::new()
            .name("mans-route-worker".into())
            .spawn(move || {
                let mut grid: Option<NavigationGrid> = None;
                while let Ok(request) = rx.recv() {
                    if current.load(Ordering::Relaxed) != request.revision {
                        continue;
                    }
                    let result = (|| {
                        if grid.is_none() {
                            grid = Some(NavigationGrid::load(&path)?);
                        }
                        grid.as_ref().unwrap().compare(
                            request.start,
                            request.goal,
                            request.policy,
                            || current.load(Ordering::Relaxed) != request.revision,
                        )
                    })();
                    if current.load(Ordering::Relaxed) == request.revision
                        && out.send((request.revision, result)).is_err()
                    {
                        break;
                    }
                }
            })
            .map_err(|e| e.to_string())?;
        Ok(Self {
            tx,
            results,
            revision,
        })
    }
    pub fn request(&self, request: Request) -> Result<(), String> {
        self.tx
            .try_send(request)
            .map_err(|e| format!("Route worker unavailable: {e}"))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::terrain_tiles::TileEntry;
    fn grid() -> NavigationGrid {
        let mut tiles = vec![];
        for z in 0..3 {
            for x in 0..3 {
                tiles.push(TileEntry {
                    id: format!("tile-{x}-{z}"),
                    x,
                    z,
                    path: String::new(),
                    bounds: [
                        997.0 + x as f64 * 2.0,
                        2001.0 - z as f64 * 2.0,
                        999.0 + x as f64 * 2.0,
                        2003.0 - z as f64 * 2.0,
                    ],
                    valid_samples: 9,
                    status: "valid".into(),
                });
            }
        }
        NavigationGrid {
            manifest: Manifest {
                name: "test measured plane".into(),
                size: 3,
                stride: 2,
                count: 3,
                cell_size: 1.0,
                min_elevation: 100.0,
                max_elevation: 104.5,
                origin: [1000.0, 2000.0, 100.0],
                crs: "test metric".into(),
                tiles,
            },
            size: 7,
            heights: (0..49)
                .map(|i| 100.0 + (i % 7) as f64 * 0.5 + (i / 7) as f64 * 0.25)
                .collect(),
            left: 997.0,
            top: 2003.0,
        }
    }
    fn policy() -> RoutePolicy {
        RoutePolicy {
            max_slope_deg: 45.0,
            slope_weight: 0.0,
            elevation_weight: 0.0,
        }
    }
    #[test]
    fn shared_edges_have_one_global_index_and_correct_tile_coordinates() {
        let g = grid();
        assert_eq!(g.point(999.0, 2001.0).unwrap(), Point { x: 2, z: 2 });
        let (tile, row, col) = g.tile_local(Point { x: 2, z: 2 });
        assert_eq!(
            (g.manifest.tiles[tile].x, g.manifest.tiles[tile].z, row, col),
            (1, 1, 0, 0)
        );
        for z in 0..7 {
            for x in 0..7 {
                let p = Point { x, z };
                let hit = g.hit(p);
                assert_eq!(g.point(hit.easting, hit.northing).unwrap(), p);
                let (t, r, c) = g.tile_local(p);
                let tile = &g.manifest.tiles[t];
                assert_eq!((tile.x * 2 + c, tile.z * 2 + r), (x, z));
            }
        }
    }
    #[test]
    fn cross_tile_route_has_continuity_metric_statistics_and_gpu_alignment() {
        let g = grid();
        let r = g
            .route([997.0, 2003.0], [1003.0, 1997.0], policy(), || false)
            .unwrap();
        assert_eq!(r.start.tile, "tile-0-0");
        assert_eq!(r.goal.tile, "tile-2-2");
        assert_eq!(r.cells, 7);
        assert!((r.horizontal - 6.0 * 2.0_f64.sqrt()).abs() < 1e-9);
        assert!((r.length - 6.0 * 2.5625_f64.sqrt()).abs() < 1e-9);
        assert!((r.gain - 4.5).abs() < 1e-9);
        assert_eq!(r.loss, 0.0);
        assert!((r.max_slope - (0.75 / 2.0_f64.sqrt()).atan().to_degrees()).abs() < 1e-9);
        let tiles: std::collections::HashSet<_> = r.segments.iter().map(|s| s.tile).collect();
        assert_eq!(tiles.len(), 3);
        for pair in r.segments.windows(2) {
            assert_eq!(pair[0].b, pair[1].a);
        }
        for (i, s) in r.segments.iter().enumerate() {
            let p = r.samples[i];
            assert!((s.a[0] as f64 - (p.x - 3.0)).abs() < 1e-6);
            assert!((s.a[2] as f64 - (p.z - 3.0)).abs() < 1e-6);
            assert!((s.a[1] as f64 - (p.elevation - 100.0)).abs() < 1e-6);
            assert!(s.a.iter().all(|v| v.is_finite()));
        }
        assert!((r.samples.last().unwrap().distance - r.length).abs() < 1e-9);
    }
    #[test]
    fn invalid_endpoints_and_missing_cross_tile_coverage_are_rejected() {
        let mut g = grid();
        g.heights[0] = f64::NAN;
        assert!(g.point(997.0, 2003.0).is_err());
        assert!(g.point(996.0, 2003.0).is_err());
        for x in 0..7 {
            g.heights[3 * 7 + x] = f64::NAN;
        }
        assert!(g
            .route([998.0, 2002.0], [1003.0, 1997.0], policy(), || false)
            .is_err());
    }
    #[test]
    fn reverse_route_swaps_gain_and_loss() {
        let g = grid();
        let r = g
            .route([1003.0, 1997.0], [997.0, 2003.0], policy(), || false)
            .unwrap();
        assert_eq!(r.gain, 0.0);
        assert!((r.loss - 4.5).abs() < 1e-9);
    }
    #[test]
    fn comparison_zero_penalties_yields_identical_routes() {
        let g = grid();
        let c = g
            .compare([997.0, 2003.0], [1003.0, 1997.0], policy(), || false)
            .unwrap();
        assert!(c.identical());
        assert_eq!(c.additional_distance(), 0.0);
        assert_eq!(c.shortest.policy.slope_weight, 0.0);
        assert_eq!(c.shortest.policy.elevation_weight, 0.0);
        assert_eq!(c.shortest.start.world, c.lower_risk.start.world);
        assert_eq!(c.shortest.goal.world, c.lower_risk.goal.world);
    }
    #[test]
    fn comparison_selects_a_longer_flatter_route_around_a_measured_rise() {
        let mut g = grid();
        g.heights.fill(100.0);
        for x in 1..6 {
            g.heights[3 * 7 + x] = 100.3;
        }
        let p = RoutePolicy {
            max_slope_deg: 45.0,
            slope_weight: 5.0,
            elevation_weight: 5.0,
        };
        let c = g
            .compare([997.0, 2000.0], [1003.0, 2000.0], p, || false)
            .unwrap();
        assert!(!c.identical());
        assert!(c.lower_risk.length > c.shortest.length);
        assert!(c.lower_risk.max_slope < c.shortest.max_slope);
        assert!(c.lower_risk.gain < c.shortest.gain);
        assert!(
            (c.additional_distance() - (c.lower_risk.length - c.shortest.length)).abs() < 1e-10
        );
        assert_eq!(
            c.shortest.policy.max_slope_deg,
            c.lower_risk.policy.max_slope_deg
        );
    }
    #[test]
    fn comparison_rejects_nodata_and_cancellation_without_partial_results() {
        let mut g = grid();
        assert!(g
            .compare([997.0, 2003.0], [1003.0, 1997.0], policy(), || true)
            .is_err());
        for x in 0..7 {
            g.heights[3 * 7 + x] = f64::NAN;
        }
        assert!(g
            .compare([997.0, 2003.0], [1003.0, 1997.0], policy(), || false)
            .is_err());
    }
}
