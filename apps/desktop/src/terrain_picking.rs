//! CPU picking uses the exact uploaded vertex positions and active LOD indices.
use crate::terrain_renderer::{Camera, Mesh};
use eframe::egui::{Pos2, Rect};
use glam::Vec3;

#[derive(Clone, Copy, Debug)]
pub struct Ray {
    pub origin: Vec3,
    pub direction: Vec3,
    pub length: f32,
}
impl Ray {
    pub fn from_viewport(camera: Camera, rect: Rect, mouse: Pos2) -> Option<Self> {
        if !rect.contains(mouse) || rect.width() <= 0.0 || rect.height() <= 0.0 {
            return None;
        }
        // Both mouse and rect are egui logical points: DPI cancels in this ratio.
        let x = 2.0 * (mouse.x - rect.left()) / rect.width() - 1.0;
        let y = 1.0 - 2.0 * (mouse.y - rect.top()) / rect.height();
        // Invert the exact GPU f32 matrix in f64 to avoid cancellation when
        // unprojecting the far plane at large camera distances.
        let inverse = camera
            .matrix(rect.width() / rect.height())
            .as_dmat4()
            .inverse();
        // glam perspective_rh and wgpu both use NDC depth [0, 1].
        let near = inverse.project_point3(glam::DVec3::new(x as f64, y as f64, 0.0));
        let far = inverse.project_point3(glam::DVec3::new(x as f64, y as f64, 1.0));
        let length = near.distance(far);
        (near.is_finite() && far.is_finite() && length > 0.0).then_some(Self {
            origin: near.as_vec3(),
            direction: ((far - near) / length).as_vec3(),
            length: length as f32,
        })
    }
    fn box_hit(&self, min: Vec3, max: Vec3, limit: f32) -> bool {
        let mut low = 0.0_f32;
        let mut high = limit.min(self.length);
        for axis in 0..3 {
            let d = self.direction[axis];
            if d.abs() < 1e-10 {
                if self.origin[axis] < min[axis] || self.origin[axis] > max[axis] {
                    return false;
                }
            } else {
                let a = (min[axis] - self.origin[axis]) / d;
                let b = (max[axis] - self.origin[axis]) / d;
                low = low.max(a.min(b));
                high = high.min(a.max(b));
            }
        }
        low <= high
    }
}

struct Block {
    min: Vec3,
    max: Vec3,
    indices: Vec<u32>,
}
pub struct PickMesh {
    positions: Vec<Vec3>,
    full: Vec<Block>,
    coarse: Vec<Block>,
}
impl PickMesh {
    pub fn new(mesh: &Mesh, coarse: &[u32], size: usize) -> Self {
        let positions: Vec<Vec3> = mesh
            .vertices
            .iter()
            .map(|v| Vec3::from(v.position))
            .collect();
        let blocks = |indices: &[u32]| {
            let count = size.div_ceil(32);
            let mut blocks: Vec<Block> = (0..count * count)
                .map(|_| Block {
                    min: Vec3::splat(f32::INFINITY),
                    max: Vec3::splat(f32::NEG_INFINITY),
                    indices: vec![],
                })
                .collect();
            for tri in indices.chunks_exact(3) {
                let first = tri[0] as usize;
                let block = &mut blocks[(first / size / 32) * count + first % size / 32];
                for i in tri {
                    block.min = block.min.min(positions[*i as usize]);
                    block.max = block.max.max(positions[*i as usize]);
                }
                block.indices.extend_from_slice(tri);
            }
            blocks
                .into_iter()
                .filter(|b| !b.indices.is_empty())
                .collect()
        };
        Self {
            full: blocks(&mesh.indices),
            coarse: blocks(coarse),
            positions,
        }
    }
    pub fn bytes(&self) -> usize {
        self.positions.len() * 12
            + self
                .full
                .iter()
                .chain(&self.coarse)
                .map(|b| b.indices.len() * 4 + std::mem::size_of::<Block>())
                .sum::<usize>()
    }
    pub fn intersect(&self, ray: Ray, coarse: bool) -> Option<Vec3> {
        let mut nearest = ray.length;
        let mut hit = false;
        for block in if coarse { &self.coarse } else { &self.full } {
            if !ray.box_hit(
                block.min - Vec3::splat(0.0001),
                block.max + Vec3::splat(0.0001),
                nearest,
            ) {
                continue;
            }
            for tri in block.indices.chunks_exact(3) {
                let [a, b, c] = [tri[0], tri[1], tri[2]].map(|i| self.positions[i as usize]);
                if let Some(t) = triangle(ray, a, b, c) {
                    if t < nearest {
                        nearest = t;
                        hit = true;
                    }
                }
            }
        }
        hit.then_some(ray.origin + nearest * ray.direction)
    }
}

fn triangle(ray: Ray, a: Vec3, b: Vec3, c: Vec3) -> Option<f32> {
    // Two-sided Möller–Trumbore; tolerance permits a shared edge in either tile.
    let e1 = b - a;
    let e2 = c - a;
    let p = ray.direction.cross(e2);
    let det = e1.dot(p);
    if det.abs() < 1e-8 {
        return None;
    }
    let s = ray.origin - a;
    let u = s.dot(p) / det;
    let q = s.cross(e1);
    let v = ray.direction.dot(q) / det;
    let t = e2.dot(q) / det;
    (u >= -1e-5 && v >= -1e-5 && u + v <= 1.0 + 1e-5 && t >= 0.0 && t <= ray.length).then_some(t)
}

#[derive(Clone, Debug)]
pub struct TerrainHit {
    pub tile: String,
    pub world: Vec3,
    pub row: f64,
    pub column: f64,
    pub easting: f64,
    pub northing: f64,
    pub elevation: f64,
    pub geographic: Option<[f64; 2]>, // latitude, east-positive longitude
    pub coarse: bool,
}

/// Only invert the supported spherical Equirectangular CRS, never infer geography
/// from raster indices. Unknown projections intentionally return None.
pub fn geographic(crs: &str, e: f64, n: f64) -> Option<[f64; 2]> {
    if !crs.contains("PROJECTION[\"Equirectangular\"]") || !crs.contains("UNIT[\"metre\",1,") {
        return None;
    }
    let parameter = |name: &str| -> Option<f64> {
        crs.split(&format!("PARAMETER[\"{name}\","))
            .nth(1)?
            .split(']')
            .next()?
            .parse()
            .ok()
    };
    let spheroid = crs.split("SPHEROID[").nth(1)?.split(']').next()?;
    let mut parts = spheroid.rsplit(',');
    let inverse_flattening: f64 = parts.next()?.parse().ok()?;
    let radius: f64 = parts.next()?.parse().ok()?;
    if inverse_flattening != 0.0 || radius <= 0.0 {
        return None;
    }
    let lat_ts = parameter("standard_parallel_1")?.to_radians();
    let lon0 = parameter("central_meridian")?;
    let lat = ((n - parameter("false_northing")?) / radius).to_degrees();
    let lon = lon0 + ((e - parameter("false_easting")?) / (radius * lat_ts.cos())).to_degrees();
    (lat.is_finite() && lon.is_finite() && lat.abs() <= 90.0)
        .then_some([lat, lon.rem_euclid(360.0)])
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::terrain_renderer::{coarse_indices, mesh};
    use eframe::egui::Vec2;

    fn vertical(x: f32, z: f32) -> Ray {
        Ray {
            origin: Vec3::new(x, 20.0, z),
            direction: -Vec3::Y,
            length: 40.0,
        }
    }
    fn flat() -> PickMesh {
        PickMesh::new(
            &mesh(5, 1.0, &[0.0; 25]).unwrap(),
            &coarse_indices(5, 4, &[1; 25]),
            5,
        )
    }
    #[test]
    fn ray_uses_viewport_offset_and_logical_pixel_scale() {
        let c = Camera::new(10.0, 0.0);
        let rect = Rect::from_min_size(Pos2::new(117.0, 83.0), Vec2::new(600.0, 400.0));
        let r = Ray::from_viewport(c, rect, rect.center()).unwrap();
        assert!(r.direction.dot((c.focus - c.eye()).normalize()) > 0.99999);
        let uv = Vec2::new(0.3, 0.7);
        let a = Ray::from_viewport(c, rect, rect.min + rect.size() * uv).unwrap();
        let scaled = Rect::from_min_size(rect.min * 2.0, rect.size() * 2.0);
        let b = Ray::from_viewport(c, scaled, scaled.min + scaled.size() * uv).unwrap();
        assert!(a.direction.distance(b.direction) < 1e-6);
        assert!(Ray::from_viewport(c, rect, rect.min - Vec2::splat(1.0)).is_none());
    }
    #[test]
    fn camera_orientations_recover_the_projected_surface_point() {
        let rect = Rect::from_min_size(Pos2::new(40.0, 80.0), Vec2::new(900.0, 500.0));
        let surface = flat();
        for yaw in [0.0, 0.7, 2.2, 4.5] {
            for pitch in [0.3, 0.8, 1.56] {
                let mut c = Camera::new(5.0, 0.0);
                c.yaw = yaw;
                c.pitch = pitch;
                c.focus = Vec3::new(0.1, 0.0, -0.2);
                let expected = Vec3::new(0.45, 0.0, -0.7);
                let ndc = c.matrix(rect.aspect_ratio()).project_point3(expected);
                let mouse = Pos2::new(
                    rect.left() + (ndc.x + 1.0) * rect.width() / 2.0,
                    rect.top() + (1.0 - ndc.y) * rect.height() / 2.0,
                );
                let ray = Ray::from_viewport(c, rect, mouse).unwrap();
                let hit = surface.intersect(ray, false).unwrap();
                assert!(hit.distance(expected) < 0.002, "{yaw} {pitch}: {hit:?}");
            }
        }
    }
    #[test]
    fn triangle_intersection_uses_measured_sloping_elevations() {
        let heights: Vec<f64> = (0..25)
            .map(|i| (i % 5) as f64 * 2.0 + (i / 5) as f64)
            .collect();
        let surface = PickMesh::new(
            &mesh(5, 1.0, &heights).unwrap(),
            &coarse_indices(5, 4, &[1; 25]),
            5,
        );
        for lod in [false, true] {
            let p = surface.intersect(vertical(0.3, -0.4), lod).unwrap();
            // Local elevation origin is 6 m; h=2*(x+2)+(z+2)-6.
            assert!((p.y - 0.2).abs() < 1e-5);
        }
    }
    #[test]
    fn picking_matches_the_active_lod_instead_of_a_hidden_full_mesh() {
        let mut heights = [0.0; 25];
        heights[12] = 10.0;
        let surface = PickMesh::new(
            &mesh(5, 1.0, &heights).unwrap(),
            &coarse_indices(5, 4, &[1; 25]),
            5,
        );
        let ray = vertical(1.2, 0.1);
        let full = surface.intersect(ray, false).unwrap();
        let coarse = surface.intersect(ray, true).unwrap();
        assert!((full.y + 5.0).abs() < 1e-5);
        assert!(coarse.y > full.y + 1.0);
    }
    #[test]
    fn neighboring_tiles_share_the_same_boundary_pick() {
        let mut left = mesh(5, 1.0, &[0.0; 25]).unwrap();
        let mut right = mesh(5, 1.0, &[0.0; 25]).unwrap();
        for v in &mut left.vertices {
            v.position[0] -= 2.0;
        }
        for v in &mut right.vertices {
            v.position[0] += 2.0;
        }
        let indices = coarse_indices(5, 4, &[1; 25]);
        let l = PickMesh::new(&left, &indices, 5);
        let r = PickMesh::new(&right, &indices, 5);
        for coarse in [false, true] {
            let a = l.intersect(vertical(0.0, 0.37), coarse).unwrap();
            let b = r.intersect(vertical(0.0, 0.37), coarse).unwrap();
            assert!(a.distance(b) < 1e-6);
            assert!(l.intersect(vertical(-0.01, 0.37), coarse).is_some());
            assert!(r.intersect(vertical(0.01, 0.37), coarse).is_some());
            assert!(l.intersect(vertical(0.01, 0.37), coarse).is_none());
        }
    }
    #[test]
    fn invalid_cells_outside_surface_and_clipped_rays_are_rejected() {
        let mut heights = [0.0; 25];
        heights[0] = f64::NAN;
        let mut valid = [1; 25];
        valid[0] = 0;
        let surface = PickMesh::new(
            &mesh(5, 1.0, &heights).unwrap(),
            &coarse_indices(5, 4, &valid),
            5,
        );
        assert!(surface.intersect(vertical(-1.9, -1.9), false).is_none());
        assert!(surface.intersect(vertical(0.0, 0.0), true).is_none());
        assert!(surface.intersect(vertical(3.0, 0.0), false).is_none());
        let mut ray = vertical(0.0, 0.0);
        ray.length = 10.0;
        assert!(surface.intersect(ray, false).is_none());
        ray.origin.y = -1.0;
        assert!(surface.intersect(ray, false).is_none());
    }
    #[test]
    fn geographic_conversion_matches_rasterio_mars_fixture() {
        // Rasterio-generated manifest bounds are independent reference values.
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../data/local/hirise-tiles/manifest.json");
        // Source assets are optional in CI; use their CRS if installed, otherwise
        // the same explicitly specified source CRS fixture.
        let crs=std::fs::read_to_string(path).ok().and_then(|s| serde_json::from_str::<serde_json::Value>(&s).ok()).and_then(|v|v["crs"].as_str().map(str::to_owned)).unwrap_or_else(||"PROJECTION[\"Equirectangular\"],SPHEROID[\"Mars\",3394839.8133163,0],PARAMETER[\"standard_parallel_1\",15],PARAMETER[\"central_meridian\",77.44],PARAMETER[\"false_easting\",0],PARAMETER[\"false_northing\",0],UNIT[\"metre\",1,]".into());
        let [lat, lon] = geographic(&crs, -728.4720903375998, 1095491.2724667683).unwrap();
        assert!((lat - 18.48895083637158).abs() < 1e-10);
        assert!((lon - 77.42727163897655).abs() < 1e-10);
        assert!(geographic("unknown CRS", 0.0, 0.0).is_none());
    }
}
