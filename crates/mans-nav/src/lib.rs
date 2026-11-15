use serde::Serialize;
use std::{cmp::Ordering, collections::BinaryHeap};
use wasm_bindgen::prelude::*;

#[derive(Clone, Copy, Debug, PartialEq, Serialize)]
pub struct Point {
    pub x: usize,
    pub z: usize,
}
#[derive(Clone, Copy, Debug)]
pub struct Estimate {
    pub x: f64,
    pub z: f64,
    pub vx: f64,
    pub vz: f64,
    pub variance: f64,
}

pub fn slope_degrees(heights: &[f64], size: usize, cell: f64, a: Point, b: Point) -> f64 {
    let dy = (heights[a.z * size + a.x] - heights[b.z * size + b.x]).abs();
    let ds = (((a.x as f64 - b.x as f64).powi(2) + (a.z as f64 - b.z as f64).powi(2)).sqrt()
        * cell)
        .max(1e-9);
    (dy / ds).atan().to_degrees()
}

#[derive(Clone, Copy, Debug)]
pub struct RoutePolicy {
    /// An analyst assumption, not a certified traversability limit.
    pub max_slope_deg: f64,
    pub slope_weight: f64,
    pub elevation_weight: f64,
}
impl Default for RoutePolicy {
    fn default() -> Self {
        Self {
            max_slope_deg: 23.0,
            slope_weight: 2.5,
            elevation_weight: 1.0,
        }
    }
}

/// Samples the two full-resolution mesh facets crossed by the other diagonal.
/// The native mesh's shared diagonal joins (x+1,z) to (x,z+1).
pub fn edge_metrics(
    heights: &[f64],
    size: usize,
    cell: f64,
    a: Point,
    b: Point,
) -> (f64, f64, f64) {
    let horizontal = ((a.x.abs_diff(b.x).pow(2) + a.z.abs_diff(b.z).pow(2)) as f64).sqrt() * cell;
    let h0 = heights[a.z * size + a.x];
    let h1 = heights[b.z * size + b.x];
    let opposite = (b.x as isize - a.x as isize) * (b.z as isize - a.z as isize) > 0;
    if opposite {
        let middle = (heights[a.z * size + b.x] + heights[b.z * size + a.x]) * 0.5;
        let half = horizontal * 0.5;
        let d0 = (middle - h0).abs();
        let d1 = (h1 - middle).abs();
        (
            half.hypot(d0) + half.hypot(d1),
            (d0.max(d1) / half).atan().to_degrees(),
            d0 + d1,
        )
    } else {
        let change = (h1 - h0).abs();
        (
            horizontal.hypot(change),
            (change / horizontal).atan().to_degrees(),
            change,
        )
    }
}

#[derive(Clone, Copy)]
struct QueueEntry {
    id: usize,
    score: f64,
    cost: f64,
}
impl PartialEq for QueueEntry {
    fn eq(&self, b: &Self) -> bool {
        self.id == b.id && self.score == b.score && self.cost == b.cost
    }
}
impl Eq for QueueEntry {}
impl PartialOrd for QueueEntry {
    fn partial_cmp(&self, b: &Self) -> Option<Ordering> {
        Some(self.cmp(b))
    }
}
impl Ord for QueueEntry {
    fn cmp(&self, b: &Self) -> Ordering {
        b.score
            .total_cmp(&self.score)
            .then_with(|| b.id.cmp(&self.id))
            .then_with(|| b.cost.total_cmp(&self.cost))
    }
}

/// Eight-connected A*, deterministic heap ordering. The planar octile distance
/// is consistent/admissible because every edge costs at least its planar length.
/// A blocked mask and missing elevation are always hard exclusions.
pub fn astar_with_policy(
    heights: &[f64],
    size: usize,
    cell: f64,
    start: Point,
    goal: Point,
    blocked: Option<&[bool]>,
    policy: RoutePolicy,
    mut cancelled: impl FnMut() -> bool,
) -> Vec<Point> {
    if size < 2
        || size.checked_mul(size) != Some(heights.len())
        || !cell.is_finite()
        || cell <= 0.0
        || start.x >= size
        || start.z >= size
        || goal.x >= size
        || goal.z >= size
        || blocked.is_some_and(|m| m.len() != heights.len())
        || !policy.max_slope_deg.is_finite()
        || !(0.0..90.0).contains(&policy.max_slope_deg)
        || !policy.slope_weight.is_finite()
        || policy.slope_weight < 0.0
        || !policy.elevation_weight.is_finite()
        || policy.elevation_weight < 0.0
    {
        return vec![];
    }
    let observed = |i: usize| heights[i].is_finite() && heights[i] > -1e10;
    let valid = |i: usize| observed(i) && !blocked.is_some_and(|m| m[i]);
    let sid = start.z * size + start.x;
    let gid = goal.z * size + goal.x;
    if !valid(sid) || !valid(gid) {
        return vec![];
    }
    let heuristic = |p: Point| {
        let dx = p.x.abs_diff(goal.x) as f64;
        let dz = p.z.abs_diff(goal.z) as f64;
        cell * (dx.max(dz) + (std::f64::consts::SQRT_2 - 1.0) * dx.min(dz))
    };
    let mut cost = vec![f64::INFINITY; heights.len()];
    let mut previous = vec![usize::MAX; heights.len()];
    let mut open = BinaryHeap::new();
    cost[sid] = 0.0;
    open.push(QueueEntry {
        id: sid,
        score: heuristic(start),
        cost: 0.0,
    });
    let mut iterations = 0usize;
    while let Some(entry) = open.pop() {
        iterations += 1;
        if iterations % 1024 == 1 && cancelled() {
            return vec![];
        }
        let id = entry.id;
        if entry.cost > cost[id] {
            continue;
        }
        if id == gid {
            let mut path = vec![];
            let mut at = gid;
            loop {
                path.push(Point {
                    x: at % size,
                    z: at / size,
                });
                if at == sid {
                    break;
                }
                at = previous[at];
            }
            path.reverse();
            return path;
        }
        let a = Point {
            x: id % size,
            z: id / size,
        };
        for dz in -1isize..=1 {
            for dx in -1isize..=1 {
                if dx == 0 && dz == 0 {
                    continue;
                }
                let x = a.x as isize + dx;
                let z = a.z as isize + dz;
                if x < 0 || z < 0 || x >= size as isize || z >= size as isize {
                    continue;
                }
                let b = Point {
                    x: x as usize,
                    z: z as usize,
                };
                let ni = b.z * size + b.x;
                if !valid(ni) {
                    continue;
                }
                // Do not invent a segment between isolated observed samples:
                // a cardinal edge must belong to a real terrain triangle.
                if dz == 0 {
                    let x = a.x.min(b.x);
                    let z = a.z;
                    if !((z + 1 < size && observed((z + 1) * size + x))
                        || (z > 0 && observed((z - 1) * size + x + 1)))
                    {
                        continue;
                    }
                } else if dx == 0 {
                    let x = a.x;
                    let z = a.z.min(b.z);
                    if !((x + 1 < size && observed(z * size + x + 1))
                        || (x > 0 && observed((z + 1) * size + x - 1)))
                    {
                        continue;
                    }
                }
                if dx != 0 && dz != 0 {
                    let sides = [Point { x: b.x, z: a.z }, Point { x: a.x, z: b.z }];
                    if sides.iter().any(|p| {
                        !valid(p.z * size + p.x)
                            || slope_degrees(heights, size, cell, a, *p) > policy.max_slope_deg
                            || slope_degrees(heights, size, cell, *p, b) > policy.max_slope_deg
                    }) {
                        continue;
                    }
                }
                let (length, slope, change) = edge_metrics(heights, size, cell, a, b);
                if slope > policy.max_slope_deg {
                    continue;
                }
                let candidate = cost[id]
                    + length * (1.0 + policy.slope_weight * (slope / 12.0).powi(2))
                    + policy.elevation_weight * change;
                if candidate < cost[ni] {
                    cost[ni] = candidate;
                    previous[ni] = id;
                    open.push(QueueEntry {
                        id: ni,
                        score: candidate + heuristic(b),
                        cost: candidate,
                    });
                }
            }
        }
    }
    vec![]
}

pub fn astar(
    heights: &[f64],
    size: usize,
    cell: f64,
    start: Point,
    goal: Point,
    safest: bool,
    blocked: bool,
) -> Vec<Point> {
    // Legacy center-patch demonstration mask, never applied to HiRISE routing.
    let mask = blocked.then(|| {
        (0..heights.len())
            .map(|i| {
                (30..=39).contains(&(i % size.max(1))) && (43..=51).contains(&(i / size.max(1)))
            })
            .collect::<Vec<_>>()
    });
    astar_with_policy(
        heights,
        size,
        cell,
        start,
        goal,
        mask.as_deref(),
        RoutePolicy {
            max_slope_deg: if safest { 23.0 } else { 89.9 },
            slope_weight: if safest { 2.5 } else { 0.02 },
            elevation_weight: 1.0,
        },
        || false,
    )
}

pub fn predict(mut e: Estimate, ax: f64, az: f64, dt: f64, noise: f64) -> Estimate {
    e.x += e.vx * dt + 0.5 * ax * dt * dt;
    e.z += e.vz * dt + 0.5 * az * dt * dt;
    e.vx += ax * dt;
    e.vz += az * dt;
    e.variance += noise * dt * dt;
    e
}
pub fn observe(mut e: Estimate, x: f64, z: f64, measurement_variance: f64) -> Estimate {
    let k = e.variance / (e.variance + measurement_variance);
    e.x += k * (x - e.x);
    e.z += k * (z - e.z);
    e.variance *= 1.0 - k;
    e
}

#[wasm_bindgen]
pub fn plan_route(
    heights: Vec<f64>,
    size: usize,
    cell: f64,
    sx: usize,
    sz: usize,
    gx: usize,
    gz: usize,
    safest: bool,
    blocked: bool,
) -> JsValue {
    serde_wasm_bindgen::to_value(&astar(
        &heights,
        size,
        cell,
        Point { x: sx, z: sz },
        Point { x: gx, z: gz },
        safest,
        blocked,
    ))
    .unwrap()
}

#[cfg(test)]
mod tests {
    use super::*;
    fn policy() -> RoutePolicy {
        RoutePolicy {
            max_slope_deg: 45.0,
            slope_weight: 0.0,
            elevation_weight: 0.0,
        }
    }
    #[test]
    fn diagonal_and_cardinal_distances_are_metric_and_deterministic() {
        let h = vec![0.0; 25];
        let a = Point { x: 0, z: 0 };
        let b = Point { x: 4, z: 4 };
        let path = astar_with_policy(&h, 5, 2.0, a, b, None, policy(), || false);
        assert_eq!(path.len(), 5);
        let length: f64 = path
            .windows(2)
            .map(|p| edge_metrics(&h, 5, 2.0, p[0], p[1]).0)
            .sum();
        assert!((length - 8.0 * std::f64::consts::SQRT_2).abs() < 1e-10);
        assert_eq!(
            path,
            astar_with_policy(&h, 5, 2.0, a, b, None, policy(), || false)
        );
        assert_eq!(edge_metrics(&h, 5, 2.0, a, Point { x: 1, z: 0 }).0, 2.0);
    }
    #[test]
    fn blocked_corners_and_nodata_cannot_be_crossed() {
        let mut h = vec![0.0; 9];
        let mut mask = vec![false; 9];
        mask[1] = true;
        mask[3] = true;
        let a = Point { x: 0, z: 0 };
        let b = Point { x: 2, z: 2 };
        assert!(astar_with_policy(&h, 3, 1.0, a, b, Some(&mask), policy(), || false).is_empty());
        h[1] = f64::NAN;
        h[3] = f64::NAN;
        assert!(astar_with_policy(&h, 3, 1.0, a, b, None, policy(), || false).is_empty());
        h[0] = f64::NAN;
        assert!(astar_with_policy(&h, 3, 1.0, a, a, None, policy(), || false).is_empty());
    }
    #[test]
    fn missing_facets_do_not_create_unsupported_cardinal_edges() {
        let mut h = vec![f64::NAN; 9];
        h[0] = 0.0;
        h[1] = 0.0;
        assert!(astar_with_policy(
            &h,
            3,
            1.0,
            Point { x: 0, z: 0 },
            Point { x: 1, z: 0 },
            None,
            policy(),
            || false
        )
        .is_empty());
    }
    #[test]
    fn diagonal_metrics_follow_both_actual_mesh_facets() {
        let h = [0.0, 2.0, 2.0, 0.0];
        let (length, slope, change) =
            edge_metrics(&h, 2, 1.0, Point { x: 0, z: 0 }, Point { x: 1, z: 1 });
        assert!((length - 2.0 * (0.5_f64 + 4.0).sqrt()).abs() < 1e-10);
        assert!(slope > 70.0);
        assert_eq!(change, 4.0);
        assert!(astar_with_policy(
            &h,
            2,
            1.0,
            Point { x: 0, z: 0 },
            Point { x: 1, z: 1 },
            None,
            policy(),
            || false
        )
        .is_empty());
    }
    #[test]
    fn cancellation_and_unreachable_goals_have_no_fallback() {
        let mut h = vec![0.0; 25];
        assert!(astar_with_policy(
            &h,
            5,
            1.0,
            Point { x: 0, z: 0 },
            Point { x: 4, z: 4 },
            None,
            policy(),
            || true
        )
        .is_empty());
        for x in 0..5 {
            h[2 * 5 + x] = f64::NAN;
        }
        assert!(astar_with_policy(
            &h,
            5,
            1.0,
            Point { x: 0, z: 0 },
            Point { x: 4, z: 4 },
            None,
            policy(),
            || false
        )
        .is_empty());
    }
    #[test]
    fn path_finds_goal() {
        let h = vec![0.0; 25];
        let p = astar(
            &h,
            5,
            1.0,
            Point { x: 0, z: 0 },
            Point { x: 4, z: 4 },
            true,
            false,
        );
        assert_eq!(p.first(), Some(&Point { x: 0, z: 0 }));
        assert_eq!(p.last(), Some(&Point { x: 4, z: 4 }));
    }
    #[test]
    fn slope_is_symmetric() {
        let h = vec![0., 2., 0., 0.];
        let a = Point { x: 0, z: 0 };
        let b = Point { x: 1, z: 0 };
        assert!((slope_degrees(&h, 2, 1., a, b) - slope_degrees(&h, 2, 1., b, a)).abs() < 1e-12);
    }
    #[test]
    fn observation_reduces_variance() {
        let e = Estimate {
            x: 0.,
            z: 0.,
            vx: 1.,
            vz: 0.,
            variance: 4.,
        };
        let p = predict(e, 0., 0., 1., 1.);
        assert!(observe(p, 1., 0., 1.).variance < p.variance);
    }
}
