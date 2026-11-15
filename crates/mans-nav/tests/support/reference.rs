//! Test-only independent graph model and Dijkstra oracle. No production cost,
//! neighbor, slope, heuristic, or statistics helper is called here.
use mans_nav::{Point, RoutePolicy};
use std::{cmp::Reverse, collections::BinaryHeap};

pub struct Model<'a> {
    pub heights: &'a [f64],
    pub size: usize,
    pub spacing: f64,
    pub blocked: Option<&'a [bool]>,
    pub policy: RoutePolicy,
}
#[derive(Clone, Copy, Debug, Default)]
pub struct Metrics {
    pub distance: f64,
    pub horizontal: f64,
    pub gain: f64,
    pub loss: f64,
    pub max_slope: f64,
    pub cost: f64,
}
impl Model<'_> {
    fn observed(&self, i: usize) -> bool {
        self.heights[i].is_finite() && self.heights[i] > -1e10
    }
    fn usable(&self, p: Point) -> bool {
        p.x < self.size
            && p.z < self.size
            && self.observed(p.z * self.size + p.x)
            && !self.blocked.is_some_and(|b| b[p.z * self.size + p.x])
    }
    pub fn edge(&self, a: Point, b: Point) -> Option<Metrics> {
        if !self.usable(a) || !self.usable(b) {
            return None;
        }
        let dx = a.x.abs_diff(b.x);
        let dz = a.z.abs_diff(b.z);
        if dx > 1 || dz > 1 || dx + dz == 0 {
            return None;
        }
        let h = |p: Point| self.heights[p.z * self.size + p.x];
        let angle = |rise: f64, run: f64| rise.abs().atan2(run).to_degrees();
        if dx == 1 && dz == 1 {
            for side in [Point { x: a.x, z: b.z }, Point { x: b.x, z: a.z }] {
                if !self.usable(side)
                    || angle(h(side) - h(a), self.spacing) > self.policy.max_slope_deg
                    || angle(h(b) - h(side), self.spacing) > self.policy.max_slope_deg
                {
                    return None;
                }
            }
        } else {
            // Independently enumerate adjacent mesh facets to establish support.
            let mut supported = false;
            for z in a.z.min(b.z).saturating_sub(1)..=a.z.min(b.z).min(self.size - 2) {
                for x in a.x.min(b.x).saturating_sub(1)..=a.x.min(b.x).min(self.size - 2) {
                    let tl = z * self.size + x;
                    let tr = tl + 1;
                    let bl = tl + self.size;
                    let br = bl + 1;
                    for triangle in [[tl, bl, tr], [tr, bl, br]] {
                        if triangle.contains(&(a.z * self.size + a.x))
                            && triangle.contains(&(b.z * self.size + b.x))
                            && triangle.iter().all(|i| self.observed(*i))
                        {
                            supported = true;
                        }
                    }
                }
            }
            if !supported {
                return None;
            }
        }
        let planar = ((dx * dx + dz * dz) as f64).sqrt() * self.spacing;
        let mut elevations = vec![h(a)];
        if dx == 1 && dz == 1 && (a.x < b.x) == (a.z < b.z) {
            elevations.push((h(Point { x: a.x, z: b.z }) + h(Point { x: b.x, z: a.z })) / 2.0);
        }
        elevations.push(h(b));
        let run = planar / (elevations.len() - 1) as f64;
        let mut m = Metrics {
            horizontal: planar,
            ..Default::default()
        };
        for pair in elevations.windows(2) {
            let dh = pair[1] - pair[0];
            m.distance += (run * run + dh * dh).sqrt();
            m.gain += dh.max(0.0);
            m.loss += (-dh).max(0.0);
            m.max_slope = m.max_slope.max(angle(dh, run));
        }
        if m.max_slope > self.policy.max_slope_deg {
            return None;
        }
        m.cost = m.distance * (1.0 + self.policy.slope_weight * (m.max_slope / 12.0).powi(2))
            + self.policy.elevation_weight * (m.gain + m.loss);
        Some(m)
    }
    pub fn measure(&self, path: &[Point], start: Point, goal: Point) -> Option<Metrics> {
        if path.first() != Some(&start) || path.last() != Some(&goal) || !self.usable(start) {
            return None;
        }
        let mut total = Metrics::default();
        for pair in path.windows(2) {
            let m = self.edge(pair[0], pair[1])?;
            total.distance += m.distance;
            total.horizontal += m.horizontal;
            total.gain += m.gain;
            total.loss += m.loss;
            total.max_slope = total.max_slope.max(m.max_slope);
            total.cost += m.cost;
        }
        Some(total)
    }
    /// Full supplied graph, no spatial crop, heuristic, or route-based pruning.
    /// At most size² nodes are settled; duplicate queue records are skipped.
    pub fn dijkstra(&self, start: Point, goal: Point) -> Option<(f64, usize)> {
        if !self.usable(start) || !self.usable(goal) {
            return None;
        }
        let mut best = vec![f64::INFINITY; self.heights.len()];
        let mut settled = vec![false; self.heights.len()];
        let mut queue = BinaryHeap::new();
        let sid = start.z * self.size + start.x;
        let gid = goal.z * self.size + goal.x;
        best[sid] = 0.0;
        queue.push(Reverse((0_f64.to_bits(), sid)));
        let mut count = 0;
        while let Some(Reverse((bits, id))) = queue.pop() {
            if settled[id] {
                continue;
            }
            let cost = f64::from_bits(bits);
            settled[id] = true;
            count += 1;
            if id == gid {
                return Some((cost, count));
            }
            let a = Point {
                x: id % self.size,
                z: id / self.size,
            };
            for (dx, dz) in [
                (1, 0),
                (0, 1),
                (-1, 0),
                (0, -1),
                (1, 1),
                (-1, 1),
                (1, -1),
                (-1, -1),
            ] {
                let x = a.x as isize + dx;
                let z = a.z as isize + dz;
                if x < 0 || z < 0 || x >= self.size as isize || z >= self.size as isize {
                    continue;
                }
                let b = Point {
                    x: x as usize,
                    z: z as usize,
                };
                let next = b.z * self.size + b.x;
                if settled[next] {
                    continue;
                }
                if let Some(m) = self.edge(a, b) {
                    let candidate = cost + m.cost;
                    if candidate < best[next] {
                        best[next] = candidate;
                        queue.push(Reverse((candidate.to_bits(), next)));
                    }
                }
            }
        }
        None
    }
}
pub fn tolerance(cost: f64) -> f64 {
    1e-7 + 1e-9 * cost.abs()
}
