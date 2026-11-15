#[path = "support/reference.rs"]
mod reference;
use mans_nav::{astar_with_policy, Point, RoutePolicy};
use reference::{tolerance, Model};

fn verify(h: &[f64], n: usize, mask: Option<&[bool]>, start: Point, goal: Point) {
    let mut routes = vec![];
    for policy in [
        RoutePolicy {
            max_slope_deg: 23.0,
            slope_weight: 0.0,
            elevation_weight: 0.0,
        },
        RoutePolicy::default(),
    ] {
        let model = Model {
            heights: h,
            size: n,
            spacing: 1.0096633268711,
            blocked: mask,
            policy,
        };
        let path = astar_with_policy(h, n, model.spacing, start, goal, mask, policy, || false);
        match model.dijkstra(start, goal) {
            None => {
                assert!(path.is_empty());
                routes.push(None);
            }
            Some((cost, _)) => {
                let metrics = model
                    .measure(&path, start, goal)
                    .expect("A* crossed an invalid edge");
                assert!(
                    (metrics.cost - cost).abs() <= tolerance(cost),
                    "A*: {} Dijkstra: {cost}",
                    metrics.cost
                );
                routes.push(Some((path, metrics)));
            }
        }
    }
    if let (Some((short, s)), Some((_, risk))) = (&routes[0], &routes[1]) {
        assert!(s.distance <= risk.distance + tolerance(risk.distance));
        let model = Model {
            heights: h,
            size: n,
            spacing: 1.0096633268711,
            blocked: mask,
            policy: RoutePolicy::default(),
        };
        assert!(
            risk.cost <= model.measure(short, start, goal).unwrap().cost + tolerance(risk.cost)
        );
        // No ordering assumption about maximum slope: total objective is tested.
    }
}
#[test]
fn flat_diagonals_and_same_point() {
    let h = vec![0.0; 81];
    verify(&h, 9, None, Point { x: 0, z: 0 }, Point { x: 8, z: 8 });
    verify(&h, 9, None, Point { x: 8, z: 0 }, Point { x: 0, z: 8 });
    verify(&h, 9, None, Point { x: 8, z: 8 }, Point { x: 0, z: 0 });
    verify(&h, 9, None, Point { x: 2, z: 2 }, Point { x: 2, z: 2 });
}
#[test]
fn steep_barrier_with_detour_and_unreachable_wall() {
    let mut h = vec![0.0; 81];
    for z in 0..8 {
        h[z * 9 + 4] = 20.0;
    }
    verify(&h, 9, None, Point { x: 0, z: 0 }, Point { x: 8, z: 0 });
    h[8 * 9 + 4] = 20.0;
    verify(&h, 9, None, Point { x: 0, z: 0 }, Point { x: 8, z: 0 });
}
#[test]
fn nodata_blocked_corners_and_invalid_endpoints() {
    let mut h = vec![0.0; 81];
    let mut mask = vec![false; 81];
    mask[1] = true;
    mask[9] = true;
    verify(
        &h,
        9,
        Some(&mask),
        Point { x: 0, z: 0 },
        Point { x: 8, z: 8 },
    );
    h[1] = f64::NAN;
    h[9] = f64::NAN;
    verify(&h, 9, None, Point { x: 0, z: 0 }, Point { x: 8, z: 8 });
    verify(&h, 9, None, Point { x: 1, z: 0 }, Point { x: 8, z: 8 });
    let mut h = vec![0.0; 81];
    for z in 1..8 {
        h[z * 9 + 4] = f64::NAN;
    }
    verify(&h, 9, None, Point { x: 0, z: 4 }, Point { x: 8, z: 4 });
}
#[test]
fn fixed_seed_terrains_verify_objectives_without_maximum_slope_assumptions() {
    let mut seed = 0x4d414e53_u64;
    for _ in 0..96 {
        let h: Vec<_> = (0..81)
            .map(|_| {
                seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
                ((seed >> 32) % 17) as f64 * 0.035
            })
            .collect();
        verify(&h, 9, None, Point { x: 0, z: 0 }, Point { x: 8, z: 8 });
    }
}
#[test]
fn oracle_analytic_geometry_and_isolated_samples() {
    let policy = RoutePolicy {
        max_slope_deg: 89.0,
        slope_weight: 0.0,
        elevation_weight: 0.0,
    };
    let h = [0.0, 2.0, 2.0, 0.0];
    let model = Model {
        heights: &h,
        size: 2,
        spacing: 1.0,
        blocked: None,
        policy,
    };
    let m = model
        .edge(Point { x: 0, z: 0 }, Point { x: 1, z: 1 })
        .unwrap();
    assert!((m.distance - 2.0 * 4.5_f64.sqrt()).abs() < 1e-12);
    assert_eq!(m.gain, 2.0);
    assert_eq!(m.loss, 2.0);
    let h = [0.0, 0.0, f64::NAN, f64::NAN];
    verify(&h, 2, None, Point { x: 0, z: 0 }, Point { x: 1, z: 0 });
}
