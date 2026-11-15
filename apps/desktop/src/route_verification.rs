//! Opt-in, bounded real-data evaluation of the native route/statistics pipeline.
#[path = "../../../crates/mans-nav/tests/support/reference.rs"]
mod reference;
use crate::terrain_navigation::NavigationGrid;
use mans_nav::{Point, RoutePolicy};
use reference::{tolerance, Model};
use serde_json::json;

#[test]
#[ignore = "Requires local HiRISE assets; run explicitly in release mode (four fixed pairs)"]
fn hirise_reference_report() {
    let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let manifest = root.join("data/local/hirise-tiles/manifest.json");
    let grid = NavigationGrid::load(&manifest).expect("Processed HiRISE assets must be installed");
    assert_eq!(
        grid.size, 1534,
        "Evaluation coordinates target the existing 3x3 asset"
    );
    let cases = [
        (
            "column-seam",
            Point { x: 500, z: 900 },
            Point { x: 525, z: 925 },
        ),
        (
            "row-seam",
            Point { x: 900, z: 500 },
            Point { x: 925, z: 525 },
        ),
        (
            "four-tile-junction",
            Point { x: 1005, z: 1005 },
            Point { x: 1040, z: 1040 },
        ),
        (
            "regional-crossing",
            Point { x: 244, z: 1268 },
            Point { x: 1271, z: 247 },
        ),
    ];
    let mut rows = vec![];
    let mut all_pass = true;
    for (id, start, goal) in cases {
        assert!(grid.heights[start.z * grid.size + start.x].is_finite());
        assert!(grid.heights[goal.z * grid.size + goal.x].is_finite());
        let a = grid.hit(start);
        let b = grid.hit(goal);
        let mut outcomes = vec![];
        let mut paths = vec![];
        let mut metrics = vec![];
        let mut passed = true;
        for policy in [
            RoutePolicy {
                max_slope_deg: 23.0,
                slope_weight: 0.0,
                elevation_weight: 0.0,
            },
            RoutePolicy::default(),
        ] {
            let model = Model {
                heights: &grid.heights,
                size: grid.size,
                spacing: grid.manifest.cell_size,
                blocked: None,
                policy,
            };
            let begun = std::time::Instant::now();
            let production = grid.route(
                [a.easting, a.northing],
                [b.easting, b.northing],
                policy,
                || false,
            );
            let optimum = model.dijkstra(start, goal);
            let seconds = begun.elapsed().as_secs_f64();
            match (production, optimum) {
                (Ok(route), Some((cost, settled))) => {
                    let path: Vec<_> = route
                        .samples
                        .iter()
                        .filter(|s| s.x.fract() == 0.0 && s.z.fract() == 0.0)
                        .map(|s| Point {
                            x: s.x as usize,
                            z: s.z as usize,
                        })
                        .collect();
                    let independently = model.measure(&path, start, goal);
                    if let Some(m) = independently {
                        let differences = [
                            route.length - m.distance,
                            route.horizontal - m.horizontal,
                            route.gain - m.gain,
                            route.loss - m.loss,
                            route.max_slope - m.max_slope,
                        ];
                        let valid = (m.cost - cost).abs() <= tolerance(cost)
                            && differences.iter().all(|d| d.abs() < 1e-7)
                            && m.max_slope <= policy.max_slope_deg + 1e-9;
                        passed &= valid;
                        outcomes.push(json!({"surfaceDistanceM":m.distance,"horizontalDistanceM":m.horizontal,"elevationGainM":m.gain,"elevationLossM":m.loss,"maximumSlopeDeg":m.max_slope,"objectiveCost":m.cost,"dijkstraOptimalCost":cost,"astarMinusDijkstraCost":m.cost-cost,"tolerance":tolerance(cost),"productionMinusReferenceStatistics":differences,"allEdgesValid":true,"dijkstraSettledNodes":settled,"routeSamples":path.len(),"seconds":seconds,"pass":valid}));
                        paths.push(Some(path));
                        metrics.push(Some(m));
                    } else {
                        passed = false;
                        outcomes.push(json!({"pass":false,"error":"Production route has invalid endpoints or terrain edges"}));
                        paths.push(None);
                        metrics.push(None);
                    }
                }
                (Err(error), None) => {
                    // These fixed real-data pairs are known reachable. A shared
                    // failure must not turn a broken integration into a pass.
                    passed = false;
                    outcomes.push(
                        json!({"reachable":false,"pass":false,"error":error,"seconds":seconds}),
                    );
                    paths.push(None);
                    metrics.push(None);
                }
                (production, optimum) => {
                    passed = false;
                    outcomes.push(json!({"pass":false,"error":"A* / Dijkstra reachability mismatch","astarReachable":production.is_ok(),"dijkstraReachable":optimum.is_some()}));
                    paths.push(None);
                    metrics.push(None);
                }
            }
        }
        let mut distance_order = None;
        let mut objective_order = None;
        if let (Some(short), Some(risk)) = (metrics[0], metrics[1]) {
            let order = short.distance <= risk.distance + tolerance(risk.distance);
            let risk_model = Model {
                heights: &grid.heights,
                size: grid.size,
                spacing: grid.manifest.cell_size,
                blocked: None,
                policy: RoutePolicy::default(),
            };
            let short_risk_cost = risk_model
                .measure(paths[0].as_ref().unwrap(), start, goal)
                .unwrap()
                .cost;
            let cost_order = risk.cost <= short_risk_cost + tolerance(short_risk_cost);
            passed &= order && cost_order;
            distance_order = Some(order);
            objective_order = Some(cost_order);
        }
        all_pass &= passed;
        rows.push(json!({"routeId":id,"start":{"column":start.x,"row":start.z,"tile":a.tile,"eastingM":a.easting,"northingM":a.northing},"destination":{"column":goal.x,"row":goal.z,"tile":b.tile,"eastingM":b.easting,"northingM":b.northing},"shortest":outcomes[0],"lowerRisk":outcomes[1],"shortestDistanceNotGreater":distance_order,"lowerRiskObjectiveNotGreaterThanShortestPath":objective_order,"pass":passed}));
        eprintln!("{id}: {}", if passed { "PASS" } else { "FAIL" });
    }
    let checksum = |path: &std::path::Path| {
        let output = std::process::Command::new("shasum")
            .args(["-a", "256"])
            .arg(path)
            .output()
            .expect("shasum available");
        assert!(output.status.success());
        String::from_utf8(output.stdout)
            .unwrap()
            .split_whitespace()
            .next()
            .unwrap()
            .to_string()
    };
    let mut assets =
        vec![json!({"file":"data/local/hirise-tiles/manifest.json","sha256":checksum(&manifest)})];
    for tile in &grid.manifest.tiles {
        for file in ["terrain.json", "valid.u8"] {
            let relative = format!("data/local/hirise-tiles/{}/{file}", tile.path);
            assets.push(json!({"file":relative,"sha256":checksum(&root.join(&relative))}));
        }
    }
    let report = json!({"dataset":grid.manifest.name,"dimensions":[grid.size,grid.size],"cellSizeM":grid.manifest.cell_size,"crs":grid.manifest.crs,"selection":"Four fixed valid endpoint pairs: three tile seams/junctions and one regional crossing. Entire 1534x1534 graph used without spatial pruning.","policies":{"hardSlopeLimitDeg":23.0,"shortest":{"slopeWeight":0.0,"elevationWeight":0.0},"lowerRisk":{"slopeWeight":2.5,"elevationWeight":1.0}},"reference":"Independent test-only Dijkstra and facet/constraint/statistics implementation; production edge helpers not called.","maximumSettledNodesPerSearch":grid.size*grid.size,"costTolerance":"1e-7 + 1e-9 * abs(Dijkstra cost)","statisticsTolerance":1e-7,"sourceAssets":assets,"routes":rows,"pass":all_pass,"limitations":"Bounded numerical checks of a discrete measured-elevation model, not exhaustive proof or astronaut safety certification. No hazards beyond represented valid-data and slope constraints were validated. No maximum-slope ordering is assumed between policies."});
    let path = root.join("docs/validation/hirise-route-verification.json");
    std::fs::create_dir_all(path.parent().unwrap()).unwrap();
    std::fs::write(
        &path,
        format!("{}\n", serde_json::to_string_pretty(&report).unwrap()),
    )
    .unwrap();
    eprintln!("Report: {}", path.display());
    assert!(
        all_pass,
        "Numerical verification failed; inspect saved report"
    );
}
