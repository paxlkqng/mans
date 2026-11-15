//! Explicit ground-level simulation; no sensor or localization observations.
use crate::terrain_navigation::NavigationRoute;
use glam::{Vec2, Vec3};

pub const EYE_HEIGHT: f32 = 1.7;
pub const SPEED: f32 = 1.2;
pub const DEVIATION_LIMIT: f32 = 10.0;
pub struct Astronaut {
    pub ground: Vec3,
    pub heading: f32, // radians clockwise from projected north (-Z)
    pub pitch: f32,
    pub waypoint: usize,
    pub error: Option<String>,
    pub focus_viewport: bool,
}
pub struct Guidance {
    pub bearing: f32,
    pub distance: f32,
    pub deviation: f32,
    pub arrived: bool,
}
impl Astronaut {
    pub fn new(ground: Vec3, route: &NavigationRoute) -> Self {
        let heading = route
            .segments
            .first()
            .map(|s| {
                let d = Vec3::from(s.b) - ground;
                d.x.atan2(-d.z).rem_euclid(std::f32::consts::TAU)
            })
            .unwrap_or(0.0);
        Self {
            ground,
            heading,
            pitch: 0.0,
            waypoint: 0,
            error: None,
            focus_viewport: true,
        }
    }
    pub fn eye(&self) -> Vec3 {
        self.ground + Vec3::Y * EYE_HEIGHT
    }
    pub fn look(&mut self, yaw: f32, pitch: f32) {
        self.heading = (self.heading + yaw).rem_euclid(std::f32::consts::TAU);
        self.pitch = (self.pitch + pitch).clamp(-1.1, 1.1);
    }
    pub fn move_ground(
        &mut self,
        input: Vec2,
        dt: f32,
        mut sample: impl FnMut(f32, f32) -> Option<f32>,
    ) {
        let Some(height) = sample(self.ground.x, self.ground.z) else {
            self.error = Some(
                "Measured terrain unavailable here. Wait for tiles or return to Mission Planning."
                    .into(),
            );
            return;
        };
        self.ground.y = height;
        self.error = None;
        let input = input.clamp_length_max(1.0);
        let forward = Vec2::new(self.heading.sin(), -self.heading.cos());
        let right = Vec2::new(self.heading.cos(), self.heading.sin());
        let delta = (right * input.x + forward * input.y) * SPEED * dt.clamp(0.0, 0.1);
        let steps = (delta.length() / 0.1).ceil().max(1.0) as usize;
        let step = delta / steps as f32;
        for _ in 0..steps {
            if step.length_squared() < 1e-12 {
                break;
            }
            let mut next = Vec3::new(
                self.ground.x + step.x,
                self.ground.y,
                self.ground.z + step.y,
            );
            let Some(h) = sample(next.x, next.z) else {
                self.error=Some("Movement stopped: no valid loaded terrain ahead. Wait for loading or return to planning.".into());
                break;
            };
            next.y = h;
            if (h - self.ground.y).abs() > step.length() * 35.0_f32.to_radians().tan() {
                self.error=Some("Movement stopped: simulated 35° walking limit. Return to the route or recalculate.".into());
                break;
            }
            // Cap 3D speed on inclines, not only projected horizontal speed.
            let scale = (step.length() / next.distance(self.ground).max(1e-9)).min(1.0);
            if scale < 1.0 {
                next = self.ground + (next - self.ground) * scale;
                let Some(h) = sample(next.x, next.z) else {
                    self.error = Some("Terrain unavailable; movement stopped.".into());
                    break;
                };
                next.y = h;
            }
            self.ground = next;
        }
    }
    pub fn guidance(&mut self, route: &NavigationRoute) -> Guidance {
        let position = Vec2::new(self.ground.x, self.ground.z);
        let mut deviation = f32::INFINITY;
        let mut nearest = 0;
        for (index, segment) in route.segments.iter().enumerate() {
            let a = Vec2::new(segment.a[0], segment.a[2]);
            let b = Vec2::new(segment.b[0], segment.b[2]);
            let d = b - a;
            let t = ((position - a).dot(d) / d.length_squared().max(1e-9)).clamp(0.0, 1.0);
            let distance = position.distance(a + t * d);
            if distance < deviation {
                deviation = distance;
                nearest = index;
            }
        }
        let goal = Vec2::new(route.goal.world.x, route.goal.world.z);
        if route.segments.is_empty() {
            deviation = position.distance(goal);
        }
        self.waypoint = self
            .waypoint
            .max(nearest)
            .min(route.segments.len().saturating_sub(1));
        while self.waypoint + 1 < route.segments.len() {
            let b = route.segments[self.waypoint].b;
            if position.distance(Vec2::new(b[0], b[2])) > 2.0 {
                break;
            }
            self.waypoint += 1;
        }
        let target = route
            .segments
            .get(self.waypoint)
            .map(|s| Vec2::new(s.b[0], s.b[2]))
            .unwrap_or(goal);
        let delta = target - position;
        Guidance {
            bearing: delta.x.atan2(-delta.y).to_degrees().rem_euclid(360.0),
            distance: delta.length(),
            deviation,
            arrived: position.distance(goal) <= 1.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn route() -> NavigationRoute {
        use crate::{
            terrain_navigation::{RouteSample, RouteSegment},
            terrain_picking::TerrainHit,
        };
        let hit = |world: Vec3| TerrainHit {
            tile: "test".into(),
            world,
            row: 0.0,
            column: 0.0,
            easting: world.x as f64,
            northing: -world.z as f64,
            elevation: world.y as f64,
            geographic: None,
            coarse: false,
        };
        NavigationRoute {
            start: hit(Vec3::ZERO),
            goal: hit(Vec3::new(10.0, 0.0, -10.0)),
            samples: vec![RouteSample {
                x: 0.0,
                z: 0.0,
                elevation: 0.0,
                distance: 0.0,
            }],
            segments: vec![
                RouteSegment {
                    tile: 0,
                    a: [0.0, 0.0, 0.0],
                    b: [0.0, 0.0, -10.0],
                },
                RouteSegment {
                    tile: 0,
                    a: [0.0, 0.0, -10.0],
                    b: [10.0, 0.0, -10.0],
                },
            ],
            length: 20.0,
            horizontal: 20.0,
            gain: 0.0,
            loss: 0.0,
            max_slope: 0.0,
            cells: 3,
            policy: mans_nav::RoutePolicy::default(),
        }
    }
    #[test]
    fn waypoint_bearing_progress_deviation_and_arrival_use_the_route() {
        let route = route();
        let mut a = Astronaut::new(Vec3::ZERO, &route);
        let g = a.guidance(&route);
        assert_eq!(g.bearing, 0.0);
        assert_eq!(g.distance, 10.0);
        assert_eq!(g.deviation, 0.0);
        a.ground = Vec3::new(0.0, 0.0, -10.0);
        let g = a.guidance(&route);
        assert_eq!(g.bearing, 90.0);
        assert_eq!(a.waypoint, 1);
        assert!(!g.arrived);
        a.ground = Vec3::new(30.0, 0.0, 0.0);
        assert!(a.guidance(&route).deviation > DEVIATION_LIMIT);
        a.ground = route.goal.world;
        assert!(a.guidance(&route).arrived);
    }
    #[test]
    fn first_person_projection_preserves_planning_camera_and_heading() {
        use crate::terrain_renderer::Camera;
        let planning = Camera::new(1548.0, 100.0);
        let before = planning.matrix(1.5);
        for heading in [0.0, std::f32::consts::FRAC_PI_2, 3.0] {
            let eye = Vec3::new(1.0, 1.7, 2.0);
            let camera = Camera::ground_view(eye, heading, 0.0, 1548.0);
            assert_eq!(camera.eye(), eye);
            let forward = Vec3::new(heading.sin(), 0.0, -heading.cos());
            let clip = camera.matrix(1.5).project_point3(eye + forward * 0.1);
            assert!(clip.x.abs() < 1e-4 && clip.y.abs() < 1e-4);
            assert!((0.0..1.0).contains(&clip.z));
        }
        assert_eq!(planning.matrix(1.5), before);
    }
    fn walker() -> Astronaut {
        Astronaut {
            ground: Vec3::ZERO,
            heading: 0.0,
            pitch: 0.0,
            waypoint: 0,
            error: None,
            focus_viewport: false,
        }
    }
    #[test]
    fn movement_heading_height_and_speed_are_consistent() {
        let mut a = walker();
        a.move_ground(Vec2::Y, 0.1, |_, z| Some(z * 0.2));
        assert!(a.ground.z < 0.0);
        assert!((a.ground.y - a.ground.z * 0.2).abs() < 1e-6);
        assert!(a.ground.length() <= SPEED * 0.1 + 1e-6);
        assert!((a.eye().y - a.ground.y - 1.7).abs() < 1e-6);
        a.look(std::f32::consts::FRAC_PI_2, 10.0);
        let p = a.ground;
        a.move_ground(Vec2::Y, 0.1, |_, z| Some(z * 0.2));
        assert!(a.ground.x > p.x);
        assert!(a.pitch <= 1.1);
    }
    #[test]
    fn diagonal_movement_is_capped_and_missing_terrain_stops_movement() {
        let mut a = walker();
        a.move_ground(Vec2::ONE, 10.0, |_, _| Some(0.0));
        assert!(a.ground.length() <= SPEED * 0.1 + 1e-6);
        let p = a.ground;
        a.move_ground(Vec2::Y, 0.1, |_, _| None);
        assert_eq!(a.ground, p);
        assert!(a.error.is_some());
    }
    #[test]
    fn excessive_measured_step_is_rejected() {
        let mut a = walker();
        a.move_ground(Vec2::Y, 0.1, |_, z| Some(if z < 0.0 { 10.0 } else { 0.0 }));
        assert_eq!(a.ground, Vec3::ZERO);
        assert!(a.error.is_some());
    }
}
