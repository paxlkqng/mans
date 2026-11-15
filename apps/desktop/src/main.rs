mod astronaut;
mod route_renderer;
mod terrain_navigation;
mod terrain_picking;
mod terrain_renderer;
mod terrain_tiles;
use eframe::egui::{self, Color32, Pos2, Rect, Sense, Stroke, StrokeKind, Vec2};
use mans_nav::{astar, slope_degrees, Point};
use serde::{Deserialize, Serialize};
use std::fs;

const TERRAIN_JSON: &str = include_str!("../../../data/real/jezero-mola-128.json");

#[derive(Debug, Deserialize)]
#[serde(rename_all = "camelCase")]
struct Terrain {
    size: usize,
    cell_size: f64,
    heights: Vec<f64>,
    source: String,
    provenance: String,
    metadata: Option<TerrainMetadata>,
}

#[derive(Debug, Deserialize)]
#[serde(rename_all = "camelCase")]
struct TerrainMetadata {
    dataset_name: String,
    product_id: String,
    source_url: String,
    coordinate_reference_system: String,
    vertical_datum: String,
    resolution: String,
    center_latitude_deg: Option<f64>,
    center_longitude_deg: Option<f64>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
enum Tool {
    Inspect,
    Start,
    Goal,
}

#[derive(Serialize, Deserialize)]
#[serde(default)]
struct SavedState {
    start: [usize; 2],
    goal: [usize; 2],
    tool: Tool,
    safest: bool,
    avoid_blocked: bool,
    show_slope: bool,
    show_grid: bool,
}

impl Default for SavedState {
    fn default() -> Self {
        Self {
            start: [3, 3],
            goal: [12, 12],
            tool: Tool::Inspect,
            safest: true,
            avoid_blocked: true,
            show_slope: true,
            show_grid: true,
        }
    }
}

struct MissionApp {
    navigation_mode: bool,
    astronaut: Option<astronaut::Astronaut>,
    navigation_route: usize,
    route_worker: Option<terrain_navigation::RouteWorker>,
    route_3d: Option<terrain_navigation::RouteComparison>,
    route_visible: [bool; 2],
    route_revision: u64,
    route_busy: bool,
    route_error: Option<String>,
    route_policy: mans_nav::RoutePolicy,
    last_terrain_click: Option<(Pos2, f64)>,
    selected_3d: Option<terrain_picking::TerrainHit>,
    start_3d: Option<terrain_picking::TerrainHit>,
    goal_3d: Option<terrain_picking::TerrainHit>,
    terrain: Terrain,
    state: SavedState,
    route: Vec<Point>,
    inspected: Option<Point>,
    message: String,
    load_warning: Option<String>,
    gpu_diagnostic: bool,
    camera: terrain_renderer::Camera,
    display_mode: terrain_renderer::DisplayMode,
    texture_warning: Option<String>,
    render_state: egui_wgpu::RenderState,
    tile_manifest: Option<terrain_tiles::Manifest>,
    dataset: usize,
    boundaries: bool,
    cache_capacity: usize,
}

fn load_hirise(path: &std::path::Path) -> Result<Terrain, String> {
    let json = fs::read_to_string(path).map_err(|error| format!("{}: {error}", path.display()))?;
    let terrain: Terrain = serde_json::from_str(&json)
        .map_err(|error| format!("{}: JSON parse error: {error}", path.display()))?;
    if terrain.size < 2 || terrain.size.checked_mul(terrain.size) != Some(terrain.heights.len()) {
        return Err(format!(
            "{}: invalid grid dimensions {} / {} heights",
            path.display(),
            terrain.size,
            terrain.heights.len()
        ));
    }
    if !terrain.cell_size.is_finite()
        || terrain.cell_size <= 0.0
        || terrain
            .heights
            .iter()
            .any(|h| !h.is_finite() || *h < -1.0e10)
    {
        return Err(format!(
            "{}: invalid spacing or elevation samples",
            path.display()
        ));
    }
    Ok(terrain)
}

impl MissionApp {
    fn new(cc: &eframe::CreationContext<'_>) -> Self {
        let state = cc
            .wgpu_render_state
            .as_ref()
            .expect("MANS requires the wgpu renderer");
        tracing::info!(adapter = ?state.adapter.get_info(), "initializing terrain GPU resources");
        let path = std::env::var_os("MANS_TERRAIN_PATH")
            .map(std::path::PathBuf::from)
            .unwrap_or_else(|| {
                std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                    .join("../../data/local/hirise-jezero/terrain.json")
            });
        tracing::info!(cwd = ?std::env::current_dir(), path = %path.display(), "loading HiRISE asset");
        let (terrain, load_warning) = match load_hirise(&path) {
            Ok(terrain) => {
                tracing::info!(
                    size = terrain.size,
                    cell_m = terrain.cell_size,
                    min_m = terrain
                        .heights
                        .iter()
                        .copied()
                        .fold(f64::INFINITY, f64::min),
                    max_m = terrain
                        .heights
                        .iter()
                        .copied()
                        .fold(f64::NEG_INFINITY, f64::max),
                    "HiRISE loaded"
                );
                (terrain, None)
            }
            Err(error) => {
                tracing::error!(%error, "HiRISE load failed; using MOLA regional fallback");
                (
                    serde_json::from_str(TERRAIN_JSON).expect("bundled MOLA grid"),
                    Some(format!("HiRISE load failed — MOLA fallback: {error}")),
                )
            }
        };
        let ortho = if load_warning.is_none() {
            terrain_renderer::load_orthophoto(
                path.parent().expect("terrain asset directory"),
                terrain.size,
            )
        } else {
            Err("Satellite imagery unavailable for MOLA regional fallback.".into())
        };
        let texture_warning = ortho.as_ref().err().cloned();
        if let Some(warning) = &texture_warning {
            tracing::warn!(%warning,"orthophoto unavailable; using elevation shading");
        }
        let display_mode = if ortho.is_ok() {
            terrain_renderer::DisplayMode::Satellite
        } else {
            terrain_renderer::DisplayMode::Elevation
        };
        state.renderer.write().callback_resources.insert(
            terrain_renderer::TerrainRenderer::new(
                &state.device,
                &state.queue,
                state.target_format,
                terrain.size,
                terrain.cell_size,
                &terrain.heights,
                ortho.as_ref().ok(),
            )
            .expect("validated terrain must produce a GPU mesh"),
        );
        let state: SavedState = cc
            .storage
            .and_then(|storage| eframe::get_value(storage, eframe::APP_KEY))
            .unwrap_or_default();
        let mut state = state;
        let last = terrain.size.saturating_sub(1);
        state.start = [state.start[0].min(last), state.start[1].min(last)];
        state.goal = [state.goal[0].min(last), state.goal[1].min(last)];
        let min = terrain
            .heights
            .iter()
            .copied()
            .fold(f64::INFINITY, f64::min);
        let max = terrain
            .heights
            .iter()
            .copied()
            .fold(f64::NEG_INFINITY, f64::max);
        let camera = terrain_renderer::Camera::new(
            ((terrain.size - 1) as f64 * terrain.cell_size) as f32,
            (max - min) as f32,
        );
        let manifest_path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../data/local/hirise-tiles/manifest.json");
        let tile_manifest = terrain_tiles::Manifest::read(&manifest_path).ok();
        let dataset = if load_warning.is_some() {
            2
        } else if tile_manifest.is_some() {
            0
        } else {
            1
        };
        if dataset == 0 {
            if let Err(error) = cc
                .wgpu_render_state
                .as_ref()
                .unwrap()
                .renderer
                .write()
                .callback_resources
                .get_mut::<terrain_renderer::TerrainRenderer>()
                .unwrap()
                .enable_tiles(&manifest_path)
            {
                tracing::warn!(%error,"tile manager initialization failed");
            }
        }
        let camera = tile_manifest
            .as_ref()
            .filter(|_| dataset == 0)
            .map(|m| {
                terrain_renderer::Camera::new(
                    m.extent(),
                    (m.max_elevation - m.min_elevation) as f32,
                )
            })
            .unwrap_or(camera);
        let mut app = Self {
            terrain,
            state,
            route: Vec::new(),
            inspected: None,
            navigation_mode: false,
            astronaut: None,
            navigation_route: 1,
            route_worker: terrain_navigation::RouteWorker::new(
                std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                    .join("../../data/local/hirise-tiles/manifest.json"),
            )
            .map_err(|e| tracing::error!(%e,"route worker creation failed"))
            .ok(),
            route_3d: None,
            route_visible: [true, true],
            route_revision: 0,
            route_busy: false,
            route_error: None,
            route_policy: mans_nav::RoutePolicy::default(),
            last_terrain_click: None,
            selected_3d: None,
            start_3d: None,
            goal_3d: None,
            message: "Select start and destination, then calculate a route.".into(),
            load_warning,
            gpu_diagnostic: true,
            camera,
            display_mode,
            texture_warning,
            render_state: cc.wgpu_render_state.as_ref().unwrap().clone(),
            tile_manifest,
            dataset,
            boundaries: false,
            cache_capacity: 9,
        };
        app.recalculate();
        app
    }

    fn switch_dataset(&mut self, dataset: usize) {
        self.navigation_mode = false;
        self.astronaut = None;
        self.invalidate_3d_route();
        let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../data/local/hirise-jezero/terrain.json");
        let terrain = if dataset == 2 {
            serde_json::from_str(TERRAIN_JSON).map_err(|e| e.to_string())
        } else {
            load_hirise(&path)
        };
        let terrain = match terrain {
            Ok(t) => t,
            Err(e) => {
                self.message = e;
                return;
            }
        };
        let ortho = if dataset == 2 {
            Err("MOLA regional reference has no HiRISE imagery.".into())
        } else {
            terrain_renderer::load_orthophoto(path.parent().unwrap(), terrain.size)
        };
        self.texture_warning = ortho.as_ref().err().cloned();
        self.display_mode = if ortho.is_ok() {
            terrain_renderer::DisplayMode::Satellite
        } else {
            terrain_renderer::DisplayMode::Elevation
        };
        let mut renderer = terrain_renderer::TerrainRenderer::new(
            &self.render_state.device,
            &self.render_state.queue,
            self.render_state.target_format,
            terrain.size,
            terrain.cell_size,
            &terrain.heights,
            ortho.as_ref().ok(),
        )
        .expect("validated terrain");
        if dataset == 0 {
            let manifest = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("../../data/local/hirise-tiles/manifest.json");
            if let Err(e) = renderer.enable_tiles(&manifest) {
                self.message = e;
                return;
            }
        }
        self.render_state
            .renderer
            .write()
            .callback_resources
            .insert(renderer);
        self.dataset = dataset;
        self.last_terrain_click = None;
        self.selected_3d = None;
        self.start_3d = None;
        self.goal_3d = None;
        self.terrain = terrain;
        let last = self.terrain.size - 1;
        self.state.start = self.state.start.map(|p| p.min(last));
        self.state.goal = self.state.goal.map(|p| p.min(last));
        self.inspected = None;
        let min = self
            .terrain
            .heights
            .iter()
            .copied()
            .fold(f64::INFINITY, f64::min);
        let max = self
            .terrain
            .heights
            .iter()
            .copied()
            .fold(f64::NEG_INFINITY, f64::max);
        self.camera = if dataset == 0 {
            let m = self.tile_manifest.as_ref().unwrap();
            terrain_renderer::Camera::new(m.extent(), (m.max_elevation - m.min_elevation) as f32)
        } else {
            terrain_renderer::Camera::new(
                (last as f64 * self.terrain.cell_size) as f32,
                (max - min) as f32,
            )
        };
        self.load_warning = if dataset == 2 {
            Some("MOLA regional reference selected".into())
        } else {
            None
        };
        self.recalculate();
    }

    fn start(&self) -> Point {
        Point {
            x: self.state.start[0],
            z: self.state.start[1],
        }
    }

    fn goal(&self) -> Point {
        Point {
            x: self.state.goal[0],
            z: self.state.goal[1],
        }
    }

    fn recalculate(&mut self) {
        self.route = astar(
            &self.terrain.heights,
            self.terrain.size,
            self.terrain.cell_size,
            self.start(),
            self.goal(),
            self.state.safest,
            self.state.avoid_blocked,
        );
        self.message = if self.route.is_empty() {
            tracing::warn!("route calculation returned no path");
            "No route satisfies the current traversal constraints.".into()
        } else {
            tracing::info!(nodes = self.route.len(), "route calculated");
            format!("Route calculated: {} nodes.", self.route.len())
        };
    }

    fn height(&self, point: Point) -> f64 {
        self.terrain.heights[point.z * self.terrain.size + point.x]
    }

    fn local_slope(&self, point: Point) -> f64 {
        let right = Point {
            x: (point.x + 1).min(self.terrain.size - 1),
            z: point.z,
        };
        let down = Point {
            x: point.x,
            z: (point.z + 1).min(self.terrain.size - 1),
        };
        slope_degrees(
            &self.terrain.heights,
            self.terrain.size,
            self.terrain.cell_size,
            point,
            right,
        )
        .max(slope_degrees(
            &self.terrain.heights,
            self.terrain.size,
            self.terrain.cell_size,
            point,
            down,
        ))
    }

    fn point_at(&self, rect: Rect, position: Pos2) -> Option<Point> {
        if !rect.contains(position) {
            return None;
        }
        let x =
            ((position.x - rect.left()) / rect.width() * self.terrain.size as f32).floor() as usize;
        let z =
            ((position.y - rect.top()) / rect.height() * self.terrain.size as f32).floor() as usize;
        Some(Point {
            x: x.min(self.terrain.size - 1),
            z: z.min(self.terrain.size - 1),
        })
    }

    fn screen_point(&self, rect: Rect, point: Point) -> Pos2 {
        Pos2::new(
            rect.left() + (point.x as f32 + 0.5) / self.terrain.size as f32 * rect.width(),
            rect.top() + (point.z as f32 + 0.5) / self.terrain.size as f32 * rect.height(),
        )
    }

    fn terrain_color(&self, point: Point) -> Color32 {
        let h = self.height(point);
        let normalized = ((h + 2700.0) / 300.0).clamp(0.0, 1.0) as f32;
        let slope = self.local_slope(point);
        let base = if normalized < 0.28 {
            Color32::from_rgb(51, 55, 48)
        } else if normalized < 0.62 {
            Color32::from_rgb(93, 83, 59)
        } else {
            Color32::from_rgb(139, 116, 78)
        };
        if self.state.show_slope && slope > 18.0 {
            Color32::from_rgb(145, 84, 54)
        } else {
            base
        }
    }

    fn route_distance(&self) -> f64 {
        self.route
            .windows(2)
            .map(|segment| {
                let dx = segment[0].x as f64 - segment[1].x as f64;
                let dz = segment[0].z as f64 - segment[1].z as f64;
                (dx * dx + dz * dz).sqrt() * self.terrain.cell_size
            })
            .sum()
    }

    fn invalidate_3d_route(&mut self) {
        self.route_revision += 1;
        if let Some(worker) = &self.route_worker {
            worker
                .revision
                .store(self.route_revision, std::sync::atomic::Ordering::Relaxed);
        }
        self.route_3d = None;
        self.route_busy = false;
        self.route_error = None;
        if let Some(renderer) = self
            .render_state
            .renderer
            .write()
            .callback_resources
            .get_mut::<terrain_renderer::TerrainRenderer>()
        {
            renderer.routes.set(&self.render_state.device, None);
        }
    }
    fn calculate_active_route(&mut self) {
        if self.dataset != 0 || !self.gpu_diagnostic {
            self.recalculate();
            return;
        }
        if self.route_busy {
            return;
        }
        let (Some(start), Some(goal)) = (&self.start_3d, &self.goal_3d) else {
            self.route_error = Some("Select a 3D start and destination first.".into());
            return;
        };
        let start = [start.easting, start.northing];
        let goal = [goal.easting, goal.northing];
        self.invalidate_3d_route();
        let result = self
            .route_worker
            .as_ref()
            .ok_or_else(|| "Route worker unavailable".to_string())
            .and_then(|worker| {
                worker.request(terrain_navigation::Request {
                    revision: self.route_revision,
                    start,
                    goal,
                    policy: self.route_policy,
                })
            });
        match result {
            Ok(()) => {
                self.route_busy = true;
                self.message = "Calculating full-resolution multi-tile route…".into();
            }
            Err(e) => self.route_error = Some(e),
        }
    }
    fn poll_3d_route(&mut self) {
        while let Some((revision, result)) = self
            .route_worker
            .as_ref()
            .and_then(|w| w.results.try_recv().ok())
        {
            if revision != self.route_revision {
                continue;
            }
            self.route_busy = false;
            match result {
                Ok(route) => {
                    if self.navigation_mode {
                        if let Some(a) = &mut self.astronaut {
                            a.waypoint = 0;
                        }
                    } else {
                        self.astronaut = None;
                    }
                    tracing::info!(
                        shortest = route.shortest.length,
                        lower_risk = route.lower_risk.length,
                        identical = route.identical(),
                        "dual-route comparison completed"
                    );
                    self.start_3d = Some(route.shortest.start.clone());
                    self.goal_3d = Some(route.shortest.goal.clone());
                    self.message = format!(
                        "Shortest {:.1} m · Lower-risk {:.1} m · full-resolution HiRISE",
                        route.shortest.length, route.lower_risk.length
                    );
                    if let Some(renderer) = self
                        .render_state
                        .renderer
                        .write()
                        .callback_resources
                        .get_mut::<terrain_renderer::TerrainRenderer>()
                    {
                        renderer.routes.set(&self.render_state.device, Some(&route));
                    }
                    self.route_3d = Some(route);
                }
                Err(e) => {
                    tracing::warn!(%e,"multi-tile route failed");
                    self.message = e.clone();
                    self.route_error = Some(e);
                }
            }
        }
    }

    fn enter_navigation(&mut self) {
        let Some(routes) = &self.route_3d else {
            self.message = "Calculate a HiRISE route before entering Astronaut Navigation.".into();
            return;
        };
        let route = if self.navigation_route == 0 {
            &routes.shortest
        } else {
            &routes.lower_risk
        };
        let p = self
            .astronaut
            .as_ref()
            .map(|a| a.ground)
            .unwrap_or(route.start.world);
        let hit = self
            .render_state
            .renderer
            .read()
            .callback_resources
            .get::<terrain_renderer::TerrainRenderer>()
            .and_then(|r| r.ground(p.x, p.z));
        let Some(hit) = hit else {
            self.message="Cannot enter navigation: measured terrain at the simulation position is not loaded. Load that region in planning first.".into();
            return;
        };
        if self.astronaut.is_none() {
            self.astronaut = Some(astronaut::Astronaut::new(hit.world, route));
        }
        if let Some(a) = &mut self.astronaut {
            a.ground = hit.world;
            a.focus_viewport = true;
        }
        self.navigation_mode = true;
    }
    fn recalculate_from_astronaut(&mut self) {
        let Some(a) = &self.astronaut else {
            return;
        };
        let hit = self
            .render_state
            .renderer
            .read()
            .callback_resources
            .get::<terrain_renderer::TerrainRenderer>()
            .and_then(|r| r.ground(a.ground.x, a.ground.z));
        if let Some(hit) = hit {
            self.start_3d = Some(hit);
            self.calculate_active_route();
        } else if let Some(a) = &mut self.astronaut {
            a.error = Some(
                "Cannot recalculate: measured terrain at the simulated position is unavailable."
                    .into(),
            );
        }
    }
    fn draw_astronaut(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.strong("Simulated astronaut · no measured localization");
            if ui
                .add_enabled(
                    !self.route_busy && self.goal_3d.is_some(),
                    egui::Button::new("Recalculate from here"),
                )
                .clicked()
            {
                self.recalculate_from_astronaut();
            }
            if ui.button("Return to planning (Esc)").clicked() {
                self.navigation_mode = false;
            }
        });
        ui.small("Click the terrain viewport to enable WASD · Arrow keys or left drag: look · 1.2 m/s simulated speed · 35° simulated walking limit");
        let (rect, response) = ui.allocate_exact_size(ui.available_size(), Sense::click_and_drag());
        let Some(a) = &mut self.astronaut else {
            ui.label("Simulation not initialized. Return to Mission Planning.");
            return;
        };
        if a.focus_viewport || response.clicked() {
            response.request_focus();
            a.focus_viewport = false;
        }
        let focused = response.has_focus() && ui.input(|i| i.focused);
        if focused {
            let (movement, turn, pitch, dt, escape) = ui.input(|i| {
                let movement = glam::Vec2::new(
                    i.key_down(egui::Key::D) as i32 as f32 - i.key_down(egui::Key::A) as i32 as f32,
                    i.key_down(egui::Key::W) as i32 as f32 - i.key_down(egui::Key::S) as i32 as f32,
                );
                (
                    movement,
                    i.key_down(egui::Key::ArrowRight) as i32 as f32
                        - i.key_down(egui::Key::ArrowLeft) as i32 as f32,
                    i.key_down(egui::Key::ArrowUp) as i32 as f32
                        - i.key_down(egui::Key::ArrowDown) as i32 as f32,
                    i.stable_dt.min(0.1),
                    i.key_pressed(egui::Key::Escape),
                )
            });
            if escape {
                self.navigation_mode = false;
            }
            a.look(turn * dt, pitch * dt);
            if response.dragged_by(egui::PointerButton::Primary) {
                let delta = ui.input(|i| i.pointer.delta());
                a.look(delta.x * 0.004, -delta.y * 0.004);
            }
            let guard = self.render_state.renderer.read();
            let renderer = guard
                .callback_resources
                .get::<terrain_renderer::TerrainRenderer>();
            a.move_ground(movement, dt, |x, z| {
                renderer.and_then(|r| r.ground(x, z)).map(|hit| hit.world.y)
            });
        }
        let m = self.tile_manifest.as_ref().unwrap();
        let camera = terrain_renderer::Camera::ground_view(a.eye(), a.heading, a.pitch, m.extent());
        let painter = ui.painter_at(rect);
        painter.add(terrain_renderer::callback(
            rect,
            camera,
            self.display_mode,
            true,
            self.boundaries,
            // Ground movement needs every adjacent tile, even if the planning
            // cache was reduced to one. Still bounded to nine resident tiles.
            9,
        ));
        let mut lines = vec![
            "SIMULATED POSITION · HiRISE terrain".to_string(),
            format!(
                "E {:.1} m  N {:.1} m  · Ground {:.1} m",
                m.origin[0] + a.ground.x as f64,
                m.origin[1] - a.ground.z as f64,
                m.origin[2] + a.ground.y as f64
            ),
            format!(
                "Grid heading {:05.1}° · Eye height 1.7 m",
                a.heading.to_degrees()
            ),
        ];
        if let Some(routes) = &self.route_3d {
            let route = if self.navigation_route == 0 {
                &routes.shortest
            } else {
                &routes.lower_risk
            };
            let guide = a.guidance(route);
            lines.push(format!(
                "{} · Next waypoint {}: {:.1} m planar · Bearing {:05.1}°",
                if self.navigation_route == 0 {
                    "Shortest"
                } else {
                    "Lower-risk"
                },
                a.waypoint + 1,
                guide.distance,
                guide.bearing
            ));
            if guide.arrived {
                lines.push("Destination reached (simulation).".into());
            }
            if guide.deviation > astronaut::DEVIATION_LIMIT {
                lines.push(format!(
                    "OFF ROUTE {:.1} m — return to the route or use Recalculate from here.",
                    guide.deviation
                ));
            }
        } else {
            lines.push(
                if self.route_busy {
                    "Recalculating route from simulated position…"
                } else {
                    "No active route. Return to planning or recalculate."
                }
                .into(),
            );
        }
        if !focused {
            lines.push("Click terrain to enable movement.".into());
        }
        if let Some(error) = &a.error {
            lines.push(error.clone());
        }
        let area = Rect::from_min_size(
            rect.min + Vec2::splat(12.0),
            Vec2::new(
                (rect.width() - 24.0).min(760.0),
                lines.len() as f32 * 21.0 + 16.0,
            ),
        );
        painter.rect_filled(area, 3.0, Color32::from_black_alpha(210));
        for (i, line) in lines.iter().enumerate() {
            painter.text(
                area.min + Vec2::new(8.0, 8.0 + i as f32 * 21.0),
                egui::Align2::LEFT_TOP,
                line,
                egui::FontId::monospace(13.0),
                if line.starts_with("OFF ROUTE") {
                    Color32::YELLOW
                } else {
                    Color32::WHITE
                },
            );
        }
        ui.ctx().request_repaint();
    }

    fn draw_viewport(&mut self, ui: &mut egui::Ui) {
        if self.gpu_diagnostic {
            ui.horizontal(|ui| {
                ui.label("Display");
                ui.add_enabled_ui(self.texture_warning.is_none(), |ui| {
                    ui.selectable_value(
                        &mut self.display_mode,
                        terrain_renderer::DisplayMode::Satellite,
                        "Satellite imagery",
                    );
                });
                ui.selectable_value(
                    &mut self.display_mode,
                    terrain_renderer::DisplayMode::Elevation,
                    "Elevation shading",
                );
                ui.selectable_value(
                    &mut self.display_mode,
                    terrain_renderer::DisplayMode::Slope,
                    "Slope visualization",
                );
            });
            ui.small(match self.display_mode {
                terrain_renderer::DisplayMode::Slope => "DTM slope: green 0 deg to red 35+ deg",
                terrain_renderer::DisplayMode::Satellite => {
                    "HiRISE RED-band grayscale DN; missing pixels shown in neutral gray"
                }
                terrain_renderer::DisplayMode::Elevation => {
                    "DTM elevation colors with directional relief lighting"
                }
            });
            let aspect = ui.available_width() / (ui.available_height() - 170.0).max(100.0);
            let reset =
                !ui.ctx().wants_keyboard_input() && ui.input(|i| i.key_pressed(egui::Key::R));
            ui.horizontal(|ui| {
                if ui.button("Reset (R)").clicked() || reset {
                    self.camera.reset(aspect);
                }
                if ui.button("Fit terrain").clicked() {
                    self.camera.fit(aspect);
                }
                if ui.button("Top-down").clicked() {
                    self.camera.top_down(aspect);
                }
                ui.small("Left drag: orbit · Right drag: pan · Wheel: zoom · Double-click: reset");
            });
        }
        let available = ui.available_size();
        let profile_height = 108.0;
        let side = (available.y - profile_height - 12.0)
            .min(available.x)
            .max(240.0);
        let viewport_size = if self.gpu_diagnostic {
            Vec2::new(
                available.x.max(1.0),
                (available.y - profile_height - 48.0).max(100.0),
            )
        } else {
            Vec2::splat(side)
        };
        let (rect, response) = ui.allocate_exact_size(viewport_size, Sense::click_and_drag());
        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 0.0, Color32::from_rgb(24, 28, 25));
        if self.gpu_diagnostic {
            let aspect = rect.width() / rect.height();
            let clicked = response.clicked_by(egui::PointerButton::Primary);
            let now = ui.input(|i| i.time);
            let reset_double_click = response.double_clicked()
                && response.interact_pointer_pos().is_some_and(|p| {
                    self.last_terrain_click.is_some_and(|(previous, time)| {
                        p.distance(previous) < 6.0 && now - time < 0.5
                    })
                });
            if clicked {
                self.last_terrain_click = response.interact_pointer_pos().map(|p| (p, now));
            }
            if reset_double_click {
                self.camera.reset(aspect);
            }
            let delta = ui.input(|i| i.pointer.delta());
            if response.dragged_by(egui::PointerButton::Primary) {
                self.camera.orbit(delta);
            }
            if response.dragged_by(egui::PointerButton::Secondary) {
                self.camera.pan(delta, rect.height());
            }
            if response.hovered() {
                let scroll = ui.input(|i| i.smooth_scroll_delta.y);
                if scroll != 0.0 {
                    self.camera.zoom(scroll);
                }
            }
            if self.dataset == 0 && clicked && !reset_double_click {
                if let Some(mouse) = response.interact_pointer_pos() {
                    let hit = self
                        .render_state
                        .renderer
                        .read()
                        .callback_resources
                        .get::<terrain_renderer::TerrainRenderer>()
                        .and_then(|r| r.pick(self.camera, rect, mouse));
                    if let Some(hit) = hit {
                        self.message = format!(
                            "{} · E {:.3} / N {:.3} m · Elevation {:.3} m",
                            hit.tile, hit.easting, hit.northing, hit.elevation
                        );
                        match self.state.tool {
                            Tool::Inspect => {}
                            Tool::Start => {
                                self.invalidate_3d_route();
                                self.start_3d = Some(hit.clone());
                            }
                            Tool::Goal => {
                                self.invalidate_3d_route();
                                self.goal_3d = Some(hit.clone());
                            }
                        }
                        tracing::info!(tile=%hit.tile,row=hit.row,column=hit.column,easting=hit.easting,northing=hit.northing,elevation=hit.elevation,coarse=hit.coarse,"3D terrain picked");
                        self.selected_3d = Some(hit);
                    } else {
                        self.message = "No valid loaded terrain at this position.".into();
                    }
                    ui.ctx().request_repaint();
                }
            }
            painter.add(terrain_renderer::callback(
                rect,
                self.camera,
                self.display_mode,
                self.dataset == 0,
                self.boundaries,
                self.cache_capacity,
            ));
            if self.dataset == 0 {
                let guard = self.render_state.renderer.read();
                if let Some(renderer) = guard
                    .callback_resources
                    .get::<terrain_renderer::TerrainRenderer>()
                {
                    for (hit, color, label) in [
                        (self.start_3d.as_ref(), Color32::LIGHT_GREEN, "S"),
                        (self.goal_3d.as_ref(), Color32::LIGHT_RED, "D"),
                        (self.selected_3d.as_ref(), Color32::YELLOW, ""),
                    ] {
                        let Some(hit) = hit else {
                            continue;
                        };
                        let Some(world) = renderer.surface_marker(hit, self.camera) else {
                            continue;
                        };
                        let clip = self.camera.matrix(aspect) * world.extend(1.0);
                        if clip.w <= 0.0 {
                            continue;
                        }
                        let ndc = clip.truncate() / clip.w;
                        if !(0.0..=1.0).contains(&ndc.z) {
                            continue;
                        }
                        let screen = Pos2::new(
                            rect.left() + (ndc.x + 1.0) * 0.5 * rect.width(),
                            rect.top() + (1.0 - ndc.y) * 0.5 * rect.height(),
                        );
                        // Occlude the marker with the same active terrain triangles.
                        if renderer
                            .pick(self.camera, rect, screen)
                            .is_some_and(|front| front.world.distance(world) < 0.2)
                        {
                            painter.circle_stroke(
                                screen,
                                5.0,
                                Stroke::new(2.0_f32, Color32::BLACK),
                            );
                            painter.circle_stroke(screen, 4.0, Stroke::new(1.5_f32, color));
                            painter.text(
                                screen + Vec2::new(7.0, -7.0),
                                egui::Align2::LEFT_BOTTOM,
                                label,
                                egui::FontId::monospace(12.0),
                                color,
                            );
                        }
                    }
                }
            }
            ui.add_space(8.0);
            ui.small(format!(
                "Yaw {:.1}° · Pitch {:.1}° · Range {:.1} m · Focus E {:.1} / S {:.1} m · Scale 1:1",
                self.camera.yaw.to_degrees(),
                self.camera.pitch.to_degrees(),
                self.camera.distance,
                self.camera.focus.x,
                self.camera.focus.z
            ));
            self.draw_profile(ui);
            return;
        }

        let cell_w = rect.width() / self.terrain.size as f32;
        let cell_h = rect.height() / self.terrain.size as f32;
        for z in 0..self.terrain.size {
            for x in 0..self.terrain.size {
                let point = Point { x, z };
                let cell = Rect::from_min_size(
                    Pos2::new(
                        rect.left() + x as f32 * cell_w,
                        rect.top() + z as f32 * cell_h,
                    ),
                    Vec2::new(cell_w + 0.2, cell_h + 0.2),
                );
                painter.rect_filled(cell, 0.0, self.terrain_color(point));
            }
        }
        if self.state.show_grid && cell_w > 5.0 {
            for step in (0..=self.terrain.size).step_by(5) {
                let x = rect.left() + step as f32 * cell_w;
                let y = rect.top() + step as f32 * cell_h;
                painter.line_segment(
                    [Pos2::new(x, rect.top()), Pos2::new(x, rect.bottom())],
                    Stroke::new(0.5_f32, Color32::from_white_alpha(36)),
                );
                painter.line_segment(
                    [Pos2::new(rect.left(), y), Pos2::new(rect.right(), y)],
                    Stroke::new(0.5_f32, Color32::from_white_alpha(36)),
                );
            }
        }
        painter.rect_stroke(
            rect,
            0.0,
            Stroke::new(1.0_f32, Color32::from_rgb(106, 113, 101)),
            StrokeKind::Inside,
        );

        if self.route.len() > 1 {
            let path: Vec<Pos2> = self
                .route
                .iter()
                .map(|point| self.screen_point(rect, *point))
                .collect();
            painter.add(egui::Shape::line(
                path,
                Stroke::new(2.0_f32, Color32::from_rgb(232, 214, 151)),
            ));
        }
        let start = self.screen_point(rect, self.start());
        let goal = self.screen_point(rect, self.goal());
        painter.circle_filled(start, 6.0, Color32::from_rgb(120, 202, 163));
        painter.circle_stroke(
            goal,
            7.0,
            Stroke::new(2.0_f32, Color32::from_rgb(242, 189, 92)),
        );

        if response.hovered() {
            if let Some(pointer) = response
                .hover_pos()
                .and_then(|pos| self.point_at(rect, pos))
            {
                let marker = self.screen_point(rect, pointer);
                painter.circle_stroke(marker, 5.0, Stroke::new(1.0_f32, Color32::WHITE));
            }
        }
        if response.clicked() {
            if let Some(point) = response
                .interact_pointer_pos()
                .and_then(|pos| self.point_at(rect, pos))
            {
                match self.state.tool {
                    Tool::Inspect => self.inspected = Some(point),
                    Tool::Start => {
                        self.state.start = [point.x, point.z];
                        self.recalculate();
                    }
                    Tool::Goal => {
                        self.state.goal = [point.x, point.z];
                        self.recalculate();
                    }
                }
            }
        }

        ui.add_space(8.0);
        self.draw_profile(ui);
    }

    fn draw_profile(&self, ui: &mut egui::Ui) {
        let width = ui.available_width();
        let (rect, _) = ui.allocate_exact_size(Vec2::new(width, 92.0), Sense::hover());
        let painter = ui.painter_at(rect);
        painter.rect_filled(rect, 0.0, Color32::from_rgb(20, 23, 21));
        painter.text(
            rect.left_top() + Vec2::new(8.0, 7.0),
            egui::Align2::LEFT_TOP,
            "ROUTE ELEVATION PROFILE",
            egui::TextStyle::Small.resolve(ui.style()),
            Color32::from_gray(178),
        );
        if self.dataset == 0 && self.gpu_diagnostic {
            if let Some(comparison) = &self.route_3d {
                let routes = [&comparison.shortest, &comparison.lower_risk];
                let visible: Vec<_> = routes
                    .iter()
                    .enumerate()
                    .filter(|(i, _)| self.route_visible[*i])
                    .collect();
                let min = visible
                    .iter()
                    .flat_map(|(_, r)| r.samples.iter())
                    .map(|p| p.elevation)
                    .fold(f64::INFINITY, f64::min);
                let max = visible
                    .iter()
                    .flat_map(|(_, r)| r.samples.iter())
                    .map(|p| p.elevation)
                    .fold(f64::NEG_INFINITY, f64::max);
                let distance = routes
                    .iter()
                    .map(|r| r.length)
                    .fold(0.0_f64, f64::max)
                    .max(1e-9);
                for (index, route) in &visible {
                    let line = route
                        .samples
                        .iter()
                        .map(|p| {
                            Pos2::new(
                                rect.left()
                                    + 8.0
                                    + (p.distance / distance) as f32 * (rect.width() - 16.0),
                                rect.bottom()
                                    - 18.0
                                    - ((p.elevation - min) / (max - min).max(1.0)) as f32
                                        * (rect.height() - 44.0),
                            )
                        })
                        .collect();
                    painter.add(egui::Shape::line(
                        line,
                        Stroke::new(
                            1.5_f32,
                            if *index == 0 {
                                Color32::from_rgb(255, 166, 20)
                            } else {
                                Color32::from_rgb(5, 237, 255)
                            },
                        ),
                    ));
                }
                if !visible.is_empty() {
                    painter.text(
                        rect.left_bottom() + Vec2::new(8.0, -2.0),
                        egui::Align2::LEFT_BOTTOM,
                        format!("0–{distance:.1} m along route · elevation {min:.1} to {max:.1} m"),
                        egui::TextStyle::Small.resolve(ui.style()),
                        Color32::GRAY,
                    );
                }
            }
            return;
        }
        if self.route.len() > 1 {
            let min = self
                .route
                .iter()
                .map(|p| self.height(*p))
                .fold(f64::INFINITY, f64::min);
            let max = self
                .route
                .iter()
                .map(|p| self.height(*p))
                .fold(f64::NEG_INFINITY, f64::max);
            let range = (max - min).max(1.0);
            let line: Vec<Pos2> = self
                .route
                .iter()
                .enumerate()
                .map(|(index, point)| {
                    let x = rect.left()
                        + 8.0
                        + index as f32 / (self.route.len() - 1) as f32 * (rect.width() - 16.0);
                    let y = rect.bottom()
                        - 8.0
                        - ((self.height(*point) - min) / range) as f32 * (rect.height() - 28.0);
                    Pos2::new(x, y)
                })
                .collect();
            painter.add(egui::Shape::line(
                line,
                Stroke::new(1.5_f32, Color32::from_rgb(154, 190, 171)),
            ));
        }
    }
}

impl eframe::App for MissionApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.poll_3d_route();
        if self.dataset == 0 {
            ctx.request_repaint_after(std::time::Duration::from_millis(30));
        }
        egui::TopBottomPanel::top("menu").show(ctx, |ui| {
            egui::menu::bar(ui, |ui| {
                ui.menu_button("File", |ui| {
                    if ui.button("Reset mission state").clicked() {
                        self.navigation_mode = false;
                        self.astronaut = None;
                        self.invalidate_3d_route();
                        self.start_3d = None;
                        self.goal_3d = None;
                        self.state = SavedState::default();
                        self.recalculate();
                        ui.close_menu();
                    }
                });
                ui.menu_button("Mission", |ui| {
                    if ui.button("Calculate route").clicked() {
                        if self.navigation_mode {
                            self.recalculate_from_astronaut();
                        } else {
                            self.calculate_active_route();
                        }
                        ui.close_menu();
                    }
                    if ui.button("Inspect terrain").clicked() {
                        self.state.tool = Tool::Inspect;
                        ui.close_menu();
                    }
                });
                ui.separator();
                if ui
                    .selectable_label(!self.navigation_mode, "Mission Planning")
                    .clicked()
                {
                    self.navigation_mode = false;
                }
                if ui
                    .add_enabled(
                        self.navigation_mode
                            || (self.route_3d.is_some()
                                && self.dataset == 0
                                && self.gpu_diagnostic),
                        egui::Button::new("Astronaut Navigation").selected(self.navigation_mode),
                    )
                    .clicked()
                {
                    self.enter_navigation();
                }
                if !self.navigation_mode {
                    let previous = self.navigation_route;
                    egui::ComboBox::from_id_salt("navigation_route")
                        .selected_text(if self.navigation_route == 0 {
                            "Navigate shortest"
                        } else {
                            "Navigate lower-risk"
                        })
                        .show_ui(ui, |ui| {
                            ui.selectable_value(&mut self.navigation_route, 0, "Navigate shortest");
                            ui.selectable_value(
                                &mut self.navigation_route,
                                1,
                                "Navigate lower-risk",
                            );
                        });
                    if previous != self.navigation_route {
                        self.astronaut = None;
                    }
                }
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    let dataset = if self.load_warning.is_some() {
                        "MOLA regional fallback"
                    } else {
                        "HiRISE local terrain"
                    };
                    ui.label(egui::RichText::new(dataset).color(Color32::from_rgb(120, 202, 163)));
                });
            });
        });

        egui::TopBottomPanel::bottom("status")
            .resizable(false)
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    ui.label(&self.message);
                    ui.separator();
                    if self.dataset == 0 && self.gpu_diagnostic {
                        ui.label(
                            self.route_3d
                                .as_ref()
                                .map(|r| {
                                    format!("Additional distance +{:.1} m", r.additional_distance())
                                })
                                .unwrap_or_else(|| "No active 3D route".into()),
                        );
                    } else {
                        ui.label(format!("route {:.0} m", self.route_distance()));
                    }
                    ui.separator();
                    ui.label("local grid / east-positive");
                });
            });

        if !self.navigation_mode {
            egui::SidePanel::left("explorer")
            .default_width(212.0)
            .resizable(true)
            .show(ctx, |ui| {
                egui::ScrollArea::vertical().id_salt("explorer_scroll").show(ui,|ui| {
                ui.heading("Data explorer");
                let mut selected = self.dataset;
                egui::ComboBox::from_id_salt("terrain_dataset")
                    .selected_text(
                        ["HiRISE 3x3 tiles", "HiRISE single patch", "MOLA regional"][selected],
                    )
                    .show_ui(ui, |ui| {
                        if self.tile_manifest.is_some() {
                            ui.selectable_value(&mut selected, 0, "HiRISE 3x3 tiles");
                        }
                        ui.selectable_value(&mut selected, 1, "HiRISE single patch");
                        ui.selectable_value(&mut selected, 2, "MOLA regional");
                    });
                if selected != self.dataset {
                    self.switch_dataset(selected);
                }
                if self.dataset == 0 {
                    ui.checkbox(&mut self.boundaries, "Tile boundaries");
                    ui.add(
                        egui::Slider::new(&mut self.cache_capacity, 1..=9).text("Tile cache limit"),
                    );
                    let guard = self.render_state.renderer.read();
                    if let Some(r) = guard
                        .callback_resources
                        .get::<terrain_renderer::TerrainRenderer>()
                    {
                        ui.small(format!(
                            "Active {}/{} · GPU tile buffers/textures {:.1} MiB",
                            r.resident_count,
                            self.cache_capacity,
                            r.gpu_tile_bytes as f64 / 1048576.0
                        ));
                        ui.small(format!(
                            "Resident footprint {:.3} km² · Failed {}",
                            r.coverage_m2 / 1e6,
                            r.tile_errors
                        ));
                        ui.small(format!(
                            "LOD full {} / coarse {} · CPU assets {:.1} MiB",
                            r.resident_count - r.coarse_tile_count,
                            r.coarse_tile_count,
                            r.cpu_tile_bytes as f64 / 1048576.0
                        ));
                    }
                    if let Some(m) = &self.tile_manifest {
                        let eye = self.camera.eye();
                        ui.small(format!(
                            "Camera E {:.1} / N {:.1} m",
                            m.origin[0] + eye.x as f64,
                            m.origin[1] - eye.z as f64
                        ));
                        ui.small(format!(
                            "Camera elevation {:.1} m",
                            m.origin[2] + eye.y as f64
                        ))
                        .on_hover_text(&m.crs);
                        ui.small(&m.name);
                        ui.small(format!(
                            "{} tiles · {:.1} m extent",
                            m.tiles.len(),
                            m.extent()
                        ));
                    }
                    ui.small("Navigation: full-resolution grid across all tiles.");
                }
                ui.checkbox(&mut self.gpu_diagnostic, "3D terrain view");
                if let Some(warning) = &self.texture_warning {
                    ui.colored_label(Color32::YELLOW, warning);
                } else {
                    ui.small(
                        "HiRISE RED orthophoto · bilinear alignment to DTM grid · DN 0 masked",
                    );
                }
                if let Some(warning) = &self.load_warning {
                    ui.colored_label(Color32::YELLOW, warning);
                }
                ui.separator();
                ui.label(egui::RichText::new("Active terrain").strong());
                if self.dataset == 0 {
                    let m = self.tile_manifest.as_ref().unwrap();
                    ui.small(&m.name);
                    ui.small(format!(
                        "{} x {} unique samples · {:.4} m",
                        m.count * m.stride + 1,
                        m.count * m.stride + 1,
                        m.cell_size
                    ));
                } else {
                    ui.small(&self.terrain.source);
                    ui.small(format!(
                        "{} x {} samples · {:.4} m",
                        self.terrain.size, self.terrain.size, self.terrain.cell_size
                    ));
                }
                ui.add_space(12.0);
                ui.label(egui::RichText::new("Layers").strong());
                ui.checkbox(&mut self.state.show_slope, "Slope alerts (>18°)");
                ui.checkbox(&mut self.state.show_grid, "5-cell reference grid");
                ui.add_space(12.0);
                ui.label(egui::RichText::new("Route policy").strong());
                if self.dataset==0 && self.gpu_diagnostic {
                    let changed=ui.add(egui::Slider::new(&mut self.route_policy.max_slope_deg,5.0..=45.0).text("Slope limit °")).changed()
                        | ui.add(egui::Slider::new(&mut self.route_policy.slope_weight,0.0..=5.0).text("Slope penalty")).changed()
                        | ui.add(egui::Slider::new(&mut self.route_policy.elevation_weight,0.0..=5.0).text("Elevation penalty")).changed();
                    if changed {self.invalidate_3d_route();}
                    ui.small("Analyst assumptions, not a safety certification. NoData is blocked; no mapped obstacle dataset is supplied.");
                } else {
                if ui
                    .checkbox(&mut self.state.safest, "Limit steep slopes")
                    .changed()
                {
                    self.recalculate();
                }
                if ui
                    .checkbox(&mut self.state.avoid_blocked, "Avoid mapped hazard area")
                    .changed()
                {
                    self.recalculate();
                }
                }
                ui.add_space(12.0);
                ui.label(egui::RichText::new("Placement tool").strong());
                ui.selectable_value(&mut self.state.tool, Tool::Inspect, "Inspect location");
                ui.selectable_value(&mut self.state.tool, Tool::Start, "Set start point");
                ui.selectable_value(&mut self.state.tool, Tool::Goal, "Set destination");
                if self.dataset==0 && self.gpu_diagnostic {
                    ui.checkbox(&mut self.route_visible[0],egui::RichText::new("Shortest traversable route").color(Color32::from_rgb(255,166,20)));
                    ui.checkbox(&mut self.route_visible[1],egui::RichText::new("Lower-risk terrain route").color(Color32::from_rgb(5,237,255)));
                    if let Some(renderer)=self.render_state.renderer.write().callback_resources.get_mut::<terrain_renderer::TerrainRenderer>() {renderer.routes.visible=self.route_visible;}
                }
                ui.add_space(8.0);
                if ui
                    .add_enabled(
                        !(self.dataset == 0 && self.gpu_diagnostic) || (!self.route_busy && self.start_3d.is_some() && self.goal_3d.is_some()),
                        egui::Button::new(if self.route_busy {"Calculating…"} else {"Calculate route"}),
                    )
                    .clicked()
                {
                    self.calculate_active_route();
                }
                if self.route_busy {ui.spinner();}
                if let Some(error)=&self.route_error {ui.colored_label(Color32::LIGHT_RED,error);}
                });
            });

            egui::SidePanel::right("inspector")
            .default_width(245.0)
            .resizable(true)
            .show(ctx, |ui| {
                egui::ScrollArea::vertical().id_salt("inspector_scroll").show(ui,|ui| {
                ui.heading("Inspector");
                ui.separator();
                if self.dataset == 0 && self.gpu_diagnostic {
                    ui.strong("3D terrain selection");
                    if let Some(hit) = &self.selected_3d {
                        ui.monospace(&hit.tile);
                        ui.monospace(format!("Row {:.3} · Col {:.3}",hit.row,hit.column));
                        ui.monospace(format!("E {:.3} m",hit.easting));
                        ui.monospace(format!("N {:.3} m",hit.northing));
                        ui.monospace(format!("Elevation {:.3} m",hit.elevation));
                        if let Some([lat,lon]) = hit.geographic {
                            ui.monospace(format!("{lat:.7}°N\n{lon:.7}°E"));
                        } else { ui.small("Geographic transformation unavailable for this CRS."); }
                        ui.small(if hit.coarse { "Active coarse LOD triangle interpolation" } else { "Full-resolution triangle interpolation" });
                        ui.small("Elevation reference: source HiRISE DTM datum. Coordinate precision is not measurement accuracy.");
                    } else { ui.small("Click a loaded terrain surface."); }
                    for (label,hit) in [("Start", &self.start_3d),("Destination", &self.goal_3d)] {
                        if let Some(hit) = hit {
                            ui.small(format!("{label}: {} · r {:.2}, c {:.2}",hit.tile,hit.row,hit.column));
                        }
                    }
                    ui.separator();
                }
                if self.dataset != 0 || !self.gpu_diagnostic {
                let point = self.inspected.unwrap_or(self.goal());
                ui.label(
                    egui::RichText::new(if self.dataset == 0 {
                        "2D center patch selection"
                    } else {
                        "Selected location"
                    })
                    .strong(),
                );
                ui.monospace(format!("GRID  X {:02}  Z {:02}", point.x, point.z));
                ui.monospace(format!("ELEV  {:.1} m", self.height(point)));
                ui.monospace(format!("SLOPE {:.1}°", self.local_slope(point)));
                }
                ui.add_space(14.0);
                if self.dataset==0 && self.gpu_diagnostic {
                    ui.strong("Route comparison");
                    if let Some(r)=&self.route_3d {
                        for (index,(label,route)) in [("Shortest traversable",&r.shortest),("Lower-risk terrain",&r.lower_risk)].into_iter().enumerate() {
                            ui.colored_label(if index==0 {Color32::from_rgb(255,166,20)} else {Color32::from_rgb(5,237,255)},label);
                            ui.monospace(format!("Distance {:.1} m",route.length));
                            ui.monospace(format!("Max slope {:.1}°",route.max_slope));
                            ui.monospace(format!("Gain +{:.1} / Loss −{:.1} m",route.gain,route.loss));
                            let extra=if index==0 {0.0} else {r.additional_distance()};
                            let percent=if r.shortest.length>0.0 {extra/r.shortest.length*100.0} else {0.0};
                            ui.small(format!("Additional distance +{extra:.1} m ({percent:.2}%)"));
                            ui.small(format!("{} samples · horizontal {:.1} m",route.cells,route.horizontal));
                        }
                        if r.identical() {ui.strong("Routes are identical.");}
                        ui.small(format!("Shared slope limit {:.1}°; lower-risk penalties: slope {:.1}, elevation {:.1}.",r.lower_risk.policy.max_slope_deg,r.lower_risk.policy.slope_weight,r.lower_risk.policy.elevation_weight));
                        ui.small("Lower-risk is a cost policy, not a measured safety rating.");
                        ui.small("Endpoints snapped to nearest measured sample (≤0.72 m). Route tiles use full geometry; unloaded route sections are hidden.");
                    } else {ui.small(if self.route_busy {"Searching full-resolution terrain…"} else {"Select endpoints and calculate a route."});}
                    if let Some(e)=&self.route_error {ui.colored_label(Color32::LIGHT_RED,e);}
                } else {
                ui.label(egui::RichText::new("2D route summary").strong());
                ui.monospace(format!(
                    "START  {:02}, {:02}",
                    self.start().x,
                    self.start().z
                ));
                ui.monospace(format!("GOAL   {:02}, {:02}", self.goal().x, self.goal().z));
                ui.monospace(format!("LENGTH {:.0} m", self.route_distance()));
                ui.monospace(format!("NODES  {}", self.route.len()));
                }
                ui.add_space(14.0);
                ui.label(egui::RichText::new("Data provenance").strong());
                ui.small(&self.terrain.provenance);
                if let Some(metadata) = &self.terrain.metadata {
                    ui.monospace(&metadata.product_id);
                    ui.small(&metadata.dataset_name);
                    ui.small(&metadata.resolution);
                    if let (Some(lat), Some(lon)) =
                        (metadata.center_latitude_deg, metadata.center_longitude_deg)
                    {
                        ui.small(format!("{lat:.3}°N, {lon:.3}°E"));
                    }
                    ui.small(&metadata.coordinate_reference_system);
                    ui.small(&metadata.vertical_datum);
                    ui.hyperlink_to("PDS source product", &metadata.source_url);
                }
                });
            });
        }
        egui::CentralPanel::default().show(ctx, |ui| {
            if self.navigation_mode {
                self.draw_astronaut(ui);
                return;
            }
            ui.horizontal(|ui| {
                ui.heading("Terrain workspace");
                ui.separator();
                ui.small("Click according to the selected placement tool");
            });
            ui.separator();
            self.draw_viewport(ui);
        });
    }

    fn save(&mut self, storage: &mut dyn eframe::Storage) {
        eframe::set_value(storage, eframe::APP_KEY, &self.state);
    }
}

#[cfg(test)]
mod loader_tests {
    use super::*;

    #[test]
    fn loader_reports_file_and_json_errors() {
        let path =
            std::env::temp_dir().join(format!("mans-invalid-terrain-{}.json", std::process::id()));
        fs::write(&path, "{ invalid json").unwrap();
        let error = load_hirise(&path).unwrap_err();
        assert!(error.contains("JSON parse error"));
        assert!(error.contains(path.file_name().unwrap().to_str().unwrap()));
        fs::write(
            &path,
            r#"{"size":2,"cellSize":1,"heights":[0],"source":"test","provenance":"test"}"#,
        )
        .unwrap();
        assert!(load_hirise(&path)
            .unwrap_err()
            .contains("invalid grid dimensions"));
        fs::remove_file(&path).unwrap();
        assert!(load_hirise(&path)
            .unwrap_err()
            .contains(path.file_name().unwrap().to_str().unwrap()));
    }
}

fn main() -> eframe::Result<()> {
    tracing_subscriber::fmt().with_target(false).try_init().ok();
    tracing::info!("starting MANS native mission planning workstation");
    let options = eframe::NativeOptions {
        renderer: eframe::Renderer::Wgpu,
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1440.0, 900.0])
            .with_min_inner_size([960.0, 620.0])
            .with_title("MANS — Mission Planning Workstation"),
        ..Default::default()
    };
    eframe::run_native(
        "MANS — Mission Planning Workstation",
        options,
        Box::new(|cc| Ok(Box::new(MissionApp::new(cc)))),
    )
}
