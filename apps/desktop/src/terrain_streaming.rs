use super::*;
impl TerrainRenderer {
    pub fn ground(&self, x: f32, z: f32) -> Option<crate::terrain_picking::TerrainHit> {
        let m = &self.manager.as_ref()?.manifest;
        let top = (m.max_elevation - m.origin[2]) as f32 + 1.0;
        let ray = crate::terrain_picking::Ray {
            origin: Vec3::new(x, top, z),
            direction: -Vec3::Y,
            length: (m.max_elevation - m.min_elevation) as f32 + 2.0,
        };
        for (index, tile) in &self.tiles {
            if let Some(world) = tile._data.picking.intersect(ray, false) {
                let entry = &m.tiles[*index];
                let easting = m.origin[0] + world.x as f64;
                let northing = m.origin[1] - world.z as f64;
                return Some(crate::terrain_picking::TerrainHit {
                    tile: entry.id.clone(),
                    world,
                    row: ((entry.bounds[3] - northing) / m.cell_size).clamp(0.0, m.stride as f64),
                    column: ((easting - entry.bounds[0]) / m.cell_size).clamp(0.0, m.stride as f64),
                    easting,
                    northing,
                    elevation: m.origin[2] + world.y as f64,
                    geographic: crate::terrain_picking::geographic(&m.crs, easting, northing),
                    coarse: false,
                });
            }
        }
        None
    }
    pub fn surface_marker(
        &self,
        hit: &crate::terrain_picking::TerrainHit,
        camera: Camera,
    ) -> Option<Vec3> {
        let m = &self.manager.as_ref()?.manifest;
        let (index, entry) = m.tiles.iter().enumerate().find(|(_, t)| t.id == hit.tile)?;
        let tile = self.tiles.get(&index)?;
        let span = m.stride as f32 * m.cell_size as f32;
        let center = Vec3::new(
            (entry.x as f32 + 0.5) * span - m.extent() / 2.0,
            0.0,
            (entry.z as f32 + 0.5) * span - m.extent() / 2.0,
        );
        let top = (m.max_elevation - m.origin[2]) as f32 + 1.0;
        tile._data.picking.intersect(
            crate::terrain_picking::Ray {
                origin: Vec3::new(hit.world.x, top, hit.world.z),
                direction: -Vec3::Y,
                length: (m.max_elevation - m.min_elevation) as f32 + 2.0,
            },
            !self.routes.contains(index) && (center - camera.eye()).length() > span * 2.5,
        )
    }

    pub fn pick(
        &self,
        camera: Camera,
        rect: egui::Rect,
        mouse: egui::Pos2,
    ) -> Option<crate::terrain_picking::TerrainHit> {
        use crate::terrain_picking::{geographic, Ray, TerrainHit};
        let ray = Ray::from_viewport(camera, rect, mouse)?;
        let m = &self.manager.as_ref()?.manifest;
        let span = m.stride as f32 * m.cell_size as f32;
        let mut nearest: Option<(f32, TerrainHit)> = None;
        for (index, tile) in &self.tiles {
            let entry = &m.tiles[*index];
            let center = Vec3::new(
                (entry.x as f32 + 0.5) * span - m.extent() / 2.0,
                0.0,
                (entry.z as f32 + 0.5) * span - m.extent() / 2.0,
            );
            let coarse =
                !self.routes.contains(*index) && (center - camera.eye()).length() > span * 2.5;
            let Some(world) = tile._data.picking.intersect(ray, coarse) else {
                continue;
            };
            let distance = world.distance(ray.origin);
            if let Some((d, hit)) = &nearest {
                if distance > *d + 0.0001
                    || ((distance - *d).abs() <= 0.0001 && entry.id >= hit.tile)
                {
                    continue;
                }
            }
            let easting = m.origin[0] + world.x as f64;
            let northing = m.origin[1] - world.z as f64;
            nearest = Some((
                distance,
                TerrainHit {
                    tile: entry.id.clone(),
                    world,
                    row: ((entry.bounds[3] - northing) / m.cell_size).clamp(0.0, m.stride as f64),
                    column: ((easting - entry.bounds[0]) / m.cell_size).clamp(0.0, m.stride as f64),
                    easting,
                    northing,
                    elevation: m.origin[2] + world.y as f64,
                    geographic: geographic(&m.crs, easting, northing),
                    coarse,
                },
            ));
        }
        nearest.map(|(_, hit)| hit)
    }

    pub fn enable_tiles(&mut self, path: &std::path::Path) -> Result<(), String> {
        self.manager = Some(TileManager::new(path)?);
        Ok(())
    }
    pub(super) fn update_tiles(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        camera: Camera,
        capacity: usize,
        enabled: bool,
    ) {
        if !enabled {
            self.tiles.clear();
            self.resident_count = 0;
            self.gpu_tile_bytes = 0;
            self.coverage_m2 = 0.0;
            self.cpu_tile_bytes = 0;
            self.coarse_tile_count = 0;
            return;
        }
        let Some(manager) = self.manager.as_mut() else {
            return;
        };
        let wanted =
            manager
                .manifest
                .wanted([camera.focus.x, camera.focus.z], camera.distance, capacity);
        self.tiles.retain(|i, _| wanted.contains(i));
        if let Some(mut data) = manager.poll() {
            if wanted.contains(&data.index) {
                let m = &manager.manifest;
                let entry = &m.tiles[data.index];
                if let Some(mesh) = data.prepared.take() {
                    let coarse = std::mem::take(&mut data.coarse);
                    let buffer = |label, bytes: &[u8], usage| {
                        device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                            label: Some(label),
                            contents: bytes,
                            usage,
                        })
                    };
                    let vertices = buffer(
                        "tile vertices",
                        bytemuck::cast_slice(&mesh.vertices),
                        wgpu::BufferUsages::VERTEX,
                    );
                    let full = buffer(
                        "tile full indices",
                        bytemuck::cast_slice(&mesh.indices),
                        wgpu::BufferUsages::INDEX,
                    );
                    let coarse_buffer = buffer(
                        "tile coarse indices",
                        bytemuck::cast_slice(&coarse),
                        wgpu::BufferUsages::INDEX,
                    );
                    let (w, h, pixels) = data
                        .image
                        .as_ref()
                        .map(|o| (o.width, o.height, o.rgba.as_slice()))
                        .unwrap_or((1, 1, &[0, 0, 0, 0]));
                    let texture = device.create_texture(&wgpu::TextureDescriptor {
                        label: Some("tile ortho"),
                        size: wgpu::Extent3d {
                            width: w,
                            height: h,
                            depth_or_array_layers: 1,
                        },
                        mip_level_count: 1,
                        sample_count: 1,
                        dimension: wgpu::TextureDimension::D2,
                        format: wgpu::TextureFormat::Rgba8UnormSrgb,
                        usage: wgpu::TextureUsages::COPY_DST | wgpu::TextureUsages::TEXTURE_BINDING,
                        view_formats: &[],
                    });
                    queue.write_texture(
                        wgpu::TexelCopyTextureInfo {
                            texture: &texture,
                            mip_level: 0,
                            origin: wgpu::Origin3d::ZERO,
                            aspect: wgpu::TextureAspect::All,
                        },
                        pixels,
                        wgpu::TexelCopyBufferLayout {
                            offset: 0,
                            bytes_per_row: Some(w * 4),
                            rows_per_image: Some(h),
                        },
                        wgpu::Extent3d {
                            width: w,
                            height: h,
                            depth_or_array_layers: 1,
                        },
                    );
                    let view = texture.create_view(&Default::default());
                    let image = device.create_bind_group(&wgpu::BindGroupDescriptor {
                        label: None,
                        layout: &self.tile_image_layout,
                        entries: &[
                            wgpu::BindGroupEntry {
                                binding: 0,
                                resource: wgpu::BindingResource::TextureView(&view),
                            },
                            wgpu::BindGroupEntry {
                                binding: 1,
                                resource: wgpu::BindingResource::Sampler(&self.sampler),
                            },
                        ],
                    });
                    let bytes = mesh.vertices.len() * std::mem::size_of::<Vertex>()
                        + (mesh.indices.len() + coarse.len()) * 4
                        + pixels.len();
                    if let Some(warning) = &data.image_warning {
                        tracing::warn!(tile=%entry.id,%warning,"tile imagery missing");
                    }
                    tracing::info!(tile=%entry.id,bytes,"tile GPU resources uploaded");
                    self.tiles.insert(
                        data.index,
                        GpuTile {
                            vertices,
                            full,
                            coarse: coarse_buffer,
                            full_count: mesh.indices.len() as u32,
                            coarse_count: coarse.len() as u32,
                            image,
                            gpu_bytes: bytes,
                            _data: data,
                        },
                    );
                }
            }
        }
        manager.request(&wanted, &self.tiles.keys().copied().collect::<HashSet<_>>());
        self.resident_count = self.tiles.len();
        self.gpu_tile_bytes = self.tiles.values().map(|t| t.gpu_bytes).sum();
        self.coverage_m2 = self.tiles.len() as f64
            * (manager.manifest.stride as f64 * manager.manifest.cell_size).powi(2);
        self.tile_errors = manager.errors.len();
        self.cpu_tile_bytes = self
            .tiles
            .values()
            .map(|t| {
                t._data.heights.len() * 8
                    + t._data.picking.bytes()
                    + t._data.normals.len() * 12
                    + t._data.valid.len()
                    + t._data.image.as_ref().map(|i| i.rgba.len()).unwrap_or(0)
            })
            .sum();
        let m = &manager.manifest;
        let span = m.stride as f32 * m.cell_size as f32;
        self.coarse_tile_count = self
            .tiles
            .keys()
            .filter(|i| {
                let t = &m.tiles[**i];
                let center = Vec3::new(
                    (t.x as f32 + 0.5) * span - m.extent() / 2.0,
                    0.0,
                    (t.z as f32 + 0.5) * span - m.extent() / 2.0,
                );
                !self.routes.contains(**i) && (center - camera.eye()).length() > span * 2.5
            })
            .count();
    }
}
