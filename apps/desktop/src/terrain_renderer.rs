use crate::terrain_tiles::{LoadedTile, TileManager};
use bytemuck::{Pod, Zeroable};
use eframe::egui::{self, PaintCallbackInfo};
use egui_wgpu::{CallbackResources, CallbackTrait, ScreenDescriptor};
use glam::{Mat4, Vec3};
use std::collections::{HashMap, HashSet};
use wgpu::util::DeviceExt;
#[path = "terrain_streaming.rs"]
mod streaming;

pub(crate) fn coarse_indices(size: usize, step: usize, valid: &[u8]) -> Vec<u32> {
    let mut result = Vec::new();
    for z in (0..size - 1).step_by(step) {
        for x in (0..size - 1).step_by(step) {
            let endx = (x + step).min(size - 1);
            let endz = (z + step).min(size - 1);
            if (z..=endz).any(|r| (x..=endx).any(|c| valid[r * size + c] == 0)) {
                continue;
            }
            let mut ring = vec![];
            for c in x..endx {
                if z == 0 || c == x {
                    ring.push(z * size + c);
                }
            }
            for r in z..endz {
                if endx == size - 1 || r == z {
                    ring.push(r * size + endx);
                }
            }
            for c in (x + 1..=endx).rev() {
                if endz == size - 1 || c == endx {
                    ring.push(endz * size + c);
                }
            }
            for r in (z + 1..=endz).rev() {
                if x == 0 || r == endz {
                    ring.push(r * size + x);
                }
            }
            let center = ((z + endz) / 2) * size + (x + endx) / 2;
            for i in 0..ring.len() {
                let a = ring[i];
                let b = ring[(i + 1) % ring.len()];
                if center != a && center != b {
                    result.extend_from_slice(&[center as u32, b as u32, a as u32]);
                }
            }
        }
    }
    result
}

struct GpuTile {
    vertices: wgpu::Buffer,
    full: wgpu::Buffer,
    coarse: wgpu::Buffer,
    full_count: u32,
    coarse_count: u32,
    image: wgpu::BindGroup,
    gpu_bytes: usize,
    _data: LoadedTile,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u32)]
pub enum DisplayMode {
    Satellite = 0,
    Elevation = 1,
    Slope = 2,
}

#[derive(serde::Deserialize)]
#[serde(rename_all = "camelCase")]
struct ImageProvenance {
    transform: Vec<f64>,
    ortho_transform: Vec<f64>,
    texture_width: u32,
    texture_height: u32,
}

pub struct Orthophoto {
    width: u32,
    height: u32,
    rgba: Vec<u8>,
}

pub fn load_orthophoto(
    directory: &std::path::Path,
    grid_size: usize,
) -> Result<Orthophoto, String> {
    let provenance = directory.join("provenance.json");
    let json = std::fs::read_to_string(&provenance)
        .map_err(|e| format!("{}: {e}", provenance.display()))?;
    let p: ImageProvenance =
        serde_json::from_str(&json).map_err(|e| format!("Unverified texture alignment: {e}"))?;
    if p.transform.len() != 9
        || p.ortho_transform.len() != 9
        || p.transform
            .iter()
            .zip(&p.ortho_transform)
            .any(|(a, b)| !a.is_finite() || !b.is_finite() || (a - b).abs() > 1e-7)
    {
        return Err(
            "Orthophoto transform does not match the DTM grid; rerun georeferenced extraction"
                .into(),
        );
    }
    let path = directory.join("ortho.tif");
    let image = image::open(&path)
        .map_err(|e| format!("{}: {e}", path.display()))?
        .to_luma8();
    if image.width() != grid_size as u32
        || image.height() != grid_size as u32
        || image.dimensions() != (p.texture_width, p.texture_height)
    {
        return Err("Orthophoto dimensions differ from verified DTM grid".into());
    }
    let mut rgba = Vec::with_capacity(grid_size * grid_size * 4);
    if image.as_raw().iter().all(|v| *v == 0) {
        return Err("Orthophoto contains no valid pixels (all DN 0)".into());
    }
    for v in image.as_raw() {
        rgba.extend_from_slice(&[*v, *v, *v, if *v == 0 { 0 } else { 255 }]);
    }
    tracing::info!(
        width = image.width(),
        height = image.height(),
        "decoded georeferenced HiRISE orthophoto once"
    );
    Ok(Orthophoto {
        width: image.width(),
        height: image.height(),
        rgba,
    })
}

#[derive(Clone, Copy, Debug)]
pub struct Camera {
    first_person: Option<(Vec3, Vec3)>,
    pub yaw: f32,
    pub pitch: f32,
    pub distance: f32,
    pub focus: Vec3,
    extent: f32,
    relief: f32,
}

impl Camera {
    pub fn new(extent: f32, relief: f32) -> Self {
        let mut c = Self {
            first_person: None,
            yaw: 0.69,
            pitch: 0.53,
            distance: 1.0,
            focus: Vec3::ZERO,
            extent: extent.max(1.0),
            relief,
        };
        c.fit(1.0);
        c
    }
    pub fn fit(&mut self, aspect: f32) {
        self.focus = Vec3::ZERO;
        let radius = (self.extent * self.extent * 0.5 + self.relief * self.relief * 0.25).sqrt();
        let half_fov = (22.5_f32.to_radians().tan() * aspect.clamp(0.1, 1.0)).atan();
        self.distance = radius / half_fov.sin() * 1.08;
        self.constrain();
    }
    pub fn reset(&mut self, aspect: f32) {
        self.yaw = 0.69;
        self.pitch = 0.53;
        self.fit(aspect);
    }
    pub fn top_down(&mut self, aspect: f32) {
        self.yaw = 0.0;
        self.pitch = std::f32::consts::FRAC_PI_2 - 0.001;
        self.fit(aspect);
    }
    fn constrain(&mut self) {
        self.pitch = self.pitch.clamp(0.15, std::f32::consts::FRAC_PI_2 - 0.001);
        // Keep the eye above the highest sample, even when panned or zoomed.
        let minimum =
            ((self.relief * 0.5 + self.extent * 0.025) / self.pitch.sin()).max(self.extent * 0.08);
        self.distance = self
            .distance
            .clamp(minimum, self.extent * 30.0 + self.relief * 10.0);
    }
    pub fn orbit(&mut self, delta: egui::Vec2) {
        self.yaw = (self.yaw - delta.x * 0.006).rem_euclid(std::f32::consts::TAU);
        self.pitch += delta.y * 0.006;
        self.constrain();
    }
    pub fn pan(&mut self, delta: egui::Vec2, viewport_height: f32) {
        let scale = 2.0 * self.distance * 22.5_f32.to_radians().tan() / viewport_height.max(1.0);
        let right = Vec3::new(self.yaw.cos(), 0.0, -self.yaw.sin());
        let forward = Vec3::new(self.yaw.sin(), 0.0, self.yaw.cos());
        self.focus += (-right * delta.x - forward * delta.y) * scale;
        self.focus.x = self.focus.x.clamp(-self.extent * 2.0, self.extent * 2.0);
        self.focus.z = self.focus.z.clamp(-self.extent * 2.0, self.extent * 2.0);
    }
    pub fn zoom(&mut self, scroll: f32) {
        self.distance *= (-scroll * 0.002).clamp(-1.0, 1.0).exp();
        self.constrain();
    }
    pub fn eye(&self) -> Vec3 {
        if let Some((eye, _)) = self.first_person {
            return eye;
        }
        self.focus
            + Vec3::new(
                self.yaw.sin() * self.pitch.cos(),
                self.pitch.sin(),
                self.yaw.cos() * self.pitch.cos(),
            ) * self.distance
    }
    pub(crate) fn matrix(&self, aspect: f32) -> Mat4 {
        if let Some((eye, direction)) = self.first_person {
            return Mat4::perspective_rh(
                70.0_f32.to_radians(),
                aspect.max(0.01),
                0.05,
                self.extent * 8.0,
            ) * Mat4::look_at_rh(eye, eye + direction, Vec3::Y);
        }
        Mat4::perspective_rh(
            45.0_f32.to_radians(),
            aspect.max(0.01),
            self.extent * 0.002,
            self.distance + self.extent * 8.0 + self.relief,
        ) * Mat4::look_at_rh(self.eye(), self.focus, Vec3::Y)
    }
    pub fn ground_view(eye: Vec3, heading: f32, pitch: f32, extent: f32) -> Self {
        let mut c = Self::new(extent, 1.0);
        c.focus = eye;
        c.distance = 0.0;
        c.first_person = Some((
            eye,
            Vec3::new(
                heading.sin() * pitch.cos(),
                pitch.sin(),
                -heading.cos() * pitch.cos(),
            ),
        ));
        c
    }
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
pub(crate) struct Vertex {
    pub position: [f32; 3],
    pub normal: [f32; 3],
    pub height: f32,
    uv: [f32; 2],
}
pub(crate) struct Mesh {
    pub vertices: Vec<Vertex>,
    pub indices: Vec<u32>,
    extent: f32,
    relief: f32,
}

pub(crate) fn mesh(size: usize, cell: f64, heights: &[f64]) -> Result<Mesh, String> {
    if size < 2 || size.checked_mul(size) != Some(heights.len()) || !cell.is_finite() || cell <= 0.0
    {
        return Err("Invalid terrain grid or metric spacing".into());
    }
    let valid: Vec<bool> = heights
        .iter()
        .map(|h| h.is_finite() && *h > -1.0e10)
        .collect();
    let min = heights
        .iter()
        .zip(&valid)
        .filter(|(_, v)| **v)
        .map(|(h, _)| *h)
        .fold(f64::INFINITY, f64::min);
    let max = heights
        .iter()
        .zip(&valid)
        .filter(|(_, v)| **v)
        .map(|(h, _)| *h)
        .fold(f64::NEG_INFINITY, f64::max);
    if !min.is_finite() {
        return Err("Terrain has no valid elevations".into());
    }
    let extent = ((size - 1) as f64 * cell) as f32;
    let origin = (min + max) * 0.5;
    let mut vertices: Vec<Vertex> = heights
        .iter()
        .enumerate()
        .map(|(i, h)| Vertex {
            position: [
                (i % size) as f32 * cell as f32 - extent * 0.5,
                if valid[i] { (*h - origin) as f32 } else { 0.0 },
                (i / size) as f32 * cell as f32 - extent * 0.5,
            ],
            normal: [0.0; 3],
            // Geometry vertices are DTM sample centers, not raster edges.
            uv: [
                ((i % size) as f32 + 0.5) / size as f32,
                ((i / size) as f32 + 0.5) / size as f32,
            ],
            height: if valid[i] {
                ((*h - min) / (max - min).max(1.0)) as f32
            } else {
                0.0
            },
        })
        .collect();
    let mut indices = Vec::with_capacity((size - 1) * (size - 1) * 6);
    for z in 0..size - 1 {
        for x in 0..size - 1 {
            let a = z * size + x;
            // Raster south is +Z; triangles face +Y.
            for tri in [[a, a + size, a + 1], [a + 1, a + size, a + size + 1]] {
                if tri.iter().all(|i| valid[*i]) {
                    let p = tri.map(|i| Vec3::from_array(vertices[i].position));
                    let n = (p[1] - p[0]).cross(p[2] - p[0]);
                    for i in tri {
                        vertices[i].normal = (Vec3::from_array(vertices[i].normal) + n).to_array();
                        indices.push(i as u32);
                    }
                }
            }
        }
    }
    if indices.is_empty() {
        return Err("Terrain has no valid triangles".into());
    }
    for v in &mut vertices {
        v.normal = Vec3::from_array(v.normal).normalize_or_zero().to_array();
    }
    Ok(Mesh {
        vertices,
        indices,
        extent,
        relief: (max - min) as f32,
    })
}

struct Targets {
    size: [u32; 2],
    color: wgpu::TextureView,
    depth: wgpu::TextureView,
    bind: wgpu::BindGroup,
}
pub struct TerrainRenderer {
    pub routes: crate::route_renderer::RouteRenderer,
    pipeline: wgpu::RenderPipeline,
    present: wgpu::RenderPipeline,
    vertices: wgpu::Buffer,
    indices: wgpu::Buffer,
    count: u32,
    uniform: wgpu::Buffer,
    camera_bind: wgpu::BindGroup,
    terrain_image_bind: wgpu::BindGroup,
    image_layout: wgpu::BindGroupLayout,
    sampler: wgpu::Sampler,
    targets: Option<Targets>,
    format: wgpu::TextureFormat,
    tile_image_layout: wgpu::BindGroupLayout,
    manager: Option<TileManager>,
    tiles: HashMap<usize, GpuTile>,
    pub resident_count: usize,
    pub gpu_tile_bytes: usize,
    pub coverage_m2: f64,
    pub tile_errors: usize,
    pub cpu_tile_bytes: usize,
    pub coarse_tile_count: usize,
}
impl TerrainRenderer {
    pub fn new(
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        format: wgpu::TextureFormat,
        size: usize,
        cell: f64,
        heights: &[f64],
        orthophoto: Option<&Orthophoto>,
    ) -> Result<Self, String> {
        let mesh = mesh(size, cell, heights)?;
        let vertices = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("terrain metric vertices"),
            contents: bytemuck::cast_slice(&mesh.vertices),
            usage: wgpu::BufferUsages::VERTEX,
        });
        let indices = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("terrain indices"),
            contents: bytemuck::cast_slice(&mesh.indices),
            usage: wgpu::BufferUsages::INDEX,
        });
        let uniform = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("terrain camera"),
            size: 80,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let camera_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: None,
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX_FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            }],
        });
        let camera_bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: None,
            layout: &camera_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: uniform.as_entire_binding(),
            }],
        });
        let terrain_image_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("orthophoto bindings"),
                entries: &[
                    wgpu::BindGroupLayoutEntry {
                        binding: 0,
                        visibility: wgpu::ShaderStages::FRAGMENT,
                        ty: wgpu::BindingType::Texture {
                            sample_type: wgpu::TextureSampleType::Float { filterable: true },
                            view_dimension: wgpu::TextureViewDimension::D2,
                            multisampled: false,
                        },
                        count: None,
                    },
                    wgpu::BindGroupLayoutEntry {
                        binding: 1,
                        visibility: wgpu::ShaderStages::FRAGMENT,
                        ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                        count: None,
                    },
                ],
            });
        let (width, height, pixels) = orthophoto
            .map(|o| (o.width, o.height, o.rgba.as_slice()))
            .unwrap_or((1, 1, &[0, 0, 0, 0]));
        let image_texture = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("observed HiRISE orthophoto (sRGB display DN)"),
            size: wgpu::Extent3d {
                width,
                height,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Rgba8UnormSrgb,
            usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
            view_formats: &[],
        });
        queue.write_texture(
            wgpu::TexelCopyTextureInfo {
                texture: &image_texture,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            pixels,
            wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(width * 4),
                rows_per_image: Some(height),
            },
            wgpu::Extent3d {
                width,
                height,
                depth_or_array_layers: 1,
            },
        );
        let image_view = image_texture.create_view(&Default::default());
        let image_sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            label: Some("orthophoto linear sampling"),
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            address_mode_u: wgpu::AddressMode::ClampToEdge,
            address_mode_v: wgpu::AddressMode::ClampToEdge,
            ..Default::default()
        });
        let terrain_image_bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: None,
            layout: &terrain_image_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: wgpu::BindingResource::TextureView(&image_view),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: wgpu::BindingResource::Sampler(&image_sampler),
                },
            ],
        });
        tracing::info!(
            width,
            height,
            available = orthophoto.is_some(),
            "terrain imagery texture uploaded once"
        );
        let shader=device.create_shader_module(wgpu::ShaderModuleDescriptor {label:Some("terrain shading"),source:wgpu::ShaderSource::Wgsl(r#"
struct Camera { vp:mat4x4<f32>, mode:vec4<u32> };
@group(0) @binding(0) var<uniform> camera:Camera;
@group(1) @binding(0) var ortho:texture_2d<f32>;
@group(1) @binding(1) var ortho_sampler:sampler;
struct Out { @builtin(position) pos:vec4<f32>, @location(0) normal:vec3<f32>, @location(1) height:f32, @location(2) uv:vec2<f32> };
@vertex fn vs(@location(0) pos:vec3<f32>, @location(1) normal:vec3<f32>, @location(2) height:f32, @location(3) uv:vec2<f32>)->Out {
 var o:Out; o.pos=camera.vp*vec4(pos,1.0); o.normal=normal; o.height=height; o.uv=uv; return o;
}
fn linear_to_srgb(c:vec3<f32>)->vec3<f32> {
 return select(c*12.92,1.055*pow(max(c,vec3(0.0)),vec3(1.0/2.4))-0.055,c>vec3(0.0031308));
}
@fragment fn fs(in:Out)->@location(0) vec4<f32> {
 let n=normalize(in.normal);
 let diffuse=max(dot(n,normalize(vec3(-0.5,1.0,-0.3))),0.0);
 let sample_color=textureSample(ortho,ortho_sampler,in.uv);
 var color=mix(vec3(0.22,0.26,0.24),vec3(0.79,0.65,0.44),clamp(in.height,0.0,1.0))*(0.28+0.72*diffuse);
 if camera.mode.x==0u {
   // Existing orbital shading remains dominant; modest extra relief lighting.
   if sample_color.a<0.99 { return vec4(0.12,0.12,0.12,1.0); }
   color=sample_color.rgb*(0.85+0.15*diffuse);
   // eframe's Bgra8Unorm target does not perform sRGB encoding.
   if camera.mode.y==1u {color=linear_to_srgb(color);}
 } else if camera.mode.x==2u {
   let angle=acos(clamp(n.y,0.0,1.0))*57.2957795;
   color=mix(vec3(0.15,0.50,0.40),vec3(0.95,0.23,0.07),clamp(angle/35.0,0.0,1.0))*(0.7+0.3*diffuse);
 }
 if camera.mode.z==1u && (in.uv.x<0.004 || in.uv.y<0.004 || in.uv.x>0.996 || in.uv.y>0.996) {color=vec3(0.95,0.65,0.1);}
 return vec4(color,1.0);
}"#.into())});
        let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: None,
            bind_group_layouts: &[&camera_layout, &terrain_image_layout],
            push_constant_ranges: &[],
        });
        let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("depth tested terrain"),
            layout: Some(&layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs"),
                compilation_options: Default::default(),
                buffers: &[wgpu::VertexBufferLayout {
                    array_stride: std::mem::size_of::<Vertex>() as u64,
                    step_mode: wgpu::VertexStepMode::Vertex,
                    attributes: &wgpu::vertex_attr_array![0=>Float32x3,1=>Float32x3,2=>Float32,3=>Float32x2],
                }],
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fs"),
                compilation_options: Default::default(),
                targets: &[Some(wgpu::ColorTargetState {
                    format,
                    blend: None,
                    write_mask: wgpu::ColorWrites::ALL,
                })],
            }),
            primitive: wgpu::PrimitiveState {
                cull_mode: Some(wgpu::Face::Back),
                ..Default::default()
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: wgpu::TextureFormat::Depth32Float,
                depth_write_enabled: true,
                depth_compare: wgpu::CompareFunction::Less,
                stencil: Default::default(),
                bias: Default::default(),
            }),
            multisample: Default::default(),
            multiview: None,
            cache: None,
        });
        let image_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: None,
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Texture {
                        sample_type: wgpu::TextureSampleType::Float { filterable: true },
                        view_dimension: wgpu::TextureViewDimension::D2,
                        multisampled: false,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 1,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                    count: None,
                },
            ],
        });
        let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            ..Default::default()
        });
        // Render into an offscreen depth attachment, then composite in egui's
        // existing pass, which does not supply a compatible depth attachment.
        let blit = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("terrain composite"),
            source: wgpu::ShaderSource::Wgsl(
                r#"
@group(0) @binding(0) var image:texture_2d<f32>;
@group(0) @binding(1) var image_sampler:sampler;
struct Out { @builtin(position) pos:vec4<f32>, @location(0) uv:vec2<f32> };
@vertex fn vs(@builtin(vertex_index) id:u32)->Out {
 let uv=vec2(f32((id<<1u)&2u),f32(id&2u));
 var o:Out; o.pos=vec4(uv*2.0-1.0,0.0,1.0); o.uv=vec2(uv.x,1.0-uv.y); return o;
}
@fragment fn fs(in:Out)->@location(0) vec4<f32> {return textureSample(image,image_sampler,in.uv);}
"#
                .into(),
            ),
        });
        let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: None,
            bind_group_layouts: &[&image_layout],
            push_constant_ranges: &[],
        });
        let present = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("terrain viewport composite"),
            layout: Some(&layout),
            vertex: wgpu::VertexState {
                module: &blit,
                entry_point: Some("vs"),
                compilation_options: Default::default(),
                buffers: &[],
            },
            fragment: Some(wgpu::FragmentState {
                module: &blit,
                entry_point: Some("fs"),
                compilation_options: Default::default(),
                targets: &[Some(wgpu::ColorTargetState {
                    format,
                    blend: None,
                    write_mask: wgpu::ColorWrites::ALL,
                })],
            }),
            primitive: Default::default(),
            depth_stencil: None,
            multisample: Default::default(),
            multiview: None,
            cache: None,
        });
        tracing::info!(
            vertices = mesh.vertices.len(),
            triangles = mesh.indices.len() / 3,
            extent_m = mesh.extent,
            relief_m = mesh.relief,
            "terrain buffers uploaded once"
        );
        Ok(Self {
            routes: crate::route_renderer::RouteRenderer::new(device, format, &camera_layout),
            pipeline,
            present,
            vertices,
            indices,
            count: mesh.indices.len() as u32,
            uniform,
            camera_bind,
            terrain_image_bind,
            image_layout,
            sampler,
            targets: None,
            format,
            tile_image_layout: terrain_image_layout,
            manager: None,
            tiles: HashMap::new(),
            resident_count: 0,
            gpu_tile_bytes: 0,
            coverage_m2: 0.0,
            tile_errors: 0,
            cpu_tile_bytes: 0,
            coarse_tile_count: 0,
        })
    }
    fn resize(&mut self, device: &wgpu::Device, size: [u32; 2]) {
        if self.targets.as_ref().is_some_and(|t| t.size == size) {
            return;
        }
        let extent = wgpu::Extent3d {
            width: size[0],
            height: size[1],
            depth_or_array_layers: 1,
        };
        let color = device
            .create_texture(&wgpu::TextureDescriptor {
                label: Some("terrain color"),
                size: extent,
                mip_level_count: 1,
                sample_count: 1,
                dimension: wgpu::TextureDimension::D2,
                format: self.format,
                usage: wgpu::TextureUsages::RENDER_ATTACHMENT
                    | wgpu::TextureUsages::TEXTURE_BINDING,
                view_formats: &[],
            })
            .create_view(&Default::default());
        let depth = device
            .create_texture(&wgpu::TextureDescriptor {
                label: Some("terrain depth"),
                size: extent,
                mip_level_count: 1,
                sample_count: 1,
                dimension: wgpu::TextureDimension::D2,
                format: wgpu::TextureFormat::Depth32Float,
                usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
                view_formats: &[],
            })
            .create_view(&Default::default());
        let bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: None,
            layout: &self.image_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: wgpu::BindingResource::TextureView(&color),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: wgpu::BindingResource::Sampler(&self.sampler),
                },
            ],
        });
        self.targets = Some(Targets {
            size,
            color,
            depth,
            bind,
        });
        tracing::info!(?size, "terrain color/depth targets resized");
    }
}
struct TerrainCallback {
    rect: egui::Rect,
    camera: Camera,
    mode: DisplayMode,
    tiled: bool,
    boundaries: bool,
    capacity: usize,
}
impl CallbackTrait for TerrainCallback {
    fn prepare(
        &self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        screen: &ScreenDescriptor,
        encoder: &mut wgpu::CommandEncoder,
        resources: &mut CallbackResources,
    ) -> Vec<wgpu::CommandBuffer> {
        let r = resources
            .get_mut::<TerrainRenderer>()
            .expect("terrain GPU resources");
        r.update_tiles(device, queue, self.camera, self.capacity, self.tiled);
        let size = [
            (self.rect.width() * screen.pixels_per_point)
                .round()
                .max(1.0) as u32,
            (self.rect.height() * screen.pixels_per_point)
                .round()
                .max(1.0) as u32,
        ];
        r.resize(device, size);
        r.routes.resize(queue, size, screen.pixels_per_point);
        queue.write_buffer(
            &r.uniform,
            0,
            bytemuck::cast_slice(
                &self
                    .camera
                    .matrix(self.rect.width() / self.rect.height())
                    .to_cols_array(),
            ),
        );
        queue.write_buffer(
            &r.uniform,
            64,
            bytemuck::cast_slice(&[
                self.mode as u32,
                u32::from(!r.format.is_srgb()),
                u32::from(self.boundaries),
                0,
            ]),
        );
        let targets = r.targets.as_ref().unwrap();
        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("HiRISE terrain depth render"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view: &targets.color,
                resolve_target: None,
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Clear(wgpu::Color {
                        r: 0.035,
                        g: 0.045,
                        b: 0.043,
                        a: 1.0,
                    }),
                    store: wgpu::StoreOp::Store,
                },
            })],
            depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                view: &targets.depth,
                depth_ops: Some(wgpu::Operations {
                    load: wgpu::LoadOp::Clear(1.0),
                    store: wgpu::StoreOp::Discard,
                }),
                stencil_ops: None,
            }),
            timestamp_writes: None,
            occlusion_query_set: None,
        });
        pass.set_pipeline(&r.pipeline);
        pass.set_bind_group(0, &r.camera_bind, &[]);
        if self.tiled && r.manager.is_some() {
            let m = &r.manager.as_ref().unwrap().manifest;
            let span = m.stride as f32 * m.cell_size as f32;
            for (i, tile) in &r.tiles {
                let entry = &m.tiles[*i];
                let center = Vec3::new(
                    (entry.x as f32 + 0.5) * span - m.extent() / 2.0,
                    0.0,
                    (entry.z as f32 + 0.5) * span - m.extent() / 2.0,
                );
                let coarse =
                    !r.routes.contains(*i) && (center - self.camera.eye()).length() > span * 2.5;
                pass.set_bind_group(1, &tile.image, &[]);
                pass.set_vertex_buffer(0, tile.vertices.slice(..));
                pass.set_index_buffer(
                    if coarse {
                        tile.coarse.slice(..)
                    } else {
                        tile.full.slice(..)
                    },
                    wgpu::IndexFormat::Uint32,
                );
                pass.draw_indexed(
                    0..if coarse {
                        tile.coarse_count
                    } else {
                        tile.full_count
                    },
                    0,
                    0..1,
                );
            }
        } else {
            pass.set_bind_group(1, &r.terrain_image_bind, &[]);
            pass.set_vertex_buffer(0, r.vertices.slice(..));
            pass.set_index_buffer(r.indices.slice(..), wgpu::IndexFormat::Uint32);
            pass.draw_indexed(0..r.count, 0, 0..1);
        }
        if self.tiled {
            r.routes.draw(
                &mut pass,
                &r.camera_bind,
                &r.tiles.keys().copied().collect(),
            );
        }
        Vec::new()
    }
    fn paint(
        &self,
        _: PaintCallbackInfo,
        pass: &mut wgpu::RenderPass<'static>,
        resources: &CallbackResources,
    ) {
        let r = resources
            .get::<TerrainRenderer>()
            .expect("terrain GPU resources");
        if let Some(t) = &r.targets {
            pass.set_pipeline(&r.present);
            pass.set_bind_group(0, &t.bind, &[]);
            pass.draw(0..3, 0..1);
        }
    }
}
pub fn callback(
    rect: egui::Rect,
    camera: Camera,
    mode: DisplayMode,
    tiled: bool,
    boundaries: bool,
    capacity: usize,
) -> egui::PaintCallback {
    egui_wgpu::Callback::new_paint_callback(
        rect,
        TerrainCallback {
            rect,
            camera,
            mode,
            tiled,
            boundaries,
            capacity,
        },
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn lod_preserves_full_resolution_boundary_and_surface_area() {
        let size = 12;
        let indices = coarse_indices(size, 4, &vec![1; size * size]);
        let mut edges: HashMap<(u32, u32), usize> = HashMap::new();
        let mut area = 0.0;
        for t in indices.chunks_exact(3) {
            let p: Vec<Vec3> = t
                .iter()
                .map(|i| {
                    Vec3::new(
                        (*i as usize % size) as f32,
                        0.0,
                        (*i as usize / size) as f32,
                    )
                })
                .collect();
            let n = (p[1] - p[0]).cross(p[2] - p[0]);
            assert!(n.y > 0.0);
            area += n.y * 0.5;
            for (a, b) in [(t[0], t[1]), (t[1], t[2]), (t[2], t[0])] {
                *edges.entry((a.min(b), a.max(b))).or_default() += 1;
            }
        }
        assert_eq!(area, ((size - 1) * (size - 1)) as f32);
        let boundary: Vec<_> = edges.iter().filter(|(_, n)| **n == 1).collect();
        assert_eq!(boundary.len(), 4 * (size - 1));
        for ((a, b), _) in boundary {
            assert!((*b - *a) == 1 || (*b - *a) == size as u32);
        }
    }
    #[test]
    fn camera_controls_preserve_clearance_and_reset() {
        let mut c = Camera::new(516.0, 81.3);
        let original = c.eye();
        c.orbit(egui::vec2(120.0, -90.0));
        assert_ne!(original, c.eye());
        for _ in 0..50 {
            c.zoom(1000.0);
        }
        assert!(c.eye().y >= 81.3 * 0.5 + 516.0 * 0.025 - 0.001);
        let distance = c.distance;
        c.zoom(-100.0);
        assert!(c.distance > distance);
        c.pan(egui::vec2(50.0, 25.0), 600.0);
        assert_ne!(c.focus, Vec3::ZERO);
        assert_eq!(c.focus.y, 0.0);
        c.reset(1.0);
        assert!(c.eye().abs_diff_eq(original, 0.001));
        c.top_down(1.0);
        assert!(c.matrix(1.0).is_finite());
        assert!(c.pitch > 1.56);
    }
    #[test]
    fn fit_contains_terrain_for_wide_and_narrow_viewports() {
        for aspect in [0.4, 1.0, 2.0] {
            let mut c = Camera::new(516.0, 81.3);
            c.fit(aspect);
            for x in [-258.0, 258.0] {
                for z in [-258.0, 258.0] {
                    for y in [-40.65, 40.65] {
                        let p = c.matrix(aspect).project_point3(Vec3::new(x, y, z));
                        assert!(p.x.abs() <= 1.0 && p.y.abs() <= 1.0 && p.z > 0.0 && p.z < 1.0);
                    }
                }
            }
        }
    }
    #[test]
    fn metric_mesh_preserves_relief_and_winding() {
        let m = mesh(2, 2.0, &[100.0, 101.0, 102.0, 103.0]).unwrap();
        assert_eq!(m.indices.len(), 6);
        assert_eq!(m.vertices[0].position, [-1.0, -1.5, -1.0]);
        assert_eq!(m.vertices[3].position, [1.0, 1.5, 1.0]);
        assert!(m.vertices.iter().all(|v| v.normal[1] > 0.0));
        assert_eq!(m.vertices[0].uv, [0.25, 0.25]);
        assert_eq!(m.vertices[3].uv, [0.75, 0.75]);
    }

    #[test]
    fn rejects_orthophoto_with_mismatched_affine() {
        let dir = std::env::temp_dir().join(format!("mans-bad-affine-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join("provenance.json"),r#"{"transform":[1,0,0,0,-1,0,0,0,1],"orthoTransform":[1,0,1.5,0,-1,0,0,0,1],"textureWidth":2,"textureHeight":2}"#).unwrap();
        let error = load_orthophoto(&dir, 2)
            .err()
            .expect("must reject mismatched georeferencing before decoding");
        assert!(error.contains("transform does not match"));
        std::fs::remove_dir_all(dir).unwrap();
    }
    #[test]
    fn missing_sample_is_not_bridged() {
        assert_eq!(
            mesh(2, 1.0, &[f64::NAN, 10.0, 11.0, 12.0]).unwrap().indices,
            vec![1, 2, 3]
        );
        assert!(mesh(2, 1.0, &[f64::NAN; 4]).is_err());
    }
}
