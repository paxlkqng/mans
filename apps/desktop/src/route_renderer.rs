use crate::terrain_navigation::NavigationRoute;
use bytemuck::{Pod, Zeroable};
use std::collections::{HashMap, HashSet};
use wgpu::util::DeviceExt;

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct RouteVertex {
    a: [f32; 3],
    b: [f32; 3],
    end: f32,
    side: f32,
}
pub struct RouteRenderer {
    pipeline: wgpu::RenderPipeline,
    viewport: wgpu::Buffer,
    bind: wgpu::BindGroup,
    batches: HashMap<usize, (wgpu::Buffer, u32)>,
}
impl RouteRenderer {
    pub fn new(
        device: &wgpu::Device,
        format: wgpu::TextureFormat,
        camera: &wgpu::BindGroupLayout,
    ) -> Self {
        let viewport = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("route viewport uniform"),
            size: 16,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let view_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: None,
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            }],
        });
        let bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: None,
            layout: &view_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: viewport.as_entire_binding(),
            }],
        });
        let shader=device.create_shader_module(wgpu::ShaderModuleDescriptor{label:Some("terrain-following route"),source:wgpu::ShaderSource::Wgsl(r#"
struct Camera {vp:mat4x4<f32>,mode:vec4<u32>};
@group(0) @binding(0) var<uniform> camera:Camera;
@group(1) @binding(0) var<uniform> viewport:vec4<f32>;
struct Out {@builtin(position) position:vec4<f32>,@location(0) side:f32};
@vertex fn vs(@location(0) a:vec3<f32>,@location(1) b:vec3<f32>,@location(2) end:f32,@location(3) side:f32)->Out {
 let p=camera.vp*vec4(a+vec3(0.0,0.06,0.0),1.0);
 let q=camera.vp*vec4(b+vec3(0.0,0.06,0.0),1.0);
 let delta=(q.xy/max(abs(q.w),0.00001)-p.xy/max(abs(p.w),0.00001))*viewport.xy;
 let normal=vec2(-delta.y,delta.x)/max(length(delta),0.00001);
 var clip=mix(p,q,end);
 clip=vec4(clip.xy+normal*side*viewport.z*2.0/viewport.xy*clip.w,clip.z,clip.w);
 var o:Out;o.position=clip;o.side=side;return o;
}
@fragment fn fs(in:Out)->@location(0) vec4<f32> {
 return vec4(select(vec3(0.02,0.93,1.0),vec3(0.01,0.08,0.09),abs(in.side)>0.72),1.0);
}
"#.into())});
        let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: None,
            bind_group_layouts: &[camera, &view_layout],
            push_constant_ranges: &[],
        });
        let pipeline=device.create_render_pipeline(&wgpu::RenderPipelineDescriptor{
            label:Some("depth-tested 3D route"),layout:Some(&layout),
            vertex:wgpu::VertexState{module:&shader,entry_point:Some("vs"),compilation_options:Default::default(),buffers:&[wgpu::VertexBufferLayout{array_stride:std::mem::size_of::<RouteVertex>() as u64,step_mode:wgpu::VertexStepMode::Vertex,attributes:&wgpu::vertex_attr_array![0=>Float32x3,1=>Float32x3,2=>Float32,3=>Float32]}]},
            fragment:Some(wgpu::FragmentState{module:&shader,entry_point:Some("fs"),compilation_options:Default::default(),targets:&[Some(wgpu::ColorTargetState{format,blend:None,write_mask:wgpu::ColorWrites::ALL})]}),
            primitive:wgpu::PrimitiveState{cull_mode:None,..Default::default()},
            depth_stencil:Some(wgpu::DepthStencilState{format:wgpu::TextureFormat::Depth32Float,depth_write_enabled:false,depth_compare:wgpu::CompareFunction::LessEqual,stencil:Default::default(),bias:wgpu::DepthBiasState{constant:-1,slope_scale:0.0,clamp:0.0}}),
            multisample:Default::default(),multiview:None,cache:None,
        });
        Self {
            pipeline,
            viewport,
            bind,
            batches: HashMap::new(),
        }
    }
    pub fn contains(&self, tile: usize) -> bool {
        self.batches.contains_key(&tile)
    }
    pub fn set(&mut self, device: &wgpu::Device, route: Option<&NavigationRoute>) {
        self.batches.clear();
        let Some(route) = route else {
            return;
        };
        let mut data: HashMap<usize, Vec<RouteVertex>> = HashMap::new();
        for s in &route.segments {
            for (end, side) in [
                (0.0, -1.0),
                (1.0, -1.0),
                (0.0, 1.0),
                (0.0, 1.0),
                (1.0, -1.0),
                (1.0, 1.0),
            ] {
                data.entry(s.tile).or_default().push(RouteVertex {
                    a: s.a,
                    b: s.b,
                    end,
                    side,
                });
            }
        }
        for (tile, vertices) in data {
            let buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("route segments uploaded on change"),
                contents: bytemuck::cast_slice(&vertices),
                usage: wgpu::BufferUsages::VERTEX,
            });
            self.batches.insert(tile, (buffer, vertices.len() as u32));
        }
        tracing::info!(
            segments = route.segments.len(),
            tiles = self.batches.len(),
            "3D route GPU buffers uploaded"
        );
    }
    pub fn resize(&self, queue: &wgpu::Queue, size: [u32; 2], pixels_per_point: f32) {
        queue.write_buffer(
            &self.viewport,
            0,
            bytemuck::cast_slice(&[size[0] as f32, size[1] as f32, 2.5 * pixels_per_point, 0.0]),
        );
    }
    pub fn draw(
        &self,
        pass: &mut wgpu::RenderPass<'_>,
        camera: &wgpu::BindGroup,
        resident: &HashSet<usize>,
    ) {
        pass.set_pipeline(&self.pipeline);
        pass.set_bind_group(0, camera, &[]);
        pass.set_bind_group(1, &self.bind, &[]);
        for (tile, (buffer, count)) in &self.batches {
            if resident.contains(tile) {
                pass.set_vertex_buffer(0, buffer.slice(..));
                pass.draw(0..*count, 0..1);
            }
        }
    }
}
