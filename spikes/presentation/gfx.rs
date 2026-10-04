//! SPIKE (get-3hd.1): the draw list's item and a wgpu renderer for it,
//! shared by the draw bench and the presenter mod, so both time the same
//! code. A draw item is a 2D shape (rect or circle) with a position, size,
//! colour and material; a material is one of four pipelines (blend modes),
//! so batching by material means a pipeline switch per batch.

use std::time::{Duration, Instant};

use bytemuck::{Pod, Zeroable};

/// One drawable, as the extract hands it on: 32 bytes, uploaded as is.
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Pod, Zeroable)]
pub struct DrawItem {
    /// Centre, in pixels from the top left.
    pub pos: [f32; 2],
    pub size: [f32; 2],
    /// RGBA8, red in the low byte.
    pub colour: u32,
    /// 0 a rect, 1 a circle.
    pub shape: u32,
    pub material: u32,
    /// Depth in 0..1; unused by the blend-only materials, kept for size.
    pub layer: f32,
}

pub const MATERIALS: u32 = 4;

const SHADER: &str = r#"
struct Globals { size: vec2<f32>, pad: vec2<f32> };
@group(0) @binding(0) var<uniform> g: Globals;

struct VsOut {
    @builtin(position) pos: vec4<f32>,
    @location(0) uv: vec2<f32>,
    @location(1) colour: vec4<f32>,
    @location(2) @interpolate(flat) shape: u32,
};

@vertex
fn vs(@builtin(vertex_index) v: u32,
      @location(0) pos: vec2<f32>, @location(1) size: vec2<f32>,
      @location(2) colour: u32, @location(3) shape: u32,
      @location(4) material: u32, @location(5) layer: f32) -> VsOut {
    let corner = vec2<f32>(f32(v & 1u), f32(v >> 1u));
    let px = pos + (corner - vec2<f32>(0.5, 0.5)) * size;
    var o: VsOut;
    o.pos = vec4<f32>(px.x / g.size.x * 2.0 - 1.0, 1.0 - px.y / g.size.y * 2.0, layer, 1.0);
    o.uv = corner * 2.0 - vec2<f32>(1.0, 1.0);
    o.colour = unpack4x8unorm(colour);
    o.shape = shape;
    return o;
}

@fragment
fn fs(i: VsOut) -> @location(0) vec4<f32> {
    if (i.shape == 1u && dot(i.uv, i.uv) > 1.0) { discard; }
    return i.colour;
}
"#;

/// How a frame's items become draw calls.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    /// A draw call per item, the pipeline set for every one (items in any
    /// material order): the naive renderer.
    PerItem,
    /// A draw call per item, items sorted by material, the pipeline set
    /// once per run.
    PerItemSorted,
    /// One instanced draw call per run of a material (items sorted).
    Instanced,
}

/// What making a renderer cost: the floor of a presenter reload once a
/// device exists.
#[derive(Clone, Copy, Debug, Default)]
pub struct MadeIn {
    pub shader: Duration,
    pub pipelines: Duration,
}

pub struct Renderer {
    pipelines: Vec<wgpu::RenderPipeline>,
    globals: wgpu::Buffer,
    bind: wgpu::BindGroup,
    instances: wgpu::Buffer,
    capacity: u64,
}

fn blend(material: u32) -> wgpu::BlendState {
    match material {
        0 => wgpu::BlendState::REPLACE,
        1 => wgpu::BlendState::ALPHA_BLENDING,
        2 => wgpu::BlendState {
            color: wgpu::BlendComponent {
                src_factor: wgpu::BlendFactor::SrcAlpha,
                dst_factor: wgpu::BlendFactor::One,
                operation: wgpu::BlendOperation::Add,
            },
            alpha: wgpu::BlendComponent::OVER,
        },
        _ => wgpu::BlendState::PREMULTIPLIED_ALPHA_BLENDING,
    }
}

impl Renderer {
    pub fn new(device: &wgpu::Device, format: wgpu::TextureFormat) -> (Renderer, MadeIn) {
        let t = Instant::now();
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("spike shapes"),
            source: wgpu::ShaderSource::Wgsl(SHADER.into()),
        });
        let shader = t.elapsed();
        let t = Instant::now();
        let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: None,
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX,
                ty: wgpu::BindingType::Buffer { ty: wgpu::BufferBindingType::Uniform, has_dynamic_offset: false, min_binding_size: None },
                count: None,
            }],
        });
        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: None,
            bind_group_layouts: &[Some(&layout)],
            immediate_size: 0,
        });
        let attributes = wgpu::vertex_attr_array![0 => Float32x2, 1 => Float32x2, 2 => Uint32, 3 => Uint32, 4 => Uint32, 5 => Float32];
        let pipelines = (0..MATERIALS)
            .map(|m| {
                device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                    label: Some("spike material"),
                    layout: Some(&pipeline_layout),
                    vertex: wgpu::VertexState {
                        module: &module,
                        entry_point: Some("vs"),
                        compilation_options: Default::default(),
                        buffers: &[Some(wgpu::VertexBufferLayout {
                            array_stride: size_of::<DrawItem>() as u64,
                            step_mode: wgpu::VertexStepMode::Instance,
                            attributes: &attributes,
                        })],
                    },
                    primitive: wgpu::PrimitiveState { topology: wgpu::PrimitiveTopology::TriangleStrip, ..Default::default() },
                    depth_stencil: None,
                    multisample: wgpu::MultisampleState::default(),
                    fragment: Some(wgpu::FragmentState {
                        module: &module,
                        entry_point: Some("fs"),
                        compilation_options: Default::default(),
                        targets: &[Some(wgpu::ColorTargetState { format, blend: Some(blend(m)), write_mask: wgpu::ColorWrites::ALL })],
                    }),
                    multiview_mask: None,
                    cache: None,
                })
            })
            .collect();
        let globals = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("spike globals"),
            size: 16,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: None,
            layout: &layout,
            entries: &[wgpu::BindGroupEntry { binding: 0, resource: globals.as_entire_binding() }],
        });
        let pipelines_took = t.elapsed();
        let (instances, capacity) = Self::instance_buffer(device, 1024);
        (Renderer { pipelines, globals, bind, instances, capacity }, MadeIn { shader, pipelines: pipelines_took })
    }

    fn instance_buffer(device: &wgpu::Device, items: u64) -> (wgpu::Buffer, u64) {
        let buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("spike instances"),
            size: items * size_of::<DrawItem>() as u64,
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        (buffer, items)
    }

    /// Uploads the frame's items whole, growing the buffer to fit.
    pub fn upload(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, size: (u32, u32), items: &[DrawItem]) {
        if items.len() as u64 > self.capacity {
            (self.instances, self.capacity) = Self::instance_buffer(device, (items.len() as u64).next_power_of_two());
        }
        queue.write_buffer(&self.globals, 0, bytemuck::cast_slice(&[size.0 as f32, size.1 as f32, 0.0, 0.0]));
        if !items.is_empty() {
            queue.write_buffer(&self.instances, 0, bytemuck::cast_slice(items));
        }
    }

    /// Records the items' draws, as `mode` says. `Instanced` and
    /// `PerItemSorted` expect the items sorted by material.
    pub fn draw(&self, pass: &mut wgpu::RenderPass<'_>, items: &[DrawItem], mode: Mode) {
        pass.set_bind_group(0, &self.bind, &[]);
        pass.set_vertex_buffer(0, self.instances.slice(..));
        match mode {
            Mode::PerItem => {
                for (i, item) in items.iter().enumerate() {
                    pass.set_pipeline(&self.pipelines[item.material as usize]);
                    pass.draw(0..4, i as u32..i as u32 + 1);
                }
            }
            Mode::PerItemSorted => {
                let mut current = u32::MAX;
                for (i, item) in items.iter().enumerate() {
                    if item.material != current {
                        current = item.material;
                        pass.set_pipeline(&self.pipelines[current as usize]);
                    }
                    pass.draw(0..4, i as u32..i as u32 + 1);
                }
            }
            Mode::Instanced => {
                let mut start = 0;
                while start < items.len() {
                    let material = items[start].material;
                    let end = start + items[start..].iter().take_while(|i| i.material == material).count();
                    pass.set_pipeline(&self.pipelines[material as usize]);
                    pass.draw(0..4, start as u32..end as u32);
                    start = end;
                }
            }
        }
    }
}

/// `n` items scattered over `size`, 4 to 16 pixels across, half circles,
/// materials in turn (so unsorted), from a fixed seed.
pub fn scatter(n: usize, size: (u32, u32), seed: u64) -> Vec<DrawItem> {
    let mut s = seed | 1;
    let mut next = move || {
        // xorshift64: deterministic and dependency-free.
        s ^= s << 13;
        s ^= s >> 7;
        s ^= s << 17;
        s
    };
    (0..n)
        .map(|i| {
            let r = next();
            let w = 4.0 + (r % 13) as f32;
            DrawItem {
                pos: [(next() % size.0 as u64) as f32, (next() % size.1 as u64) as f32],
                size: [w, w],
                colour: (next() as u32) | 0x8000_0000,
                shape: (i % 2) as u32,
                material: (i as u32) % MATERIALS,
                layer: 0.5,
            }
        })
        .collect()
}

/// The items stably sorted by material: what a batching extract keeps.
pub fn by_material(items: &[DrawItem]) -> Vec<DrawItem> {
    let mut sorted = items.to_vec();
    sorted.sort_by_key(|i| i.material);
    sorted
}

/// The Vulkan adapter whose name contains `want` (any case), or the first
/// discrete GPU, or the first adapter.
pub fn adapter(instance: &wgpu::Instance, want: Option<&str>) -> wgpu::Adapter {
    let adapters = pollster::block_on(instance.enumerate_adapters(wgpu::Backends::VULKAN));
    let names: Vec<String> = adapters.iter().map(|a| a.get_info().name).collect();
    let pick = match want {
        Some(w) => adapters.iter().position(|a| a.get_info().name.to_lowercase().contains(&w.to_lowercase())),
        None => adapters.iter().position(|a| a.get_info().device_type == wgpu::DeviceType::DiscreteGpu).or(Some(0)),
    };
    let i = pick.unwrap_or_else(|| panic!("no Vulkan adapter matching {want:?} among {names:?}"));
    adapters.into_iter().nth(i).expect("picked from the list")
}

/// A GPU made by one library and, in the spike's shared mode, used from
/// another's copy of wgpu.
pub struct SharedGpu {
    pub instance: wgpu::Instance,
    pub adapter: wgpu::Adapter,
    pub device: wgpu::Device,
    pub queue: wgpu::Queue,
}

impl SharedGpu {
    pub fn new(want: Option<&str>) -> SharedGpu {
        let instance = instance();
        let adapter = adapter(&instance, want.filter(|w| !w.is_empty()));
        let (device, queue) = pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor::default())).expect("a device");
        SharedGpu { instance, adapter, device, queue }
    }
}

pub fn instance() -> wgpu::Instance {
    wgpu::Instance::new(wgpu::InstanceDescriptor {
        backends: wgpu::Backends::VULKAN,
        ..wgpu::InstanceDescriptor::new_without_display_handle()
    })
}
