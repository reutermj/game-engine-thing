//! SPIKE (get-3hd.1): draw throughput. Renders 1k, 10k and 100k shapes to
//! an offscreen 1280x720 target, one draw call per item against one
//! instanced draw per material, and times whole frames: the upload, the
//! encode, the submit and the wait for the GPU. The median of `FRAMES`
//! frames after a warm-up.
//!
//!     ./bazel run --config=bench //spikes/presentation:draw_bench
//!     SPIKE_ADAPTER=llvmpipe ./bazel run --config=bench //spikes/presentation:draw_bench

use std::time::Instant;

use presentation_gfx::{DrawItem, Mode, Renderer, adapter, by_material, instance, scatter};

const SIZE: (u32, u32) = (1280, 720);

fn median(mut xs: Vec<f64>) -> f64 {
    xs.sort_by(|a, b| a.total_cmp(b));
    xs[xs.len() / 2]
}

fn main() {
    let want = std::env::var("SPIKE_ADAPTER").ok();
    let frames: usize = std::env::var("FRAMES").ok().and_then(|f| f.parse().ok()).unwrap_or(30);
    let counts: Vec<usize> =
        std::env::var("COUNTS").ok().map(|c| c.split(',').map(|n| n.parse().unwrap()).collect()).unwrap_or(vec![1_000, 10_000, 100_000]);
    let instance = instance();
    let adapter = adapter(&instance, want.as_deref());
    let info = adapter.get_info();
    let (device, queue) = pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor::default())).expect("a device");
    let format = wgpu::TextureFormat::Rgba8UnormSrgb;
    let target = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("offscreen"),
        size: wgpu::Extent3d { width: SIZE.0, height: SIZE.1, depth_or_array_layers: 1 },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
        view_formats: &[],
    });
    let view = target.create_view(&Default::default());
    let (mut renderer, made) = Renderer::new(&device, format);
    println!("{} ({}): shader {:.2} ms, 4 pipelines {:.2} ms", info.name, info.driver_info, ms(made.shader), ms(made.pipelines));
    println!("ms a frame (upload + encode + submit + wait), the median of {frames}; encode alone in brackets\n");
    println!("| items | per item, unsorted | per item, by material | instanced, by material | pixel hash |");
    println!("|---|---|---|---|---|");
    for n in counts {
        let unsorted = scatter(n, SIZE, 7);
        let sorted = by_material(&unsorted);
        let mut cells = Vec::new();
        for (mode, items) in [(Mode::PerItem, &unsorted), (Mode::PerItemSorted, &sorted), (Mode::Instanced, &sorted)] {
            let (whole, encode) = time(&device, &queue, &view, &mut renderer, items, mode, frames);
            cells.push(format!("{whole:.3} ({encode:.3})"));
        }
        let hash = pixels(&device, &queue, &target);
        println!("| {n} | {} | {hash:016x} |", cells.join(" | "));
    }
}

fn ms(d: std::time::Duration) -> f64 {
    d.as_secs_f64() * 1e3
}

fn time(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    view: &wgpu::TextureView,
    renderer: &mut Renderer,
    items: &[DrawItem],
    mode: Mode,
    frames: usize,
) -> (f64, f64) {
    let (mut whole, mut encode) = (Vec::new(), Vec::new());
    for f in 0..frames + 5 {
        let t = Instant::now();
        renderer.upload(device, queue, SIZE, items);
        let mut encoder = device.create_command_encoder(&Default::default());
        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: None,
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view,
                    depth_slice: None,
                    resolve_target: None,
                    ops: wgpu::Operations { load: wgpu::LoadOp::Clear(wgpu::Color::BLACK), store: wgpu::StoreOp::Store },
                })],
                depth_stencil_attachment: None,
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            });
            renderer.draw(&mut pass, items, mode);
        }
        let commands = encoder.finish();
        let encoded = t.elapsed();
        queue.submit([commands]);
        device.poll(wgpu::PollType::wait_indefinitely()).expect("the frame");
        if f >= 5 {
            whole.push(ms(t.elapsed()));
            encode.push(ms(encoded));
        }
    }
    (median(whole), median(encode))
}

/// FNV-1a of the target's pixels, as the last frame left them: whether two
/// runs, or two adapters, drew the same image.
fn pixels(device: &wgpu::Device, queue: &wgpu::Queue, target: &wgpu::Texture) -> u64 {
    let row = SIZE.0 as u64 * 4;
    let buffer = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("readback"),
        size: row * SIZE.1 as u64,
        usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
        mapped_at_creation: false,
    });
    let mut encoder = device.create_command_encoder(&Default::default());
    encoder.copy_texture_to_buffer(
        wgpu::TexelCopyTextureInfo { texture: target, mip_level: 0, origin: wgpu::Origin3d::ZERO, aspect: wgpu::TextureAspect::All },
        wgpu::TexelCopyBufferInfo {
            buffer: &buffer,
            layout: wgpu::TexelCopyBufferLayout { offset: 0, bytes_per_row: Some(row as u32), rows_per_image: Some(SIZE.1) },
        },
        wgpu::Extent3d { width: SIZE.0, height: SIZE.1, depth_or_array_layers: 1 },
    );
    queue.submit([encoder.finish()]);
    buffer.map_async(wgpu::MapMode::Read, .., |r| r.expect("mapped"));
    device.poll(wgpu::PollType::wait_indefinitely()).expect("the copy");
    let bytes = buffer.get_mapped_range(..).expect("the pixels");
    bytes.iter().fold(0xcbf2_9ce4_8422_2325, |h, &b| (h ^ b as u64).wrapping_mul(0x100_0000_01b3))
}
