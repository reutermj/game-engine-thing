//! SPIKE (get-3hd.1): `spike_present`, the window presenter, reloadable:
//! each build makes its own surface on the platform's window, and draws
//! the frame's `DrawList` with wgpu. Its load times every step of making
//! the GPU side (`stats`), which is what a presenter reload costs.
//!
//! Settings, read at load:
//! - `SPIKE_ADAPTER=<name part>`: which Vulkan adapter (default: the
//!   discrete GPU);
//! - `SPIKE_SHARED=1`: the experiment D9 rules out. Borrow the platform's
//!   device (made by the platform's copy of wgpu) instead of making one,
//!   so a reload keeps the device;
//! - `SPIKE_DRAW=per_item`: a draw call per item (default instanced).
//!
//! Messages: `stats`; `hazard on` (shared mode: register a work-done
//! closure on every submit, which the platform's device keeps, so a
//! reload without a wait leaves it pointing into this build); `provoke`
//! (a validation error, to see whose panic handler runs).

use std::ffi::c_void;
use std::ptr::NonNull;
use std::time::Instant;

use engine_api::{Cx, Mod, See, Systems, export_mod, phase};
use presentation_gfx::{DrawItem, Mode, Renderer, SharedGpu};
use spike_draw::{DrawList, Item};
use wgpu::rwh::{RawDisplayHandle, RawWindowHandle, XlibDisplayHandle, XlibWindowHandle};

engine_api::mod_state! {
    #[derive(Default)]
    struct Present {
        loads: u64,
    }
}

struct Gpu {
    // Dropped in this order: the surface before the device it was
    // configured with, everything before the instance.
    renderer: Renderer,
    surface: wgpu::Surface<'static>,
    config: wgpu::SurfaceConfiguration,
    queue: wgpu::Queue,
    device: wgpu::Device,
    _adapter: wgpu::Adapter,
    _instance: wgpu::Instance,
}

#[derive(Default)]
pub struct Presenter {
    gpu: Option<Gpu>,
    made: Vec<(&'static str, f64)>,
    shared: bool,
    mode: Option<Mode>,
    hazard: bool,
    /// The kept copy a delta list is applied to.
    kept: Vec<Item>,
    frames: u64,
    first_frame_ms: f64,
    frame_us: f64,
    made_by: String,
}

fn ms(t: Instant) -> f64 {
    t.elapsed().as_secs_f64() * 1e3
}

impl Presenter {
    fn make(&mut self, cx: &mut Cx) -> Result<(), String> {
        let all = Instant::now();
        let handles = spike_platform::window(cx).map_err(|e| e.to_string())?;
        if handles.display == 0 {
            return Err("the platform has no window".into());
        }
        let want = std::env::var("SPIKE_ADAPTER").ok();
        self.shared = std::env::var("SPIKE_SHARED").is_ok_and(|s| s == "1");
        self.mode = Some(if std::env::var("SPIKE_DRAW").is_ok_and(|s| s == "per_item") { Mode::PerItemSorted } else { Mode::Instanced });
        let (instance, adapter, device, queue) = if self.shared {
            let t = Instant::now();
            let address = spike_platform::shared_gpu(cx, want.unwrap_or_default()).map_err(|e| e.to_string())?;
            // SAFETY (spike only): the platform is resident and keeps the
            // `SharedGpu` alive until it closes, after this mod; the type is
            // the same rlib's in both libraries. What this does *not* make
            // safe is the experiment's question (presentation-spike.md).
            let gpu = unsafe { &*(address as *const SharedGpu) };
            let got = (gpu.instance.clone(), gpu.adapter.clone(), gpu.device.clone(), gpu.queue.clone());
            self.made.push(("borrow the platform's device", ms(t)));
            got
        } else {
            let t = Instant::now();
            let instance = presentation_gfx::instance();
            self.made.push(("instance", ms(t)));
            let t = Instant::now();
            let adapter = presentation_gfx::adapter(&instance, want.as_deref());
            self.made.push(("adapter", ms(t)));
            let t = Instant::now();
            let (device, queue) =
                pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor::default())).map_err(|e| e.to_string())?;
            self.made.push(("device", ms(t)));
            (instance, adapter, device, queue)
        };
        let t = Instant::now();
        let target = wgpu::SurfaceTargetUnsafe::RawHandle {
            raw_display_handle: Some(RawDisplayHandle::Xlib(XlibDisplayHandle::new(
                NonNull::new(handles.display as *mut c_void),
                handles.screen,
            ))),
            raw_window_handle: RawWindowHandle::Xlib(XlibWindowHandle::new(handles.window as _)),
        };
        // SAFETY (spike only): the handles are the platform's window, which
        // outlives this build (resident, closed after it).
        let surface = unsafe { instance.create_surface_unsafe(target) }.map_err(|e| e.to_string())?;
        let mut config =
            surface.get_default_config(&adapter, handles.width, handles.height).ok_or("the adapter can't present to the window")?;
        config.present_mode = wgpu::PresentMode::AutoNoVsync;
        surface.configure(&device, &config);
        self.made.push(("surface", ms(t)));
        let (renderer, made_in) = Renderer::new(&device, config.format);
        self.made.push(("shader", made_in.shader.as_secs_f64() * 1e3));
        self.made.push(("pipelines", made_in.pipelines.as_secs_f64() * 1e3));
        self.made.push(("all", ms(all)));
        cx.log(format!(
            "on {} ({}): {}",
            adapter.get_info().name,
            if self.shared { "the platform's device" } else { "its own device" },
            self.made_summary()
        ));
        self.gpu = Some(Gpu { renderer, surface, config, queue, device, _adapter: adapter, _instance: instance });
        Ok(())
    }

    fn made_summary(&self) -> String {
        self.made.iter().map(|(k, v)| format!("{k} {v:.2} ms")).collect::<Vec<_>>().join(", ")
    }
}

impl Present {
    fn present(&mut self, p: &mut Presenter, cx: &mut Cx, list: See<DrawList>) {
        let t = Instant::now();
        let Some(gpu) = &mut p.gpu else { return };
        if let Ok(h) = spike_platform::window(cx)
            && h.width > 0
            && (h.width, h.height) != (gpu.config.width, gpu.config.height)
        {
            (gpu.config.width, gpu.config.height) = (h.width, h.height);
            gpu.surface.configure(&gpu.device, &gpu.config);
        }
        if !list.full {
            p.kept.resize(list.len as usize, Item::default());
            for &(slot, item) in &list.changed {
                p.kept[slot as usize] = item;
            }
        }
        let items: &[Item] = if list.full { &list.items } else { &p.kept };
        // SAFETY (spike only): `Item` and `DrawItem` are both `repr(C)`,
        // eight 4-byte fields in the same order, no padding.
        let items: &[DrawItem] = unsafe { std::slice::from_raw_parts(items.as_ptr() as *const DrawItem, items.len()) };
        if p.made_by != list.made_by {
            p.made_by.clone_from(&list.made_by);
            cx.log(format!("drawing {} items from a {} extract", list.len, list.made_by));
        }
        let frame = match gpu.surface.get_current_texture() {
            wgpu::CurrentSurfaceTexture::Success(f) | wgpu::CurrentSurfaceTexture::Suboptimal(f) => f,
            wgpu::CurrentSurfaceTexture::Outdated | wgpu::CurrentSurfaceTexture::Lost => {
                gpu.surface.configure(&gpu.device, &gpu.config);
                return;
            }
            _ => return,
        };
        let view = frame.texture.create_view(&Default::default());
        gpu.renderer.upload(&gpu.device, &gpu.queue, (gpu.config.width, gpu.config.height), items);
        let mut encoder = gpu.device.create_command_encoder(&Default::default());
        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: None,
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    depth_slice: None,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color { r: 0.02, g: 0.02, b: 0.05, a: 1.0 }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: None,
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            });
            gpu.renderer.draw(&mut pass, items, p.mode.unwrap_or(Mode::Instanced));
        }
        gpu.queue.submit([encoder.finish()]);
        if p.hazard {
            gpu.queue.on_submitted_work_done(|| {});
        }
        gpu.queue.present(frame);
        let took = t.elapsed().as_secs_f64();
        if p.frames == 0 {
            p.first_frame_ms = took * 1e3;
        } else {
            p.frame_us += took * 1e6;
        }
        p.frames += 1;
    }
}

impl Mod for Present {
    type Transient = Presenter;

    fn systems(s: &mut Systems<Self>) {
        s.add("present", Self::present).phase(phase::RENDER).after("spike_draw::extract");
    }

    fn load(&mut self, p: &mut Presenter, cx: &mut Cx) {
        self.loads += 1;
        if let Err(e) = p.make(cx) {
            cx.log(format!("no GPU: {e}"));
        }
    }

    fn unload(&mut self, p: &mut Presenter, cx: &mut Cx) {
        let t = Instant::now();
        if let Some(gpu) = &p.gpu
            && !p.hazard
        {
            // Nothing of this build's may be left for the device to call:
            // wait for the GPU, which fires (and drops) every callback.
            let _ = gpu.device.poll(wgpu::PollType::wait_indefinitely());
        }
        p.gpu = None;
        cx.log(format!("dropped its GPU side in {:.2} ms", ms(t)));
    }

    fn close(&mut self, p: &mut Presenter, cx: &mut Cx) {
        self.unload(p, cx);
    }

    fn message(&mut self, p: &mut Presenter, _: &mut Cx, message: &str) -> Result<String, String> {
        match message {
            "stats" => Ok(format!(
                "loads {} shared {} made: {}; first frame {:.2} ms; then {:.1} µs a frame over {} frames",
                self.loads,
                p.shared,
                p.made_summary(),
                p.first_frame_ms,
                p.frame_us / (p.frames.max(2) - 1) as f64,
                p.frames
            )),
            "hazard on" => {
                p.hazard = true;
                Ok("registering a work-done closure on every submit, and not waiting at unload".into())
            }
            "provoke" => {
                let gpu = p.gpu.as_ref().ok_or("no GPU")?;
                let buffer = gpu.device.create_buffer(&wgpu::BufferDescriptor {
                    label: Some("too small"),
                    size: 4,
                    usage: wgpu::BufferUsages::COPY_DST,
                    mapped_at_creation: false,
                });
                gpu.queue.write_buffer(&buffer, 0, &[0u8; 64]);
                Ok("wrote 64 bytes into a 4-byte buffer, and nothing panicked".into())
            }
            _ => Err("usage: stats | hazard on | provoke".into()),
        }
    }
}

export_mod!(Present);
