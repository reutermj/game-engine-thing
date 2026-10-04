//! SPIKE (get-3hd.1): `spike_platform`, a resident mod owning winit's
//! event loop and one window, pumped (never run) by whoever calls the
//! `pump` service: the windowed bootstrap, once a frame. Resident because
//! winit's X11 connection installs an error handler in libX11 that points
//! into this library, and one event loop per process is all winit allows.
//!
//! In the experiment `shared_gpu` serves, it also owns a wgpu device that
//! the presenter, a reloadable mod with its own copy of wgpu, borrows.

use std::sync::Arc;
use std::time::{Duration, Instant};

use engine_api::{Cx, Mod, export_mod};
use presentation_gfx::SharedGpu;
use spike_platform::{Pumped, WindowHandles};
use winit::application::ApplicationHandler;
use winit::event::WindowEvent;
use winit::event_loop::{ActiveEventLoop, EventLoop};
use winit::platform::pump_events::EventLoopExtPumpEvents;
use winit::raw_window_handle::{HasDisplayHandle, HasWindowHandle, RawDisplayHandle, RawWindowHandle};
use winit::window::{Window, WindowId};

engine_api::mod_state! {
    #[derive(Default)]
    struct Platform {
        pumps: u64,
    }
}

#[derive(Default)]
struct App {
    window: Option<Window>,
    close: bool,
    events: u32,
    keys: Vec<String>,
}

impl ApplicationHandler for App {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        if self.window.is_none() {
            let attributes =
                Window::default_attributes().with_title("presentation spike").with_inner_size(winit::dpi::PhysicalSize::new(1280, 720));
            self.window = event_loop.create_window(attributes).ok();
        }
    }

    fn window_event(&mut self, _: &ActiveEventLoop, _: WindowId, event: WindowEvent) {
        self.events += 1;
        match event {
            WindowEvent::CloseRequested => self.close = true,
            WindowEvent::KeyboardInput { event, .. } if event.state.is_pressed() => {
                self.keys.push(format!("{:?}", event.logical_key));
            }
            _ => {}
        }
    }
}

/// The event loop, and what it made. Fields drop in order: the GPU (whose
/// surfaces, if any, the presenter dropped first), the window, then the
/// event loop, while this library is still mapped.
#[derive(Default)]
pub struct Host {
    gpu: Option<Arc<SharedGpu>>,
    app: App,
    event_loop: Option<EventLoop<()>>,
}

impl Host {
    fn pump(&mut self) -> Pumped {
        let t = Instant::now();
        self.app.events = 0;
        if let Some(event_loop) = &mut self.event_loop {
            event_loop.pump_app_events(Some(Duration::ZERO), &mut self.app);
        }
        let (width, height) = self.app.window.as_ref().map_or((0, 0), |w| (w.inner_size().width, w.inner_size().height));
        Pumped {
            close: self.app.close,
            width,
            height,
            events: self.app.events,
            keys: std::mem::take(&mut self.app.keys),
            micros: t.elapsed().as_micros() as u32,
        }
    }
}

impl Mod for Platform {
    type Transient = Host;

    fn load(&mut self, host: &mut Host, cx: &mut Cx) {
        let t = Instant::now();
        match EventLoop::new() {
            Ok(event_loop) => host.event_loop = Some(event_loop),
            Err(e) => {
                cx.log(format!("no event loop: {e}"));
                return;
            }
        }
        // The first pump delivers `resumed`, where the window is made.
        host.pump();
        cx.log(format!(
            "event loop and window in {:.1} ms: {}",
            t.elapsed().as_secs_f64() * 1e3,
            if host.app.window.is_some() { "window open" } else { "no window" }
        ));
    }

    fn close(&mut self, host: &mut Host, cx: &mut Cx) {
        let t = Instant::now();
        let shared = host.gpu.take().map(|g| Arc::strong_count(&g));
        host.app.window = None;
        host.event_loop = None;
        cx.log(format!("closed the window and event loop in {:.1} ms (shared gpu refs left: {shared:?})", t.elapsed().as_secs_f64() * 1e3));
    }
}

impl spike_platform::Platform for Platform {
    fn pump(&mut self, host: &mut Host, _: &mut Cx) -> Pumped {
        self.pumps += 1;
        host.pump()
    }

    fn window(&mut self, host: &mut Host, _: &mut Cx) -> WindowHandles {
        let Some(window) = &host.app.window else { return WindowHandles::default() };
        let size = window.inner_size();
        let display = window.display_handle().map(|h| h.as_raw());
        let handle = window.window_handle().map(|h| h.as_raw());
        match (display, handle) {
            (Ok(RawDisplayHandle::Xlib(d)), Ok(RawWindowHandle::Xlib(w))) => WindowHandles {
                display: d.display.map_or(0, |p| p.as_ptr() as u64),
                screen: d.screen,
                window: w.window,
                width: size.width,
                height: size.height,
            },
            _ => WindowHandles::default(),
        }
    }

    fn shared_gpu(&mut self, host: &mut Host, cx: &mut Cx, adapter: String) -> u64 {
        let gpu = host.gpu.get_or_insert_with(|| {
            let t = Instant::now();
            let gpu = Arc::new(SharedGpu::new(Some(&adapter)));
            cx.log(format!("made a shared gpu on {} in {:.1} ms", gpu.adapter.get_info().name, t.elapsed().as_secs_f64() * 1e3));
            gpu
        });
        Arc::as_ptr(gpu) as u64
    }
}

export_mod!(Platform, provides = [spike_platform::Platform]);
