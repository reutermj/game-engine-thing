//! SPIKE (get-3hd.1): does wgpu build and find a GPU? Lists every Vulkan
//! adapter, opens a device on each, and times instance, adapter and device
//! creation: the floor of a presenter reload.

use std::time::Instant;

fn main() {
    let t = Instant::now();
    let instance = wgpu::Instance::new(wgpu::InstanceDescriptor {
        backends: wgpu::Backends::VULKAN,
        ..wgpu::InstanceDescriptor::new_without_display_handle()
    });
    let t_instance = t.elapsed();
    let adapters = pollster::block_on(instance.enumerate_adapters(wgpu::Backends::VULKAN));
    println!("instance {:.2} ms, {} adapters", t_instance.as_secs_f64() * 1e3, adapters.len());
    for adapter in adapters {
        let info = adapter.get_info();
        let t = Instant::now();
        let device = pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor::default()));
        let took = t.elapsed();
        println!(
            "{:?} {} ({:?}, driver {} {}): device {} in {:.2} ms",
            info.device_type,
            info.name,
            info.backend,
            info.driver,
            info.driver_info,
            if device.is_ok() { "made" } else { "FAILED" },
            took.as_secs_f64() * 1e3
        );
    }
}
