//! Owns the frame loop in real time: paces frames at a fixed rate, publishes
//! the `Clock`, and steps every other mod.

use std::time::{Duration, Instant};

use clock::Clock;
use engine_api::{Cx, Entity, Mod, Status, export_mod};

const FRAME: Duration = Duration::from_nanos(1_000_000_000 / 60);

#[derive(Default)]
struct Bootstrap {
    frame: u64,
    next_frame: Option<Instant>,
    clock: Option<Entity>,
}

impl Mod for Bootstrap {
    fn load(&mut self, cx: &mut Cx) {
        cx.log(format!(
            "loaded (generation {}), frame loop at frame {}",
            cx.generation(),
            self.frame
        ));
    }

    fn step(&mut self, cx: &mut Cx) -> Status {
        self.frame += 1;
        let clock = Clock { frame: self.frame, dt: FRAME.as_secs_f32() };
        let mut world = cx.world();
        let entity = *self.clock.get_or_insert_with(|| world.spawn());
        world.insert(entity, clock);
        cx.step_mods();

        let next = self.next_frame.unwrap_or_else(Instant::now) + FRAME;
        let now = Instant::now();
        if next > now {
            std::thread::sleep(next - now);
            self.next_frame = Some(next);
        } else {
            // Fell behind (e.g. paused in a debugger); don't try to catch up.
            self.next_frame = Some(now);
        }
        Status::OK
    }

    fn close(&mut self, cx: &mut Cx) {
        if let Some(e) = self.clock.take() {
            cx.world().despawn(e);
        }
    }
}

export_mod!(Bootstrap);
