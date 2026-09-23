//! One source, several builds of the mod `counter`, selected by crate
//! feature. Each build adds a different amount per step and stamps its
//! number, so a test can tell which code ran and whether state carried over.

use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use engine_api::{Cx, Entity, Mod, Status, export_mod};
use test_probe::Probe;

#[cfg(feature = "v1")]
const BUILD: (u32, u64) = (1, 1);
#[cfg(feature = "v2")]
const BUILD: (u32, u64) = (2, 100);
#[cfg(feature = "v3")]
const BUILD: (u32, u64) = (3, 1000);
// Panics on its first step only, and counts like v1 after that, so a loader
// that kept stepping a failed mod would visibly move the total.
#[cfg(feature = "panic")]
const BUILD: (u32, u64) = (4, 1);

/// Reset by every fresh mapping of the library, unlike `Mod` state.
static LOADS: AtomicU32 = AtomicU32::new(0);
static PANICKED: AtomicBool = AtomicBool::new(false);

#[derive(Default)]
struct Counter {
    total: u64,
    probe: Option<Entity>,
    /// Only in v3, to give it an incompatible state layout.
    #[cfg(feature = "v3")]
    _grown: [u64; 4],
}

impl Counter {
    fn write_probe(&self, cx: &mut Cx) {
        let probe = Probe {
            value: self.total,
            build: BUILD.0,
            loads_seen_by_statics: LOADS.load(Ordering::Relaxed),
        };
        cx.world().insert(self.probe.expect("spawned in load"), probe);
    }
}

impl Mod for Counter {
    fn load(&mut self, cx: &mut Cx) {
        LOADS.fetch_add(1, Ordering::Relaxed);
        if self.probe.is_none() {
            self.probe = Some(cx.world().spawn());
        }
        self.write_probe(cx);
    }

    fn step(&mut self, cx: &mut Cx) -> Status {
        if cfg!(feature = "panic") && !PANICKED.swap(true, Ordering::Relaxed) {
            panic!("this build of counter panics on its first step");
        }
        self.total += BUILD.1;
        self.write_probe(cx);
        Status::OK
    }

    /// The state is going away, so the entity it tracks would be orphaned.
    fn close(&mut self, cx: &mut Cx) {
        if let Some(e) = self.probe.take() {
            cx.world().despawn(e);
        }
    }
}

export_mod!(Counter);
