//! One source, several builds of the mod `counter`, selected by crate
//! feature. Each build adds a different amount per step and stamps its
//! number, so a test can tell which code ran and whether state carried over.

use std::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use engine_api::{Cx, Entity, Mod, Query, Systems, export_mod};
use test_probe::Probe;

#[cfg(feature = "v1")]
const BUILD: (u32, u64) = (1, 1);
#[cfg(feature = "v2")]
const BUILD: (u32, u64) = (2, 100);
#[cfg(feature = "v3")]
const BUILD: (u32, u64) = (3, 1000);
#[cfg(feature = "v4")]
const BUILD: (u32, u64) = (5, 10);
// Panics on its first step only, and counts like v1 after that, so a loader
// that kept stepping a failed mod would visibly move the total.
#[cfg(feature = "panic")]
const BUILD: (u32, u64) = (4, 1);

/// Reset by every fresh mapping of the library, unlike `Mod` state.
static LOADS: AtomicU32 = AtomicU32::new(0);
static PANICKED: AtomicBool = AtomicBool::new(false);

#[cfg(not(any(feature = "v3", feature = "v4")))]
engine_api::mod_state! {
    #[derive(Default)]
    struct Counter {
        total: u64,
        probe: Option<Entity>,
    }
}

// v3 declares that its state means something else (version 1), so the loader
// has v1 drop its state and v3 start over.
#[cfg(feature = "v3")]
engine_api::mod_state! {
    #[derive(Default)]
    struct Counter, version = 1 {
        total: u64,
        probe: Option<Entity>,
        #[allow(dead_code)]
        grown: [u64; 4],
    }
}

// v4 only adds a field, so the loader migrates the state and keeps the rest.
#[cfg(feature = "v4")]
engine_api::mod_state! {
    #[derive(Default)]
    struct Counter {
        total: u64,
        probe: Option<Entity>,
        #[allow(dead_code)]
        grown: [u64; 4],
    }
}

impl Counter {
    fn probe_value(&self) -> Probe {
        Probe { value: self.total, build: BUILD.0, loads_seen_by_statics: LOADS.load(Ordering::Relaxed) }
    }

    /// Outside a frame, where the world can be changed directly.
    fn write_probe(&self, cx: &mut Cx) {
        cx.world().insert(self.probe.expect("spawned in load"), self.probe_value());
    }
}

impl Counter {
    fn count(&mut self, _: &mut (), _: &mut Cx, mut probes: Query<&mut Probe>) {
        if cfg!(feature = "panic") && !PANICKED.swap(true, Ordering::Relaxed) {
            panic!("this build of counter panics on its first step");
        }
        self.total += BUILD.1;
        let (probe, entity) = (self.probe_value(), self.probe.expect("spawned in load"));
        probes.with(entity, |_, mut p| *p = probe);
    }
}

impl Mod for Counter {
    type Transient = ();

    fn systems(s: &mut Systems<Self>) {
        s.add("count", Self::count);
    }

    fn load(&mut self, _: &mut (), cx: &mut Cx) {
        LOADS.fetch_add(1, Ordering::Relaxed);
        if self.probe.is_none() {
            self.probe = Some(cx.world().spawn(()));
        }
        self.write_probe(cx);
    }

    /// `get` replies with the total; `add <n>` adds to it.
    fn message(&mut self, _: &mut (), cx: &mut Cx, message: &str) -> Result<String, String> {
        match message.split_once(' ') {
            None if message == "get" => Ok(self.total.to_string()),
            // Not resident, so the loader refuses.
            None if message == "pump" => Ok(format!("{:?}", cx.pump_loader(std::time::Duration::ZERO, |_, _| Err(String::new())))),
            Some(("add", n)) => {
                self.total += n.parse::<u64>().map_err(|e| format!("{n:?}: {e}"))?;
                self.write_probe(cx);
                Ok(self.total.to_string())
            }
            _ => Err(format!("counter doesn't understand {message:?}")),
        }
    }

    /// The state is going away, so the entity it tracks would be orphaned.
    fn close(&mut self, _: &mut (), cx: &mut Cx) {
        if let Some(e) = self.probe.take() {
            cx.world().despawn(e);
        }
    }
}

export_mod!(Counter);
