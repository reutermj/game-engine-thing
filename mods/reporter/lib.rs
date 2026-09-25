//! Prints once a second how many bodies are moving, and where the first one
//! is, so a reload elsewhere is visible here.

use engine_api::{Cx, Mod, Query, Systems, export_mod, phase};
use physics::{Position, Velocity};

/// Slower than this counts as still.
const STILL: f32 = 0.1;

engine_api::mod_state! {
    #[derive(Default)]
    struct Reporter {
        frames: u64,
    }
}

impl Reporter {
    fn report(&mut self, _: &mut (), cx: &mut Cx, mut bodies: Query<(&Position, &Velocity)>) {
        self.frames += 1;
        if !self.frames.is_multiple_of(60) {
            return;
        }
        let (mut n, mut moving, mut first) = (0, 0, None);
        bodies.for_each(|row, (p, v)| {
            n += 1;
            if v.x.hypot(v.y) >= STILL {
                moving += 1;
            }
            first.get_or_insert((row.entity().index, *p));
        });
        match first {
            Some((e, p)) => cx.log(format!("{moving} of {n} bodies moving; entity {e} at ({:.2}, {:.2})", p.x, p.y)),
            None => cx.log("no bodies"),
        }
    }
}

impl Mod for Reporter {
    type Transient = ();

    fn systems(s: &mut Systems<Self>) {
        s.add("report", Self::report).phase(phase::LATE);
    }
}

export_mod!(Reporter);
