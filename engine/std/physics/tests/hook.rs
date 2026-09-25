//! A game's system between finding contacts and solving them (a pre-solve
//! hook), for the pile: what it changes has to wake sleeping bodies as if a
//! game changed it before the step. A mod of its own, so the pile the
//! benchmarks run has no system of its own in the step.
//!
//!   kick <vx> <vy>  set the velocity of the first body dropped, the next
//!              time the hook runs
//!   despawn    despawn it, the next time the hook runs

use engine_api::{Cx, Despawns, Entity, Mod, Query, Systems, With, export_mod};
use physics::{Body, Velocity};

engine_api::mod_state! {
    #[derive(Default)]
    struct Hook {
        /// For the next run: a velocity to set, and whether to despawn.
        kick: Vec<f32>,
        despawn: u32,
    }
}

impl Hook {
    fn between(
        &mut self,
        _: &mut (),
        _: &mut Cx,
        mut ids: Query<&Body, With<Velocity>>,
        mut bodies: Query<&mut Velocity, With<Body>, Despawns>,
    ) {
        if self.kick.is_empty() && self.despawn == 0 {
            return;
        }
        let mut first = None;
        ids.for_each(|row, _| first = Some(first.map_or(row.entity(), |f: Entity| f.min(row.entity()))));
        let Some(first) = first else { return };
        if let [x, y] = self.kick[..] {
            bodies.with(first, |_, mut v| *v = Velocity { x, y });
        }
        if self.despawn != 0 {
            bodies.with(first, |row, _| row.despawn());
        }
        (self.kick, self.despawn) = (Vec::new(), 0);
    }
}

impl Mod for Hook {
    type Transient = ();

    fn systems(s: &mut Systems<Self>) {
        s.add("between", Self::between).phase("physics::step").after("physics::find_contacts").before("physics::solve");
    }

    fn message(&mut self, _: &mut (), _: &mut Cx, message: &str) -> Result<String, String> {
        match message.split_once(' ') {
            Some(("kick", args)) => {
                let args: Result<Vec<f32>, String> =
                    args.split_whitespace().map(|a| a.parse::<f32>().map_err(|e| format!("{a:?}: {e}"))).collect();
                let &[x, y] = args?.as_slice() else { return Err("kick <vx> <vy>".into()) };
                self.kick = vec![x, y];
                Ok("kicking".into())
            }
            None if message.trim() == "despawn" => {
                self.despawn = 1;
                Ok("despawning".into())
            }
            _ => Err("commands: kick <vx> <vy> | despawn".into()),
        }
    }
}

export_mod!(Hook);
