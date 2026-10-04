//! SPIKE (get-3hd.1): `spike_sink`, a presenter without a GPU, for the
//! extract bench: it reads the draw list another mod made, and does what
//! the GPU presenter's upload does with it, a copy into memory it owns (a
//! whole list) or the changed slots written into its kept copy (a delta).
//! `stats` replies the frames it saw and a checksum, so the bench can tell
//! it really read them.

use engine_api::{Cx, Mod, See, Systems, export_mod, phase};
use spike_draw::{DrawList, Item};

engine_api::mod_state! {
    #[derive(Default)]
    struct Sink {
        frames: u64,
        checksum: f64,
    }
}

#[derive(Default)]
pub struct Staging {
    items: Vec<Item>,
}

impl Sink {
    fn read(&mut self, staging: &mut Staging, _: &mut Cx, list: See<DrawList>) {
        if list.full {
            staging.items.clear();
            staging.items.extend_from_slice(&list.items);
        } else {
            staging.items.resize(list.len as usize, Item::default());
            for &(slot, item) in &list.changed {
                staging.items[slot as usize] = item;
            }
        }
        self.frames += 1;
        if let Some(last) = staging.items.last() {
            self.checksum += last.pos[0] as f64;
        }
    }
}

impl Mod for Sink {
    type Transient = Staging;

    fn systems(s: &mut Systems<Self>) {
        s.add("read", Self::read).phase(phase::RENDER).after("spike_draw::extract");
    }

    fn message(&mut self, staging: &mut Staging, _: &mut Cx, message: &str) -> Result<String, String> {
        match message {
            "stats" => Ok(format!("frames {} items {} checksum {}", self.frames, staging.items.len(), self.checksum)),
            _ => Err("usage: stats".into()),
        }
    }
}

export_mod!(Sink);
