//! SPIKE (get-3hd.1): `spike_draw`, the extract: turns `Place` and `Look`
//! into the frame's `DrawList`, in one of three modes (`mode <m>`):
//!
//! - `full`: every drawable, walked from the world every frame (Bevy's
//!   extract);
//! - `incremental`: a list kept between frames (this build's transient
//!   part), updated from the rows written since last frame
//!   (`for_each_written`), then copied whole into the flow, since a flow
//!   lives one frame;
//! - `delta`: the same kept list, but the flow carries only what changed
//!   (`changed`), and a presenter keeps its own copy.
//!
//! A row arriving or leaving rebuilds the kept list whole: a spike's
//! shortcut, which the scenes here never take after the first frame.

use engine_api::{Cx, Make, Mod, Query, Systems, export_mod, phase};
use spike_draw::{DrawList, Item, Look, Place};

engine_api::mod_state! {
    #[derive(Default)]
    struct Draw {
        /// 0 full, 1 incremental, 2 delta.
        mode: u32,
        since: u32,
        rebuilds: u64,
        walked: u64,
        frames: u64,
    }
}

/// The list kept between frames, and each entity's slot in it.
#[derive(Default)]
pub struct Kept {
    items: Vec<Item>,
    /// By entity index: its slot in `items`, or `u32::MAX`.
    slot: Vec<u32>,
    valid: bool,
}

const MODES: [&str; 3] = ["full", "incremental", "delta"];

impl Draw {
    fn extract(&mut self, kept: &mut Kept, _: &mut Cx, mut q: Query<(&Place, &Look)>, mut out: Make<DrawList>) {
        out.made_by.push_str(MODES[self.mode as usize]);
        if self.mode == 0 {
            out.full = true;
            q.for_each(|_, (p, l)| out.items.push(Item::of(p, l)));
            out.len = out.items.len() as u32;
            self.since = q.now();
            return;
        }
        let rebuild = !kept.valid || q.arrived_since(self.since) || q.left_since(self.since);
        self.frames += 1;
        if rebuild {
            self.rebuilds += 1;
            kept.items.clear();
            kept.slot.clear();
            q.for_each(|row, (p, l)| {
                let e = row.entity().index as usize;
                if kept.slot.len() <= e {
                    kept.slot.resize(e + 1, u32::MAX);
                }
                kept.slot[e] = kept.items.len() as u32;
                kept.items.push(Item::of(p, l));
            });
            kept.valid = true;
        } else {
            let delta = self.mode == 2;
            let walked = &mut self.walked;
            q.for_each_written(self.since, |row, (p, l)| {
                *walked += 1;
                let slot = kept.slot[row.entity().index as usize];
                let item = Item::of(p, l);
                kept.items[slot as usize] = item;
                if delta {
                    out.changed.push((slot, item));
                }
            });
        }
        self.since = q.now();
        out.len = kept.items.len() as u32;
        if rebuild || self.mode == 1 {
            out.full = true;
            out.items.extend_from_slice(&kept.items);
            out.changed.clear();
        }
    }
}

impl Mod for Draw {
    type Transient = Kept;

    fn systems(s: &mut Systems<Self>) {
        s.add("extract", Self::extract).phase(phase::RENDER);
    }

    fn message(&mut self, kept: &mut Kept, _: &mut Cx, message: &str) -> Result<String, String> {
        match message.split_whitespace().collect::<Vec<_>>()[..] {
            ["mode", m] => {
                self.mode = MODES.iter().position(|x| *x == m).ok_or(format!("modes: {MODES:?}"))? as u32;
                kept.valid = false;
                Ok(format!("mode {m}"))
            }
            ["stats"] => Ok(format!("frames {} rebuilds {} rows found written {}", self.frames, self.rebuilds, self.walked)),
            _ => Err("usage: mode full|incremental|delta | stats".into()),
        }
    }
}

export_mod!(Draw);
