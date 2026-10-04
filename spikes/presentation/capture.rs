//! SPIKE (get-3hd.1): `spike_capture`, a presenter that keeps the last
//! frame's draw list and hands it out as text, for the replay tool
//! (`pong_replay.rs`), which rasterises it with tiny-skia. It reads the
//! list after `spike_pong_view` has drawn pong into it, so the tool draws
//! exactly what the window's presenter is given. Only the replay games
//! load it, in place of `spike_present`.
//!
//! `items` replies `canvas <w> <h>`, then one line per item: `x y w h
//! colour shape`, floats as Rust prints them (the shortest text that
//! parses back to the same bits).

use std::fmt::Write;

use engine_api::{Cx, Mod, See, Systems, export_mod, phase};
use spike_draw::{DrawList, Item};

engine_api::mod_state! {
    #[derive(Default)]
    struct Capture {
        frames: u64,
    }
}

#[derive(Default)]
pub struct Kept {
    items: Vec<Item>,
    canvas: (f32, f32),
}

impl Capture {
    fn read(&mut self, kept: &mut Kept, _: &mut Cx, list: See<DrawList>) {
        if list.full {
            kept.items.clear();
            kept.items.extend_from_slice(&list.items);
        } else {
            kept.items.resize(list.len as usize, Item::default());
            for &(slot, item) in &list.changed {
                kept.items[slot as usize] = item;
            }
        }
        kept.canvas = (list.canvas_w, list.canvas_h);
        self.frames += 1;
    }
}

impl Mod for Capture {
    type Transient = Kept;

    fn systems(s: &mut Systems<Self>) {
        s.add("read", Self::read).phase(phase::RENDER).after("spike_draw::extract").after("spike_pong_view::draw");
    }

    fn message(&mut self, kept: &mut Kept, _: &mut Cx, message: &str) -> Result<String, String> {
        match message.trim() {
            "items" => {
                let mut out = format!("canvas {} {}", kept.canvas.0, kept.canvas.1);
                for i in &kept.items {
                    let _ = write!(out, "\n{} {} {} {} {} {}", i.pos[0], i.pos[1], i.size[0], i.size[1], i.colour, i.shape);
                }
                Ok(out)
            }
            "stats" => Ok(format!("frames {} items {}", self.frames, kept.items.len())),
            _ => Err("usage: items | stats".into()),
        }
    }
}

export_mod!(Capture);
