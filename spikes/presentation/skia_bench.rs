//! SPIKE (get-3hd.1): the deterministic pixel presenter's cost. Rasterises
//! the draw bench's shapes (rects and circles, 4 to 16 px, translucent)
//! with tiny-skia into a 1280x720 pixmap on one thread, anti-aliased and
//! not, and hashes the pixels, so two runs (or two machines) can be
//! compared. The median of `FRAMES` frames.
//!
//!     ./bazel run --config=bench //spikes/presentation:skia_bench

use std::time::Instant;

use presentation_gfx::{DrawItem, scatter};
use tiny_skia::{Color, FillRule, Paint, PathBuilder, Pixmap, Rect, Transform};

const SIZE: (u32, u32) = (1280, 720);

fn draw(pixmap: &mut Pixmap, items: &[DrawItem], anti_alias: bool) {
    pixmap.fill(Color::BLACK);
    let mut paint = Paint { anti_alias, ..Paint::default() };
    for item in items {
        let [r, g, b, a] = item.colour.to_le_bytes();
        paint.set_color_rgba8(r, g, b, a);
        let ([x, y], [w, h]) = (item.pos, item.size);
        if item.shape == 1 {
            if let Some(path) = PathBuilder::from_circle(x, y, w / 2.0) {
                pixmap.fill_path(&path, &paint, FillRule::Winding, Transform::identity(), None);
            }
        } else if let Some(rect) = Rect::from_xywh(x - w / 2.0, y - h / 2.0, w, h) {
            pixmap.fill_rect(rect, &paint, Transform::identity(), None);
        }
    }
}

fn fnv(bytes: &[u8]) -> u64 {
    bytes.iter().fold(0xcbf2_9ce4_8422_2325, |h, &b| (h ^ b as u64).wrapping_mul(0x100_0000_01b3))
}

fn main() {
    let frames: usize = std::env::var("FRAMES").ok().and_then(|f| f.parse().ok()).unwrap_or(20);
    let mut pixmap = Pixmap::new(SIZE.0, SIZE.1).expect("a pixmap");
    println!("ms a frame on one thread, 1280x720, the median of {frames}; pixel hash\n");
    println!("| shapes | anti-aliased | aliased | hash (aa) |");
    println!("|---|---|---|---|");
    for n in [1_000, 10_000, 100_000] {
        let items = scatter(n, SIZE, 7);
        let mut cells = Vec::new();
        let mut hash = 0;
        for aa in [true, false] {
            let mut times = Vec::new();
            for _ in 0..frames + 2 {
                let t = Instant::now();
                draw(&mut pixmap, &items, aa);
                times.push(t.elapsed().as_secs_f64() * 1e3);
            }
            let h = fnv(pixmap.data());
            if aa {
                hash = h;
            }
            // Twice more: the same pixels every time, or it isn't a golden image.
            draw(&mut pixmap, &items, aa);
            assert_eq!(fnv(pixmap.data()), h, "the same shapes rasterised twice differ");
            times.drain(..2);
            times.sort_by(|a, b| a.total_cmp(b));
            cells.push(format!("{:.2}", times[times.len() / 2]));
        }
        println!("| {n} | {} | {hash:016x} |", cells.join(" | "));
    }
}
