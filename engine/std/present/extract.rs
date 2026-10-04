//! The extract's work, apart from the mod so its tests run it on a world
//! of their own (`tests/extract_test.rs`): the presentation components to
//! the frame's `DrawList`.

use std::collections::BTreeMap;

use engine_api::{Entity, Query};
use present::{Camera, Circle, DrawList, Item, Label, Labelled, Line, Look, Place, Rect, Run, Shape, TEXT_ADVANCE, Text, TextItem};

/// What the extract keeps between frames for its allocations only: the
/// list itself is rebuilt whole (D2).
#[derive(Default)]
pub struct Scratch {
    order: Vec<u32>,
    items: Vec<Item>,
    entities: Vec<Entity>,
    classes: BTreeMap<String, u32>,
}

/// An axis-aligned box, by its corners.
#[derive(Clone, Copy)]
struct Bounds {
    min: [f32; 2],
    max: [f32; 2],
}

impl Bounds {
    fn around(pos: [f32; 2], half: [f32; 2]) -> Bounds {
        Bounds { min: [pos[0] - half[0], pos[1] - half[1]], max: [pos[0] + half[0], pos[1] + half[1]] }
    }

    fn of(item: &Item) -> Bounds {
        let [c, s] = [item.rot[0].abs(), item.rot[1].abs()];
        let [w, h] = [item.size[0] / 2.0, item.size[1] / 2.0];
        Bounds::around(item.pos, [c * w + s * h, s * w + c * h])
    }

    fn of_text(at: &Place, t: &Text) -> Bounds {
        let size = [t.text.chars().count() as f32 * t.size * TEXT_ADVANCE, t.size];
        let min = [at.x - t.anchor[0] * size[0], at.y - t.anchor[1] * size[1]];
        Bounds { min, max: [min[0] + size[0], min[1] + size[1]] }
    }

    fn join(a: Option<Bounds>, b: Bounds) -> Bounds {
        match a {
            None => b,
            Some(a) => {
                Bounds { min: [a.min[0].min(b.min[0]), a.min[1].min(b.min[1])], max: [a.max[0].max(b.max[0]), a.max[1].max(b.max[1])] }
            }
        }
    }
}

fn rect(at: &Place, r: &Rect, look: &Look) -> Item {
    Item {
        pos: [at.x, at.y],
        size: [r.w, r.h],
        rot: [at.c, at.s],
        colour: look.colour,
        shape: Shape::Rect,
        layer: look.layer,
        material: look.material,
    }
}

fn circle(at: &Place, c: &Circle, look: &Look) -> Item {
    let d = 2.0 * c.radius;
    Item { size: [d, d], shape: Shape::Circle, ..rect(at, &Rect::default(), look) }
}

/// A rectangle from the place to its far end, `width` thick, turned to
/// point along it. A line of no length keeps the place's turn.
fn line(at: &Place, l: &Line, look: &Look) -> Item {
    let d = [at.c * l.dx - at.s * l.dy, at.s * l.dx + at.c * l.dy];
    let len = d[0].hypot(d[1]);
    let rot = if len > 0.0 { [d[0] / len, d[1] / len] } else { [at.c, at.s] };
    Item { pos: [at.x + d[0] / 2.0, at.y + d[1] / 2.0], size: [len, l.width], rot, shape: Shape::Line, ..rect(at, &Rect::default(), look) }
}

/// Fills `out` (empty, as `Make` hands it over) from the world.
pub fn extract(
    out: &mut DrawList,
    scratch: &mut Scratch,
    (rects, circles, lines): (&mut Query<(&Place, &Rect, &Look)>, &mut Query<(&Place, &Circle, &Look)>, &mut Query<(&Place, &Line, &Look)>),
    texts: &mut Query<(&Place, &Text, &Look)>,
    labels: &mut Query<(&Place, &Label)>,
    cameras: &mut Query<&Camera>,
) {
    shapes(out, scratch, rects, circles, lines);
    texts.for_each(|row, (at, t, look)| {
        out.texts.push(TextItem {
            entity: row.entity(),
            text: t.text.clone(),
            pos: [at.x, at.y],
            size: t.size,
            anchor: t.anchor,
            colour: look.colour,
            layer: look.layer,
        });
    });
    // Stable, so text of one layer keeps the order it was found in.
    if !out.texts.is_sorted_by_key(|t| t.layer) {
        out.texts.sort_by_key(|t| t.layer);
    }
    labelled(out, scratch, labels, (rects, circles, lines), texts);
    view(out, cameras, texts);
}

fn shapes(
    out: &mut DrawList,
    scratch: &mut Scratch,
    rects: &mut Query<(&Place, &Rect, &Look)>,
    circles: &mut Query<(&Place, &Circle, &Look)>,
    lines: &mut Query<(&Place, &Line, &Look)>,
) {
    let DrawList { items, entities, .. } = out;
    rects.for_each(|row, (at, r, look)| {
        items.push(rect(at, r, look));
        entities.push(row.entity());
    });
    circles.for_each(|row, (at, c, look)| {
        items.push(circle(at, c, look));
        entities.push(row.entity());
    });
    lines.for_each(|row, (at, l, look)| {
        items.push(line(at, l, look));
        entities.push(row.entity());
    });
    // A game whose drawables share a layer and material, or that spawns
    // them in order, is already sorted: one pass to see it, no sort.
    let key = |i: &Item| (i.layer, i.material);
    if !items.is_sorted_by_key(key) {
        // Sorted by index, then gathered, so `entities` follows `items`.
        // Stable, so one run's items keep the order the extract found
        // them in, which is the world's and the same every run.
        let Scratch { order, items: sorted, entities: sorted_entities, .. } = scratch;
        order.clear();
        order.extend(0..items.len() as u32);
        order.sort_by_key(|&i| key(&items[i as usize]));
        sorted.clear();
        sorted.extend(order.iter().map(|&i| items[i as usize]));
        sorted_entities.clear();
        sorted_entities.extend(order.iter().map(|&i| entities[i as usize]));
        std::mem::swap(items, sorted);
        std::mem::swap(entities, sorted_entities);
    }
    let mut start = 0;
    while start < items.len() {
        let (layer, material) = key(&items[start]);
        let len = items[start..].iter().take_while(|i| key(i) == (layer, material)).count();
        out.runs.push(Run { layer, material, start: start as u32, len: len as u32 });
        start += len;
    }
}

fn labelled(
    out: &mut DrawList,
    scratch: &mut Scratch,
    labels: &mut Query<(&Place, &Label)>,
    (rects, circles, lines): (&mut Query<(&Place, &Rect, &Look)>, &mut Query<(&Place, &Circle, &Look)>, &mut Query<(&Place, &Line, &Look)>),
    texts: &mut Query<(&Place, &Text, &Look)>,
) {
    // Classes are numbered by first meeting each frame; the map only
    // saves the allocations.
    scratch.classes.clear();
    labels.for_each(|row, (at, label)| {
        let e = row.entity();
        let mut b = None;
        // Looked up by entity: a query with an optional term would walk
        // every shape for the few that are labelled.
        if let Some(i) = rects.with(e, |_, (at, r, look)| Bounds::of(&rect(at, r, look))) {
            b = Some(Bounds::join(b, i));
        }
        if let Some(i) = circles.with(e, |_, (at, c, look)| Bounds::of(&circle(at, c, look))) {
            b = Some(Bounds::join(b, i));
        }
        if let Some(i) = lines.with(e, |_, (at, l, look)| Bounds::of(&line(at, l, look))) {
            b = Some(Bounds::join(b, i));
        }
        if let Some(i) = texts.with(e, |_, (at, t, _)| Bounds::of_text(at, t)) {
            b = Some(Bounds::join(b, i));
        }
        let b = b.unwrap_or(Bounds::around([at.x, at.y], [0.0, 0.0]));
        let next = scratch.classes.len() as u32;
        let what = match scratch.classes.get(label.what.as_str()) {
            Some(&what) => what,
            None => {
                scratch.classes.insert(label.what.clone(), next);
                out.names.push(label.what.clone());
                next
            }
        };
        out.labels.push(Labelled {
            entity: e,
            what,
            name: label.name.clone(),
            pos: [(b.min[0] + b.max[0]) / 2.0, (b.min[1] + b.max[1]) / 2.0],
            size: [b.max[0] - b.min[0], b.max[1] - b.min[1]],
        });
    });
}

fn view(out: &mut DrawList, cameras: &mut Query<&Camera>, texts: &mut Query<(&Place, &Text, &Look)>) {
    cameras.for_each(|_, camera| {
        out.cameras += 1;
        if out.view.is_none() {
            out.view = Some(*camera);
        }
    });
    if out.view.is_some() {
        return;
    }
    // No camera: show everything there is, so a game is visible before it
    // says what to look at.
    let mut b = None;
    for item in &out.items {
        b = Some(Bounds::join(b, Bounds::of(item)));
    }
    texts.for_each(|_, (at, t, _)| b = Some(Bounds::join(b, Bounds::of_text(at, t))));
    for l in &out.labels {
        b = Some(Bounds::join(b, Bounds::around(l.pos, [l.size[0] / 2.0, l.size[1] / 2.0])));
    }
    out.view = b.map(|b| Camera { x: b.min[0], y: b.min[1], w: b.max[0] - b.min[0], h: b.max[1] - b.min[1], ..Camera::default() });
}
