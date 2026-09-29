//! What the 3D baseline records (`baseline.rs`, 2D's, shared;
//! physics-testing.md, "The baseline"), and the scenes each suite runs:
//! the quality tests' piles and stacks (group `quality`, checked by
//! `:quality_test`) and the behaviour tests' scenes (`behaviour`,
//! `:behaviour_test`), the default suite's in `baseline.txt` and the long
//! suite's in `baseline_long.txt`. Every value comes from the runs the
//! tests make (`runs.rs`); `:baseline` prints the comparison and writes
//! the files.

use std::sync::atomic::{AtomicUsize, Ordering};

use crate::baseline::{Band, Better, Entry};
use crate::behave::Behaviour;
use crate::bounces;
use crate::measure::Run;
use crate::runs;
use crate::scenes::Kind;

pub const DEFAULT: &str = include_str!("baseline.txt");
pub const LONG: &str = include_str!("baseline_long.txt");

pub const WRITE: &str = "./bazel run //bench/physics3d:baseline -- --write";
pub const WRITE_LONG: &str = "./bazel run //bench/physics3d:baseline -- --long --write";

/// The piles of the default suite: `scenes.rs`'s, each size its own seeded
/// drop, so its own pile, and judged over the four.
pub const SIZES: [usize; 4] = [200, 300, 400, 500];
/// Stacks of turning cubes.
pub const STACKS: [usize; 4] = [5, 10, 15, 20];
/// The long suite's piles: one a size, since each is minutes in every
/// engine.
pub const BIG: [usize; 2] = [1000, 10000];
/// The long suite's wider families: nine sizes from 200 to 1000, each its
/// own seeded drop, and four from 2000 to 5000.
pub const MID: [usize; 9] = [200, 300, 400, 500, 600, 700, 800, 900, 1000];
pub const LARGE: [usize; 4] = [2000, 3000, 4000, 5000];
/// (kind, turning, sizes): cubes turning and locked, planks turning and
/// the mixed pile (`Kind::Mixed`) both ways at `MID`; cubes and planks
/// turning at `LARGE`.
pub const WIDE: [(Kind, bool, &[usize]); 7] = [
    (Kind::BoxPile, true, &MID),
    (Kind::BoxPile, false, &MID),
    (Kind::PlankPile, true, &MID),
    (Kind::Mixed, true, &MID),
    (Kind::Mixed, false, &MID),
    (Kind::BoxPile, true, &LARGE),
    (Kind::PlankPile, true, &LARGE),
];

/// Energies a body under this are rounding, not motion (2D's `STILL`).
pub const STILL: f64 = 1e-8;

/// The bands, each set just outside the spread measured under changes
/// that shouldn't matter: every pile at another size's seed (`--offset` 1
/// to 11), and two reassociated sums planted in the solver
/// (physics-testing.md, "The baseline", has the spreads). Rest is the step
/// from which every body stays under the threshold, looked at every step.
///
/// A pile family (four or nine sizes, each a random drop): its median
/// rest moved by up to 84 steps (but for the big planks'); its median
/// depth at the end by 41%, landing depth's median by 31% and the greatest
/// mean overlap by 23%, the least partners a body by 20%; its median
/// energy not past a factor of 1. The worsts aren't recorded, one pile
/// deciding them: the worst rest went from 505 to 1462, from never to 94,
/// the worst energy from 6e-12 to 1e-3, the worst depth by 76%.
const REST_MEDIAN: Band = Band::Steps(90.0, 0.3);
const PILE_DEPTH: Band = Band::Rel(0.5, 1e-4);
const PILE_LANDING: Band = Band::Rel(0.35, 1e-4);
const PILE_SHAPE: Band = Band::Rel(0.25, 0.0);
const ENERGY: Band = Band::Log(10.0, STILL);
/// One big pile (the long suite's, one run a size): its rest moved by 700
/// steps, planted, so it isn't recorded; at the next sizes' seeds its
/// depth at the end by 55%, the greatest mean overlap by 19%, partners a
/// body by 14%; its energy by a factor of 6.6. One run is a weak record:
/// the wider families (`quality(true)`) are the long suite's statistics.
const BIG_DEPTH: Band = Band::Rel(0.6, 1e-4);
const BIG_MEAN: Band = Band::Rel(0.25, 1e-4);
const BIG_SHAPE: Band = Band::Rel(0.15, 0.0);
/// A stack: rest didn't move; its top by 9%, depth by 3%, energy by a
/// factor of 1.13, and its tilt by 0.014°, under the measure's resolution
/// (0.02°, `measure.rs`'s tests).
const STAND_REST: Band = Band::Steps(10.0, 0.0);
const STAND_LENGTH: Band = Band::Rel(0.15, 1e-4);
const STAND_TILT: Band = Band::Rel(0.1, 0.05);
const STAND_ENERGY: Band = Band::Log(3.0, STILL);
/// Behaviour: lengths within 2.6% (a heavy cube's jitter, ten-millionths,
/// by more but under the floor), rest not a step apart.
const MOTION: Band = Band::Rel(0.03, 1e-4);
const STEPS_TO: Band = Band::Steps(10.0, 0.0);
const ANALYTIC: Band = Band::Rel(0.002, 0.0);
const CREEP: Band = Band::Rel(0.05, 2e-5);
const APEX: Band = Band::Rel(0.01, 0.0);
const COUNT: Band = Band::Exact;

/// Added to every pile size by `:baseline`'s `--offset`, recorded under
/// the sizes' names: how the spread over sizes is measured. 0 in tests.
static OFFSET: AtomicUsize = AtomicUsize::new(0);

pub fn set_offset(n: usize) {
    OFFSET.store(n, Ordering::Relaxed);
}

fn shifted(n: usize) -> usize {
    n + OFFSET.load(Ordering::Relaxed)
}

fn turned(rotate: bool) -> &'static str {
    if rotate { " turning" } else { "" }
}

fn rest(r: &Run) -> f64 {
    r.settled_at.map_or(f64::INFINITY, |s| s as f64)
}

fn energy(r: &Run) -> f64 {
    r.quality.kinetic_energy / r.quality.bodies.max(1) as f64
}

fn median(mut v: Vec<f64>) -> f64 {
    v.sort_by(f64::total_cmp);
    v[v.len() / 2]
}

fn most(v: impl Iterator<Item = f64>) -> f64 {
    v.fold(f64::NEG_INFINITY, f64::max)
}

fn least(v: impl Iterator<Item = f64>) -> f64 {
    v.fold(f64::INFINITY, f64::min)
}

/// One pile: how soon it rests, how deep it sinks at the end and while
/// settling, what energy is left, whether anything escaped.
fn pile(group: &'static str, name: String, r: &Run) -> Vec<Entry> {
    let e = |measure: &str, value: f64, band: Band, better: Better| Entry::new(group, name.clone(), measure, value, band, better);
    vec![
        e("depth_end", r.quality.pen_max as f64, BIG_DEPTH, Better::Lower),
        e("mean_during", r.pen_mean_during as f64, BIG_MEAN, Better::Lower),
        e("energy_end", energy(r), ENERGY, Better::Lower),
        e("partners", r.quality.contacts_per_body, BIG_SHAPE, Better::Neither),
        e("escaped", r.quality.escaped as f64, COUNT, Better::Lower),
    ]
}

/// A pile at several sizes (each its own seeded drop): the family's
/// statistics, the median and the worst over the sizes, never a size's
/// own value, as in 2D.
fn piles(group: &'static str, kind: Kind, sizes: &[usize], rotate: bool) -> Vec<Entry> {
    let runs = runs::par(sizes, |&n| runs::ours(kind, shifted(n), rotate));
    let name = format!("{} {}-{}{}", kind.name(), sizes[0], sizes[sizes.len() - 1], turned(rotate));
    let e = |measure: &str, value: f64, band: Band, better: Better| Entry::new(group, name.clone(), measure, value, band, better);
    let of = |f: fn(&Run) -> f64| runs.iter().map(|r| f(r)).collect::<Vec<f64>>();
    let (rests, depth_end, depth_during, energies) =
        (of(rest), of(|r| r.quality.pen_max as f64), of(|r| r.pen_max_during as f64), of(energy));
    let mut v = Vec::new();
    // The big piles' median rest isn't a result: at another seed, planks
    // of 2000-5000 went from 417 to 1416.
    if sizes.len() > 4 || sizes[0] < 2000 {
        v.push(e("rest_median", median(rests), REST_MEDIAN, Better::Lower));
    }
    v.extend([
        e("depth_end_median", median(depth_end), PILE_DEPTH, Better::Lower),
        e("depth_during_median", median(depth_during), PILE_LANDING, Better::Lower),
        e("mean_during_worst", most(runs.iter().map(|r| r.pen_mean_during as f64)), PILE_LANDING, Better::Lower),
        e("energy_end_median", median(energies), ENERGY, Better::Lower),
        e("partners_least", least(runs.iter().map(|r| r.quality.contacts_per_body)), PILE_SHAPE, Better::Neither),
        e("escaped", runs.iter().map(|r| r.quality.escaped as f64).sum(), COUNT, Better::Lower),
    ]);
    v
}

/// A stack that stands.
fn stands(group: &'static str, n: usize) -> Vec<Entry> {
    let r = runs::ours(Kind::Stack, n, true);
    let name = format!("stack {n} turning");
    let e = |measure: &str, value: f64, band: Band, better: Better| Entry::new(group, name.clone(), measure, value, band, better);
    vec![
        e("rest", rest(&r), STAND_REST, Better::Lower),
        e("top_moved", r.top_moved as f64, STAND_LENGTH, Better::Lower),
        e("depth_end", r.quality.pen_max as f64, STAND_LENGTH, Better::Lower),
        e("tilt", r.quality.tilt as f64, STAND_TILT, Better::Lower),
        e("energy_end", energy(&r), STAND_ENERGY, Better::Lower),
        e("escaped", r.quality.escaped as f64, COUNT, Better::Lower),
    ]
}

type Job<'a> = Box<dyn Fn() -> Vec<Entry> + Send + Sync + 'a>;

/// The quality group: the piles and stacks `:quality_test` bounds, or with
/// `long` the big piles `:quality_long_test` does.
pub fn quality(long: bool) -> Vec<Entry> {
    const G: &str = "quality";
    let mut jobs: Vec<Job> = Vec::new();
    if !long {
        jobs.push(Box::new(|| piles(G, Kind::BoxPile, &SIZES, true)));
        jobs.push(Box::new(|| piles(G, Kind::BoxPile, &SIZES, false)));
        jobs.push(Box::new(|| piles(G, Kind::PlankPile, &SIZES, true)));
        for n in STACKS {
            jobs.push(Box::new(move || stands(G, n)));
        }
    } else {
        for kind in [Kind::BoxPile, Kind::PlankPile] {
            for n in BIG {
                jobs.push(Box::new(move || pile(G, format!("{} {n} turning", kind.name()), &runs::ours(kind, shifted(n), true))));
            }
        }
        for &(kind, rotate, sizes) in &WIDE {
            jobs.push(Box::new(move || piles(G, kind, sizes, rotate)));
        }
    }
    runs::par(&jobs, |j| j()).concat()
}

/// A bounce family's statistic: 2D's bands (its `record::bounce_band`),
/// but a count's, whose grids here are no more than 3000.
fn bounce_band(name: &str) -> (Band, Better) {
    match name {
        "gain median" => (Band::Rel(0.05, 1e-3), Better::Toward(0.0)),
        "bounced below" | "flat above" => (Band::Abs(1.0), Better::Lower),
        "tangent most" | "tangent median" => (Band::Rel(0.01, 1e-4), Better::Neither),
        "rise worst" | "decay worst" | "decay median" => (Band::Rel(0.01, 1e-3), Better::Toward(1.0)),
        "momentum worst" => (Band::Rel(0.5, 1e-4), Better::Lower),
        _ => (Band::Rel(0.05, 1e-3), Better::Lower),
    }
}

/// The bounce families' statistics (`bounces.rs`), on the short grids or
/// the long.
fn bouncing(group: &'static str, long: bool) -> Vec<Entry> {
    let mut v = Vec::new();
    for f in bounces::families(long) {
        let runs: Vec<Behaviour> = runs::par(&f.hits, |h| (*runs::hit(h)).clone());
        for (name, value) in bounces::stats(&f, &runs) {
            let (band, better) = bounce_band(name);
            v.push(Entry::new(group, format!("bounces {}", f.name), name.replace(' ', "_"), value, band, better));
        }
    }
    v
}

/// The behaviour group: every scene `:behaviour_test` bounds, the ignored
/// tests' too, and the bounce families on their short grids; with `long`,
/// on their long ones alone (`:behaviour_long_test`).
pub fn behaviour(long: bool) -> Vec<Entry> {
    const G: &str = "behaviour";
    if long {
        return bouncing(G, true);
    }
    let cases = [
        (Kind::RampHold, 1),
        (Kind::RampSlide, 1),
        (Kind::RampRoll, 1),
        (Kind::Bounce, 25),
        (Kind::Bounce, 50),
        (Kind::Bounce, 75),
        (Kind::Bounce, 100),
        (Kind::Ratio, 10),
        (Kind::Ratio, 100),
        (Kind::Ratio, 1000),
    ];
    let runs = runs::par(&cases, |&(kind, n)| runs::behaved(kind, n));
    let mut v = Vec::new();
    for ((kind, n), b) in cases.iter().zip(&runs) {
        let name = format!("{} {n}", kind.name());
        let e = |measure: &str, band: Band, better: Better| {
            Entry::new(G, name.clone(), measure.replace(' ', "_"), b.get(measure), band, better)
        };
        let (lower, higher) = (Better::Lower, Better::Higher);
        match kind {
            Kind::RampHold => v.push(e("crept", CREEP, lower)),
            Kind::RampSlide => v.push(e("a", ANALYTIC, Better::Toward(b.get("expected a")))),
            Kind::RampRoll => {
                v.push(e("a", ANALYTIC, Better::Toward(b.get("expected a"))));
                v.push(e("slip", MOTION, Better::Neither));
            }
            Kind::Bounce => {
                let e2 = b.get("expected");
                v.push(e("first apex", APEX, Better::Toward(e2)));
                if *n >= 100 {
                    v.push(e("most apex", APEX, Better::Toward(1.0)));
                    v.push(e("last apex", APEX, Better::Toward(1.0)));
                }
            }
            _ => v.extend([
                e("top sank", MOTION, lower),
                e("deepest at end", MOTION, lower),
                e("stands", COUNT, higher),
                e("escaped", COUNT, lower),
                e("at rest from", STEPS_TO, lower),
                e("jitter", MOTION, lower),
            ]),
        }
    }
    v.extend(bouncing(G, false));
    v
}
