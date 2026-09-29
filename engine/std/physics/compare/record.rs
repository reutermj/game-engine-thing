//! What the 2D baseline records (`baseline.rs`; physics-testing.md, "The
//! baseline"), and the scenes each suite runs, which the tests bound and
//! this records: the quality tests' settling (group `quality`, checked by
//! `:quality_test`) and the behaviour tests' scenes and families (group
//! `behaviour`, `:behaviour_test`), the default suite's in `baseline.txt`
//! and the long suite's in `baseline_long.txt`. Every value comes from the
//! runs the tests make (`runs.rs`), so checking it costs the suite nothing
//! it didn't run already; `:baseline` prints the comparison and writes the
//! files.

use std::sync::atomic::{AtomicI32, Ordering};

use crate::baseline::{Band, Better, Entry};
use crate::behave::{self, Behaviour};
use crate::family;
use crate::runs;
use crate::scene::Scene;
use crate::settle::Settling;

/// The files, as the tests check them.
pub const DEFAULT: &str = include_str!("baseline.txt");
pub const LONG: &str = include_str!("baseline_long.txt");

/// What writes them, which a failing test names.
pub const WRITE: &str = "./bazel run //engine/std/physics/compare:baseline -- --write";
pub const WRITE_LONG: &str = "./bazel run //engine/std/physics/compare:baseline -- --long --write";

/// Steps every default scene is settled: past the latest bound on rest (a
/// turning pile's, 500), so "at rest from" means it stayed so for a while.
/// The references were measured over the same steps.
pub const STEPS: u32 = 700;
/// The long suite's: long enough to see the references stay at rest, or
/// not (at 10 000 their turning piles don't).
pub const LONG_STEPS: u32 = 2500;

/// The piles: the comparison's, 41 wide, staggered, at five sizes, each
/// its own drop. Settling is chaotic (docs/lore), so a pile is judged over
/// its sizes, never on one.
pub const PILES: [u32; 5] = [400, 600, 800, 1000, 1200];
pub const PILE_WIDTH: f32 = 41.0;
/// The big piles, 401 wide.
pub const BIG_PILES: [u32; 3] = [9000, 10000, 11000];
pub const BIG_WIDTH: f32 = 401.0;
/// Pyramids, which stand and don't vary so: one scene each.
pub const PYRAMIDS: [u32; 3] = [15, 20, 25];
pub const STACKS: [u32; 2] = [10, 20];
/// The orders a turning pyramid 40 wide is solved in, and a 50 wide one
/// (colored, and in the long suite in three orders).
pub const PYRAMID_40_ORDERS: [&str; 2] = ["rot", "rot/order=3"];
pub const PYRAMID_50_COLORED: [&str; 1] = ["rot/colored=4"];
pub const PYRAMID_50_ORDERS: [&str; 3] = ["rot", "rot/order=2", "rot/order=3"];
/// The long suite's wider pile families (width, sizes, steps): each width
/// from about a third full to the top of the box, 7 to 10 sizes, locked
/// and turning; the 41 and 401 wide ones include the default's and the
/// big piles' sizes, which the run cache shares.
pub const WIDE_PILES: [(f32, &[u32], u32); 5] = [
    (21.0, &[150, 200, 250, 300, 350, 400, 450], STEPS),
    (41.0, &[300, 400, 500, 600, 700, 800, 900, 1000, 1100, 1200], STEPS),
    (81.0, &[800, 1000, 1200, 1400, 1600, 1800, 2000, 2200, 2400], 1500),
    (161.0, &[2000, 2400, 2800, 3200, 3600, 4000, 4400, 4800], 1500),
    (401.0, &[5000, 6000, 7000, 8000, 9000, 10000, 11000, 12000], LONG_STEPS),
];
/// Mixed shapes and materials (`Scene::Mixed`), 41 wide.
pub const MIXED: [u32; 9] = [400, 500, 600, 700, 800, 900, 1000, 1100, 1200];
/// Pyramids at more widths (bases, steps): 30-60 over the default's
/// steps, and 70-120 around the 5050 (base 100) over the long suite's.
pub const PYRAMID_FAMILIES: [(&[u32], u32); 2] =
    [(&[30, 35, 40, 45, 50, 55, 60], STEPS), (&[70, 80, 90, 95, 100, 105, 110, 120], LONG_STEPS)];
/// The pile sleeping is tested on, and the most steps it is given.
pub const SLEEP_PILE: Scene = Scene::Pile { n: 1000, width: 41.0, stagger: true };
pub const SLEEP_MAX: u32 = 1500;

/// Energies under this are rounding, not motion: every body slower than
/// about 1e-4, a five-hundredth of the sleep threshold.
pub const STILL: f64 = 1e-8;

/// The bands, each set just outside the spread measured under changes
/// that shouldn't matter: every pile family at sizes 2% to 10% either way
/// (`:baseline` `--offset`), and three reassociated sums planted in the
/// solver (physics-testing.md, "The baseline", has the spreads).
///
/// A pile family of five sizes or more: its median rest moved by up to 90
/// steps, its median first look at rest by 120; its depths by up to 41%,
/// the greatest mean overlap by 24%, contacts a body by 1.6%; its median
/// energy by a factor of 1.04 where it comes to rest. Its worst rest and
/// worst first rest aren't recorded: one pile in the family decides them,
/// and they went from 240 to 390, from 400 to 1050, from never to 1380.
const REST_MEDIAN: Band = Band::Steps(100.0, 0.2);
const FIRST_MEDIAN: Band = Band::Steps(130.0, 0.25);
/// Three sizes (the big piles): the medians moved by up to 240.
const FEW_MEDIAN: Band = Band::Steps(250.0, 0.3);
const PILE_DEPTH: Band = Band::Rel(0.45, 1e-4);
const PILE_MEAN: Band = Band::Rel(0.3, 1e-4);
const PILE_ENERGY: Band = Band::Log(10.0, STILL);
const SHAPE: Band = Band::Rel(0.03, 0.0);
/// A scene that stands (a pyramid, a stack): rest didn't move at all (a
/// look is 10 steps), its top and depth by 0.2%, its tilt by 4%, its
/// energy by a factor of 1.13.
const STAND_REST: Band = Band::Steps(10.0, 0.0);
const STAND_LENGTH: Band = Band::Rel(0.02, 1e-4);
const STAND_TILT: Band = Band::Rel(0.1, 0.01);
const STAND_ENERGY: Band = Band::Log(3.0, STILL);
/// When a pile of 1000 is all asleep: 50 steps later, planted.
const ASLEEP: Band = Band::Steps(60.0, 0.0);
/// A behaviour scene: every length and speed within 1.2%, rest and
/// separation not a step apart.
const MOTION: Band = Band::Rel(0.03, 1e-4);
const STEPS_TO: Band = Band::Steps(10.0, 0.0);
/// A hand calculation's value: it didn't move, and the law tests allow 1%.
const ANALYTIC: Band = Band::Rel(0.002, 0.0);
/// Counts and flags.
const COUNT: Band = Band::Exact;
/// An edge family's count of runs that did what they should: a
/// reassociated sum flipped one of four dominoes' spacings and one of 135
/// card houses. One run, or 2% of a long grid.
fn family_band(runs: usize) -> Band {
    Band::Abs((0.02 * runs as f64).floor().max(1.0))
}

/// Every pile size moved by this many percent by `:baseline`'s
/// `--offset`: the neighbouring piles, recorded under the names of these,
/// which is how the spread over sizes the bands are set outside of is
/// measured. A share, not a count, since the families span 150 to 12 000
/// bodies. 0 in the tests.
static OFFSET: AtomicI32 = AtomicI32::new(0);

pub fn set_offset(percent: i32) {
    assert!(percent.abs() < 50, "--offset is a percentage of each size");
    OFFSET.store(percent, Ordering::Relaxed);
}

fn shifted(n: u32) -> u32 {
    (n as f64 * (1.0 + OFFSET.load(Ordering::Relaxed) as f64 / 100.0)).round() as u32
}

fn turned(turning: bool) -> &'static str {
    if turning { " turning" } else { "" }
}

/// The look a run stayed at rest from, or never.
fn rest(r: Option<u32>) -> f64 {
    r.map_or(f64::INFINITY, f64::from)
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

/// A pile family: one pile at several sizes, recorded as the family's
/// statistics over its sizes, never a size's own value: a size's rest
/// moves by up to 90 steps to the next size, or to never, its depth by a
/// third and its energy ten thousandfold (physics-testing.md, "The
/// baseline", has the spread). Medians, and the worst only of depth and
/// mean overlap, which no one pile decides.
/// `mixed`: the mixed shapes and materials (`Scene::Mixed`) in place of
/// the pile's circles and boxes.
fn piles(group: &'static str, sizes: &[u32], width: f32, turning: bool, steps: u32, stable: Rests, mixed: bool) -> Vec<Entry> {
    let scene = |n: u32| if mixed { Scene::Mixed { n, width } } else { Scene::Pile { n, width, stagger: true } };
    let runs = runs::par(sizes, |&n| runs::settled(scene(shifted(n)), turning, steps, ""));
    let kind = if mixed { "mixed" } else { "piles" };
    let name = format!("{kind} {}-{} {width}{}", sizes[0], sizes[sizes.len() - 1], turned(turning));
    let rests: Vec<f64> = runs.iter().map(|r| rest(r.rest_from)).collect();
    let firsts: Vec<f64> = runs.iter().map(|r| rest(r.first_rest)).collect();
    let e = |measure: &str, value: f64, band: Band, better: Better| Entry::new(group, name.clone(), measure, value, band, better);
    let of = |f: fn(&Settling) -> f64| runs.iter().map(f).collect::<Vec<f64>>();
    let (depth_end, depth_during) = (of(|r| r.end.max_depth as f64), of(|r| r.deepest_during as f64));
    let (rest_band, first_band) = if sizes.len() <= 3 { (FEW_MEDIAN, FEW_MEDIAN) } else { (REST_MEDIAN, FIRST_MEDIAN) };
    let mut v = Vec::new();
    if stable == Rests::Median {
        v.push(e("rest_median", median(rests), rest_band, Better::Lower));
    }
    v.extend([
        e("first_rest_median", median(firsts), first_band, Better::Lower),
        e("depth_end_median", median(depth_end.clone()), PILE_DEPTH, Better::Lower),
        e("depth_end_worst", most(depth_end.into_iter()), PILE_DEPTH, Better::Lower),
        e("depth_during_median", median(depth_during), PILE_DEPTH, Better::Lower),
        e("mean_during_worst", most(runs.iter().map(|r| r.mean_during)), PILE_MEAN, Better::Lower),
    ]);
    if stable == Rests::Median {
        v.push(e("energy_end_median", median(of(|r| r.end.energy)), PILE_ENERGY, Better::Lower));
    }
    v.extend([
        e("contacts_least", least(runs.iter().map(|r| r.end.contacts_per_body)), SHAPE, Better::Neither),
        e("escaped", runs.iter().map(|r| r.end.escaped as f64).sum(), COUNT, Better::Lower),
    ]);
    v
}

/// Whether a pile family's median rest from is a result, not rounding.
#[derive(Clone, Copy, PartialEq)]
enum Rests {
    /// It is: its median rest from, and its median first look at rest.
    Median,
    /// Only its first look at rest is: the big turning piles, whose median
    /// rest from is 350, 690, 740 or never at neighbouring sizes, a body
    /// now and then moving again (as in Box2D and Rapier), and the locked
    /// piles 81 wide, whose median went from 260 to 510 at sizes 10% more,
    /// piles moving again after they rest (get-emj.63). Nor its energy.
    First,
}

/// Which rests of a pile family `width` wide are results.
fn rests(width: f32, turning: bool) -> Rests {
    if (turning && width >= BIG_WIDTH) || (!turning && width == 81.0) { Rests::First } else { Rests::Median }
}

/// A scene that stands (a pyramid, a stack): how soon it rests, how far
/// its top moved, how deep it sinks, how far it leans, what energy is left
/// in its last `settle::TAIL` steps, and whether anything left it.
fn stands(group: &'static str, name: String, r: &Settling) -> Vec<Entry> {
    let e = |measure: &str, value: f64, band: Band, better: Better| Entry::new(group, name.clone(), measure, value, band, better);
    vec![
        e("rest", rest(r.rest_from), STAND_REST, Better::Lower),
        e("top_moved", r.top_moved.unwrap_or(0.0) as f64, STAND_LENGTH, Better::Lower),
        e("depth_end", r.end.max_depth as f64, STAND_LENGTH, Better::Lower),
        e("tilt", r.end.tilt as f64, STAND_TILT, Better::Lower),
        e("energy_tail", r.energy_tail, STAND_ENERGY, Better::Lower),
        e("escaped", r.end.escaped as f64, COUNT, Better::Lower),
    ]
}

/// Pyramids at several widths, judged as a family: the median and the
/// worst rest, the median and worst of how far the top moved, the median
/// depth at the end, the worst tilt, the median energy in the last
/// `settle::TAIL` steps, and how many stand (no box leaning past 5°,
/// nothing out of the scene: not the top's move, which in Box2D and
/// Rapier is mostly the sinking of a hundred soft rows, 1.5 at 100 wide).
/// One pyramid is steady under rounding (a reassociated sum moved none of
/// their rests), but a big turning one's rest grows with its width (290
/// at 70 wide to 1200 at 120), so the family says whether the 5050's 780
/// (get-emj.62) is one scene's or the trend's.
fn pyramids(group: &'static str, bases: &[u32], turning: bool, steps: u32) -> Vec<Entry> {
    let runs = runs::par(bases, |&base| runs::settled(Scene::Pyramid { base }, turning, steps, ""));
    let name = format!("pyramids {}-{}{}", bases[0], bases[bases.len() - 1], turned(turning));
    let e = |measure: &str, value: f64, band: Band, better: Better| Entry::new(group, name.clone(), measure, value, band, better);
    let of = |f: fn(&Settling) -> f64| runs.iter().map(f).collect::<Vec<f64>>();
    let (rests, tops) = (of(|r| rest(r.rest_from)), of(|r| r.top_moved.unwrap_or(0.0) as f64));
    let stand = runs.iter().filter(|r| r.end.tilt < 5.0 && r.end.escaped == 0).count();
    vec![
        e("rest_median", median(rests.clone()), STAND_REST, Better::Lower),
        e("rest_worst", most(rests.into_iter()), STAND_REST, Better::Lower),
        e("top_moved_median", median(tops.clone()), STAND_LENGTH, Better::Lower),
        e("top_moved_worst", most(tops.into_iter()), STAND_LENGTH, Better::Lower),
        e("depth_end_median", median(of(|r| r.end.max_depth as f64)), STAND_LENGTH, Better::Lower),
        e("tilt_worst", most(runs.iter().map(|r| r.end.tilt as f64)), STAND_TILT, Better::Lower),
        e("energy_tail_median", median(of(|r| r.energy_tail)), STAND_ENERGY, Better::Lower),
        e("stand", stand as f64, family_band(runs.len()), Better::Higher),
        e("escaped", runs.iter().map(|r| r.end.escaped as f64).sum(), COUNT, Better::Lower),
    ]
}

type Job<'a> = Box<dyn Fn() -> Vec<Entry> + Send + Sync + 'a>;

/// Each job in a thread of its own, their entries in order.
fn gather(jobs: Vec<Job>) -> Vec<Entry> {
    runs::par(&jobs, |j| j()).concat()
}

fn stand_job(group: &'static str, name: String, scene: Scene, turning: bool, steps: u32, spec: &'static str) -> Job<'static> {
    Box::new(move || stands(group, name.clone(), &runs::settled(scene, turning, steps, &runs::with_solver(spec))))
}

/// The quality group: every settling scene `:quality_test` bounds, or with
/// `long` those only `:quality_long_test` does.
pub fn quality(long: bool) -> Vec<Entry> {
    const G: &str = "quality";
    let mut jobs: Vec<Job> = Vec::new();
    if !long {
        for turning in [false, true] {
            jobs.push(Box::new(move || piles(G, &PILES, PILE_WIDTH, turning, STEPS, rests(PILE_WIDTH, turning), false)));
            for base in PYRAMIDS {
                let name = format!("pyramid {base}{}", turned(turning));
                jobs.push(stand_job(G, name, Scene::Pyramid { base }, turning, STEPS, ""));
            }
        }
        for n in STACKS {
            jobs.push(stand_job(G, format!("stack {n}"), Scene::Stack { n }, false, STEPS, ""));
        }
        jobs.push(stand_job(G, "stack 10 turning".into(), Scene::Stack { n: 10 }, true, STEPS, ""));
        jobs.push(Box::new(|| {
            let r = runs::settled_on_mod(Scene::Stack { n: 20 }, true, 6, STEPS);
            stands(G, "stack 20 turning, 6 substeps".into(), &r)
        }));
        for spec in PYRAMID_40_ORDERS {
            jobs.push(stand_job(G, format!("pyramid 40 turning {spec}"), Scene::Pyramid { base: 40 }, true, STEPS, spec));
        }
        for spec in PYRAMID_50_COLORED {
            jobs.push(stand_job(G, format!("pyramid 50 turning {spec}"), Scene::Pyramid { base: 50 }, true, STEPS, spec));
        }
        for turning in [false, true] {
            jobs.push(Box::new(move || {
                let at = runs::asleep_at(SLEEP_PILE, turning, SLEEP_MAX).map_or(f64::INFINITY, f64::from);
                let name = format!("{}{} sleeping", SLEEP_PILE.text(), turned(turning));
                vec![Entry::new(G, name, "asleep_at", at, ASLEEP, Better::Lower)]
            }));
        }
    } else {
        for turning in [false, true] {
            jobs.push(Box::new(move || piles(G, &BIG_PILES, BIG_WIDTH, turning, LONG_STEPS, rests(BIG_WIDTH, turning), false)));
            for &(width, sizes, steps) in &WIDE_PILES {
                jobs.push(Box::new(move || piles(G, sizes, width, turning, steps, rests(width, turning), false)));
            }
            jobs.push(Box::new(move || piles(G, &MIXED, PILE_WIDTH, turning, STEPS, rests(PILE_WIDTH, turning), true)));
            for &(bases, steps) in &PYRAMID_FAMILIES {
                jobs.push(Box::new(move || pyramids(G, bases, turning, steps)));
            }
            let name = format!("pyramid 100{}", turned(turning));
            jobs.push(stand_job(G, name, Scene::Pyramid { base: 100 }, turning, LONG_STEPS, ""));
        }
        for spec in PYRAMID_50_ORDERS {
            jobs.push(stand_job(G, format!("pyramid 50 turning {spec}"), Scene::Pyramid { base: 50 }, true, STEPS, spec));
        }
    }
    gather(jobs)
}

/// How far a box that holds on a ramp crept in 2 s: tenths of a
/// thousandth, which moved by 0.9%, where a hundred-thousandth is rounding.
const CREEP: Band = Band::Rel(0.05, 2e-5);
/// A bounce's apex over its drop: it didn't move.
const APEX: Band = Band::Rel(0.01, 0.0);
/// How far a ladder slid: nothing where it stands, over a unit where not.
const SLID: Band = Band::Rel(0.03, 1e-4);
/// How far overlap recovery's top ends from where it rests once apart:
/// 1.2%, of a hundredth.
const TOP_OFF: Band = Band::Rel(0.05, 1e-3);

/// What the baseline records of a behaviour scene: the values its test
/// bounds (`behaviour_test.rs`), each by its kind, the ignored tests'
/// included, so that fixing what they found shows as a better value.
fn behaviour_of(group: &'static str, scene: &Scene, b: &Behaviour) -> Vec<Entry> {
    let name = scene.text();
    let e = |measure: &str, band: Band, better: Better| {
        Entry::new(group, name.clone(), measure.replace(' ', "_"), b.get(measure), band, better)
    };
    let (lower, higher) = (Better::Lower, Better::Higher);
    match *scene {
        Scene::Ramp { circle: false, .. } if b.get("expected a") <= 0.0 => vec![e("crept", CREEP, lower)],
        Scene::Ramp { circle, .. } => {
            let mut v = vec![e("a", ANALYTIC, Better::Toward(b.get("expected a")))];
            if circle {
                v.push(e("slip", MOTION, Better::Neither));
            }
            v
        }
        Scene::Bounce { e: r } if r >= 1.0 => {
            vec![
                e("first apex", APEX, Better::Toward(1.0)),
                e("most apex", APEX, Better::Toward(1.0)),
                e("last apex", APEX, Better::Toward(1.0)),
            ]
        }
        Scene::Bounce { e: r } => vec![e("first apex", APEX, Better::Toward((r * r) as f64))],
        Scene::Ratio { .. } | Scene::BigOnSmall => vec![
            e("top sank", MOTION, lower),
            e("deepest at end", MOTION, lower),
            e("stands", COUNT, higher),
            e("escaped", COUNT, lower),
            e("at rest from", STEPS_TO, lower),
            e("jitter", MOTION, lower),
        ],
        Scene::Overlap { .. } => vec![
            e("peak speed", MOTION, lower),
            e("separated at", STEPS_TO, lower),
            e("at rest from", STEPS_TO, lower),
            e("top off", TOP_OFF, Better::Toward(0.0)),
            e("escaped", COUNT, lower),
        ],
        Scene::Ladder { .. } => vec![e("slid", SLID, Better::Neither)],
        Scene::Dominoes { .. } => vec![
            e("toppled", COUNT, higher),
            e("in order", COUNT, higher),
            e("wave", MOTION, Better::Neither),
            e("last lean", MOTION, Better::Neither),
            e("at rest from", STEPS_TO, lower),
        ],
        _ => Vec::new(),
    }
}

/// The bullets at each wall and speed, each at the four phases: how many
/// pass through.
fn bullets() -> Vec<(f32, f32, Vec<Scene>)> {
    let mut v = Vec::new();
    for thick in [0.1, 1.0] {
        for speed in behave::BULLET_SPEEDS {
            let phases = behave::BULLET_PHASES.map(|phase| Scene::Bullet { speed, radius: 0.25, thick, phase }).to_vec();
            v.push((thick, speed, phases));
        }
    }
    v
}

/// The behaviour group: every behaviour scene of `:behaviour_test`, the
/// ignored tests' too, and the families on their short grids; with `long`,
/// the families on their long grids (`:behaviour_long_test`).
pub fn behaviour(long: bool) -> Vec<Entry> {
    const G: &str = "behaviour";
    let singles: Vec<Scene> = if long {
        Vec::new()
    } else {
        behave::scenes().into_iter().filter(|s| !matches!(s, Scene::Cards { .. } | Scene::Bullet { .. })).collect()
    };
    let bullets = if long { Vec::new() } else { bullets() };
    let families = family::families(long);
    let mut all = singles.clone();
    all.extend(bullets.iter().flat_map(|(_, _, s)| s.iter().copied()));
    all.extend(families.iter().flat_map(|f| f.scenes.iter().copied()));
    // Each run once, all at once; below they are read from `runs`.
    runs::par(&all, |&s| runs::behaved(s));
    let mut v = Vec::new();
    for s in &singles {
        v.extend(behaviour_of(G, s, &runs::behaved(*s)));
    }
    for (thick, speed, phases) in &bullets {
        let through: f64 = phases.iter().map(|&s| runs::behaved(s).get("through")).sum();
        v.push(Entry::new(G, format!("bullet {speed} 0.25 {thick}"), "through", through, COUNT, Better::Lower));
    }
    for f in &families {
        let runs: Vec<Behaviour> = f.scenes.iter().map(|&s| runs::behaved(s)).collect();
        let (yes, _) = family::share(f, &runs);
        v.push(Entry::new(G, format!("family {}", f.name), "good", yes as f64, family_band(runs.len()), Better::Higher));
    }
    v
}
