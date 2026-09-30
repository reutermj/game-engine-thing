//! SPIKE, not engine code: the measurements behind
//! docs/architecture/parallel-relations.md. Kept as a bench target so its
//! numbers can be taken again; nothing depends on it.
//!
//! Two questions, on the solver's inputs as the step on arrays hands them
//! over (`tests/arrays.rs`, bit for bit the mod's), bodies turning:
//!
//! 1. **Colors kept across steps** against colored afresh each step, as
//!    `solver::solve_across` colors them. Every step's contacts are
//!    captured from the first, then replayed: afresh is `lanes::group`'s
//!    greedy coloring in pair order and the placing into batches that
//!    follows it; kept is Box2D's constraint graph (`constraint_graph.c`):
//!    a contact takes a color when it begins and keeps it while it lasts,
//!    and each body's taken colors are kept too, changed only by contacts
//!    beginning and ending. Timed apart: the walk that finds what began and
//!    ended (which the mod's merge makes anyway), the changes themselves,
//!    and placing from kept colors, on one thread and across threads.
//! 2. **A pass over bodies in pages** against the solver's copy: the same
//!    contact kernel over colored contacts, bodies in one dense array, in
//!    pages of 16 rows (9.5 filled, as spatial pages are) with each
//!    contact's rows found once a step, and found at every touch.
//!
//!     taskset -c 0-7 ./bazel run --config=bench //engine/std/physics2d/compare:colors_spike
//!
//! `REPS` (5) replays, the median of each step's; `THREADS` (8) for
//! placing across threads (`tests/pool.rs`'s kept threads); `ONLY=pile` or
//! `ONLY=pyramid`.

#[allow(dead_code)]
#[path = "colors_spike_arrays.rs"]
mod arrays;
#[allow(dead_code)]
mod ecs;
#[path = "colors_spike_narrow.rs"]
mod narrow;
#[allow(dead_code)]
#[path = "colors_spike_pool.rs"]
mod pool;
#[allow(dead_code)]
mod scene;
#[allow(dead_code)]
mod sim;
// solver.rs with one line patched (the BUILD's `colors_spike_srcs`).
#[allow(dead_code)]
#[path = "colors_spike_solver.rs"]
mod solver;
#[allow(dead_code)]
#[path = "colors_spike_split_impulse.rs"]
mod split_impulse;
#[allow(dead_code)]
mod variants;

use std::cell::{Cell, RefCell};
use std::collections::HashMap;
use std::hint::black_box;
use std::ops::RangeInclusive;
use std::rc::Rc;
use std::sync::Mutex;
use std::time::{Duration, Instant};

use std::sync::Arc;

use engine_ecs::Executor;
pub use sim::{Dyn, Sim};

use scene::Scene;
use solver::{Constraint, PARAMS, Points, SolverBody, Spinning, Wide};

/// Colors handed to the next solve on this thread in place of its own
/// coloring (`colors_spike_solver.rs`, patched by the BUILD's genrule).
pub mod kept_colors {
    use std::cell::RefCell;
    thread_local! {
        static NEXT: RefCell<Option<(Vec<u32>, usize)>> = const { RefCell::new(None) };
    }
    pub fn set(colors: Vec<u32>, n: usize) {
        NEXT.with(|x| *x.borrow_mut() = Some((colors, n)));
    }
    pub fn take() -> Option<(Vec<u32>, usize)> {
        NEXT.with(|x| x.borrow_mut().take())
    }
}

const COLORS: u32 = 64;
const OVERFLOW: u32 = u32::MAX - 1;
const UNSOLVED: u32 = u32::MAX;
const LANES: usize = 4;

/// A step's contacts, in the solver's order (pair order by entity), each
/// contact's identity (its ends' solver indices, lesser first, and which of
/// its equals it is: every static is one body to the solver) sorted, with
/// where it is in that order, and which bodies move.
struct Step {
    pairs: Vec<u64>,
    ids: Vec<(u128, u32)>,
    moves: Vec<bool>,
}

fn ids_of(pairs: &[u64]) -> Vec<(u128, u32)> {
    let mut ids: Vec<(u128, u32)> = pairs
        .iter()
        .enumerate()
        .map(|(k, &p)| {
            let (a, b) = (a_of(p) as u128, b_of(p) as u128);
            (((a.min(b)) << 64) | (a.max(b) << 8), k as u32)
        })
        .collect();
    ids.sort_unstable();
    for k in 1..ids.len() {
        if ids[k].0 >> 8 == ids[k - 1].0 >> 8 {
            ids[k].0 = ids[k - 1].0 + 1;
        }
    }
    ids
}

fn ends(id: u128) -> [usize; 2] {
    [(id >> 64) as usize, ((id >> 8) & 0xffff_ffff) as usize]
}

#[derive(Clone)]
struct Input {
    bodies: Vec<SolverBody>,
    spinning: Vec<Spinning>,
    contacts: Vec<Constraint>,
    points: Vec<Points>,
}

fn a_of(p: u64) -> usize {
    (p >> 32) as usize
}

fn b_of(p: u64) -> usize {
    (p & 0xffff_ffff) as usize
}

fn moves_of(bodies: &[SolverBody], spinning: &[Spinning]) -> Vec<bool> {
    let mut m: Vec<bool> = bodies.iter().map(|b| b.inv_mass > 0.0).collect();
    for s in spinning {
        if s.inv_inertia > 0.0 {
            m[s.body as usize] = true;
        }
    }
    m
}

/// Every step's contacts up to `last`, and the whole input at steps `keep`.
fn capture(scene: &Scene, last: u32, keep: &[u32]) -> (Vec<Step>, Vec<Input>) {
    let steps: Rc<RefCell<Vec<Step>>> = Rc::default();
    let inputs: Rc<RefCell<Vec<Input>>> = Rc::default();
    let n = Rc::new(Cell::new(0u32));
    let (st, inp, c, keep) = (steps.clone(), inputs.clone(), n.clone(), keep.to_vec());
    let solve: variants::Boxed = Box::new(move |bodies, spinning, contacts, points, dt| {
        c.set(c.get() + 1);
        let pairs: Vec<u64> = contacts.iter().map(|k| ((k.a as u64) << 32) | k.b as u64).collect();
        let ids = ids_of(&pairs);
        st.borrow_mut().push(Step { pairs, ids, moves: moves_of(bodies, spinning) });
        if keep.contains(&c.get()) {
            inp.borrow_mut().push(Input {
                bodies: bodies.to_vec(),
                spinning: spinning.to_vec(),
                contacts: contacts.to_vec(),
                points: points.to_vec(),
            });
        }
        solver::solve_points(bodies, spinning, contacts, points, dt);
    });
    let mut flat = ecs::Flat::new(scene, true, solve, "capture");
    flat.step(last);
    drop(flat);
    (Rc::try_unwrap(steps).ok().unwrap().into_inner(), Rc::try_unwrap(inputs).ok().unwrap().into_inner())
}

/// The lowest color free at both ends, and not color 0 for a contact with
/// an end that doesn't move: `lanes::group`'s rule, and Box2D v3's.
#[inline(always)]
fn pick(used: &mut [u64], moves: &[bool], a: usize, b: usize) -> u32 {
    if !moves[a] && !moves[b] {
        return UNSOLVED;
    }
    let at = |e: usize| if moves[e] { used[e] } else { 0 };
    let mut free = !(at(a) | at(b));
    if !moves[a] || !moves[b] {
        free &= !1;
    }
    let k = free.trailing_zeros();
    if k >= COLORS {
        return OVERFLOW;
    }
    for e in [a, b] {
        if moves[e] {
            used[e] |= 1u64 << k;
        }
    }
    k
}

/// Afresh: every contact colored in pair order, as `solve_across` does
/// each step.
fn afresh(step: &Step) -> Vec<u32> {
    let mut used = vec![0u64; step.moves.len()];
    step.pairs.iter().map(|&p| pick(&mut used, &step.moves, a_of(p), b_of(p))).collect()
}

/// Each contact's batch and lane, as `head` counts them and `solve_across`
/// places them: the overflow first, one a batch, then each color's.
fn place(colors: &[u32]) -> Vec<(u32, u32)> {
    let n = colors.iter().filter(|k| **k < COLORS).map(|k| *k as usize + 1).max().unwrap_or(0);
    let mut count = vec![0usize; n];
    let mut overflow = 0;
    for &k in colors {
        match k {
            UNSOLVED => (),
            OVERFLOW => overflow += 1,
            k => count[k as usize] += 1,
        }
    }
    let mut batches = overflow;
    let first: Vec<usize> = count
        .iter()
        .map(|c| {
            let f = batches;
            batches += c.div_ceil(LANES);
            f
        })
        .collect();
    let mut filled = vec![0usize; n];
    let mut over = 0u32;
    colors
        .iter()
        .map(|&k| match k {
            UNSOLVED => (u32::MAX, 0),
            OVERFLOW => {
                over += 1;
                (over - 1, 0)
            }
            k => {
                let j = filled[k as usize];
                filled[k as usize] += 1;
                ((first[k as usize] + j / LANES) as u32, (j % LANES) as u32)
            }
        })
        .collect()
}

/// `place` split over the executor's threads: each chunk counts its
/// colors, the counts are summed in chunk order, and each chunk places its
/// own. The same slots as `place` at any thread count.
fn place_across(colors: &[u32], exec: &dyn Executor) -> Vec<(u32, u32)> {
    const O: usize = COLORS as usize;
    let t = exec.threads();
    let cut = |k: usize| colors.len() * k / t;
    let counts: Vec<Mutex<[usize; O + 1]>> = (0..t).map(|_| Mutex::new([0; O + 1])).collect();
    exec.run(t, &|k| {
        let mut c = counts[k].try_lock().expect("a chunk's own");
        for &x in &colors[cut(k)..cut(k + 1)] {
            match x {
                UNSOLVED => (),
                OVERFLOW => c[O] += 1,
                x => c[x as usize] += 1,
            }
        }
    });
    let per: Vec<[usize; O + 1]> = counts.into_iter().map(|m| m.into_inner().unwrap()).collect();
    let mut total = [0usize; O + 1];
    for c in &per {
        for (t, n) in total.iter_mut().zip(c) {
            *t += n;
        }
    }
    let mut first = [0usize; O];
    let mut batches = total[O];
    for (f, n) in first.iter_mut().zip(&total) {
        *f = batches;
        batches += n.div_ceil(LANES);
    }
    let mut start: Vec<[usize; O + 1]> = Vec::with_capacity(t);
    let mut run = [0usize; O + 1];
    for c in &per {
        start.push(run);
        for (r, n) in run.iter_mut().zip(c) {
            *r += n;
        }
    }
    let mut slots = vec![(u32::MAX, 0u32); colors.len()];
    let mut parts = Vec::with_capacity(t);
    let mut rest = slots.as_mut_slice();
    for k in 0..t {
        let (p, r) = std::mem::take(&mut rest).split_at_mut(cut(k + 1) - cut(k));
        rest = r;
        parts.push(Mutex::new(p));
    }
    exec.run(t, &|k| {
        let mut out = parts[k].try_lock().expect("a chunk's own");
        let mut at = start[k];
        for (s, &x) in out.iter_mut().zip(&colors[cut(k)..cut(k + 1)]) {
            *s = match x {
                UNSOLVED => (u32::MAX, 0),
                OVERFLOW => {
                    at[O] += 1;
                    ((at[O] - 1) as u32, 0)
                }
                x => {
                    let j = at[x as usize];
                    at[x as usize] += 1;
                    ((first[x as usize] + j / LANES) as u32, (j % LANES) as u32)
                }
            };
        }
    });
    drop(parts);
    slots
}

/// Colors kept across steps (Box2D's constraint graph): each contact's,
/// and each body's taken, changed only as contacts begin and end.
#[derive(Default)]
struct Kept {
    used: Vec<u64>,
    /// Last step's contacts by identity, sorted, and each one's color.
    ids: Vec<u128>,
    colors: Vec<u32>,
    moves: Vec<bool>,
}

/// What the walk found: the ended contacts with their colors, the begun
/// ones by position, and every persisting contact's color.
struct Diff {
    ended: Vec<(u128, u32)>,
    began: Vec<usize>,
    colors: Vec<u32>,
}

impl Kept {
    /// Last step's contacts against this step's, both in pair order: the
    /// walk the mod's merge makes anyway. A contact at a body whose moving
    /// changed ends and begins again.
    fn diff(&self, step: &Step) -> Diff {
        let same = self.moves == step.moves;
        let changed = |id: u128| {
            let [a, b] = ends(id);
            !same && (self.moves.get(a) != step.moves.get(a) || self.moves.get(b) != step.moves.get(b))
        };
        let mut d = Diff { ended: Vec::new(), began: Vec::new(), colors: vec![UNSOLVED; step.pairs.len()] };
        let (old, mut i) = (&self.ids, 0);
        for &(id, j) in &step.ids {
            let j = j as usize;
            while i < old.len() && old[i] < id {
                d.ended.push((old[i], self.colors[i]));
                i += 1;
            }
            if i < old.len() && old[i] == id {
                if changed(id) {
                    d.ended.push((id, self.colors[i]));
                    d.began.push(j);
                } else {
                    d.colors[j] = self.colors[i];
                }
                i += 1;
            } else {
                d.began.push(j);
            }
        }
        d.ended.extend(old[i..].iter().copied().zip(self.colors[i..].iter().copied()));
        // Begun in the solver's order, as they'd be colored as they begin.
        d.began.sort_unstable();
        d
    }

    /// The ended free their colors at the ends they took them at; then the
    /// begun take the lowest free, in pair order.
    fn change(&mut self, step: &Step, d: &mut Diff) {
        if self.used.len() < step.moves.len() {
            self.used.resize(step.moves.len(), 0);
        }
        for &(id, k) in &d.ended {
            if k < COLORS {
                for e in ends(id) {
                    if self.moves.get(e) == Some(&true) {
                        self.used[e] &= !(1u64 << k);
                    }
                }
            }
        }
        for &j in &d.began {
            let p = step.pairs[j];
            d.colors[j] = pick(&mut self.used, &step.moves, a_of(p), b_of(p));
        }
    }

    fn keep(&mut self, step: &Step, d: Diff) {
        self.ids = step.ids.iter().map(|x| x.0).collect();
        self.colors = step.ids.iter().map(|&(_, j)| d.colors[j as usize]).collect();
        self.moves.clone_from(&step.moves);
    }
}

/// Panics unless no two contacts of a color share a body that moves.
fn check(step: &Step, colors: &[u32]) {
    let mut seen = vec![0u64; step.moves.len()];
    for (&p, &k) in step.pairs.iter().zip(colors) {
        if k >= COLORS {
            continue;
        }
        for e in [a_of(p), b_of(p)] {
            if step.moves[e] {
                assert!(seen[e] & (1u64 << k) == 0, "color {k} has body {e} twice");
                seen[e] |= 1u64 << k;
            }
        }
    }
}

fn sizes(colors: &[u32]) -> (Vec<usize>, usize) {
    let mut s = vec![0usize; COLORS as usize];
    let mut over = 0;
    for &k in colors {
        match k {
            UNSOLVED => (),
            OVERFLOW => over += 1,
            k => s[k as usize] += 1,
        }
    }
    while s.last() == Some(&0) {
        s.pop();
    }
    (s, over)
}

fn median(mut v: Vec<f64>) -> f64 {
    v.sort_by(f64::total_cmp);
    v[v.len() / 2]
}

fn us(t: Instant) -> f64 {
    t.elapsed().as_secs_f64() * 1e6
}

const TIMED: [&str; 6] = [
    "afresh: coloring",
    "afresh: placing",
    "kept: the walk (the merge's)",
    "kept: the changes",
    "kept: placing, one thread",
    "kept: placing across threads",
];

/// The merge's walk, last step's contacts against this step's found, over
/// contacts kept in pair order (one run) and kept in color order, each
/// color in pair order (a run a color, merged by pair as the walk goes):
/// µs each, the median of 9.
fn merge_walks(step: &Step, colors: &[u32]) -> (f64, f64) {
    let found: Vec<u128> = step.ids.iter().map(|x| x.0).collect();
    let n = colors.iter().filter(|k| **k < COLORS).map(|k| *k as usize + 1).max().unwrap_or(0);
    let mut runs: Vec<Vec<u128>> = vec![Vec::new(); n + 1];
    for &(id, j) in &step.ids {
        let k = colors[j as usize];
        runs[if k < COLORS { k as usize } else { n }].push(id);
    }
    let one = |stored: &[u128]| {
        let (mut i, mut hits) = (0, 0);
        for &s in stored {
            while i < found.len() && found[i] < s {
                i += 1;
            }
            if i < found.len() && found[i] == s {
                hits += 1;
                i += 1;
            }
        }
        hits
    };
    let many = |runs: &[Vec<u128>]| {
        let mut heads = vec![0usize; runs.len()];
        let (mut i, mut hits) = (0, 0);
        loop {
            let mut best: Option<(u128, usize)> = None;
            for (r, run) in runs.iter().enumerate() {
                if let Some(&x) = run.get(heads[r])
                    && best.is_none_or(|b| x < b.0)
                {
                    best = Some((x, r));
                }
            }
            let Some((s, r)) = best else { break };
            heads[r] += 1;
            while i < found.len() && found[i] < s {
                i += 1;
            }
            if i < found.len() && found[i] == s {
                hits += 1;
                i += 1;
            }
        }
        hits
    };
    let time = |f: &dyn Fn() -> usize| {
        let mut t = Vec::new();
        for _ in 0..9 {
            let at = Instant::now();
            black_box(f());
            t.push(us(at));
        }
        median(t)
    };
    assert_eq!(one(&found), many(&runs), "both walks find every contact");
    (time(&|| one(&found)), time(&|| many(&runs)))
}

/// A window step's facts: afresh's color sizes and overflow, kept's, how
/// many began and ended, and how many persisting contacts afresh recolored.
type Stats = (Vec<usize>, usize, Vec<usize>, usize, usize, usize, usize);

/// Replays every step `reps` times; each window step's times, the median
/// of the replays', averaged over the window.
/// Returns, at each window's middle step, the kept colors and the repacked
/// ones, in the solver's order.
fn colors(
    name: &str,
    steps: &[Step],
    windows: &[(&str, RangeInclusive<u32>)],
    reps: usize,
    exec: &dyn Executor,
) -> HashMap<u32, (Vec<u32>, Vec<u32>)> {
    let middles: Vec<u32> = windows.iter().map(|(_, w)| (w.start() + w.end()) / 2).collect();
    let mut at_middles: HashMap<u32, (Vec<u32>, Vec<u32>)> = HashMap::new();
    let mut times: HashMap<u32, Vec<[f64; 6]>> = HashMap::new();
    let mut stats: HashMap<u32, Stats> = HashMap::new();
    // Kept, but recolored afresh every 60 steps: what a periodic repacking
    // would leave. Only its colors, from the first replay.
    let mut packed_sizes: HashMap<u32, Vec<usize>> = HashMap::new();
    let mut walks: HashMap<u32, (f64, f64)> = HashMap::new();
    for rep in 0..reps {
        let mut kept = Kept::default();
        let mut packed = Kept::default();
        let mut last_afresh: Option<(Vec<u128>, Vec<u32>)> = None;
        for (s, step) in steps.iter().enumerate() {
            let n = s as u32 + 1;
            let timed = windows.iter().any(|(_, w)| w.contains(&n));
            let t = Instant::now();
            let mut d = kept.diff(step);
            let t_walk = us(t);
            let t = Instant::now();
            kept.change(step, &mut d);
            let t_change = us(t);
            if timed {
                let t = Instant::now();
                let fresh = afresh(step);
                let t_color = us(t);
                let t = Instant::now();
                let slots = black_box(place(&fresh));
                let t_place = us(t);
                let t = Instant::now();
                let kept_slots = black_box(place(&d.colors));
                let t_kept_place = us(t);
                let t = Instant::now();
                let across = black_box(place_across(&d.colors, exec));
                let t_across = us(t);
                assert!(across == kept_slots, "placing across threads is placing on one");
                drop(slots);
                times.entry(n).or_default().push([t_color, t_place, t_walk, t_change, t_kept_place, t_across]);
                if rep == 0 {
                    check(step, &fresh);
                    check(step, &d.colors);
                    let (fs, fo) = sizes(&fresh);
                    let (ks, ko) = sizes(&d.colors);
                    // Persisting contacts whose afresh color moved since the last step.
                    let mut moved = 0;
                    if let Some((ids, cols)) = &last_afresh {
                        let mut i = 0;
                        for &(id, j) in &step.ids {
                            while i < ids.len() && ids[i] < id {
                                i += 1;
                            }
                            if i < ids.len() && ids[i] == id && cols[i] != fresh[j as usize] {
                                moved += 1;
                            }
                        }
                    }
                    stats.insert(n, (fs, fo, ks, ko, d.began.len(), d.ended.len(), moved));
                    let by_id = step.ids.iter().map(|&(_, j)| fresh[j as usize]).collect();
                    last_afresh = Some((step.ids.iter().map(|x| x.0).collect(), by_id));
                }
            } else if rep == 0 {
                last_afresh = None;
            }
            if rep == 0 {
                let mut pd = packed.diff(step);
                packed.change(step, &mut pd);
                if n.is_multiple_of(60) {
                    pd.colors = afresh(step);
                    packed.used = vec![0; step.moves.len()];
                    for (&p, &k) in step.pairs.iter().zip(&pd.colors) {
                        for e in [a_of(p), b_of(p)] {
                            if k < COLORS && step.moves[e] {
                                packed.used[e] |= 1u64 << k;
                            }
                        }
                    }
                }
                if timed {
                    check(step, &pd.colors);
                    packed_sizes.insert(n, sizes(&pd.colors).0);
                }
                if middles.contains(&n) {
                    at_middles.insert(n, (d.colors.clone(), pd.colors.clone()));
                }
                if windows.iter().any(|(_, w)| *w.end() == n) {
                    walks.insert(n, merge_walks(step, &d.colors));
                }
                packed.keep(step, pd);
            }
            kept.keep(step, d);
        }
    }
    for (label, w) in windows {
        let n = w.clone().count() as f64;
        let contacts = w.clone().map(|s| steps[s as usize - 1].pairs.len()).sum::<usize>() as f64 / n;
        println!("\n### {name}, {label} (steps {w:?}): {contacts:.0} contacts\n");
        println!("| stage | µs a step (median of {reps}, mean over the window) |");
        println!("|---|---|");
        for (m, what) in TIMED.iter().enumerate() {
            let v = w.clone().map(|s| median(times[&s].iter().map(|t| t[m]).collect())).sum::<f64>() / n;
            println!("| {what} | {v:.1} |");
        }
        let mean = |f: &dyn Fn(&Stats) -> f64| w.clone().map(|s| f(&stats[&s])).sum::<f64>() / n;
        let most = |f: &dyn Fn(&Stats) -> usize| w.clone().map(|s| f(&stats[&s])).max().unwrap();
        println!(
            "\nbegan {:.1}, ended {:.1} a step; persisting contacts whose afresh color moved {:.1} a step (the window's first step not counted)",
            mean(&|x| x.4 as f64),
            mean(&|x| x.5 as f64),
            mean(&|x| x.6 as f64)
        );
        println!(
            "colors: afresh at most {}, kept at most {}; overflow: afresh at most {}, kept at most {}",
            most(&|x| x.0.len()),
            most(&|x| x.2.len()),
            most(&|x| x.1),
            most(&|x| x.3)
        );
        let end = stats[w.end()].clone();
        println!("sizes at step {}: afresh {:?}, kept {:?}", w.end(), end.0, end.2);
        let most_packed = w.clone().map(|s| packed_sizes[&s].len()).max().unwrap();
        println!("kept, recolored afresh every 60 steps: at most {most_packed} colors; at step {}: {:?}", w.end(), packed_sizes[w.end()]);
        let (one, many) = walks[w.end()];
        println!("the merge's walk at step {}: pair order {one:.1} µs, color order ({} runs) {many:.1} µs", w.end(), end.2.len());
    }
    at_middles
}

/// The solve across the pool's threads on colors of its own, on kept
/// colors and on repacked ones (handed over, so its coloring is skipped;
/// placing stays), and on one thread: µs, the median of 21, the variants
/// taken in turn, after the threads are kept busy 300 ms (the cores
/// clocked up: docs/lore/idle-cores-run-a-parallel-solve-at-half-speed.md).
fn solves(name: &str, input: &Input, (kept, packed): &(Vec<u32>, Vec<u32>), gang: &engine_ecs::Workers) {
    let n = |c: &[u32]| c.iter().filter(|k| **k < COLORS).map(|k| *k as usize + 1).max().unwrap_or(0);
    let once = |colors: Option<&Vec<u32>>, across: bool| {
        let mut i = input.clone();
        if let Some(c) = colors {
            kept_colors::set(c.clone(), n(c));
        }
        let at = Instant::now();
        if across {
            solver::solve_across(&PARAMS, (&mut i.bodies, &mut i.spinning), &mut i.contacts, &mut i.points, arrays::DT, gang);
        } else {
            solver::solve_with(&PARAMS, (&mut i.bodies, &mut i.spinning), &mut i.contacts, &mut i.points, arrays::DT);
        }
        let t = us(at);
        assert!(kept_colors::take().is_none(), "the solve took the colors");
        t
    };
    let variants: [Option<&Vec<u32>>; 3] = [None, Some(kept), Some(packed)];
    let mut t = vec![Vec::new(); 6];
    let warm = Instant::now();
    while warm.elapsed() < Duration::from_millis(300) {
        engine_ecs::Workers::run(gang, gang.threads(), |_| {
            let t = Instant::now();
            while t.elapsed() < Duration::from_millis(1) {
                std::hint::spin_loop();
            }
        });
    }
    for _ in 0..21 {
        for (v, c) in variants.iter().enumerate() {
            t[v].push(once(*c, true));
        }
    }
    for _ in 0..7 {
        for (v, c) in variants.iter().enumerate() {
            t[3 + v].push(once(*c, false));
        }
    }
    let m: Vec<f64> = t.into_iter().map(median).collect();
    // Any valid coloring is solved bit for bit alike on one thread and
    // across: the kept colors as much as the solver's own.
    for c in variants {
        let bits = |across: bool| {
            let mut i = input.clone();
            if let Some(c) = c {
                kept_colors::set(c.clone(), n(c));
            }
            if across {
                solver::solve_across(&PARAMS, (&mut i.bodies, &mut i.spinning), &mut i.contacts, &mut i.points, arrays::DT, gang);
            } else {
                solver::solve_with(&PARAMS, (&mut i.bodies, &mut i.spinning), &mut i.contacts, &mut i.points, arrays::DT);
            }
            let mut out: Vec<u32> = i.bodies.iter().flat_map(|b| [b.v.x, b.v.y, b.moved.x, b.moved.y]).map(f32::to_bits).collect();
            out.extend(i.spinning.iter().flat_map(|s| [s.w, s.angle]).map(f32::to_bits));
            out.extend(i.contacts.iter().flat_map(|c| [c.jn, c.jt]).map(f32::to_bits));
            out
        };
        assert!(bits(true) == bits(false), "a coloring solved across threads is its solve on one, bit for bit");
    }
    println!("\n### {name}: the solve, {} threads, µs (median of 21, one thread of 7)\n", gang.threads());
    println!("| colors | across threads | one thread |");
    println!("|---|---|---|");
    println!("| its own ({} colors) | {:.0} | {:.0} |", n(&afresh_of(input)), m[0], m[3]);
    println!("| kept, handed over ({} colors) | {:.0} | {:.0} |", n(kept), m[1], m[4]);
    println!("| kept and repacked every 60 steps, handed over ({} colors) | {:.0} | {:.0} |", n(packed), m[2], m[5]);
}

/// The solver's own coloring of an input, for its count of colors.
fn afresh_of(i: &Input) -> Vec<u32> {
    let pairs: Vec<u64> = i.contacts.iter().map(|k| ((k.a as u64) << 32) | k.b as u64).collect();
    afresh(&Step { ids: Vec::new(), pairs, moves: moves_of(&i.bodies, &i.spinning) })
}

/// A body's velocity, as the solver's states hold it.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct St {
    vx: f32,
    vy: f32,
    w: f32,
    pad: f32,
}

/// A contact reduced to one point's normal: the kernel's inputs, and
/// where its ends are (by body, or by page row).
#[derive(Clone, Copy)]
struct Row {
    a: u32,
    b: u32,
    nx: f32,
    ny: f32,
    rax: f32,
    ray: f32,
    rbx: f32,
    rby: f32,
    ma: f32,
    mb: f32,
    ia: f32,
    ib: f32,
    mass: f32,
    bias: f32,
    j: f32,
}

#[inline(always)]
fn kernel(r: &mut Row, va: St, vb: St) -> (St, St) {
    let dvx = vb.vx - vb.w * r.rby - (va.vx - va.w * r.ray);
    let dvy = vb.vy + vb.w * r.rbx - (va.vy + va.w * r.rax);
    let vn = dvx * r.nx + dvy * r.ny;
    let new = (r.j - r.mass * (vn + r.bias)).max(0.0);
    let d = new - r.j;
    r.j = new;
    let (px, py) = (d * r.nx, d * r.ny);
    let a = St { vx: va.vx - r.ma * px, vy: va.vy - r.ma * py, w: va.w - r.ia * (r.rax * py - r.ray * px), pad: 0.0 };
    let b = St { vx: vb.vx + r.mb * px, vy: vb.vy + r.mb * py, w: vb.w + r.ib * (r.rbx * py - r.rby * px), pad: 0.0 };
    (a, b)
}

const PAGE: usize = 16;
const PASSES: usize = 20;

/// Bodies in pages of `PAGE` rows, filled 9 and 10 in turn (9.5, as a
/// spatial table's are on average), each page its own allocation, as the
/// world's are; `at` is each body's row, `page * PAGE + row`.
// Boxed: each page its own allocation, as the world's are, which is the
// point.
#[allow(clippy::vec_box)]
struct Paged {
    pages: Vec<Box<[St; PAGE]>>,
    at: Vec<u32>,
}

impl Paged {
    fn of(s: &[St]) -> Paged {
        let (mut pages, mut at): (Vec<Box<[St; PAGE]>>, Vec<u32>) = (Vec::new(), Vec::with_capacity(s.len()));
        let (mut row, mut fill) = (0, 0);
        for x in s {
            if row == fill {
                pages.push(Box::new([St::default(); PAGE]));
                (row, fill) = (0, 9 + pages.len() % 2);
            }
            let p = pages.len() - 1;
            pages[p][row] = *x;
            at.push((p * PAGE + row) as u32);
            row += 1;
        }
        Paged { pages, at }
    }

    #[inline(always)]
    fn get(&self, loc: u32) -> St {
        self.pages[loc as usize / PAGE][loc as usize % PAGE]
    }

    #[inline(always)]
    fn set(&mut self, loc: u32, v: St) {
        self.pages[loc as usize / PAGE][loc as usize % PAGE] = v;
    }
}

/// The kernel's rows for an input, in colored order (the overflow first,
/// as the solver's batches), and the bodies' states.
fn rows_of(i: &Input) -> (Vec<Row>, Vec<St>) {
    let mut inertia = vec![0.0f32; i.bodies.len()];
    let mut s: Vec<St> = i.bodies.iter().map(|b| St { vx: b.v.x, vy: b.v.y, ..St::default() }).collect();
    for sp in &i.spinning {
        inertia[sp.body as usize] = sp.inv_inertia;
        s[sp.body as usize].w = sp.w;
    }
    let order = solver::order(Wide::Colored(4), &i.bodies, &i.spinning, &i.contacts);
    let rows = order
        .iter()
        .map(|&k| {
            let c = &i.contacts[k];
            let (a, b) = (c.a as usize, c.b as usize);
            // Arms along the normal: the kernel's arithmetic, not a real
            // contact's points.
            let (rax, ray) = (0.25 * c.normal.x, 0.25 * c.normal.y);
            let (rbx, rby) = (-rax, -ray);
            let (ma, mb, ia, ib) = (i.bodies[a].inv_mass, i.bodies[b].inv_mass, inertia[a], inertia[b]);
            let ca = rax * c.normal.y - ray * c.normal.x;
            let cb = rbx * c.normal.y - rby * c.normal.x;
            let k = ma + mb + ia * ca * ca + ib * cb * cb;
            Row {
                a: a as u32,
                b: b as u32,
                nx: c.normal.x,
                ny: c.normal.y,
                rax,
                ray,
                rbx,
                rby,
                ma,
                mb,
                ia,
                ib,
                mass: if k > 0.0 { 1.0 / k } else { 0.0 },
                bias: c.depth.min(0.0) * 6.0,
                j: 0.0,
            }
        })
        .collect();
    (rows, s)
}

/// The median of `reps` runs of `f` (at least 9), and its last result.
fn time(reps: usize, f: &mut dyn FnMut() -> Vec<St>) -> (f64, Vec<St>) {
    let mut t = Vec::new();
    let mut out = Vec::new();
    for _ in 0..reps.max(9) {
        let at = Instant::now();
        out = f();
        t.push(us(at));
    }
    (median(t), out)
}

/// The same passes over bodies stored three ways, each checked to end bit
/// for bit as the copy does, and what the copy costs.
fn layout(name: &str, input: &Input, reps: usize) {
    let (rows, s) = rows_of(input);
    let base = Paged::of(&s);
    // The copy: gathered from the pages into one array, solved there,
    // scattered back.
    let (gather, _) = time(reps, &mut || base.at.iter().map(|&l| base.get(l)).collect());
    let (dense, want) = time(reps, &mut || {
        let mut st = s.clone();
        let mut rs = rows.clone();
        for _ in 0..PASSES {
            for r in rs.iter_mut() {
                let (a, b) = kernel(r, st[r.a as usize], st[r.b as usize]);
                st[r.a as usize] = a;
                st[r.b as usize] = b;
            }
        }
        st
    });
    let mut back = Paged::of(&s);
    let at = back.at.clone();
    let (scatter, _) = time(reps, &mut || {
        for (&l, x) in at.iter().zip(&want) {
            back.set(l, *x);
        }
        Vec::new()
    });
    // In the pages, each contact's rows found once, at the step's start.
    let (resolve, _) = time(reps, &mut || {
        let rs: Vec<Row> = rows.iter().map(|r| Row { a: base.at[r.a as usize], b: base.at[r.b as usize], ..*r }).collect();
        black_box(rs);
        Vec::new()
    });
    let (make, _) = time(reps, &mut || {
        let p = Paged::of(&s);
        black_box(p.pages.len());
        Vec::new()
    });
    let (paged, got) = time(reps, &mut || {
        let mut p = Paged::of(&s);
        let mut rs: Vec<Row> = rows.iter().map(|r| Row { a: p.at[r.a as usize], b: p.at[r.b as usize], ..*r }).collect();
        for _ in 0..PASSES {
            for r in rs.iter_mut() {
                let (a, b) = kernel(r, p.get(r.a), p.get(r.b));
                p.set(r.a, a);
                p.set(r.b, b);
            }
        }
        p.at.iter().map(|&l| p.get(l)).collect()
    });
    assert!(got == want, "the pages' pass is the copy's");
    // In the pages, each end's row looked up at every touch, as by entity.
    let (touch, got) = time(reps, &mut || {
        let mut p = Paged::of(&s);
        let at = p.at.clone();
        let mut rs = rows.clone();
        for _ in 0..PASSES {
            for r in rs.iter_mut() {
                let (a, b) = kernel(r, p.get(at[r.a as usize]), p.get(at[r.b as usize]));
                p.set(at[r.a as usize], a);
                p.set(at[r.b as usize], b);
            }
        }
        p.at.iter().map(|&l| p.get(l)).collect()
    });
    assert!(got == want, "the looked-up pass is the copy's");
    println!("\n### {name}: {} contacts, {} bodies, {PASSES} passes, one thread\n", rows.len(), s.len());
    println!("| bodies | µs a solve (median) |");
    println!("|---|---|");
    println!("| copied: the passes | {dense:.0} |");
    println!("| copied: gathering from pages, scattering back | {gather:.1} + {scatter:.1} |");
    println!("| copied: all | {:.0} |", dense + gather + scatter);
    println!("| in pages, rows found once a step (finding them: {resolve:.1}) | {:.0} |", paged - make);
    println!("| in pages, rows looked up at every touch | {:.0} |", touch - make);
}

fn main() {
    let reps: usize = std::env::var("REPS").ok().and_then(|r| r.parse().ok()).unwrap_or(5);
    let threads: usize = std::env::var("THREADS").ok().and_then(|r| r.parse().ok()).unwrap_or(8);
    let only = std::env::var("ONLY").unwrap_or_default();
    let exec = Arc::new(pool::Pool::new(threads));
    let gang = engine_ecs::Workers::new(Some(exec.clone() as Arc<dyn Executor>));
    let warm = Instant::now();
    while warm.elapsed() < Duration::from_millis(300) {
        exec.run(threads, &|_| {
            let t = Instant::now();
            while t.elapsed() < Duration::from_millis(1) {
                std::hint::spin_loop();
            }
        });
    }
    let pile = Scene::Pile { n: 10000, width: 401.0, stagger: true };
    let pyramid = Scene::Pyramid { base: 100 };
    type Case<'a> = (&'a str, Scene, u32, Vec<(&'a str, RangeInclusive<u32>)>);
    let cases: Vec<Case> = vec![
        ("pile 10 000, turning", pile, 460, vec![("falling", 2..=61), ("settled", 401..=460)]),
        ("pyramid 5050, turning", pyramid, 660, vec![("standing", 601..=660)]),
    ];
    for (name, scene, last, windows) in cases.iter().filter(|c| only.is_empty() || c.0.contains(only.as_str())) {
        // The inputs the layout is timed on, and the orders checked on: each
        // window's middle.
        let keep: Vec<u32> = windows.iter().map(|(_, w)| (w.start() + w.end()) / 2).collect();
        let (steps, inputs) = capture(scene, *last, &keep);
        // Afresh here is the solver's own coloring: the same order.
        for (input, &at) in inputs.iter().zip(&keep) {
            let fresh = afresh(&steps[at as usize - 1]);
            let rank = |k: u32| match k {
                OVERFLOW => 0,
                UNSOLVED => u32::MAX,
                k => k + 1,
            };
            let mut mine: Vec<usize> = (0..fresh.len()).collect();
            mine.sort_by_key(|&i| rank(fresh[i]));
            assert!(
                mine == solver::order(Wide::Colored(4), &input.bodies, &input.spinning, &input.contacts),
                "afresh is the solver's coloring"
            );
        }
        let middles = colors(name, &steps, windows, reps, &*exec);
        for ((input, (label, _)), at) in inputs.iter().zip(windows).zip(&keep) {
            solves(&format!("{name}, {label}, step {at}"), input, &middles[at], &gang);
            layout(&format!("{name}, {label}"), input, reps);
        }
    }
}
