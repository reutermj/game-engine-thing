//! SPIKE, not engine code: the second user measured for
//! docs/architecture/flows-spike.md. Transform propagation down `ChildOf`
//! as a pipeline of flows, against the two ways hierarchy_spike.rs
//! measured for working-sets.md: each node's `Global` is its parent's
//! composed with its own `Local`, a root's its `Local`.
//!
//! - **hand, by level**: what a game writes today (hierarchy_spike.rs's):
//!   globals in a vector by entity index, the children walked once a
//!   level.
//! - **hand, sorted each frame**: physics's shape, in one system: gather,
//!   index, sort by depth, propagate, write back, fresh vectors.
//! - **flows**: the same shape as four systems and three flows, their
//!   allocations kept:
//!
//!   ```text
//!   gather     world -> Make<Nodes>        (rows, depth order, parents)
//!   propagate  See<Nodes> -> Make<Globals> (a level at a time, each level
//!                                           across the threads)
//!   extent     See<Globals> -> Make<Extent> (the forest's box: a reduction
//!                                           in a fixed order)
//!   scatter    See<Nodes>, Take<Globals> -> world
//!   ```
//!
//!   `extent` is the second reader a flow makes free: a system no other
//!   knows about, seeing what `propagate` made.
//! - **flows, in walk order**: the same pipeline with the idiom's order on
//!   the copy: rows as walked, each parent found through the index, as
//!   many passes as it takes. What the flows cost apart from the sort.
//!
//! Every frame each way is checked bit for bit against a recursive
//! reference, and the extent against one computed in order.
//!
//!     taskset -c 0-7 ./bazel run --config=bench //engine/ecs:flows_hierarchy_spike
//!
//! `REPS` (41), the median of each; `THREADS` (8) for the flows' parallel
//! levels on kept threads.

#[allow(dead_code)]
#[path = "../flows_hierarchy_pool.rs"]
mod pool;

use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use engine_ecs::harness::{Cx, IntoSystem, Schedule};
use engine_ecs::{ChildOf, Entity, Executor, Query, Spawner, With, Without, Workers, World, component};
use flows::{Make, See, Take, flow};

component! {
    /// A transform relative to the parent: a rotation as cosine and sine.
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Local: "spike::Local" { pub x: f32, pub y: f32, pub c: f32, pub s: f32 }
}

component! {
    #[derive(Debug, Default, PartialEq, Copy)]
    pub struct Global: "spike::Global" { pub x: f32, pub y: f32, pub c: f32, pub s: f32 }
}

#[inline]
fn compose(p: &Global, l: &Local) -> Global {
    Global { x: p.x + p.c * l.x - p.s * l.y, y: p.y + p.s * l.x + p.c * l.y, c: p.c * l.c - p.s * l.s, s: p.s * l.c + p.c * l.s }
}

#[inline]
fn root(l: &Local) -> Global {
    Global { x: l.x, y: l.y, c: l.c, s: l.s }
}

/// `Slots` as physics has it, refilled in place: entities to places, by
/// entity index.
#[derive(Clone, Default)]
struct Slots(Vec<(u32, u32)>);

impl Slots {
    fn fill(&mut self, entities: impl Iterator<Item = Entity> + Clone) {
        let len = entities.clone().map(|e| e.index as usize + 1).max().unwrap_or(0);
        self.0.clear();
        self.0.resize(len, (u32::MAX, u32::MAX));
        for (k, e) in entities.enumerate() {
            self.0[e.index as usize] = (e.generation, k as u32);
        }
    }

    #[inline]
    fn get(&self, e: Entity) -> Option<u32> {
        self.0.get(e.index as usize).filter(|(g, k)| *g == e.generation && *k != u32::MAX).map(|(_, k)| *k)
    }
}

impl flows::Recycle for Slots {
    fn recycle(&mut self) {
        self.0.clear();
    }
}

const NONE: u32 = u32::MAX;

flow! {
    /// The nodes in depth order: each one's entity, local and parent's
    /// place (`NONE` for a root), each level's first place, and the index
    /// by entity; the rest is the sort's scratch, kept for its allocations.
    pub struct Nodes: "spike::Nodes" {
        pub entities: Vec<Entity>,
        pub local: Vec<Local>,
        pub parent: Vec<u32>,
        pub levels: Vec<usize>,
        index: Slots,
        rows: Vec<(Entity, Entity)>,
        locals: Vec<Local>,
        walk: Slots,
        depth: Vec<u32>,
        stack: Vec<usize>,
        placed: Vec<u32>,
    }
}

flow! {
    pub struct Globals: "spike::Globals" {
        pub global: Vec<Global>,
    }
}

flow! {
    /// The box around every node's position.
    pub struct Extent: "spike::Extent" {
        pub bounds: Option<[f32; 4]>,
    }
}

/// `hierarchy_spike.rs`'s `order`, into the flow's kept vectors: roots,
/// then each level's nodes, siblings in walk order.
fn order_into(n: &mut Nodes) {
    let Nodes { entities, local, parent, levels, index, rows, locals, walk, depth, stack, placed } = n;
    walk.fill(rows.iter().map(|r| r.0));
    depth.clear();
    depth.resize(rows.len(), u32::MAX);
    for i in 0..rows.len() {
        let mut at = i;
        while depth[at] == u32::MAX {
            let (e, p) = rows[at];
            if p == e {
                depth[at] = 0;
                break;
            }
            stack.push(at);
            at = walk.get(p).expect("a parent among the nodes") as usize;
        }
        while let Some(j) = stack.pop() {
            depth[j] = depth[walk.get(rows[j].1).unwrap() as usize] + 1;
        }
    }
    let count = depth.iter().max().map_or(0, |d| *d as usize + 1);
    levels.clear();
    levels.resize(count + 1, 0);
    depth.iter().for_each(|&d| levels[d as usize + 1] += 1);
    for l in 0..count {
        levels[l + 1] += levels[l];
    }
    // `levels[l]` is now level `l`'s first place; `placed` takes each
    // node's from a running copy.
    let mut next: Vec<usize> = levels[..count].to_vec();
    placed.clear();
    placed.extend(depth.iter().map(|&d| {
        let k = next[d as usize];
        next[d as usize] += 1;
        k as u32
    }));
    entities.clear();
    entities.resize(rows.len(), Entity { index: 0, generation: 0 });
    local.clear();
    local.resize(rows.len(), Local::default());
    parent.clear();
    parent.resize(rows.len(), NONE);
    for (i, &(e, p)) in rows.iter().enumerate() {
        let k = placed[i] as usize;
        entities[k] = e;
        local[k] = locals[i];
        if p != e {
            parent[k] = placed[walk.get(p).unwrap() as usize];
        }
    }
    index.fill(entities.iter().copied());
}

/// Each level's globals from the levels before: across the threads, a
/// level's slice split in chunks, the levels before read-only.
fn propagate_levels(workers: &Workers, n: &Nodes, global: &mut Vec<Global>) {
    global.clear();
    global.resize(n.entities.len(), Global::default());
    for l in 0..n.levels.len() - 1 {
        let (from, to) = (n.levels[l], n.levels[l + 1]);
        let (done, level) = global.split_at_mut(from);
        let done = &*done;
        flows::par_for_each_mut(workers, &mut level[..to - from], 512, |i, g| {
            let k = from + i;
            *g = match n.parent[k] {
                NONE => root(&n.local[k]),
                p => compose(&done[p as usize], &n.local[k]),
            };
        });
    }
}

/// Nodes in walk order, roots first (`levels` is `[0, roots]`), each
/// one's parent's place found through the index: the idiom's order on the
/// copy, propagated in as many passes as it takes for every parent to be
/// done before its children. One, where `ChildOf`'s order is topological.
fn gather_walk(roots: &mut Query<&Local, Without<ChildOf>>, children: &mut Query<(&ChildOf, &Local)>, n: &mut Nodes) {
    let Nodes { entities, local, parent, levels, index, rows, .. } = n;
    roots.for_each_page(|page, l| {
        entities.extend_from_slice(page.entities());
        local.extend(page.rows().map(|r| l[r]));
    });
    let tops = entities.len();
    children.for_each_page(|page, (of, l)| {
        for r in page.rows() {
            entities.push(page.entity(r));
            local.push(l[r]);
            rows.push((page.entity(r), of[r].parent));
        }
    });
    index.fill(entities.iter().copied());
    parent.resize(tops, NONE);
    parent.extend(rows.iter().map(|&(_, p)| index.get(p).expect("a parent among the nodes")));
    levels.extend([0, tops]);
}

fn propagate_walk(n: &Nodes, global: &mut Vec<Global>) {
    const UNSET: f32 = f32::NAN;
    global.clear();
    global.resize(n.entities.len(), Global { c: UNSET, ..Global::default() });
    let tops = n.levels[1];
    for k in 0..tops {
        global[k] = root(&n.local[k]);
    }
    let mut left = n.entities.len() - tops;
    while left > 0 {
        let before = left;
        for k in tops..n.entities.len() {
            if !global[k].c.is_nan() {
                continue;
            }
            let p = global[n.parent[k] as usize];
            if !p.c.is_nan() {
                global[k] = compose(&p, &n.local[k]);
                left -= 1;
            }
        }
        assert!(left < before, "every child has a parent among the nodes");
    }
}

fn extent_of(global: &[Global]) -> [f32; 4] {
    global.iter().fold([f32::INFINITY, f32::INFINITY, f32::NEG_INFINITY, f32::NEG_INFINITY], |b, g| {
        [b[0].min(g.x), b[1].min(g.y), b[2].max(g.x), b[3].max(g.y)]
    })
}

type Roots<'w, 'a> = Query<'w, (&'a Local, &'a mut Global), Without<ChildOf>>;
type Children<'w, 'a> = Query<'w, (&'a ChildOf, &'a Local, &'a mut Global)>;

static TIMES: Mutex<Vec<(&'static str, f64)>> = Mutex::new(Vec::new());
static EXTENT: Mutex<Option<[f32; 4]>> = Mutex::new(None);

fn timed<R>(name: &'static str, f: impl FnOnce() -> R) -> R {
    let t = Instant::now();
    let out = f();
    TIMES.lock().unwrap().push((name, t.elapsed().as_secs_f64() * 1e6));
    out
}

/// The pipeline; `walk` keeps the nodes in walk order rather than sorting
/// them by depth.
fn flows_pipeline(world: &World, walk: bool) -> Schedule {
    let gather =
        move |_: &mut Cx, mut roots: Query<&Local, Without<ChildOf>>, mut children: Query<(&ChildOf, &Local)>, mut n: Make<Nodes>| {
            if walk {
                return timed("gather", || gather_walk(&mut roots, &mut children, &mut n));
            }
            timed("gather", || {
                let Nodes { rows, locals, .. } = &mut *n;
                roots.for_each_page(|page, l| {
                    for r in page.rows() {
                        rows.push((page.entity(r), page.entity(r)));
                        locals.push(l[r]);
                    }
                });
                children.for_each_page(|page, (of, l)| {
                    for r in page.rows() {
                        rows.push((page.entity(r), of[r].parent));
                        locals.push(l[r]);
                    }
                });
                order_into(&mut n);
            })
        };
    let propagate = move |_: &mut Cx, workers: Workers, n: See<Nodes>, mut g: Make<Globals>| {
        if walk {
            return timed("propagate", || propagate_walk(&n, &mut g.global));
        }
        timed("propagate", || propagate_levels(&workers, &n, &mut g.global))
    };
    let extent = |_: &mut Cx, workers: Workers, g: See<Globals>, mut out: Make<Extent>| {
        timed("extent", || {
            let fold = |a: [f32; 4], b: [f32; 4]| [a[0].min(b[0]), a[1].min(b[1]), a[2].max(b[2]), a[3].max(b[3])];
            out.bounds = flows::reduce(&workers, &g.global, 1024, extent_of, fold);
            *EXTENT.lock().unwrap() = out.bounds;
        })
    };
    let scatter = |_: &mut Cx, n: See<Nodes>, g: Take<Globals>, mut roots: Roots, mut children: Children| {
        timed("scatter", || {
            let (index, global) = (&n.index, &g.global);
            roots.for_each_page(|page, (_, mut out)| {
                let out = out.write_all();
                for (r, &e) in page.entities().iter().enumerate() {
                    out[r] = global[index.get(e).unwrap() as usize];
                }
            });
            children.for_each_page(|page, (_, _, mut out)| {
                let out = out.write_all();
                for (r, &e) in page.entities().iter().enumerate() {
                    out[r] = global[index.get(e).unwrap() as usize];
                }
            });
        })
    };
    let s = Schedule {
        systems: vec![
            gather.system(world, "gather"),
            propagate.system(world, "propagate"),
            extent.system(world, "extent"),
            scatter.system(world, "scatter"),
        ],
    };
    // Extent is made for whoever wants it, and read by nothing here: the
    // check notes it, and the next frame's `Make` recycles it.
    assert_eq!(flows::check(world, &s), Ok(vec!["spike::Extent: made by extent, read by nothing".to_string()]));
    s
}

/// What a game writes today (hierarchy_spike.rs's by level).
fn by_level(world: &World, span: usize) -> Schedule {
    let scratch: Mutex<Vec<Global>> = Mutex::default();
    let sys = move |_: &mut Cx, mut roots: Roots, mut children: Children| {
        let t = Instant::now();
        const UNSET: f32 = f32::NAN;
        let mut scratch = scratch.lock().unwrap();
        scratch.clear();
        scratch.resize(span, Global { c: UNSET, ..Global::default() });
        roots.for_each(|row, (l, mut g)| {
            let r = root(l);
            *g = r;
            scratch[row.entity().index as usize] = r;
        });
        let mut left = children.len();
        while left > 0 {
            let before = left;
            children.for_each(|row, (of, l, mut g)| {
                let i = row.entity().index as usize;
                if !scratch[i].c.is_nan() {
                    return;
                }
                let p = scratch[of.parent.index as usize];
                if p.c.is_nan() {
                    return;
                }
                let r = compose(&p, l);
                *g = r;
                scratch[i] = r;
                left -= 1;
            });
            assert!(left < before, "every child has a parent among the nodes");
        }
        TIMES.lock().unwrap().push(("by level", t.elapsed().as_secs_f64() * 1e6));
    };
    Schedule { systems: vec![sys.system(world, "by level")] }
}

/// Physics's shape in one system, fresh vectors each frame
/// (hierarchy_spike.rs's sorted).
fn sorted(world: &World) -> Schedule {
    let sys = |_: &mut Cx, mut roots: Roots, mut children: Children| {
        let t = Instant::now();
        let mut n = Nodes::default();
        roots.for_each_page(|page, (l, _)| {
            for r in page.rows() {
                n.rows.push((page.entity(r), page.entity(r)));
                n.locals.push(l[r]);
            }
        });
        children.for_each_page(|page, (of, l, _)| {
            for r in page.rows() {
                n.rows.push((page.entity(r), of[r].parent));
                n.locals.push(l[r]);
            }
        });
        order_into(&mut n);
        let mut global = Vec::new();
        propagate_levels(&Workers::default(), &n, &mut global);
        let index = &n.index;
        roots.for_each_page(|page, (_, mut g)| {
            let g = g.write_all();
            for (r, &e) in page.entities().iter().enumerate() {
                g[r] = global[index.get(e).unwrap() as usize];
            }
        });
        children.for_each_page(|page, (_, _, mut g)| {
            let g = g.write_all();
            for (r, &e) in page.entities().iter().enumerate() {
                g[r] = global[index.get(e).unwrap() as usize];
            }
        });
        TIMES.lock().unwrap().push(("sorted", t.elapsed().as_secs_f64() * 1e6));
    };
    Schedule { systems: vec![sys.system(world, "sorted")] }
}

/// The forest: `trees` trees, each node with `branch` children down to
/// `depth`, spawned a tree at a time, depth first (hierarchy_spike.rs's).
fn forest(world: &World, trees: usize, branch: usize, depth: usize, scrambled: bool) -> usize {
    if scrambled {
        let total = trees * (branch.pow(depth as u32 + 1) - 1) / (branch - 1);
        let held: Arc<Mutex<Vec<Entity>>> = Arc::default();
        let h = held.clone();
        let fill = move |_: &mut Cx, spawner: Spawner<(Global,)>| {
            let mut h = h.lock().unwrap();
            (0..total).for_each(|_| h.push(spawner.spawn((Global::default(),))));
        };
        Schedule { systems: vec![fill.system(world, "fill")] }.run_sequential(world);
        let empty = move |_: &mut Cx, mut q: Query<&Global, Without<Local>, engine_ecs::Despawns>| {
            let held = held.lock().unwrap();
            let n = held.len();
            for i in 0..n {
                let e = held[(i * 7919 + 13) % n];
                q.with(e, |row, _| row.despawn());
            }
        };
        Schedule { systems: vec![empty.system(world, "empty")] }.run_sequential(world);
    }
    let count = Arc::new(Mutex::new(0usize));
    let c = count.clone();
    let spawn = move |_: &mut Cx, roots: Spawner<(Local, Global)>, nodes: Spawner<(Local, Global, ChildOf)>| {
        let mut n = 0;
        let local = |i: usize| {
            let a = (i % 11) as f32 * 0.05;
            Local { x: 1.0 + (i % 7) as f32 * 0.1, y: (i % 5) as f32 * 0.1, c: a.cos(), s: a.sin() }
        };
        fn grow(
            nodes: &Spawner<(Local, Global, ChildOf)>,
            parent: Entity,
            branch: usize,
            left: usize,
            n: &mut usize,
            local: &dyn Fn(usize) -> Local,
        ) {
            if left == 0 {
                return;
            }
            for _ in 0..branch {
                *n += 1;
                let e = nodes.spawn((local(*n), Global::default(), ChildOf { parent }));
                grow(nodes, e, branch, left - 1, n, local);
            }
        }
        for _ in 0..trees {
            n += 1;
            let r = roots.spawn((local(n), Global::default()));
            grow(&nodes, r, branch, depth, &mut n, &local);
        }
        *c.lock().unwrap() = n;
    };
    Schedule { systems: vec![spawn.system(world, "forest")] }.run_sequential(world);
    *count.lock().unwrap()
}

/// Every local written each frame: an animated skeleton.
fn animate(world: &World) -> Schedule {
    let tick = Mutex::new(0u32);
    let sys = move |_: &mut Cx, mut roots: Query<&mut Local, Without<ChildOf>>, mut nodes: Query<&mut Local, With<ChildOf>>| {
        let mut t = tick.lock().unwrap();
        *t += 1;
        let turn = |l: &mut Local, k: u32| {
            let a = (k % 13) as f32 * 0.01;
            (l.c, l.s) = (l.c * a.cos() - l.s * a.sin(), l.s * a.cos() + l.c * a.sin());
        };
        roots.for_each(|_, mut l| {
            l.x += 0.01;
            turn(&mut l, *t);
        });
        nodes.for_each(|row, mut l| turn(&mut l, *t + row.entity().index));
    };
    Schedule { systems: vec![sys.system(world, "animate")] }
}

fn reference(world: &World) -> HashMap<Entity, [u32; 4]> {
    let locals: HashMap<Entity, Local> = world.values::<Local>().unwrap().into_iter().collect();
    let parents: HashMap<Entity, Entity> = world.values::<ChildOf>().unwrap().into_iter().map(|(e, c)| (e, c.parent)).collect();
    let mut out: HashMap<Entity, Global> = HashMap::new();
    fn of(e: Entity, locals: &HashMap<Entity, Local>, parents: &HashMap<Entity, Entity>, out: &mut HashMap<Entity, Global>) -> Global {
        if let Some(g) = out.get(&e) {
            return *g;
        }
        let g = match parents.get(&e) {
            None => root(&locals[&e]),
            Some(&p) => compose(&of(p, locals, parents, out), &locals[&e]),
        };
        out.insert(e, g);
        g
    }
    for &e in locals.keys() {
        of(e, &locals, &parents, &mut out);
    }
    out.into_iter().map(|(e, g)| (e, [g.x.to_bits(), g.y.to_bits(), g.c.to_bits(), g.s.to_bits()])).collect()
}

fn globals(world: &World) -> HashMap<Entity, [u32; 4]> {
    world.values::<Global>().unwrap().into_iter().map(|(e, g)| (e, [g.x.to_bits(), g.y.to_bits(), g.c.to_bits(), g.s.to_bits()])).collect()
}

fn median(mut xs: Vec<f64>) -> f64 {
    xs.sort_by(|a, b| a.total_cmp(b));
    xs[xs.len() / 2]
}

fn warm(gang: &Workers) {
    let start = Instant::now();
    while start.elapsed() < Duration::from_millis(5) {
        gang.run(gang.threads(), |_| {
            let t = Instant::now();
            while t.elapsed() < Duration::from_micros(200) {
                std::hint::spin_loop();
            }
        });
    }
}

fn main() {
    let reps: usize = std::env::var("REPS").ok().and_then(|r| r.parse().ok()).unwrap_or(41);
    let threads: usize = std::env::var("THREADS").ok().and_then(|t| t.parse().ok()).unwrap_or(8);
    let pool: Arc<dyn Executor> = Arc::new(pool::Pool::new(threads));
    let gang = Workers::new(Some(pool.clone()));
    warm(&gang);
    println!("| forest | way | gather | propagate | extent | scatter | the systems' bodies | the frame |");
    println!("|---|---|---|---|---|---|---|---|");
    for (trees, branch, depth, scrambled) in [(500, 4, 2, false), (160, 2, 5, false), (160, 2, 5, true)] {
        let world = World::new();
        flows::reset(&world);
        let n = forest(&world, trees, branch, depth, scrambled);
        let span = world.values::<Local>().unwrap().iter().map(|(e, _)| e.index as usize + 1).max().unwrap();
        let drive = animate(&world);
        let ways: Vec<(String, Schedule, bool)> = vec![
            ("hand, by level".into(), by_level(&world, span), false),
            ("hand, sorted each frame".into(), sorted(&world), false),
            ("flows, sorted by depth, 1 thread".into(), flows_pipeline(&world, false), false),
            (format!("flows, sorted by depth, {threads} threads"), flows_pipeline(&world, false), true),
            ("flows, in walk order".into(), flows_pipeline(&world, true), false),
        ];
        let mut times: HashMap<usize, Vec<(HashMap<&'static str, f64>, f64)>> = HashMap::new();
        for rep in 0..reps + 1 {
            drive.run_sequential(&world);
            let want = reference(&world);
            let mut order: Vec<usize> = (0..ways.len()).collect();
            order.rotate_left(rep % ways.len());
            for i in order {
                let (name, s, threaded) = &ways[i];
                let clear = |_: &mut Cx, mut g: Query<&mut Global>| g.for_each(|_, mut g| *g = Global::default());
                Schedule { systems: vec![clear.system(&world, "clear")] }.run_sequential(&world);
                world.set_executor(threaded.then(|| pool.clone()));
                if *threaded {
                    warm(&gang);
                }
                TIMES.lock().unwrap().clear();
                let t = Instant::now();
                s.run_sequential(&world);
                let frame = t.elapsed().as_secs_f64() * 1e6;
                world.set_executor(None);
                assert!(globals(&world) == want, "{name} propagates otherwise than the reference");
                if let Some(got) = EXTENT.lock().unwrap().take() {
                    let all: Vec<Global> = world.values::<Global>().unwrap().into_iter().map(|(_, g)| g).collect();
                    let want = extent_of(&all);
                    // A box doesn't depend on the order it's folded in.
                    assert_eq!(got.map(f32::to_bits), want.map(f32::to_bits), "{name}: the extent");
                }
                if rep > 0 {
                    times.entry(i).or_default().push((TIMES.lock().unwrap().iter().copied().collect(), frame));
                }
            }
        }
        let indices = if scrambled { ", scrambled" } else { "" };
        for (i, (name, _, _)) in ways.iter().enumerate() {
            let t = &times[&i];
            let stage = |k: &str| {
                let xs: Vec<f64> = t.iter().filter_map(|(m, _)| m.get(k).copied()).collect();
                if xs.is_empty() { "–".to_string() } else { format!("{:.1}", median(xs)) }
            };
            let bodies = median(t.iter().map(|(m, _)| m.values().sum::<f64>()).collect());
            println!(
                "| {n} nodes, {depth} deep{indices} | {name} | {} | {} | {} | {} | {bodies:.1} | {:.1} |",
                stage("gather"),
                if i < 2 { stage(if i == 0 { "by level" } else { "sorted" }) } else { stage("propagate") },
                stage("extent"),
                stage("scatter"),
                median(t.iter().map(|(_, f)| *f).collect())
            );
        }
    }
}
