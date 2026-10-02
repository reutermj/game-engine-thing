//! SPIKE, not engine code: the second user measured for
//! docs/architecture/working-sets.md. Transform propagation down `ChildOf`:
//! each node's `Global` is its parent's `Global` composed with its own
//! `Local`, and a root's `Global` is its `Local`. Kept as a bench target so
//! its numbers can be taken again; nothing depends on it.
//!
//! Four ways, on the same world, each frame's result checked bit for bit
//! against a recursive reference:
//!
//! - **hand, by level**: what a game writes today. A query can't read a
//!   parent's `Global` while it writes its children's in the same table, so
//!   globals go into a vector by entity index, roots first; then the
//!   children are walked once per level, each taking its parent's from the
//!   vector once the parent has one.
//! - **hand, sorted**: physics's shape. Each frame the nodes are gathered
//!   dense, indexed by entity (`Slots`), each one's parent found through
//!   the index, sorted by depth, propagated in one pass, and written back.
//! - **working set, kept**: the index by entity, the depth order and each
//!   node's parent slot kept across frames, rebuilt only when a node
//!   arrives or leaves or a `ChildOf` is written. A frame gathers the locals
//!   into their slots, propagates in one pass and writes back.
//! - **working set, kept, changed locals**: the same, gathering only the
//!   locals written since its last frame (change ticks).
//!
//! Three kinds of frame: every `Local` written (an animated skeleton), only
//! the roots' (things carried by what moves), and only the roots' with two
//! leaves swapping parents (something picked up: the kept ways then
//! rebuild, the spike's fallback, where a world would patch its order).
//!
//!     taskset -c 0-7 ./bazel run --config=bench //engine/ecs:hierarchy_spike
//!
//! `REPS` (41), the median of each.

use std::collections::HashMap;
use std::sync::{Arc, Mutex};
use std::time::Instant;

use engine_ecs::harness::{Cx, IntoSystem, Schedule};
use engine_ecs::{ChildOf, Entity, Query, Spawner, With, Without, World, component};

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

/// `Slots` as physics has it: entities to places, by entity index.
#[derive(Clone, Default)]
struct Slots(Vec<(u32, u32)>);

impl Slots {
    fn of(entities: &[Entity]) -> Slots {
        let len = entities.iter().map(|e| e.index as usize + 1).max().unwrap_or(0);
        let mut slots = vec![(u32::MAX, u32::MAX); len];
        for (k, e) in entities.iter().enumerate() {
            slots[e.index as usize] = (e.generation, k as u32);
        }
        Slots(slots)
    }

    #[inline]
    fn get(&self, e: Entity) -> Option<u32> {
        self.0.get(e.index as usize).filter(|(g, k)| *g == e.generation && *k != u32::MAX).map(|(_, k)| *k)
    }
}

const NONE: u32 = u32::MAX;

/// Nodes in an order where every parent comes before its children, each
/// with its parent's place in that order (`NONE` for a root), and the index
/// by entity: what both sorted ways build, one each frame, one kept.
#[derive(Default)]
struct Order {
    entities: Vec<Entity>,
    parent: Vec<u32>,
    index: Slots,
}

/// Roots, then each level's nodes, from rows in any order: `(entity,
/// parent)`, a root's parent itself.
fn order(rows: &[(Entity, Entity)]) -> Order {
    let walk = Slots::of(&rows.iter().map(|r| r.0).collect::<Vec<_>>());
    // Each node's depth, by following parents: memoized, so each is found
    // once.
    let mut depth = vec![u32::MAX; rows.len()];
    let mut stack = Vec::new();
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
    // A counting sort by depth: stable, so siblings keep their walk order.
    let levels = depth.iter().max().map_or(0, |d| *d as usize + 1);
    let mut start = vec![0usize; levels + 1];
    depth.iter().for_each(|&d| start[d as usize + 1] += 1);
    for l in 0..levels {
        start[l + 1] += start[l];
    }
    let mut placed = vec![0u32; rows.len()];
    for (i, &d) in depth.iter().enumerate() {
        placed[i] = start[d as usize] as u32;
        start[d as usize] += 1;
    }
    let mut entities = vec![Entity { index: 0, generation: 0 }; rows.len()];
    let mut parent = vec![NONE; rows.len()];
    for (i, &(e, p)) in rows.iter().enumerate() {
        let k = placed[i] as usize;
        entities[k] = e;
        if p != e {
            parent[k] = placed[walk.get(p).unwrap() as usize];
        }
    }
    let index = Slots::of(&entities);
    Order { entities, parent, index }
}

fn propagate(parent: &[u32], local: &[Local], global: &mut [Global]) {
    for k in 0..parent.len() {
        global[k] = match parent[k] {
            NONE => root(&local[k]),
            p => {
                assert!((p as usize) < k, "a parent before its child");
                compose(&global[p as usize], &local[k])
            }
        };
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Hash, Debug)]
enum Way {
    ByLevel,
    Sorted,
    Kept,
    KeptChanged,
}

const WAYS: [Way; 4] = [Way::ByLevel, Way::Sorted, Way::Kept, Way::KeptChanged];

impl Way {
    fn name(self) -> &'static str {
        match self {
            Way::ByLevel => "hand, by level",
            Way::Sorted => "hand, sorted each frame",
            Way::Kept => "working set, kept",
            Way::KeptChanged => "working set, kept, changed locals",
        }
    }
}

/// One way's state, kept between its frames.
#[derive(Default)]
struct State {
    /// For by level: globals by entity index, and how many indices.
    scratch: Vec<Global>,
    span: usize,
    kept: Order,
    kept_local: Vec<Local>,
    kept_global: Vec<Global>,
    /// The tick the kept ways last looked at.
    since: u32,
    rebuilt: u32,
    /// µs of the last frame: gather (with any rebuild), propagate, write
    /// back; and of the rebuild alone.
    gather: f64,
    pass: f64,
    scatter: f64,
    rebuild: f64,
}

type Roots<'w, 'a> = Query<'w, (&'a Local, &'a mut Global), Without<ChildOf>>;
type Children<'w, 'a> = Query<'w, (&'a ChildOf, &'a Local, &'a mut Global)>;
/// What the kept ways watch: the links, for nodes arriving, leaving or
/// changing parents.
type Links<'w, 'a> = Query<'w, &'a ChildOf>;
/// The locals alone, for which were written (`Global` is a term of the
/// others, and every frame writes it).
type Locals<'w, 'a> = (Query<'w, &'a Local, Without<ChildOf>>, Query<'w, &'a Local, With<ChildOf>>);

fn us(t: Instant) -> f64 {
    t.elapsed().as_secs_f64() * 1e6
}

fn run(way: Way, s: &mut State, roots: &mut Roots, children: &mut Children, links: &mut Links, locals: &mut Locals) {
    let start = Instant::now();
    s.rebuild = 0.0;
    match way {
        Way::ByLevel => {
            const UNSET: f32 = f32::NAN;
            s.scratch.clear();
            s.scratch.resize(s.span, Global { c: UNSET, ..Global::default() });
            let scratch = &mut s.scratch;
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
            s.pass = us(start);
            (s.gather, s.scatter) = (0.0, 0.0);
        }
        Way::Sorted => {
            let mut rows = Vec::with_capacity(roots.len() + children.len());
            let mut locals = Vec::with_capacity(rows.capacity());
            roots.for_each_page(|page, (l, _)| {
                for r in page.rows() {
                    rows.push((page.entity(r), page.entity(r)));
                    locals.push(l[r]);
                }
            });
            children.for_each_page(|page, (of, l, _)| {
                for r in page.rows() {
                    rows.push((page.entity(r), of[r].parent));
                    locals.push(l[r]);
                }
            });
            let o = order(&rows);
            let walk = Slots::of(&rows.iter().map(|r| r.0).collect::<Vec<_>>());
            let local: Vec<Local> = o.entities.iter().map(|e| locals[walk.get(*e).unwrap() as usize]).collect();
            s.gather = us(start);
            let t = Instant::now();
            let mut global = vec![Global::default(); rows.len()];
            propagate(&o.parent, &local, &mut global);
            s.pass = us(t);
            let t = Instant::now();
            let index = &o.index;
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
            s.scatter = us(t);
        }
        Way::Kept | Way::KeptChanged => {
            // Kept while no node came or went and no link was written: a
            // look per table, and a page walk that skips unwritten pages.
            let mut changed = s.kept.entities.is_empty() || links.arrived_since(s.since) || links.left_since(s.since);
            changed = changed || roots.arrived_since(s.since) || roots.left_since(s.since);
            if !changed {
                links.for_each_written(s.since, |_, _| changed = true);
            }
            let now = links.now();
            if changed {
                let t = Instant::now();
                let mut rows = Vec::with_capacity(roots.len() + children.len());
                roots.for_each_page(|page, _| rows.extend(page.entities().iter().map(|&e| (e, e))));
                children.for_each_page(|page, (of, _, _)| rows.extend(page.rows().map(|r| (page.entity(r), of[r].parent))));
                s.kept = order(&rows);
                s.kept_local = vec![Local::default(); rows.len()];
                s.kept_global = vec![Global::default(); rows.len()];
                s.rebuilt += 1;
                s.rebuild = us(t);
            }
            let State { kept, kept_local, since, .. } = s;
            let index = &kept.index;
            if way == Way::Kept || changed {
                roots.for_each_page(|page, (l, _)| {
                    for (r, &e) in page.entities().iter().enumerate() {
                        kept_local[index.get(e).unwrap() as usize] = l[r];
                    }
                });
                children.for_each_page(|page, (_, l, _)| {
                    for (r, &e) in page.entities().iter().enumerate() {
                        kept_local[index.get(e).unwrap() as usize] = l[r];
                    }
                });
            } else {
                locals.0.for_each_written(*since, |row, l| kept_local[index.get(row.entity()).unwrap() as usize] = *l);
                locals.1.for_each_written(*since, |row, l| kept_local[index.get(row.entity()).unwrap() as usize] = *l);
            }
            s.gather = us(start);
            let t = Instant::now();
            propagate(&s.kept.parent, &s.kept_local, &mut s.kept_global);
            s.pass = us(t);
            let t = Instant::now();
            let State { kept, kept_global, .. } = s;
            let index = &kept.index;
            roots.for_each_page(|page, (_, mut g)| {
                let g = g.write_all();
                for (r, &e) in page.entities().iter().enumerate() {
                    g[r] = kept_global[index.get(e).unwrap() as usize];
                }
            });
            children.for_each_page(|page, (_, _, mut g)| {
                let g = g.write_all();
                for (r, &e) in page.entities().iter().enumerate() {
                    g[r] = kept_global[index.get(e).unwrap() as usize];
                }
            });
            s.scatter = us(t);
            // After this frame's own writes, so the next frame sees only
            // what others wrote.
            s.since = locals.0.now().max(now);
        }
    }
}

/// The forest: `trees` trees, each node with `branch` children down to
/// `depth`, spawned a tree at a time, depth first, as a game spawns
/// prefabs.
fn forest(world: &World, trees: usize, branch: usize, depth: usize, scrambled: bool) -> usize {
    if scrambled {
        // Indices freed in a scattered order, which the forest's spawns
        // then take: what a world that has run a while hands out, where a
        // child's index is as likely below its parent's as above.
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

#[derive(Clone, Copy, PartialEq, Debug)]
enum Frame {
    Animated,
    RootsMove,
    Reparented,
}

/// What happens to the forest before each frame's propagation.
fn driver(world: &World, frame: Frame) -> Schedule {
    let tick = Arc::new(Mutex::new(0u32));
    let animate = move |_: &mut Cx, mut roots: Query<&mut Local, Without<ChildOf>>, mut nodes: Query<(&mut ChildOf, &mut Local)>| {
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
        if frame == Frame::RootsMove {
            return;
        }
        // Reparented frames move only the roots, as things picked up and
        // carried do: a link written while the rest of the forest is still.
        let mut links = Vec::new();
        nodes.for_each(|row, (of, mut l)| {
            if frame == Frame::Animated {
                turn(&mut l, *t + row.entity().index);
            }
            links.push((row.entity(), of.parent));
        });
        if frame == Frame::Reparented {
            // Two leaves with different parents swap them: the same depth
            // each, so the forest stays one, and a link written each frame.
            let parents: std::collections::HashSet<Entity> = links.iter().map(|l| l.1).collect();
            let mut leaves = links.iter().filter(|l| !parents.contains(&l.0));
            let a = *leaves.next().expect("a leaf");
            let b = *leaves.find(|l| l.1 != a.1).expect("a leaf of another parent");
            nodes.with(a.0, |_, (mut of, _)| of.parent = b.1);
            nodes.with(b.0, |_, (mut of, _)| of.parent = a.1);
        }
    };
    Schedule { systems: vec![animate.system(world, "animate")] }
}

/// Every node's `Global` by recursion over the world: the reference.
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

fn main() {
    let reps: usize = std::env::var("REPS").ok().and_then(|r| r.parse().ok()).unwrap_or(41);
    // (trees, branch, depth): 10 500 nodes two deep, and 10 080 five deep;
    // indices fresh (each child's above its parent's) or scrambled.
    for (trees, branch, depth, scrambled) in [(500, 4, 2, false), (160, 2, 5, false), (160, 2, 5, true)] {
        for frame in [Frame::Animated, Frame::RootsMove, Frame::Reparented] {
            let world = World::new();
            let n = forest(&world, trees, branch, depth, scrambled);
            let span = world.values::<Local>().unwrap().iter().map(|(e, _)| e.index as usize + 1).max().unwrap();
            let states: Vec<Arc<Mutex<State>>> = WAYS.iter().map(|_| Arc::new(Mutex::new(State { span, ..State::default() }))).collect();
            let schedules: Vec<Schedule> = WAYS
                .iter()
                .zip(&states)
                .map(|(&way, st)| {
                    let st = st.clone();
                    let sys = move |_: &mut Cx, mut roots: Roots, mut children: Children, mut links: Links, mut locals: Locals| {
                        run(way, &mut st.lock().unwrap(), &mut roots, &mut children, &mut links, &mut locals)
                    };
                    Schedule { systems: vec![sys.system(&world, way.name())] }
                })
                .collect();
            let drive = driver(&world, frame);
            let mut times: HashMap<Way, Vec<(f64, f64, f64, f64, f64)>> = HashMap::new();
            for rep in 0..reps + 1 {
                drive.run_sequential(&world);
                let want = reference(&world);
                let mut order: Vec<usize> = (0..WAYS.len()).collect();
                order.rotate_left(rep % WAYS.len());
                for i in order {
                    // Each way writes every `Global`; clearing them first
                    // makes a way that skipped one fail the check.
                    let clear = |_: &mut Cx, mut g: Query<&mut Global>| g.for_each(|_, mut g| *g = Global::default());
                    Schedule { systems: vec![clear.system(&world, "clear")] }.run_sequential(&world);
                    let start = Instant::now();
                    schedules[i].run_sequential(&world);
                    let frame_us = us(start);
                    assert!(globals(&world) == want, "{} propagates otherwise than the reference", WAYS[i].name());
                    let s = states[i].lock().unwrap();
                    // The first frame builds what the kept ways keep.
                    if rep > 0 {
                        times.entry(WAYS[i]).or_default().push((s.gather, s.pass, s.scatter, s.gather + s.pass + s.scatter, frame_us));
                    }
                }
            }
            let indices = if scrambled { "indices scrambled" } else { "indices fresh" };
            println!("\n### {trees} trees, {branch} children a node, {depth} deep ({n} nodes, {indices}): {frame:?}\n");
            println!("µs a frame, the median of {reps}; every way bit for bit the reference\n");
            println!("| way | gather | propagate | write back | all | the system, run by the harness | rebuilds |");
            println!("|---|---|---|---|---|---|---|");
            for (i, way) in WAYS.iter().enumerate() {
                let t = &times[way];
                let m = |f: fn(&(f64, f64, f64, f64, f64)) -> f64| median(t.iter().map(f).collect());
                let gather = if *way == Way::ByLevel { "–".to_string() } else { format!("{:.1}", m(|x| x.0)) };
                let scatter = if *way == Way::ByLevel { "–".to_string() } else { format!("{:.1}", m(|x| x.2)) };
                println!(
                    "| {} | {gather} | {:.1} | {scatter} | {:.1} | {:.1} | {} |",
                    way.name(),
                    m(|x| x.1),
                    m(|x| x.3),
                    m(|x| x.4),
                    states[i].lock().unwrap().rebuilt
                );
            }
        }
    }
}
