//! A small executor for the graph, outside the engine: systems are plain
//! functions of a `Cx` and parameters, run sequentially or on threads. The
//! engine's own tests of storage and scheduling drive it, and so does the
//! benchmark; the engine runs mods' systems through its own scheduler.

use std::sync::{Condvar, Mutex};
use std::time::{Duration, Instant};

use crate::graph::{Footprint, SystemView, applies_overlap, bound, exact, system_apply_overlap, systems_overlap};
use crate::query::{Change, Declare, FrameCx, Log, Param, ParamDecl, check_conflicts};
use crate::world::{Build, Structural, World};

/// What every harness system gets first: a place to log to.
#[derive(Default)]
pub struct Cx {
    pub messages: Vec<String>,
}

impl Cx {
    pub fn log(&mut self, message: impl Into<String>) {
        self.messages.push(message.into());
    }
}

pub type RunFn = Box<dyn for<'w> Fn(&FrameCx<'w>, &'w [ParamDecl], &mut Cx) + Send + Sync>;

/// A function that can be a harness system: `fn(&mut Cx, P1, .., Pn)`.
pub trait IntoSystem<P> {
    /// Declares it, installing the components it names in `world`.
    fn system(self, world: &World, name: &str) -> SystemDecl;
}

static OWNERS: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);

macro_rules! into_system {
    ($($p:ident),*) => {
        impl<Func, $($p: Param),*> IntoSystem<($($p,)*)> for Func
        where
            Func: Send + Sync + 'static,
            for<'a> &'a Func: Fn(&mut Cx, $($p),*) + Fn(&mut Cx, $($p::Item<'_>),*),
        {
            #[allow(non_snake_case, unused_mut, unused_variables)]
            fn system(self, world: &World, name: &str) -> SystemDecl {
                let mut d = Declare::new(world);
                let params = vec![$($p::declare(&mut d)),*];
                if let Some(e) = d.error {
                    panic!("{e}");
                }
                crate::between::install_all(world, &d.components, &d.events, &Build::default()).unwrap();
                if let Err(e) = check_conflicts(world, name, &params) {
                    panic!("{e}");
                }
                let run: RunFn = Box::new(move |frame, decls, cx| {
                    let mut decls = decls.iter();
                    $(let $p = $p::fetch(frame, decls.next().unwrap());)*
                    // Picks the `Fn` over the parameters' items, whatever
                    // lifetimes they were fetched with.
                    fn call<$($p),*>(f: impl Fn(&mut Cx, $($p),*), cx: &mut Cx, $($p: $p),*) {
                        f(cx, $($p),*)
                    }
                    call(&self, cx, $($p),*);
                });
                let owner = OWNERS.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
                SystemDecl { name: name.into(), owner, params, run }
            }
        }
    };
}
into_system!();
into_system!(P1);
into_system!(P1, P2);
into_system!(P1, P2, P3);
into_system!(P1, P2, P3, P4);
into_system!(P1, P2, P3, P4, P5);
into_system!(P1, P2, P3, P4, P5, P6);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Node {
    System(usize),
    Apply(usize),
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum State {
    Pending,
    Running,
    Done,
}

/// Systems in plan order.
pub struct Schedule {
    pub systems: Vec<SystemDecl>,
}

/// A system as the harness runs it: its declaration, and the closure that
/// fetches its parameters and calls it.
pub struct SystemDecl {
    pub name: String,
    /// Systems with one owner never run at once (one mod's).
    pub owner: usize,
    pub params: Vec<ParamDecl>,
    pub run: RunFn,
}

impl SystemDecl {
    fn view(&self) -> SystemView<'_> {
        SystemView { owner: self.owner, params: &self.params }
    }
}

/// One frame's progress.
pub struct FrameState {
    pub nodes: Vec<Node>,
    pub state: Vec<State>,
    /// By system: its change log, once it has run and until its apply takes it.
    logs: Vec<Option<Vec<Change>>>,
    /// By system: its apply's footprint while the apply runs, its log being
    /// out. Others' readiness must still see it.
    running: Vec<Option<Footprint>>,
}

#[derive(Clone, Debug)]
pub struct Span {
    pub node: String,
    pub thread: usize,
    pub start: Duration,
    pub end: Duration,
}

impl Schedule {
    pub fn node_name(&self, n: Node) -> String {
        match n {
            Node::System(i) => self.systems[i].name.clone(),
            Node::Apply(i) => format!("apply({})", self.systems[i].name),
        }
    }

    pub fn frame(&self) -> FrameState {
        let mut nodes = Vec::new();
        for (i, s) in self.systems.iter().enumerate() {
            nodes.push(Node::System(i));
            if s.params.iter().any(ParamDecl::changes) {
                nodes.push(Node::Apply(i));
            }
        }
        let state = vec![State::Pending; nodes.len()];
        let n = self.systems.len();
        FrameState { nodes, state, logs: (0..n).map(|_| None).collect(), running: vec![None; n] }
    }

    /// An apply node's footprint now: exact if its system has run.
    pub fn apply_footprint(&self, world: &World, fs: &FrameState, system: usize) -> Footprint {
        if let Some(fp) = &fs.running[system] {
            return fp.clone();
        }
        match &fs.logs[system] {
            Some(log) => exact(world, log),
            None => bound(world, &self.systems[system].params),
        }
    }

    fn overlap(&self, world: &World, fs: &FrameState, a: Node, b: Node) -> bool {
        match (a, b) {
            (Node::System(x), Node::System(y)) => systems_overlap(world, self.systems[x].view(), self.systems[y].view()),
            (Node::System(x), Node::Apply(y)) | (Node::Apply(y), Node::System(x)) => {
                system_apply_overlap(world, self.systems[x].view(), &self.apply_footprint(world, fs, y))
            }
            (Node::Apply(x), Node::Apply(y)) => applies_overlap(&self.apply_footprint(world, fs, x), &self.apply_footprint(world, fs, y)),
        }
    }

    /// Whether node `i` may start: it's pending, an apply's system has run,
    /// and no earlier unfinished node overlaps it.
    pub fn ready(&self, world: &World, fs: &FrameState, i: usize) -> bool {
        if fs.state[i] != State::Pending {
            return false;
        }
        if let Node::Apply(s) = fs.nodes[i]
            && fs.logs[s].is_none()
            && fs.running[s].is_none()
        {
            return false;
        }
        self.blockers(world, fs, i).is_empty()
    }

    /// The earlier, unfinished nodes node `i` overlaps: what it waits for.
    pub fn blockers(&self, world: &World, fs: &FrameState, i: usize) -> Vec<String> {
        (0..i)
            .filter(|&j| fs.state[j] != State::Done && self.overlap(world, fs, fs.nodes[j], fs.nodes[i]))
            .map(|j| self.node_name(fs.nodes[j]))
            .collect()
    }

    /// Sets node `i`'s state, as a test driving the scheduler by hand does.
    pub fn mark(&self, fs: &mut FrameState, i: usize, state: State) {
        fs.state[i] = state;
    }

    /// Runs one node. An apply gets its footprint and log, taken from the
    /// frame state; a system returns its log.
    fn run_node(&self, world: &World, node: Node, apply: Option<(Footprint, Vec<Change>)>) -> Option<Vec<Change>> {
        match node {
            Node::System(s) => {
                let decl = &self.systems[s];
                let log = Log::default();
                let mut cx = Cx::default();
                // The parameters, and their guards, are dropped when `run`
                // returns: before the node is marked done.
                // The harness has no clock: a frame is 1/60 s.
                let frame = FrameCx { world, log: &log, system: &decl.name, dt: 1.0 / 60.0 };
                (decl.run)(&frame, &decl.params, &mut cx);
                Some(log.into_inner())
            }
            Node::Apply(_) => {
                let (fp, log) = apply.expect("an apply runs with its log");
                let mut st = Structural::new(world);
                for shape in &fp.tables {
                    debug_assert_eq!(shape.lower, shape.upper, "an exact footprint's tables are concrete");
                    let t = world.table_for(&shape.lower);
                    st.lock_table(t);
                }
                for &(c, _) in &fp.sparse {
                    st.lock_sparse(c);
                }
                for &(q, _) in &fp.events {
                    st.events(q);
                }
                for change in log {
                    change.apply(&mut st);
                }
                None
            }
        }
    }

    /// Runs node `i` on this thread and marks it done, whatever else is
    /// pending: the sequential executor, and tests driving a frame by hand.
    pub fn step(&self, world: &World, fs: &mut FrameState, i: usize) {
        let node = fs.nodes[i];
        let apply = match node {
            Node::Apply(s) => {
                let fp = self.apply_footprint(world, fs, s);
                Some((fp, fs.logs[s].take().expect("the apply's system has run")))
            }
            Node::System(_) => None,
        };
        let out = self.run_node(world, node, apply);
        if let (Node::System(s), Some(log)) = (node, out) {
            fs.logs[s] = Some(log);
        }
        fs.state[i] = State::Done;
    }

    /// Runs one frame on this thread, in plan order: the reference.
    pub fn run_sequential(&self, world: &World) -> Vec<Span> {
        let _frame = OpenFrame::begin(world);
        self.sequential(world)
    }

    fn sequential(&self, world: &World) -> Vec<Span> {
        let mut fs = self.frame();
        let base = Instant::now();
        let mut spans = Vec::new();
        for i in 0..fs.nodes.len() {
            let start = base.elapsed();
            self.step(world, &mut fs, i);
            spans.push(Span { node: self.node_name(fs.nodes[i]), thread: 0, start, end: base.elapsed() });
        }
        spans
    }

    /// Runs one frame on `threads` workers, each starting the first ready
    /// node in plan order. A node that panics fails the frame: the other
    /// workers stop taking nodes, and the panic is raised here.
    pub fn run_parallel(&self, world: &World, threads: usize) -> Vec<Span> {
        let _frame = OpenFrame::begin(world);
        self.parallel(world, threads)
    }

    fn parallel(&self, world: &World, threads: usize) -> Vec<Span> {
        type Panic = Box<dyn std::any::Any + Send>;
        let shared = Mutex::new((self.frame(), Vec::<Span>::new(), None::<Panic>));
        let wake = Condvar::new();
        let base = Instant::now();
        std::thread::scope(|scope| {
            for thread in 0..threads {
                let (shared, wake) = (&shared, &wake);
                scope.spawn(move || {
                    loop {
                        let mut guard = shared.lock().unwrap();
                        let (i, node, apply) = loop {
                            if guard.2.is_some() {
                                return;
                            }
                            let fs = &mut guard.0;
                            if fs.state.iter().all(|s| *s == State::Done) {
                                wake.notify_all();
                                return;
                            }
                            if let Some(i) = (0..fs.nodes.len()).find(|&i| self.ready(world, fs, i)) {
                                let node = fs.nodes[i];
                                let apply = match node {
                                    Node::Apply(s) => {
                                        let fp = self.apply_footprint(world, fs, s);
                                        Some((fp, fs.logs[s].take().unwrap()))
                                    }
                                    Node::System(_) => None,
                                };
                                fs.state[i] = State::Running;
                                if let (Node::Apply(s), Some((fp, _))) = (node, &apply) {
                                    fs.running[s] = Some(fp.clone());
                                }
                                break (i, node, apply);
                            }
                            guard = wake.wait(guard).unwrap();
                        };
                        drop(guard);
                        let start = base.elapsed();
                        let run = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| self.run_node(world, node, apply)));
                        let end = base.elapsed();
                        let mut guard = shared.lock().unwrap();
                        let out = match run {
                            Ok(out) => out,
                            Err(panic) => {
                                guard.2.get_or_insert(panic);
                                wake.notify_all();
                                return;
                            }
                        };
                        if let (Node::System(s), Some(log)) = (node, out) {
                            guard.0.logs[s] = Some(log);
                        }
                        guard.0.state[i] = State::Done;
                        guard.1.push(Span { node: self.node_name(node), thread, start, end });
                        wake.notify_all();
                    }
                });
            }
        });
        let (_, mut spans, panic) = shared.into_inner().unwrap();
        if let Some(panic) = panic {
            std::panic::resume_unwind(panic);
        }
        spans.sort_by_key(|s| s.start);
        spans
    }
}

/// Pairs of nodes that ran at the same time, by name.
pub fn overlaps(spans: &[Span]) -> Vec<(String, String)> {
    let mut out = Vec::new();
    for (i, a) in spans.iter().enumerate() {
        for b in &spans[i + 1..] {
            if a.start < b.end && b.start < a.end {
                out.push((a.node.clone(), b.node.clone()));
            }
        }
    }
    out
}

/// An open frame, ended when dropped: also when a node's panic unwinds out
/// of it, as the loader ends a frame its scheduler panicked in.
struct OpenFrame<'w>(&'w World);

impl<'w> OpenFrame<'w> {
    fn begin(world: &'w World) -> OpenFrame<'w> {
        world.begin_frame();
        OpenFrame(world)
    }
}

impl Drop for OpenFrame<'_> {
    fn drop(&mut self) {
        self.0.end_frame();
    }
}
