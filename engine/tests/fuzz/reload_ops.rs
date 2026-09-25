//! Hot reload against a model: the driver the reload fuzzer
//! (`//engine/tests/fuzz:reload`), its corpus replay and the seeded soak
//! (`:replay_test`) share, so an input the fuzzer finds replays exactly.
//!
//! A session is a real `Engine` loading the test mods in `mods/`, each built
//! several ways (see BUILD.bazel), driven by operations decoded from the
//! choices: loads and reloads of any build, alone and in batches, unloads,
//! frames (the loader's own, and the lockstep bootstrap's from a message),
//! and messages that spawn, despawn and write components, send events and
//! call services, or trip the failures planted in the mods. The model
//! predicts each operation from the documented rules (hot-reload.md,
//! mod-deps.md, ecs.md's "Layout changes", storage.md's events,
//! relationships.md's ordered tables), and after every one the driver
//! checks:
//!
//! - the engine's reply, exactly (a refusal from `dlopen` or the ABI check,
//!   by what it must mention);
//! - `list`: each mod's generation, residency and whether it has failed;
//! - each running mod's own report: its build, its state, what it saw;
//! - the world through the driver's mirror of each layout: every item and
//!   flag, and the ranks in the order their table keeps them;
//! - the frame and live entity counts;
//! - the builds mapped (`/proc/self/maps`): exactly the running builds and
//!   those whose code the world holds for its values.
//!
//! And after the session, dropping the engine must unmap every build.
//!
//! The model mirrors what each test mod's builds do; a change to a mod is a
//! change here too.

use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::sync::OnceLock;
use std::sync::atomic::{AtomicU64, Ordering};

use engine_loader::engine::Engine;

pub use ecs_ops::{Bytes, Choices, Rng};

// ---- The builds ----

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Kind {
    Keeper,
    Herald,
    Hearer,
    Ranker,
    Anchor,
    Tether,
    Clock,
    Lockstep,
    Sequential,
    /// Not a library.
    Garbage,
    /// A library without the entry points.
    NoEntry,
    /// A mod built against another mod API.
    WrongApi,
}

/// One build, as the model knows it: what BUILD.bazel builds it as.
struct Spec {
    /// The variable its library's path is passed in.
    var: &'static str,
    /// The mod it's a build of.
    name: &'static str,
    kind: Kind,
    /// The letter it reports itself by.
    v: &'static str,
    /// Its interface, by class: builds of one class have one digest.
    iface: &'static str,
    /// The interfaces it was built against: (mod, class).
    deps: &'static [(&'static str, &'static str)],
    resident: bool,
    provides: Option<&'static str>,
}

const fn build(var: &'static str, name: &'static str, kind: Kind, v: &'static str) -> Spec {
    Spec { var, name, kind, v, iface: "", deps: &[], resident: false, provides: None }
}

impl Spec {
    const fn iface(self, iface: &'static str) -> Spec {
        Spec { iface, ..self }
    }
    const fn deps(self, deps: &'static [(&'static str, &'static str)]) -> Spec {
        Spec { deps, ..self }
    }
    const fn resident(self) -> Spec {
        Spec { resident: true, ..self }
    }
    const fn provides(self, service: &'static str) -> Spec {
        Spec { provides: Some(service), ..self }
    }
}

const HERALD: &str = "herald::Herald";

static SPECS: &[Spec] = &[
    build("FZ_KEEPER_A", "keeper", Kind::Keeper, "a"),
    build("FZ_KEEPER_B", "keeper", Kind::Keeper, "b"),
    build("FZ_KEEPER_C", "keeper", Kind::Keeper, "c"),
    build("FZ_KEEPER_D", "keeper", Kind::Keeper, "d"),
    build("FZ_KEEPER_E", "keeper", Kind::Keeper, "e"),
    build("FZ_KEEPER_F", "keeper", Kind::Keeper, "f"),
    build("FZ_KEEPER_S", "keeper", Kind::Keeper, "s"),
    build("FZ_HERALD_A", "herald", Kind::Herald, "a").iface("herald1").provides(HERALD),
    build("FZ_HERALD_B", "herald", Kind::Herald, "b").iface("herald1").provides(HERALD),
    build("FZ_HERALD_C", "herald", Kind::Herald, "c").iface("herald2").provides(HERALD),
    build("FZ_HERALD_P", "herald", Kind::Herald, "p").iface("herald1").provides(HERALD),
    build("FZ_HEARER_A", "hearer", Kind::Hearer, "a").deps(&[("herald", "herald1")]),
    build("FZ_HEARER_B", "hearer", Kind::Hearer, "b").deps(&[("herald", "herald1")]),
    build("FZ_HEARER_C", "hearer", Kind::Hearer, "c").deps(&[("herald", "herald2")]),
    build("FZ_RANKER_A", "ranker", Kind::Ranker, "a"),
    build("FZ_RANKER_B", "ranker", Kind::Ranker, "b"),
    build("FZ_RANKER_C", "ranker", Kind::Ranker, "c"),
    build("FZ_ANCHOR_A", "anchor", Kind::Anchor, "a").iface("anchor").resident(),
    build("FZ_ANCHOR_B", "anchor", Kind::Anchor, "b").iface("anchor").resident(),
    build("FZ_ANCHOR_LOOSE", "anchor", Kind::Anchor, "loose").iface("anchor"),
    build("FZ_TETHER", "tether", Kind::Tether, "").deps(&[("anchor", "anchor")]).resident(),
    build("FZ_CLOCK", "clock", Kind::Clock, "").iface("clock").resident(),
    build("FZ_LOCKSTEP", "lockstep", Kind::Lockstep, "").deps(&[("clock", "clock")]).resident(),
    build("FZ_SEQUENTIAL", "sequential", Kind::Sequential, "").provides("engine_api::scheduler::Scheduler"),
    build("FZ_GARBAGE", "", Kind::Garbage, ""),
    build("FZ_NO_ENTRY", "", Kind::NoEntry, ""),
    build("FZ_WRONG_API", "", Kind::WrongApi, ""),
];

/// Every mod name the driver uses: each mod's, and a second name a herald
/// build can be loaded as, which two providers of one service can't share.
const NAMES: [&str; 10] = ["keeper", "herald", "hearer", "ranker", "anchor", "tether", "clock", "lockstep", "sequential", "herald2"];

/// Each build's library, from the paths in the environment.
fn libs() -> &'static [PathBuf] {
    static LIBS: OnceLock<Vec<PathBuf>> = OnceLock::new();
    LIBS.get_or_init(|| {
        let runfiles = runfiles::Runfiles::create().expect("runfiles");
        let libs: Vec<PathBuf> = SPECS
            .iter()
            .map(|s| {
                let at = std::env::var(s.var)
                    .unwrap_or_else(|_| panic!("${} is not set: run through //engine/tests/fuzz:reload or :replay_test", s.var));
                runfiles.rlocation(&at).unwrap_or_else(|| panic!("{at} is not in the runfiles"))
            })
            .collect();
        // The model tells builds apart as the engine does, by content: two
        // alike would be 'unchanged' where the model expects a reload.
        let contents: Vec<Vec<u8>> = libs.iter().map(|l| std::fs::read(l).expect("a build's library")).collect();
        for (i, a) in contents.iter().enumerate() {
            for (j, b) in contents.iter().enumerate().skip(i + 1) {
                assert!(a != b, "{} and {} are the same library", SPECS[i].var, SPECS[j].var);
            }
        }
        libs
    })
}

/// The test mods' components, one mirror per layout, for reading the world
/// without the mods' code. `World::values` hands back a layout's values
/// only while it's the one installed, by fingerprint.
mod mirror {
    use engine_api::{OrderKey, component};

    component! {
        #[derive(Default)]
        pub struct ItemA: "fz::Item" {
            pub id: u32,
            pub count: u32,
            pub name: String,
            pub tags: Vec<String>,
        }
    }

    component! {
        #[derive(Default)]
        pub struct ItemC: "fz::Item" {
            pub weight: f64,
            pub name: String,
            pub count: u64,
            pub id: u32,
        }
    }

    component! {
        #[derive(Default)]
        pub struct ItemD: "fz::Item", version = 1 {
            pub weight: f64,
            pub name: String,
            pub count: u64,
            pub id: u32,
        }
    }

    component! {
        #[derive(Default)]
        pub struct FlagA: "fz::Flag", storage = sparse {
            pub level: u8,
            pub note: String,
        }
    }

    component! {
        #[derive(Default)]
        pub struct FlagC: "fz::Flag", storage = sparse {
            pub note: String,
            pub level: i32,
            pub seen: bool,
        }
    }

    component! {
        #[derive(Default)]
        pub struct FlagS: "fz::Flag" {
            pub level: u8,
            pub note: String,
        }
    }

    component! {
        #[derive(Default)]
        pub struct RankA: "fz::Rank", order = key {
            pub n: u32,
        }
    }

    component! {
        #[derive(Default)]
        pub struct RankC: "fz::Rank", order = key {
            pub bonus: u8,
            pub n: u64,
        }
    }

    // Never called: the mirrors are only read. `order = key` needs them.
    impl OrderKey for RankA {
        fn key(&self) -> u128 {
            self.n as u128
        }
    }

    impl OrderKey for RankC {
        fn key(&self) -> u128 {
            self.n as u128
        }
    }
}

// ---- The model ----

/// `fz::Item`'s layouts: A; C (reordered, `count` widened, `weight` added,
/// `tags` removed); D, C's fields at version 1.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum ItemLayout {
    A,
    C,
    D,
}

impl ItemLayout {
    fn of(v: &str) -> ItemLayout {
        match v {
            "c" => ItemLayout::C,
            "d" => ItemLayout::D,
            _ => ItemLayout::A,
        }
    }
    fn version(self) -> u32 {
        if self == ItemLayout::D { 1 } else { 0 }
    }
    /// Each layout's `Default`, which every migrated value starts from.
    fn default_item(self) -> Item {
        let (name, weight) = match self {
            ItemLayout::A => ("anon", 0.0),
            ItemLayout::C => ("anon-c", 1.5),
            ItemLayout::D => ("anon-d", 2.5),
        };
        let tags = if self == ItemLayout::A { vec!["new".to_string()] } else { Vec::new() };
        Item { id: 0, count: 0, name: name.into(), tags, weight, flag: None }
    }
    /// A count as this layout holds it: `u32` in A, `u64` otherwise.
    fn count(self, count: u64) -> u64 {
        if self == ItemLayout::A { count as u32 as u64 } else { count }
    }
}

/// `fz::Flag`'s layouts: A and C sparse (C's level widened to `i32`, and
/// `seen` added); S, A's fields in tables.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum FlagLayout {
    A,
    C,
    S,
}

impl FlagLayout {
    fn of(v: &str) -> FlagLayout {
        match v {
            "c" | "d" => FlagLayout::C,
            "s" => FlagLayout::S,
            _ => FlagLayout::A,
        }
    }
    /// A level as this layout holds it, from any other's, with `as`.
    fn level(self, level: i64) -> i64 {
        if self == FlagLayout::C { level as i32 as i64 } else { level as u8 as i64 }
    }
}

/// `fz::Rank`'s layouts: A (`n: u32`), C (`n` widened, `bonus` added).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum RankLayout {
    A,
    C,
}

/// The order a build's `fz::Rank` glue keys by.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Glue {
    Up,
    Down,
}

impl Glue {
    fn key(self, n: u64) -> u64 {
        match self {
            Glue::Up => n,
            Glue::Down => u32::MAX as u64 - n,
        }
    }
}

/// An item, with the fields of every layout; which count depends on the
/// layout installed.
#[derive(Clone, Debug)]
struct Item {
    id: u32,
    count: u64,
    name: String,
    tags: Vec<String>,
    weight: f64,
    flag: Option<Flag>,
}

#[derive(Clone, Debug)]
struct Flag {
    level: i64,
    note: String,
    seen: bool,
}

/// A queued `fz::Note`.
#[derive(Clone, Debug)]
struct Note {
    seq: u64,
    /// Dropped as the frame after this one ends.
    frame: u64,
    n: u64,
    from: String,
    weight: Option<u32>,
}

/// A mod's state, with the fields of every build's layout of it.
#[derive(Clone, Debug, PartialEq)]
enum State {
    Keeper { loads: u32, spawned: u64, journal: Vec<String>, extra: Option<u64> },
    Herald { calls: u64, pending: Vec<u64>, loads: u32 },
    Hearer { early: Vec<String>, late: Vec<String>, at_load: String, loads: u32, heard: Option<u64> },
    Ranker { seen: Vec<u64>, loads: u32 },
    Anchor { pings: u32, loads: u32 },
    Tether { pings: u32 },
    Lockstep { frame: u64 },
    Empty,
}

/// A build's state layout and version. Builds with equal ones carry state
/// over as is; with equal versions, migrate it field by field; otherwise
/// the old build drops it and the new one starts from `Default`.
fn state_layout(s: &Spec) -> (u32, u32) {
    match (s.kind, s.v) {
        (Kind::Keeper, "c") => (1, 0),
        (Kind::Keeper, "d") => (1, 1),
        (Kind::Keeper, _) => (0, 0),
        (Kind::Hearer, "c") => (3, 0),
        (Kind::Hearer, _) => (2, 0),
        (kind, _) => (10 + kind as u32, 0),
    }
}

/// `s`'s `Default` state.
fn fresh(s: &Spec) -> State {
    match s.kind {
        Kind::Keeper => {
            State::Keeper { loads: 0, spawned: 0, journal: Vec::new(), extra: (ItemLayout::of(s.v) != ItemLayout::A).then_some(7) }
        }
        Kind::Herald => State::Herald { calls: 0, pending: Vec::new(), loads: 0 },
        Kind::Hearer => {
            State::Hearer { early: Vec::new(), late: Vec::new(), at_load: String::new(), loads: 0, heard: (s.v == "c").then_some(0) }
        }
        Kind::Ranker => State::Ranker { seen: Vec::new(), loads: 0 },
        Kind::Anchor => State::Anchor { pings: 0, loads: 0 },
        Kind::Tether => State::Tether { pings: 0 },
        Kind::Lockstep => State::Lockstep { frame: 0 },
        _ => State::Empty,
    }
}

/// Migrates `state` to `to`'s layout (the same version, another layout),
/// returning the loader's report of it: each new field in declaration
/// order, then each dropped one.
fn migrate(state: &mut State, to: &Spec) -> String {
    match state {
        State::Keeper { spawned, extra, .. } if extra.is_none() => {
            *extra = Some(7);
            "kept journal, converted spawned u32 -> u64, kept loads, added extra (default)".into()
        }
        State::Keeper { spawned, extra, .. } => {
            *spawned = *spawned as u32 as u64;
            *extra = None;
            "kept loads, converted spawned u64 -> u32, kept journal, dropped extra".into()
        }
        State::Hearer { heard, .. } if to.v == "c" => {
            *heard = Some(0);
            "kept early, kept late, kept at_load, kept loads, added heard (default)".into()
        }
        State::Hearer { heard, .. } => {
            *heard = None;
            "kept early, kept late, kept at_load, kept loads, dropped heard".into()
        }
        other => unreachable!("only keeper and hearer change their state's layout, not {other:?}"),
    }
}

struct Mod {
    name: String,
    /// The running build, into `SPECS`.
    spec: usize,
    /// When it was loaded: `ModContext::loaded_at`, which also names the
    /// build in the model's account of what is mapped.
    at: u64,
    generation: u32,
    failed: bool,
    state: State,
}

impl Mod {
    fn spec(&self) -> &'static Spec {
        &SPECS[self.spec]
    }
}

/// What the engine should be doing, by the rules.
#[derive(Default)]
struct Model {
    frame: u64,
    /// The last `loaded_at` handed out.
    loads: u64,
    /// In the engine's order: load order, a reload keeping its place.
    mods: Vec<Mod>,
    /// Whether `fz::Flag` was first declared (so is stored for the
    /// session) as a table component: by keeper_s, and no other.
    flag_in_tables: Option<bool>,
    /// Each installed layout, and the build it was installed from: whose
    /// code the world holds for the values.
    item: Option<(ItemLayout, u64)>,
    flag: Option<(FlagLayout, u64)>,
    rank: Option<(RankLayout, Glue, u64)>,
    /// `fz::Note`'s layout (`true` for herald's interface v2) and holder.
    note: Option<(bool, u64)>,
    items: Vec<Item>,
    ranks: Vec<u64>,
    /// The glue the rank table was last sorted with: its rows are in that
    /// order until something next re-sorts it, even after a reload changed
    /// the glue (relationships.md, "Open questions").
    sorted_by: Option<Glue>,
    notes: Vec<Note>,
    next_seq: u64,
    /// Each reading system's next sequence number, by `mod::system`.
    cursors: HashMap<String, u64>,
    /// Whether lockstep has published its clock, an entity of its own.
    clock: bool,
}

/// What the engine should answer.
#[derive(Debug)]
enum Expect {
    Exactly(Result<String, String>),
    /// A refusal that starts with this and mentions that: the rest of its
    /// text is the system's (`dlopen`'s), not the engine's.
    Refused(String, &'static str),
}

fn resident_note(name: &str) -> String {
    format!("{name} is resident: restart the engine to load its new build")
}

/// What hearer's build `v` records for a note it reads.
fn hearer_entry(v: &str, note: &Note) -> String {
    let entry = match note.weight {
        Some(w) => format!("{}@{}#{w}", note.n, note.from),
        None => format!("{}@{}", note.n, note.from),
    };
    assert_eq!(v == "c", note.weight.is_some(), "model: hearer:{v} read a note of the other layout: {note:?}");
    if v == "b" { format!("B{entry}") } else { entry }
}

/// Orders a batch so each build comes after the builds in it that it
/// depends on, first ready first, as the loader does.
fn dependency_order(mut pending: Vec<(String, usize)>) -> Vec<(String, usize)> {
    let mut ordered = Vec::new();
    while !pending.is_empty() {
        let ready = pending
            .iter()
            .position(|(_, s)| SPECS[*s].deps.iter().all(|(dep, _)| !pending.iter().any(|(p, _)| p == dep)))
            .expect("mod_deps have no cycles");
        ordered.push(pending.remove(ready));
    }
    ordered
}

impl Model {
    fn position(&self, name: &str) -> Option<usize> {
        self.mods.iter().position(|m| m.name == name)
    }

    fn find(&self, name: &str) -> Option<&Mod> {
        self.mods.iter().find(|m| m.name == name)
    }

    /// `Engine::load`: a batch of one, but asked for by name, so a resident
    /// mod's new build is refused rather than kept.
    fn load(&mut self, name: &str, spec: usize) -> Expect {
        if self.find(name).is_some_and(|m| m.spec().resident && m.spec != spec) {
            return Expect::Exactly(Err(resident_note(name)));
        }
        self.batch(&[(name.to_string(), spec)])
    }

    /// `Engine::load_batch`: every changed build opened and checked, then
    /// all swapped in, or none.
    fn batch(&mut self, batch: &[(String, usize)]) -> Expect {
        let (mut opened, mut unchanged, mut kept) = (Vec::new(), Vec::new(), Vec::new());
        for (i, (name, s)) in batch.iter().enumerate() {
            if batch[..i].iter().any(|(n, _)| n == name) {
                return Expect::Exactly(Err(format!("{name} is in the batch twice")));
            }
            let running = self.find(name);
            if running.is_some_and(|m| m.spec == *s) {
                unchanged.push(format!("{name} unchanged"));
                continue;
            }
            if running.is_some_and(|m| m.spec().resident) {
                kept.push(resident_note(name));
                continue;
            }
            // Opening a build runs nothing of it but its declarations, which
            // intern what they name whether or not the batch commits.
            match SPECS[*s].kind {
                Kind::Garbage => return Expect::Refused(format!("{name}: "), ""),
                Kind::NoEntry => return Expect::Refused(format!("{name}: "), "engine_mod_info"),
                Kind::WrongApi => return Expect::Refused(format!("{name} was built against mod API v"), ""),
                Kind::Keeper => {
                    let tables = SPECS[*s].v == "s";
                    let first = *self.flag_in_tables.get_or_insert(tables);
                    if first != tables {
                        let storage = |t: bool| if t { "Table" } else { "Sparse" };
                        return Expect::Exactly(Err(format!(
                            "{name}: fz::Flag is stored as {}, and this build stores it as {}; restart the engine to change a component's storage",
                            storage(first),
                            storage(tables)
                        )));
                    }
                }
                _ => {}
            }
            opened.push((name.clone(), *s));
        }
        let problems = self.check_links(&opened);
        if !problems.is_empty() {
            return Expect::Exactly(Err(problems.join("; ")));
        }
        let mut report = self.commit(dependency_order(opened));
        report.extend(kept);
        report.extend(unchanged);
        Expect::Exactly(Ok(report.join("; ")))
    }

    /// Why the mods after the batch couldn't run together, in the order
    /// the loader finds it (mod-deps.md, "What the engine enforces";
    /// hot-reload.md, "Resident mods"): every dependency loaded, with the
    /// interface each dependent was built against; residency flowing
    /// downward; one provider per service.
    fn check_links(&self, opened: &[(String, usize)]) -> Vec<String> {
        let in_batch = |name: &str| opened.iter().find(|(n, _)| n == name).map(|(_, s)| &SPECS[*s]);
        let mut after: Vec<(&str, &Spec, bool)> =
            self.mods.iter().filter(|m| in_batch(&m.name).is_none()).map(|m| (m.name.as_str(), m.spec(), false)).collect();
        after.extend(opened.iter().map(|(n, s)| (n.as_str(), &SPECS[*s], true)));
        let resident_after = |name: &str| match in_batch(name) {
            Some(s) => s.resident,
            None => self.find(name).is_some_and(|m| m.spec().resident),
        };
        let resident_now = |name: &str| self.find(name).is_some_and(|m| m.spec().resident);
        let mut problems = Vec::new();
        let mut stranded: Vec<(&str, Vec<&str>)> = Vec::new();
        for &(name, spec, batched) in &after {
            if resident_after(name) {
                for (dep, _) in spec.deps.iter().filter(|(dep, _)| !resident_after(dep)) {
                    problems.push(format!("{name} is resident, so everything it depends on must be too; {dep} isn't"));
                }
            }
            for &(dep, iface) in spec.deps {
                match after.iter().find(|a| a.0 == dep) {
                    None => problems.push(format!("{name} depends on {dep}, which isn't loaded")),
                    Some(&(_, found, dep_batched)) if found.iface != iface => {
                        if dep_batched && !batched {
                            match stranded.iter_mut().find(|(d, _)| *d == dep) {
                                Some((_, names)) => names.push(name),
                                None => stranded.push((dep, vec![name])),
                            }
                        } else {
                            let restart =
                                if !dep_batched && resident_now(dep) { format!("; {}", resident_note(dep)) } else { String::new() };
                            let which = if dep_batched { "new" } else { "running" };
                            problems.push(format!("{name} was built against a different interface of {dep} than the {which} one{restart}"));
                        }
                    }
                    Some(_) => {}
                }
            }
        }
        for (dep, names) in stranded {
            let were = if names.len() == 1 { "was" } else { "were" };
            problems.push(format!(
                "{dep}'s interface changed, and {} {were} built against the old one; reload them in the same batch",
                names.join(", ")
            ));
        }
        let mut providers: Vec<(&str, &str)> = Vec::new();
        for &(name, spec, _) in &after {
            let Some(service) = spec.provides else { continue };
            match providers.iter().find(|(s, _)| *s == service) {
                Some((_, other)) => problems.push(format!("{name} and {other} both provide {service}")),
                None => providers.push((service, name)),
            }
        }
        problems
    }

    /// Swaps in a checked batch, in dependency order: each replaced build's
    /// state handed over (dependents first), every new build's layouts
    /// installed, then each new build loaded. What the loader reports.
    fn commit(&mut self, order: Vec<(String, usize)>) -> Vec<String> {
        let mut notes: Vec<(String, String)> = Vec::new();
        for (name, s) in order.iter().rev() {
            let Some(m) = self.mods.iter_mut().find(|m| m.name == *name) else { continue };
            let (old, new) = (state_layout(m.spec()), state_layout(&SPECS[*s]));
            if old == new {
                continue;
            }
            let note = if old.1 == new.1 {
                format!("state migrated: {}", migrate(&mut m.state, &SPECS[*s]))
            } else {
                m.state = fresh(&SPECS[*s]);
                "state layout changed so state was reset".to_string()
            };
            notes.push((name.clone(), note));
        }
        let ats: Vec<u64> = order
            .iter()
            .map(|_| {
                self.loads += 1;
                self.loads
            })
            .collect();
        for ((_, s), &at) in order.iter().zip(&ats) {
            self.install(&SPECS[*s], at);
        }
        let mut report = Vec::new();
        for ((name, s), &at) in order.iter().zip(&ats) {
            match self.mods.iter_mut().find(|m| m.name == *name) {
                Some(m) => {
                    (m.spec, m.at, m.failed) = (*s, at, false);
                    m.generation += 1;
                    let note = notes.iter().find(|(n, _)| n == name).map_or(String::new(), |(_, n)| format!(", {n}"));
                    report.push(format!("reloaded {name} (generation {}{note})", m.generation));
                }
                None => {
                    let state = fresh(&SPECS[*s]);
                    self.mods.push(Mod { name: name.clone(), spec: *s, at, generation: 0, failed: false, state });
                    report.push(format!("loaded {name}"));
                }
            }
        }
        for (name, _) in &order {
            self.on_load(name);
        }
        report
    }

    /// What a build's load installs: the components and events its systems
    /// name. A newer build takes a component over (ecs.md, "Layout
    /// changes"): same layout, its code replaces the older build's; another
    /// layout at the same version, the values migrate field by field;
    /// another version, they reset to its `Default`. An event whose layout
    /// changed drops what's queued.
    fn install(&mut self, s: &Spec, at: u64) {
        match s.kind {
            Kind::Keeper => {
                self.install_item(ItemLayout::of(s.v), at);
                self.install_flag(FlagLayout::of(s.v), at);
            }
            Kind::Ranker => {
                let layout = if s.v == "c" { RankLayout::C } else { RankLayout::A };
                // Values below u32::MAX, so `n` converts both ways intact.
                self.rank = Some((layout, if s.v == "b" { Glue::Down } else { Glue::Up }, at));
            }
            Kind::Herald | Kind::Hearer => {
                let v2 = s.v == "c";
                if self.note.is_some_and(|(layout, _)| layout != v2) {
                    self.notes.clear();
                }
                self.note = Some((v2, at));
            }
            _ => {}
        }
    }

    fn install_item(&mut self, layout: ItemLayout, at: u64) {
        if let Some((current, _)) = self.item.filter(|(current, _)| *current != layout) {
            for item in &mut self.items {
                let flag = item.flag.take();
                let mut new = layout.default_item();
                if current.version() == layout.version() {
                    // By name: `id`, `name` and `count` carry over (`count`
                    // converted), the rest is the new layout's default.
                    (new.id, new.count) = (item.id, layout.count(item.count));
                    new.name = std::mem::take(&mut item.name);
                }
                *item = Item { flag, ..new };
            }
        }
        self.item = Some((layout, at));
    }

    fn install_flag(&mut self, layout: FlagLayout, at: u64) {
        if let Some((current, _)) = self.flag.filter(|(current, _)| *current != layout) {
            assert!(current != FlagLayout::S && layout != FlagLayout::S, "model: Flag in tables and sparse");
            for flag in self.items.iter_mut().filter_map(|i| i.flag.as_mut()) {
                flag.level = layout.level(flag.level);
                if layout == FlagLayout::C {
                    flag.seen = true;
                }
            }
        }
        self.flag = Some((layout, at));
    }

    /// What a build's `load` does.
    fn on_load(&mut self, name: &str) {
        let at_load = match self.call_herald(None) {
            Ok(build) => build,
            Err(kind) => format!("err {kind}"),
        };
        let i = self.position(name).expect("just loaded");
        let m = &mut self.mods[i];
        let v = m.spec().v;
        match &mut m.state {
            State::Keeper { .. } if v == "e" => m.failed = true,
            State::Keeper { loads, .. } | State::Herald { loads, .. } | State::Ranker { loads, .. } | State::Anchor { loads, .. } => {
                *loads += 1
            }
            State::Hearer { loads, at_load: at, .. } => {
                *loads += 1;
                *at = at_load;
            }
            _ => {}
        }
    }

    /// A call to herald's service, `bump` or (with `None`) `build`: the
    /// answer, or the `CallErrorKind` it fails with.
    fn call_herald(&mut self, bump: Option<u64>) -> Result<String, &'static str> {
        let Some(p) = self.mods.iter_mut().find(|m| m.spec().provides == Some(HERALD)) else {
            return Err("NotProvided");
        };
        if p.failed {
            return Err("ProviderFailed");
        }
        let v = SPECS[p.spec].v;
        let Some(by) = bump else { return Ok(format!("herald:{v}")) };
        if v == "p" {
            p.failed = true;
            return Err("Panicked");
        }
        let State::Herald { calls, .. } = &mut p.state else { unreachable!("a herald's state") };
        *calls = calls.wrapping_add(by);
        Ok(calls.to_string())
    }

    fn unload(&mut self, name: &str) -> Expect {
        let Some(i) = self.position(name) else { return Expect::Exactly(Err(format!("{name} is not loaded"))) };
        if self.mods[i].spec().resident {
            return Expect::Exactly(Err(format!("{name} is resident: it unloads when the engine exits")));
        }
        let dependents: Vec<&str> =
            self.mods.iter().filter(|m| m.spec().deps.iter().any(|(d, _)| *d == name)).map(|m| m.name.as_str()).collect();
        if !dependents.is_empty() {
            return Expect::Exactly(Err(format!("{name} is needed by {}; unload them first", dependents.join(", "))));
        }
        self.mods.remove(i);
        Expect::Exactly(Ok(format!("unloaded {name}")))
    }

    /// One frame: each phase's systems in load order, those of failed mods
    /// skipped. hearer reads in `input`, keeper ticks and herald emits in
    /// `update` (published as `emit` returns), hearer reads again and
    /// ranker reports in `late`. Then the notes published before this
    /// frame expire (storage.md, "Events").
    fn frame(&mut self) {
        self.frame += 1;
        for i in 0..self.mods.len() {
            if !self.mods[i].failed && self.mods[i].spec().kind == Kind::Hearer {
                self.read(i, "early");
            }
        }
        for i in 0..self.mods.len() {
            if self.mods[i].failed {
                continue;
            }
            let v = self.mods[i].spec().v;
            match self.mods[i].spec().kind {
                Kind::Keeper => {
                    let (layout, _) = self.item.expect("keeper installed fz::Item");
                    assert_eq!(layout, ItemLayout::of(v), "model: keeper ticks another layout");
                    let step = match v {
                        "b" => 2,
                        "c" => 3,
                        "d" => 5,
                        _ => 1,
                    };
                    for item in &mut self.items {
                        item.count = layout.count(item.count.wrapping_add(step));
                    }
                }
                Kind::Herald => {
                    let State::Herald { pending, .. } = &mut self.mods[i].state else { unreachable!() };
                    let mut pending = std::mem::take(pending);
                    if v == "b" {
                        pending.reverse();
                    }
                    for n in pending {
                        self.publish(n, v, self.frame);
                    }
                }
                _ => {}
            }
        }
        for i in 0..self.mods.len() {
            if self.mods[i].failed {
                continue;
            }
            match self.mods[i].spec().kind {
                Kind::Hearer => self.read(i, "late"),
                Kind::Ranker => {
                    let order = self.rank_order();
                    let State::Ranker { seen, .. } = &mut self.mods[i].state else { unreachable!() };
                    *seen = order;
                }
                _ => {}
            }
        }
        let frame = self.frame;
        self.notes.retain(|n| n.frame >= frame);
    }

    fn publish(&mut self, n: u64, v: &str, frame: u64) {
        let weight = (v == "c").then_some(7);
        self.notes.push(Note { seq: self.next_seq, frame, n, from: format!("herald:{v}"), weight });
        self.next_seq += 1;
    }

    /// hearer's system `system` reading the notes it hasn't: those from
    /// its cursor on, which then moves past the last one queued.
    fn read(&mut self, i: usize, system: &str) {
        let key = format!("{}::{system}", self.mods[i].name);
        let cursor = self.cursors.get(&key).copied().unwrap_or(0);
        let v = self.mods[i].spec().v;
        let seen: Vec<String> = self.notes.iter().filter(|n| n.seq >= cursor).map(|n| hearer_entry(v, n)).collect();
        if let Some(last) = self.notes.last() {
            self.cursors.insert(key, last.seq + 1);
        }
        let State::Hearer { early, late, heard, .. } = &mut self.mods[i].state else { unreachable!() };
        if let Some(heard) = heard {
            *heard += seen.len() as u64;
        }
        if system == "early" { early } else { late }.extend(seen);
    }

    /// The ranks in their table's order: by the glue it was last sorted with.
    fn rank_order(&self) -> Vec<u64> {
        let mut ranks = self.ranks.clone();
        if let Some(glue) = self.sorted_by {
            ranks.sort_by_key(|&n| glue.key(n));
        }
        ranks
    }

    /// Something re-sorted the rank table: with the glue installed now.
    fn resort(&mut self) {
        self.sorted_by = self.rank.map(|(_, glue, _)| glue);
    }

    /// `Engine::send`: the mod's reply, or why it has none.
    fn send(&mut self, name: &str, message: &str) -> Expect {
        let Some(i) = self.position(name) else { return Expect::Exactly(Err(format!("{name} is not loaded"))) };
        if self.mods[i].failed {
            return Expect::Exactly(Err(format!("{name} has failed; reload it first")));
        }
        let words: Vec<&str> = message.split(' ').collect();
        if words[0] == "boom" && self.mods[i].spec().kind != Kind::Lockstep {
            // Every test mod panics on it, which fails it.
            self.mods[i].failed = true;
            return Expect::Exactly(Err(format!("{name} failed handling the message")));
        }
        let num = |k: usize| words.get(k).and_then(|w| w.parse::<u64>().ok()).expect("the driver sends numbers");
        let v = self.mods[i].spec().v;
        Expect::Exactly(match self.mods[i].spec().kind {
            Kind::Keeper => self.keeper(i, &words),
            Kind::Herald => match words[0] {
                "now" => {
                    self.publish(num(1), v, self.frame + 1);
                    Ok(format!("herald:{v} sent {}", num(1)))
                }
                "queue" => {
                    let State::Herald { pending, .. } = &mut self.mods[i].state else { unreachable!() };
                    pending.push(num(1));
                    Ok(format!("herald:{v} queued {}", pending.len()))
                }
                _ => {
                    let State::Herald { calls, pending, loads } = &self.mods[i].state else { unreachable!() };
                    Ok(format!("herald:{v} calls={calls} loads={loads} pending={pending:?}"))
                }
            },
            Kind::Hearer => match words[0] {
                "ask" => Ok(match self.call_herald(Some(num(1))) {
                    Ok(calls) => format!("hearer:{v} got {calls}"),
                    Err(kind) => format!("hearer:{v} err {kind}"),
                }),
                "clear" => {
                    let State::Hearer { early, late, .. } = &mut self.mods[i].state else { unreachable!() };
                    early.clear();
                    late.clear();
                    Ok(format!("hearer:{v} cleared"))
                }
                _ => {
                    let State::Hearer { early, late, at_load, loads, heard } = &self.mods[i].state else { unreachable!() };
                    let heard = heard.map_or("-".to_string(), |h| h.to_string());
                    Ok(format!(
                        "hearer:{v} loads={loads} at_load={at_load} heard={heard} early=[{}] late=[{}]",
                        early.join(","),
                        late.join(",")
                    ))
                }
            },
            Kind::Ranker => self.ranker(i, &words),
            Kind::Anchor => {
                let State::Anchor { pings, loads } = &mut self.mods[i].state else { unreachable!() };
                if words[0] == "ping" {
                    *pings += 1;
                }
                Ok(format!("anchor:{v} pings={pings} loads={loads}"))
            }
            Kind::Tether => {
                let State::Tether { pings } = &mut self.mods[i].state else { unreachable!() };
                if words[0] == "ping" {
                    *pings += 1;
                }
                Ok(format!("tether:anchor pings={pings}"))
            }
            Kind::Lockstep => {
                if words[0] == "step" {
                    for _ in 0..num(1) {
                        let State::Lockstep { frame } = &mut self.mods[i].state else { unreachable!() };
                        *frame += 1;
                        // Published before each frame, on an entity of its own.
                        self.clock = true;
                        self.frame();
                    }
                }
                let State::Lockstep { frame } = &self.mods[i].state else { unreachable!() };
                Ok(format!("frame {frame}"))
            }
            _ => Err(format!("{name} doesn't take messages")),
        })
    }

    fn keeper(&mut self, i: usize, words: &[&str]) -> Result<String, String> {
        let v = self.mods[i].spec().v;
        let (layout, flags) = (ItemLayout::of(v), FlagLayout::of(v));
        let num = |k: usize| words.get(k).and_then(|w| w.parse::<u64>().ok()).expect("the driver sends numbers");
        let word = |k: usize| words.get(k).map(|w| w.to_string()).unwrap_or_default();
        let id = || num(1) as u32;
        let matching = |items: &mut Vec<Item>| items.iter_mut().filter(|item| item.id == id()).count();
        match words[0] {
            "dump" => {
                let State::Keeper { loads, spawned, journal, extra } = &self.mods[i].state else { unreachable!() };
                let extra = extra.map_or("-".to_string(), |x| x.to_string());
                Ok(format!(
                    "keeper:{v} loads={loads} spawned={spawned} extra={extra} journal=[{}] items=[{}]",
                    journal.join(","),
                    self.describe_items().join(" ")
                ))
            }
            "spawn" => {
                self.items.push(Item { id: id(), name: word(2), ..layout.default_item() });
                let State::Keeper { spawned, journal, .. } = &mut self.mods[i].state else { unreachable!() };
                *spawned += 1;
                journal.push(format!("{v}+{}", id()));
                Ok(format!("keeper:{v} spawned {}", id()))
            }
            "despawn" => {
                let before = self.items.len();
                self.items.retain(|item| item.id != id());
                Ok(format!("keeper:{v} despawned {}", before - self.items.len()))
            }
            "tag" if layout != ItemLayout::A => Err(format!("keeper:{v} has no tags")),
            "rename" | "bulk" | "tag" | "flag" | "unflag" => {
                let n = matching(&mut self.items);
                for item in self.items.iter_mut().filter(|item| item.id == id()) {
                    match words[0] {
                        "rename" => item.name = word(2),
                        "bulk" => item.count = layout.count(item.count.wrapping_add(num(2))),
                        "tag" => item.tags.push(word(2)),
                        "flag" => {
                            let level = flags.level(num(2) as i64);
                            item.flag = Some(Flag { level, note: format!("{v}{level}"), seen: false });
                        }
                        _ => item.flag = None,
                    }
                }
                let did = match words[0] {
                    "rename" => "renamed",
                    "bulk" => "bulked",
                    "tag" => "tagged",
                    "flag" => "flagged",
                    _ => "unflagged",
                };
                Ok(format!("keeper:{v} {did} {n}"))
            }
            _ => Err(format!("keeper doesn't understand {:?}", words.join(" "))),
        }
    }

    /// Every item as keeper describes it, in the layouts installed, sorted.
    fn describe_items(&self) -> Vec<String> {
        let (Some((items, _)), Some((flags, _))) = (self.item, self.flag) else { return Vec::new() };
        let mut all: Vec<String> = self.items.iter().map(|item| describe(item, items, flags)).collect();
        all.sort();
        all
    }

    fn ranker(&mut self, i: usize, words: &[&str]) -> Result<String, String> {
        let v = self.mods[i].spec().v;
        let num = |k: usize| words.get(k).and_then(|w| w.parse::<u64>().ok()).expect("the driver sends numbers");
        match words[0] {
            "add" if self.ranks.contains(&num(1)) => Err(format!("ranker:{v} has {}", num(1))),
            "add" => {
                self.ranks.push(num(1));
                self.resort();
                Ok(format!("ranker:{v} added {}", num(1)))
            }
            "set" if !self.ranks.contains(&num(1)) || self.ranks.contains(&num(2)) => {
                Err(format!("ranker:{v} can't set {} to {}", num(1), num(2)))
            }
            "set" => {
                let (from, to) = (num(1), num(2));
                self.ranks.iter_mut().filter(|n| **n == from).for_each(|n| *n = to);
                self.resort();
                Ok(format!("ranker:{v} set {from} to {to}"))
            }
            "del" => {
                let before = self.ranks.len();
                self.ranks.retain(|&n| n != num(1));
                let deleted = before - self.ranks.len();
                if deleted > 0 {
                    self.resort();
                }
                Ok(format!("ranker:{v} deleted {deleted}"))
            }
            _ => {
                let State::Ranker { seen, loads } = &self.mods[i].state else { unreachable!() };
                Ok(format!("ranker:{v} loads={loads} seen={seen:?}"))
            }
        }
    }

    /// The builds the engine should have mapped: each running build, and
    /// each build whose code the world holds, having installed the layout
    /// of a component or event it still stores.
    fn mapped(&self) -> Vec<u64> {
        let mut builds: Vec<u64> = self.mods.iter().map(|m| m.at).collect();
        builds.extend(self.item.map(|(_, at)| at));
        builds.extend(self.flag.map(|(_, at)| at));
        builds.extend(self.rank.map(|(_, _, at)| at));
        builds.extend(self.note.map(|(_, at)| at));
        builds.sort();
        builds.dedup();
        builds
    }
}

/// An item as keeper's dump (and the driver's reading of the world) writes
/// it, in the layouts installed.
fn describe(item: &Item, items: ItemLayout, flags: FlagLayout) -> String {
    let flag = match (&item.flag, flags) {
        (None, _) => "-".to_string(),
        (Some(f), FlagLayout::C) => format!("{}:{}:{}", f.level, f.note, f.seen),
        (Some(f), _) => format!("{}:{}", f.level, f.note),
    };
    match items {
        ItemLayout::A => format!("{}/{}/{}/{}/{flag}", item.id, item.count, item.name, item.tags.join(",")),
        _ => format!("{}/{}/{}/w{}/{flag}", item.id, item.count, item.name, item.weight),
    }
}

// ---- Reading the engine ----

/// Every item in the world, with its flag, read through the mirrors of the
/// layouts installed and described as keeper describes them.
fn world_items(world: &engine_api::World, items: ItemLayout, flags: FlagLayout) -> Result<Vec<String>, String> {
    use mirror::*;
    let missing = |what: &str| format!("{what} isn't installed with the model's layout");
    let flag = |level: i64, note: String, seen: bool| Flag { level, note, seen };
    let flag_of: HashMap<engine_api::Entity, Flag> = match flags {
        FlagLayout::A => {
            world.values::<FlagA>().ok_or(missing("fz::Flag"))?.into_iter().map(|(e, f)| (e, flag(f.level as i64, f.note, false))).collect()
        }
        FlagLayout::C => world
            .values::<FlagC>()
            .ok_or(missing("fz::Flag"))?
            .into_iter()
            .map(|(e, f)| (e, flag(f.level as i64, f.note, f.seen)))
            .collect(),
        FlagLayout::S => {
            world.values::<FlagS>().ok_or(missing("fz::Flag"))?.into_iter().map(|(e, f)| (e, flag(f.level as i64, f.note, false))).collect()
        }
    };
    let item = |id: u32, count: u64, name: String, tags: Vec<String>, weight: f64| Item { id, count, name, tags, weight, flag: None };
    let found: Vec<(engine_api::Entity, Item)> = match items {
        ItemLayout::A => world
            .values::<ItemA>()
            .ok_or(missing("fz::Item"))?
            .into_iter()
            .map(|(e, i)| (e, item(i.id, i.count as u64, i.name, i.tags, 0.0)))
            .collect(),
        ItemLayout::C => world
            .values::<ItemC>()
            .ok_or(missing("fz::Item"))?
            .into_iter()
            .map(|(e, i)| (e, item(i.id, i.count, i.name, Vec::new(), i.weight)))
            .collect(),
        ItemLayout::D => world
            .values::<ItemD>()
            .ok_or(missing("fz::Item"))?
            .into_iter()
            .map(|(e, i)| (e, item(i.id, i.count, i.name, Vec::new(), i.weight)))
            .collect(),
    };
    let flagged = found.iter().filter(|(e, _)| flag_of.contains_key(e)).count();
    if flagged != flag_of.len() {
        return Err(format!("{} flag(s) on entities without an item", flag_of.len() - flagged));
    }
    let mut all: Vec<String> = found
        .into_iter()
        .map(|(e, mut item)| {
            item.flag = flag_of.get(&e).cloned();
            describe(&item, items, flags)
        })
        .collect();
    all.sort();
    Ok(all)
}

/// The ranks in the world, in their table's order.
fn world_ranks(world: &engine_api::World, layout: RankLayout) -> Option<Vec<u64>> {
    Some(match layout {
        RankLayout::A => world.values::<mirror::RankA>()?.into_iter().map(|(_, r)| r.n as u64).collect(),
        RankLayout::C => world.values::<mirror::RankC>()?.into_iter().map(|(_, r)| r.n).collect(),
    })
}

/// How many builds staged in `dir` are mapped: each has a file of its own
/// there, deleted once opened, which the maps still name.
fn mapped_builds(dir: &Path) -> usize {
    let maps = std::fs::read_to_string("/proc/self/maps").expect("/proc/self/maps");
    let dir = format!("{}/", dir.display());
    let mut paths: Vec<&str> =
        maps.lines().filter_map(|line| Some(line.splitn(6, ' ').nth(5)?.trim_start())).filter(|path| path.starts_with(&dir)).collect();
    paths.sort();
    paths.dedup();
    paths.len()
}

/// A directory of the session's own to stage builds in, so what it maps is
/// told apart from other sessions', and named as the maps name it.
fn staging_dir() -> PathBuf {
    static SESSIONS: AtomicU64 = AtomicU64::new(0);
    let base = std::env::var_os("TEST_TMPDIR").map(PathBuf::from).unwrap_or_else(std::env::temp_dir);
    let n = SESSIONS.fetch_add(1, Ordering::Relaxed);
    let dir = base.join(format!("reload-fuzz-{}-{n}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap_or_else(|e| panic!("{}: {e}", dir.display()));
    std::fs::canonicalize(&dir).expect("the staging directory")
}

// ---- Operations ----

#[derive(Debug)]
enum Op {
    Load(String, usize),
    Batch(Vec<(String, usize)>),
    Unload(String),
    /// Frames the loader runs itself (`Engine::step_all`); lockstep's are
    /// messages.
    Frames(u64),
    Send(String, String),
}

const WORDS: [&str; 4] = ["x", "yy", "anon", "z9"];

fn pick<T: Copy>(ch: &mut impl Choices, from: &[T]) -> T {
    from[ch.below(from.len() as u64) as usize]
}

/// A build to load, and the name to load it as: its own mod's, but a bad
/// build's is any, and a herald's is sometimes a second name, which can't
/// provide the service herald does.
fn target(ch: &mut impl Choices) -> (String, usize) {
    // By mod, then by build: the mods that only matter together (herald and
    // hearer, anchor and tether) as often as the rest, whatever their
    // number of builds. Picking a build straight from all of them, the
    // fuzzer went 30 minutes without once reloading herald between two of
    // hearer's reads of a queued note.
    let kinds: &[Kind] = match ch.below(20) {
        0..4 => &[Kind::Keeper],
        4..8 => &[Kind::Herald],
        8..12 => &[Kind::Hearer],
        12..15 => &[Kind::Ranker],
        15..17 => &[Kind::Anchor, Kind::Tether],
        17..19 => &[Kind::Clock, Kind::Lockstep, Kind::Sequential],
        _ => &[Kind::Garbage, Kind::NoEntry, Kind::WrongApi],
    };
    let of: Vec<usize> = (0..SPECS.len()).filter(|&s| kinds.contains(&SPECS[s].kind)).collect();
    let s = pick(ch, &of);
    let name = match SPECS[s].kind {
        Kind::Garbage | Kind::NoEntry | Kind::WrongApi => pick(ch, &NAMES),
        Kind::Herald if ch.below(6) == 0 => "herald2",
        _ => SPECS[s].name,
    };
    (name.to_string(), s)
}

fn choose(ch: &mut impl Choices) -> Op {
    match ch.below(100) {
        0..22 => {
            let (name, s) = target(ch);
            Op::Load(name, s)
        }
        22..30 => Op::Batch((0..2 + ch.below(3)).map(|_| target(ch)).collect()),
        30..35 => Op::Unload(pick(ch, &NAMES).to_string()),
        35..47 => Op::Frames(1 + ch.below(3)),
        47..53 => Op::Send("lockstep".into(), format!("step {}", 1 + ch.below(3))),
        _ => {
            let (to, message) = message(ch);
            Op::Send(to.into(), message)
        }
    }
}

/// A message some mod takes; `boom`, which fails it, now and then.
fn message(ch: &mut impl Choices) -> (&'static str, String) {
    let id = ch.below(4);
    match ch.below(20) {
        0..6 => (
            "keeper",
            match ch.below(20) {
                0..4 => format!("spawn {id} {}", pick(ch, &WORDS)),
                4..6 => format!("despawn {id}"),
                6..8 => format!("rename {id} {}", pick(ch, &WORDS)),
                // Now and then far past what a u32 holds, so migrating the
                // count between widths shows.
                8..10 => {
                    let by = if ch.below(4) == 0 { ch.word() } else { ch.below(10) };
                    format!("bulk {id} {by}")
                }
                10..12 => format!("tag {id} {}", pick(ch, &WORDS)),
                12..14 => format!("flag {id} {}", ch.below(600)),
                14 => format!("unflag {id}"),
                15..19 => "dump".into(),
                _ => "boom".into(),
            },
        ),
        6..10 => (
            if ch.below(4) == 0 { "herald2" } else { "herald" },
            match ch.below(12) {
                0..5 => format!("now {}", ch.below(100)),
                5..10 => format!("queue {}", ch.below(100)),
                10 => "get".into(),
                _ => "boom".into(),
            },
        ),
        10..13 => (
            "hearer",
            match ch.below(12) {
                0..4 => "dump".into(),
                4..6 => "clear".into(),
                6..11 => format!("ask {}", ch.below(1000)),
                _ => "boom".into(),
            },
        ),
        13..17 => (
            "ranker",
            match ch.below(16) {
                0..6 => format!("add {}", ch.below(8)),
                6..10 => format!("set {} {}", ch.below(8), ch.below(8)),
                10..13 => format!("del {}", ch.below(8)),
                13..15 => "dump".into(),
                _ => "boom".into(),
            },
        ),
        17 => (pick(ch, &["anchor", "tether"]), pick(ch, &["ping", "get"]).to_string()),
        _ => (pick(ch, &["lockstep", "clock", "sequential"]), "frame".to_string()),
    }
}

// ---- Sessions ----

/// What a session did, for tests that a seeded run reaches what it should:
/// how often each kind of reply came back.
#[derive(Default, Debug)]
pub struct Stats {
    pub ops: usize,
    /// Replies, by what they said: `loaded`, `state migrated`, `is
    /// resident`, and so on (see `TALLIED`).
    pub replies: HashMap<&'static str, usize>,
}

/// What `Stats` counts replies by.
pub const TALLIED: &[&str] = &[
    "loaded ",
    "reloaded ",
    "unloaded ",
    " unchanged",
    "state migrated",
    "state layout changed",
    "is resident: restart",
    "is resident: it unloads",
    "is resident, so everything",
    "isn't loaded",
    "is not loaded",
    "interface changed",
    "a different interface",
    "both provide",
    "is needed by",
    "in the batch twice",
    "restart the engine to change a component's storage",
    "mod API",
    "engine_mod_info",
    "has failed; reload it first",
    "failed handling the message",
    "err Panicked",
    "err ProviderFailed",
    "has no tags",
    "@herald",
];

struct Session {
    engine: Box<Engine>,
    dir: PathBuf,
    model: Model,
    /// Every operation so far, with the engine's reply, for the report.
    log: Vec<String>,
    stats: Stats,
}

/// Runs one session of up to `steps` operations from `ch`, checking each,
/// and panics at the first difference from the model, with the session so
/// far. A fuzzer's input runs out before `steps` does.
pub fn run(ch: &mut impl Choices, steps: usize) -> Stats {
    let mut left = steps;
    run_ops(&mut || {
        if left == 0 || ch.done() {
            return None;
        }
        left -= 1;
        Some(choose(ch))
    })
}

fn run_ops(next: &mut dyn FnMut() -> Option<Op>) -> Stats {
    let dir = staging_dir();
    let mut s = Session { engine: Engine::new(None, dir.clone()), dir, model: Model::default(), log: Vec::new(), stats: Stats::default() };
    while let Some(op) = next() {
        s.apply(op);
        s.check();
        s.stats.ops += 1;
    }
    let Session { engine, dir, log, stats, .. } = s;
    // Closing every mod, and dropping the world's values, lets go of every
    // build.
    drop(engine);
    let left = mapped_builds(&dir);
    assert!(left == 0, "{left} build(s) still mapped after the engine was dropped\n{}", log.join("\n"));
    let _ = std::fs::remove_dir_all(&dir);
    stats
}

impl Session {
    fn fail(&self, what: &str) -> ! {
        let from = self.log.len().saturating_sub(60);
        panic!(
            "reload fuzz: after {} operation(s), {what}\nthe session (last {} operations):\n  {}",
            self.log.len(),
            self.log.len() - from,
            self.log[from..].join("\n  ")
        )
    }

    fn apply(&mut self, op: Op) {
        let libs = libs();
        if trace() {
            eprintln!("[reload fuzz] {}", op.script());
        }
        let (actual, expected) = match &op {
            Op::Load(name, s) => (self.engine.load(name, &libs[*s]), self.model.load(name, *s)),
            Op::Batch(batch) => {
                let paths: Vec<(String, PathBuf)> = batch.iter().map(|(n, s)| (n.clone(), libs[*s].clone())).collect();
                (self.engine.load_batch(&paths), self.model.batch(batch))
            }
            Op::Unload(name) => (self.engine.unload(name), self.model.unload(name)),
            Op::Frames(n) => {
                for _ in 0..*n {
                    self.engine.step_all();
                    self.model.frame();
                }
                (Ok(String::new()), Expect::Exactly(Ok(String::new())))
            }
            Op::Send(name, message) => (self.engine.send(name, message), self.model.send(name, message)),
        };
        let what = op.script();
        self.log.push(format!("{what} -> {actual:?}"));
        let reply = match &actual {
            Ok(r) | Err(r) => r.as_str(),
        };
        for tally in TALLIED.iter().filter(|t| reply.contains(**t)) {
            *self.stats.replies.entry(tally).or_default() += 1;
        }
        let matches = match &expected {
            Expect::Exactly(expected) => actual == *expected,
            Expect::Refused(start, mentions) => actual.as_ref().is_err_and(|e| e.starts_with(start.as_str()) && e.contains(mentions)),
        };
        if !matches {
            self.fail(&format!("{what}\n  replied {actual:?}\n  expected {expected:?}"));
        }
    }

    /// Everything the engine shows, against the model.
    fn check(&mut self) {
        let list = self.engine.list();
        let mut lines = list.lines();
        let mut expected = vec![format!("{} mod(s) loaded", self.model.mods.len())];
        for m in &self.model.mods {
            let resident = if m.spec().resident { " [resident]" } else { "" };
            let failed = if m.failed { " [failed]" } else { "" };
            let deps: Vec<&str> = m.spec().deps.iter().map(|(d, _)| *d).collect();
            let needs = if deps.is_empty() { String::new() } else { format!(" needs {}", deps.join(",")) };
            expected.push(format!("  {} gen {}{resident}{failed}{needs}  ", m.name, m.generation));
        }
        for (i, want) in expected.iter().enumerate() {
            let line = lines.next().unwrap_or("");
            // The mods' lines end with the path they were loaded from.
            if !(line.starts_with(want.as_str()) && (i > 0 || line == want)) {
                self.fail(&format!("the mods listed as\n{list}\nand the model has\n{}", expected.join("\n")));
            }
        }

        let world = self.engine.world();
        let model = &self.model;
        if world.frame() != model.frame {
            self.fail(&format!("the world is at frame {}, and the model at {}", world.frame(), model.frame));
        }
        let entities = model.items.len() + model.ranks.len() + model.clock as usize;
        if world.entities.count() != entities {
            self.fail(&format!("{} entities alive, and the model has {entities}", world.entities.count()));
        }
        if let (Some((items, _)), Some((flags, _))) = (model.item, model.flag) {
            match world_items(world, items, flags) {
                Ok(found) if found == model.describe_items() => {}
                found => self.fail(&format!("the items are\n  {found:?}\nand the model has\n  {:?}", model.describe_items())),
            }
        } else if !model.items.is_empty() {
            self.fail("the model has items with no layout installed");
        }
        if let Some((layout, _, _)) = model.rank {
            let found = world_ranks(world, layout);
            if found.as_ref() != Some(&model.rank_order()) {
                self.fail(&format!("the rank table holds {found:?}, and the model {:?}", model.rank_order()));
            }
        }
        let (mapped, builds) = (mapped_builds(&self.dir), model.mapped());
        if mapped != builds.len() {
            self.fail(&format!("{mapped} build(s) mapped, and the model expects {}: the builds loaded at {builds:?}", builds.len()));
        }

        // Each running mod's own report of itself: none of these change
        // anything, in the engine or the model.
        for i in 0..self.model.mods.len() {
            let m = &self.model.mods[i];
            let message = match m.spec().kind {
                Kind::Keeper | Kind::Hearer | Kind::Ranker => "dump",
                Kind::Herald | Kind::Anchor | Kind::Tether => "get",
                Kind::Lockstep => "frame",
                _ => continue,
            };
            let name = m.name.clone();
            let actual = self.engine.send(&name, message);
            let Expect::Exactly(expected) = self.model.send(&name, message) else { unreachable!() };
            if actual != expected {
                self.fail(&format!("{name} reports\n  {actual:?}\nand the model expects\n  {expected:?}"));
            }
        }
    }
}

/// Whether to print each operation as it starts (`RELOAD_FUZZ_TRACE=1`): a
/// crash leaves no report, and this says what it was doing.
fn trace() -> bool {
    static TRACE: OnceLock<bool> = OnceLock::new();
    *TRACE.get_or_init(|| std::env::var_os("RELOAD_FUZZ_TRACE").is_some())
}

/// Runs a session written out, one operation a line, checked as `run`
/// checks it: for regression tests a person can read. Builds are named by
/// their target (`keeper_a`, `lockstep`); a line is one of
///
/// ```text
/// load <build> [as <name>]
/// batch <build>[=<name>] ...
/// unload <name>
/// frames <n>
/// send <name> <message>
/// ```
pub fn script(text: &str) -> Stats {
    let build = |target: &str| {
        let var = format!("FZ_{}", target.to_uppercase());
        SPECS.iter().position(|s| s.var == var).unwrap_or_else(|| panic!("no build {target}"))
    };
    let named = |s: usize, name: Option<&str>| name.unwrap_or(SPECS[s].name).to_string();
    let ops: Vec<Op> = text
        .lines()
        .map(str::trim)
        .filter(|l| !l.is_empty() && !l.starts_with('#'))
        .map(|line| {
            let words: Vec<&str> = line.split_whitespace().collect();
            match words[0] {
                "load" => {
                    let s = build(words[1]);
                    Op::Load(named(s, words.get(3).copied()), s)
                }
                "batch" => Op::Batch(
                    words[1..]
                        .iter()
                        .map(|w| {
                            let (target, name) = w.split_once('=').map_or((*w, None), |(t, n)| (t, Some(n)));
                            let s = build(target);
                            (named(s, name), s)
                        })
                        .collect(),
                ),
                "unload" => Op::Unload(words[1].into()),
                "frames" => Op::Frames(words[1].parse().expect("a frame count")),
                "send" => Op::Send(words[1].into(), words[2..].join(" ")),
                other => panic!("not an operation: {other}"),
            }
        })
        .collect();
    let mut ops = ops.into_iter();
    run_ops(&mut || ops.next())
}

impl Op {
    /// The operation as a line of a `script`.
    fn script(&self) -> String {
        let target = |s: usize| SPECS[s].var.trim_start_matches("FZ_").to_lowercase();
        let named = |name: &str, s: usize, sep: &str| {
            if name == SPECS[s].name { target(s) } else { format!("{}{sep}{name}", target(s)) }
        };
        match self {
            Op::Load(name, s) => format!("load {}", named(name, *s, " as ")),
            Op::Batch(batch) => {
                let each: Vec<String> = batch.iter().map(|(n, s)| named(n, *s, "=")).collect();
                format!("batch {}", each.join(" "))
            }
            Op::Unload(name) => format!("unload {name}"),
            Op::Frames(n) => format!("frames {n}"),
            Op::Send(name, message) => format!("send {name} {message}"),
        }
    }
}
