//! The set of loaded mods and the host services they call back into.
//!
//! Single-threaded. The loader has no loop of its own: the bootstrap mod,
//! which is resident, runs the session in one `Op::RUN` and hands the loader
//! control each frame (`Cx::pump_loader`). Requests queue on a channel until
//! then, and are served only when every mod on the stack is resident, so a
//! reload never swaps out code that is running.

use std::alloc::Layout;
use std::cell::{Cell, OnceCell, RefCell};
use std::collections::HashMap;
use std::rc::Rc;
use std::sync::Arc;
use std::ffi::c_void;
use std::path::{Path, PathBuf};
use std::sync::mpsc::{Receiver, Sender};
use std::time::Duration;

use engine_api::{
    API_VERSION, CallStatus, CallTarget, Declarations, DefaultFn, DropFn, ErasedFn, Host, INFO_SYMBOL,
    BUILD_SYMBOL, BuildFn, InfoFn, MAIN_SYMBOL, MainFn, ModContext, ModInfo, Op, PhaseDesc, Pumped, Status, SystemDesc,
};
use engine_api::scheduler::{FramePlan, PlannedNode, Ran};
use engine_ecs::schema::{self, Field};
use engine_ecs::{Build, Change, FrameCx, Keepalive, Log, Structural, World};
use engine_control::Request;
use libloading::Library;

use crate::schedule::{self, ModDecls, Plan};


pub struct Engine {
    host: Host,
    mods: RefCell<Vec<Loaded>>,
    bootstrap: Option<String>,
    /// The frame being run, while one is open: what a scheduler's
    /// `run_node` ids refer to.
    frame: RefCell<Option<OpenFrame>>,
    staging_dir: PathBuf,
    staged_count: Cell<u64>,
    /// Outlives every mod build: component values keep the builds whose
    /// code they need mapped (see engine_ecs's world.rs).
    world: World,
    /// Source of `ModContext::loaded_at`.
    load_count: Cell<u64>,
    reload_hint: RefCell<Option<String>>,
    /// The reply to the message being delivered, built up by `host_reply`.
    reply: RefCell<String>,
    /// Each loaded mod's current library, by `ModContext` address, for
    /// `library_of`. Separate from `mods` because it is read while `mods` is
    /// borrowed for a load.
    libs: RefCell<HashMap<usize, Arc<Library>>>,
    /// The rustc this engine was built with, which every mod must match.
    rustc: OnceCell<Option<String>>,
    /// Control requests waiting for the next pump; see [`Engine::requests`].
    requests: Sender<Pending>,
    inbox: Receiver<Pending>,
    /// Set once a `quit` request is served.
    quitting: Cell<bool>,
    /// The order systems run in, rebuilt after the builds change.
    plan: RefCell<Option<Rc<Plan>>>,
    /// The time the next frame covers, as its bootstrap said
    /// (`run_frame_for`), and reset to `DEFAULT_FRAME` by it.
    frame_time: Cell<f32>,
    /// Seconds each fixed-rate group has accumulated toward its next step,
    /// by the group's first phase: bookkeeping like the plan's, kept across
    /// reloads. Time policy (real or lockstep) is the bootstrap's.
    accumulated: RefCell<HashMap<String, f64>>,

}

/// A frame in progress: its plan, its nodes in order (each system, then its
/// apply node if it can change the world), and the logs of systems that
/// have run and whose applies haven't.
struct OpenFrame {
    plan: Rc<Plan>,
    nodes: Vec<(FrameNode, String)>,
    logs: HashMap<usize, Vec<Change>>,
}

#[derive(Clone, Copy)]
enum FrameNode {
    /// A system, by its id in the plan, and the seconds its run covers.
    System(usize, f32),
    /// A system's apply node: the system's id, and the node that ran it,
    /// whose log it applies (a fixed-rate system runs more than once).
    Apply(usize, usize),
}

/// A frame whose time its bootstrap didn't give.
const DEFAULT_FRAME: f32 = 1.0 / 60.0;
/// Steps a fixed-rate group may take in one frame: after a long stall, the
/// simulation slows rather than spiralling into ever longer frames.
const MAX_STEPS: u32 = 8;

/// A control request waiting for the engine, and how to answer it.
pub struct Pending {
    pub text: String,
    /// Sends the reply (`ok ...` or `err ...`).
    pub reply: Box<dyn FnOnce(String) + Send>,
}

struct Loaded {
    /// `ctx.name` points into this.
    name: Box<str>,
    ctx: *mut ModContext,
    /// The running build's state schema, and the address ranges its library
    /// occupies, for noticing state that points into it.
    state: StateSchema,
    image: Vec<(usize, usize)>,
    info: ModInfo,
    main: MainFn,
    /// Shared with the world, which keeps a build's library mapped while
    /// component values still need its code.
    lib: Arc<Library>,
    source: PathBuf,
    /// Hash of the library file, to recognize an unchanged build.
    content: u64,
    /// This build's interface digest, and the digests it was built against.
    interface: String,
    deps: Vec<(String, String)>,
    /// The services this build provides.
    services: Vec<Service>,
    /// This build's systems and phases.
    systems: Vec<SystemDesc>,
    phases: Vec<PhaseDesc>,
    /// Loaded once and never swapped: `engine_mod(resident = True)`.
    resident: bool,
    /// Set when the mod returned `Status::ERROR` (e.g. it panicked). Cleared by a reload.
    failed: Cell<bool>,
    /// Set while any of the mod's code is on the stack (a hook, a message, a
    /// call into one of its services), so a call back into it is refused.
    running: Cell<bool>,
}

/// A service a build provides: its name and methods, copied out of the
/// build's `ModInfo`. The methods are that build's code.
struct Service {
    name: String,
    methods: Vec<(String, ErasedFn)>,
}

/// A new build, opened and validated but not yet swapped in.
struct Opened {
    name: String,
    path: PathBuf,
    content: u64,
    lib: Arc<Library>,
    image: Vec<(usize, usize)>,
    info: ModInfo,
    main: MainFn,
    state: StateSchema,
    interface: String,
    deps: Vec<(String, String)>,
    services: Vec<Service>,
    decls: Declarations,
    resident: bool,
}

/// A build's state layout and schema, copied out of its `ModInfo`. `drops`
/// and `default` are that build's code.
struct StateSchema {
    layout: Layout,
    version: u32,
    fields: Vec<Field>,
    drops: Vec<Option<DropFn>>,
    default: DefaultFn,
}

impl Drop for Loaded {
    fn drop(&mut self) {
        unsafe {
            let ctx = Box::from_raw(self.ctx);
            std::alloc::dealloc(ctx.state as *mut u8, self.state.layout);
        }
    }
}

impl Engine {
    pub fn new(bootstrap: Option<String>, staging_dir: PathBuf) -> Box<Engine> {
        let (requests, inbox) = std::sync::mpsc::channel();
        let mut engine = Box::new(Engine {
            host: Host {
                userdata: std::ptr::null_mut(),
                log: host_log,
                step_mods: host_step_mods,
                reply: host_reply,
                begin_call: host_begin_call,
                end_call: host_end_call,
                pump: host_pump,
                begin_frame: host_begin_frame,
                run_node: host_run_node,
                end_frame: host_end_frame,
                set_frame_time: host_set_frame_time,
                world: std::ptr::null(),
                keepalive: host_keepalive,
            },
            mods: RefCell::new(Vec::new()),
            bootstrap,
            frame: RefCell::new(None),
            staging_dir,
            staged_count: Cell::new(0),
            world: World::new(),
            load_count: Cell::new(0),
            reload_hint: RefCell::new(None),
            reply: RefCell::new(String::new()),
            libs: RefCell::new(HashMap::new()),
            rustc: OnceCell::new(),
            requests,
            inbox,
            quitting: Cell::new(false),
            plan: RefCell::new(None),
            frame_time: Cell::new(DEFAULT_FRAME),
            accumulated: RefCell::new(HashMap::new()),
        });
        engine.host.userdata = &*engine as *const Engine as *mut c_void;
        engine.host.world = &engine.world;
        engine
    }

    /// Loads `path` as mod `name`, or hot-reloads it if `name` is already loaded.
    /// A batch of one: see [`Engine::load_batch`].
    pub fn load(&self, name: &str, path: &Path) -> Result<String, String> {
        // Asked for by name, so refuse rather than skip as a batch does.
        if let Some(m) = self.mods.borrow().iter().find(|m| &*m.name == name && m.resident) {
            if m.content != content_hash(path)? {
                return Err(resident_note(name));
            }
        }
        self.load_batch(&[(name.to_string(), path.to_path_buf())])
    }

    /// Loads or reloads every mod in `batch` as one step: either every changed
    /// build is swapped in, or none is and the running builds are untouched.
    ///
    /// A build whose contents match the running one is skipped, so sending a
    /// whole game reloads only the mods an edit affected. The rest are checked
    /// before anything changes: each build must open and match the mod API,
    /// and after the batch every mod must be running against the interfaces
    /// it was built against (see docs/architecture/mod-deps.md).
    pub fn load_batch(&self, batch: &[(String, PathBuf)]) -> Result<String, String> {
        let mut mods = self.mods.borrow_mut();
        let mut opened = Vec::new();
        let mut unchanged = Vec::new();
        let mut kept = Vec::new();
        for (i, (name, path)) in batch.iter().enumerate() {
            if batch[..i].iter().any(|(n, _)| n == name) {
                return Err(format!("{name} is in the batch twice"));
            }
            let content = content_hash(path)?;
            if mods.iter().any(|m| &*m.name == name && m.content == content) {
                unchanged.push(name.as_str());
                continue;
            }
            // A resident mod keeps its build for the session; the rest of the
            // batch still goes ahead, checked against the build that stays.
            if mods.iter().any(|m| &*m.name == name && m.resident) {
                kept.push(resident_note(name));
                continue;
            }
            opened.push(self.open(name, path, content)?);
        }
        self.check_links(&mods, &opened)?;
        let opened = in_dependency_order(opened)?;
        check_plan(&mods, &opened, None)?;
        *self.plan.borrow_mut() = None;

        // Retire the builds being replaced, dependents before what they depend
        // on, and hand each one's state to its successor: as is, migrated, or
        // dropped by the old build and started over.
        let mut notes = Vec::new();
        for o in opened.iter().rev() {
            let Some(m) = mods.iter_mut().find(|m| *m.name == *o.name) else { continue };
            let (old, new) = (&m.state, &o.state);
            let same = (old.layout, old.version, &old.fields) == (new.layout, new.version, &new.fields);
            let migrate = !same && old.version == new.version && !old.fields.is_empty() && !new.fields.is_empty();
            let note = if same || migrate {
                m.call(Op::UNLOAD);
                if m.state_points_into_its_build() {
                    eprintln!(
                        "[engine] {}'s state holds pointers into the build being replaced (a closure, a \
                         trait object, a &'static str?); it is reset instead of carried over. Keep such \
                         things in the mod's Transient.",
                        m.name
                    );
                    m.call(Op::CLOSE);
                    m.fresh_state(new);
                    Some("state held pointers into the old build, so it was reset".to_string())
                } else if migrate {
                    Some(format!("state migrated: {}", m.migrate_state(new)))
                } else {
                    None
                }
            } else {
                m.call(Op::CLOSE);
                m.fresh_state(new);
                Some("state layout changed so state was reset".to_string())
            };
            if let Some(note) = note {
                notes.push((o.name.clone(), note));
            }
        }

        // The new builds' components and events, now the old builds have let
        // go of theirs: a changed layout migrates stored values here. The
        // checks above rule out a refusal (an older layout, a changed storage).
        let loaded_at: Vec<u64> = opened.iter().map(|_| self.next_load()).collect();
        for (o, &loaded_at) in opened.iter().zip(&loaded_at) {
            let build = Build { name: o.name.clone(), loaded_at, keepalive: Some(o.lib.clone() as Keepalive) };
            match engine_ecs::between::install_all(&self.world, &o.decls.components, &o.decls.events, &build) {
                Ok(reports) => reports.iter().for_each(|r| println!("[engine] {r}")),
                Err(e) => eprintln!("[engine] {}: {e}", o.name),
            }
        }

        // Swap in the new builds, then load them, dependencies first.
        let mut report = Vec::new();
        let mut to_load = Vec::new();
        for (o, loaded_at) in opened.into_iter().zip(loaded_at) {
            match mods.iter_mut().find(|m| *m.name == *o.name) {
                Some(m) => {
                    let generation = unsafe {
                        (*m.ctx).generation += 1;
                        (*m.ctx).loaded_at = loaded_at;
                        (*m.ctx).generation
                    };
                    // Releasing the old library unmaps the previous build, unless
                    // the world still needs its code.
                    m.lib = o.lib;
                    self.libs.borrow_mut().insert(m.ctx as usize, m.lib.clone());
                    (m.main, m.info, m.source, m.content) = (o.main, o.info, o.path, o.content);
                    (m.interface, m.deps, m.state, m.image) = (o.interface, o.deps, o.state, o.image);
                    m.services = o.services;
                    (m.systems, m.phases) = (o.decls.systems, o.decls.phases);
                    m.resident = o.resident;
                    m.failed.set(false);
                    to_load.push(m.ctx);
                    let note = match notes.iter().find(|(name, _)| *name == o.name) {
                        Some((_, note)) => format!(", {note}"),
                        None => String::new(),
                    };
                    report.push(format!("reloaded {} (generation {generation}{note})", o.name));
                }
                None => {
                    let name: Box<str> = o.name.as_str().into();
                    let ctx = Box::into_raw(Box::new(ModContext {
                        host: &self.host,
                        name: name.as_ptr(),
                        name_len: name.len(),
                        generation: 0,
                        loaded_at,
                        state: new_state(&o.state),
                        transient: std::ptr::null_mut(),
                        message: std::ptr::null(),
                        message_len: 0,
                    }));
                    let lib = o.lib;
                    self.libs.borrow_mut().insert(ctx as usize, lib.clone());
                    mods.push(Loaded {
                        name,
                        ctx,
                        state: o.state,
                        image: o.image,
                        info: o.info,
                        main: o.main,
                        lib,
                        source: o.path,
                        content: o.content,
                        interface: o.interface,
                        deps: o.deps,
                        services: o.services,
                        systems: o.decls.systems,
                        phases: o.decls.phases,
                        resident: o.resident,
                        failed: Cell::new(false),
                        running: Cell::new(false),
                    });
                    to_load.push(ctx);
                    report.push(format!("loaded {}", o.name));
                }
            }
        }
        // With the list only shared, so a mod's `load` can call services of the
        // mods it depends on, which are already loaded: dependencies first.
        drop(mods);
        let mods = self.mods.borrow();
        for ctx in to_load {
            if let Some(m) = mods.iter().find(|m| m.ctx == ctx) {
                m.call(Op::LOAD);
            }
        }
        report.extend(kept);
        report.extend(unchanged.iter().map(|name| format!("{name} unchanged")));
        Ok(report.join("; "))
    }

    /// Opens a build and reads what it says about itself, without running any
    /// of its mod code.
    fn open(&self, name: &str, path: &Path, content: u64) -> Result<Opened, String> {
        self.check_rustc(name, path)?;
        let (lib, image) = self.open_staged(name, path).map_err(|e| format!("{name}: {e}"))?;
        let info = unsafe {
            let info_fn = *lib.get::<InfoFn>(INFO_SYMBOL).map_err(|e| format!("{name}: {}", describe(e)))?;
            info_fn()
        };
        // Checked before reading any other field: a build from another API
        // version may lay `ModInfo` out differently.
        if info.api_version != API_VERSION {
            return Err(format!(
                "{name} was built against mod API v{}, engine is v{API_VERSION}",
                info.api_version
            ));
        }
        let main = unsafe { *lib.get::<MainFn>(MAIN_SYMBOL).map_err(|e| format!("{name}: {}", describe(e)))? };
        // A zero-sized mod still gets a real allocation: `alloc_zeroed` with a
        // zero-size layout is undefined behavior.
        let layout = Layout::from_size_align(info.state_size.max(1), info.state_align)
            .map_err(|e| format!("{name}: bad state layout: {e}"))?;
        let (fields, drops) = unsafe {
            schema::read_fields(info.state_fields, info.state_field_count, info.state_size, info.state_align)
        }
        .ok_or_else(|| format!("{name}: its state's schema doesn't describe its layout"))?;
        let state = StateSchema { layout, version: info.state_version, fields, drops, default: info.state_default };
        let interface = unsafe { copy_str(info.interface, info.interface_len) };
        let deps = unsafe { copy_str(info.deps, info.deps_len) };
        let deps = deps
            .split(',')
            .filter(|d| !d.is_empty())
            .map(|d| {
                let (dep, digest) = d.split_once(':').ok_or_else(|| format!("{name}: malformed deps {deps:?}"))?;
                Ok((dep.to_string(), digest.to_string()))
            })
            .collect::<Result<_, String>>()?;
        let services = unsafe { read_services(&info) }.map_err(|e| format!("{name}: {e}"))?;
        let mut decls = Declarations::default();
        if !unsafe { (info.declare)(&mut decls as *mut Declarations as *mut c_void, &self.world) } {
            return Err(format!("{name}: declaring its systems panicked"));
        }
        if let Some(e) = decls.error.take() {
            return Err(format!("{name}: {e}"));
        }
        let resident = info.resident;
        if self.bootstrap.as_deref() == Some(name) && !info.bootstrap {
            return Err(bootstrap_note(name));
        }
        Ok(Opened {
            name: name.into(),
            path: path.into(),
            content,
            lib: Arc::new(lib),
            image,
            info,
            main,
            state,
            interface,
            deps,
            services,
            decls,
            resident,
        })
    }

    /// Checks that after swapping in `opened`, every mod runs against the
    /// interfaces it was built against: each dependency is loaded, with the
    /// same interface digest the dependent recorded.
    fn check_links(&self, mods: &[Loaded], opened: &[Opened]) -> Result<(), String> {
        // (name, interface, deps, in this batch) for every mod after the batch.
        let mut after: Vec<(&str, &str, &[(String, String)], bool)> = mods
            .iter()
            .filter(|m| !opened.iter().any(|o| *o.name == *m.name))
            .map(|m| (&*m.name, m.interface.as_str(), m.deps.as_slice(), false))
            .collect();
        after.extend(opened.iter().map(|o| (o.name.as_str(), o.interface.as_str(), o.deps.as_slice(), true)));
        // Residency after the batch, which a new build may change.
        let resident_after = |name: &str| match opened.iter().find(|o| o.name == name) {
            Some(o) => o.resident,
            None => mods.iter().any(|m| &*m.name == name && m.resident),
        };

        let is_resident = |name: &str| mods.iter().any(|m| &*m.name == name && m.resident);
        let mut problems = Vec::new();
        // Dependency -> running dependents it would leave on its old interface.
        let mut stranded: Vec<(&str, Vec<&str>)> = Vec::new();
        for &(name, _, deps, in_batch) in &after {
            // engine_mod enforces this at build time; this is the backstop for
            // libraries built otherwise.
            if resident_after(name) {
                for (dep, _) in deps.iter().filter(|(dep, _)| !resident_after(dep)) {
                    problems.push(format!("{name} is resident, so everything it depends on must be too; {dep} isn't"));
                }
            }
            for (dep, digest) in deps {
                match after.iter().find(|a| a.0 == dep) {
                    None => problems.push(format!("{name} depends on {dep}, which isn't loaded")),
                    Some(&(_, interface, _, dep_in_batch)) if interface != digest => {
                        if dep_in_batch && !in_batch {
                            match stranded.iter_mut().find(|(d, _)| d == dep) {
                                Some((_, names)) => names.push(name),
                                None => stranded.push((dep, vec![name])),
                            }
                        } else {
                            // A resident dependency's running build is the one
                            // that stays, so only a restart can bring these together.
                            let restart = if !dep_in_batch && is_resident(dep) {
                                format!("; {}", resident_note(dep))
                            } else {
                                String::new()
                            };
                            problems.push(format!(
                                "{name} was built against a different interface of {dep} than the {} one{restart}",
                                if dep_in_batch { "new" } else { "running" }
                            ));
                        }
                    }
                    Some(_) => {}
                }
            }
        }
        for (dep, names) in stranded {
            // A dependent is never resident here: a resident mod depends only
            // on resident mods, which aren't swapped.
            let how = match self.reload_hint.borrow().as_deref() {
                Some(label) => format!("reload them together with `./bazel run {label}`"),
                None => "reload them in the same batch".to_string(),
            };
            problems.push(format!(
                "{dep}'s interface changed, and {} {} built against the old one; {how}",
                names.join(", "),
                if names.len() == 1 { "was" } else { "were" }
            ));
        }
        // One provider per service, among the mods after the batch.
        let providers = mods
            .iter()
            .filter(|m| !opened.iter().any(|o| *o.name == *m.name))
            .map(|m| (&*m.name, &m.services))
            .chain(opened.iter().map(|o| (o.name.as_str(), &o.services)));
        let mut seen: Vec<(&str, &str)> = Vec::new();
        for (name, services) in providers {
            for service in services {
                match seen.iter().find(|(s, _)| *s == service.name) {
                    Some((_, other)) => {
                        problems.push(format!("{name} and {other} both provide {}", service.name))
                    }
                    None => seen.push((&service.name, name)),
                }
            }
        }
        if problems.is_empty() { Ok(()) } else { Err(problems.join("; ")) }
    }

    /// Delivers `message` to the mod `name` and returns its reply. `Err` if the
    /// mod declined it (the reply says why), or failed handling it.
    ///
    /// The mod list is only shared-borrowed, so a handler may step other mods:
    /// that is how a bootstrap mod can advance time on request.
    pub fn send(&self, name: &str, message: &str) -> Result<String, String> {
        let mods = self.mods.try_borrow().map_err(|_| "the mods are being modified".to_string())?;
        let m = mods.iter().find(|m| &*m.name == name).ok_or_else(|| format!("{name} is not loaded"))?;
        if m.failed.get() {
            return Err(format!("{name} has failed; reload it first"));
        }
        if m.running.get() {
            return Err(format!("{name} is running, so it can't take a message now"));
        }
        self.reply.borrow_mut().clear();
        unsafe {
            (*m.ctx).message = message.as_ptr();
            (*m.ctx).message_len = message.len();
        }
        let status = m.call(Op::MESSAGE);
        unsafe {
            (*m.ctx).message = std::ptr::null();
            (*m.ctx).message_len = 0;
        }
        let reply = std::mem::take(&mut *self.reply.borrow_mut());
        match status {
            Status::OK => Ok(reply),
            Status::REFUSED => Err(reply),
            _ => Err(format!("{name} failed handling the message")),
        }
    }

    /// The library of the mod whose context this is, for the world to keep
    /// mapped while it holds that build's code.
    fn library_of(&self, ctx: *const ModContext) -> Option<Keepalive> {
        let lib = self.libs.borrow().get(&(ctx as usize)).cloned()?;
        Some(lib as Keepalive)
    }

    /// Refuses a build from another rustc than the engine's. Components hold
    /// std types (`Vec`, `String`), whose layout is only the same between
    /// builds of one compiler, and every build's code touches every other's
    /// values. A library with no rustc version recorded (not Rust) passes.
    fn check_rustc(&self, name: &str, path: &Path) -> Result<(), String> {
        let ours = self.rustc.get_or_init(|| rustc_version(Path::new("/proc/self/exe")));
        let theirs = rustc_version(path);
        match (ours, theirs) {
            (Some(ours), Some(theirs)) if *ours != theirs => Err(format!(
                "{name} was built by {theirs}, the engine by {ours}; restart the engine to switch compilers"
            )),
            _ => Ok(()),
        }
    }

    /// The label the user runs to reload the whole game, named in errors.
    pub fn set_reload_hint(&self, label: Option<String>) {
        *self.reload_hint.borrow_mut() = label;
    }

    pub fn unload(&self, name: &str) -> Result<String, String> {
        let mut mods = self.mods.borrow_mut();
        let index = mods
            .iter()
            .position(|m| &*m.name == name)
            .ok_or_else(|| format!("{name} is not loaded"))?;
        if mods[index].resident {
            return Err(format!("{name} is resident: it unloads when the engine exits"));
        }
        let dependents: Vec<&str> =
            mods.iter().filter(|m| m.deps.iter().any(|(d, _)| d == name)).map(|m| &*m.name).collect();
        if !dependents.is_empty() {
            return Err(format!("{name} is needed by {}; unload them first", dependents.join(", ")));
        }
        check_plan(&mods, &[], Some(name))?;
        *self.plan.borrow_mut() = None;
        let m = mods.remove(index);
        m.close_state();
        self.libs.borrow_mut().remove(&(m.ctx as usize));
        Ok(format!("unloaded {name}"))
    }

    pub fn world(&self) -> &World {
        &self.world
    }

    fn next_load(&self) -> u64 {
        let n = self.load_count.get() + 1;
        self.load_count.set(n);
        n
    }

    /// Closes every mod, most recently loaded first.
    pub fn shutdown(&self) {
        let mut mods = self.mods.borrow_mut();
        while let Some(m) = mods.pop() {
            m.close_state();
        }
    }

    pub fn list(&self) -> String {
        let mods = self.mods.borrow();
        let mut out = format!("{} mod(s) loaded", mods.len());
        for m in mods.iter() {
            let bootstrap = if self.bootstrap.as_deref() == Some(&*m.name) { " [bootstrap]" } else { "" };
            let failed = if m.failed.get() { " [failed]" } else { "" };
            let failed = if m.resident { format!(" [resident]{failed}") } else { failed.to_string() };
            let generation = unsafe { (*m.ctx).generation };
            let needs: Vec<&str> = m.deps.iter().map(|(d, _)| d.as_str()).collect();
            let needs = if needs.is_empty() { String::new() } else { format!(" needs {}", needs.join(",")) };
            out += &format!(
                "\n  {} gen {generation}{bootstrap}{failed}{needs}  {}",
                m.name,
                m.source.display()
            );
        }
        out += &format!("\n{}", self.world.summary());
        out
    }

    /// The Bazel label mod `name`'s running build was built as, asked of that
    /// build's own code: so a test swapping two builds of the same code can
    /// tell which image is mapped, where the generation only says the loader
    /// swapped something. `None` if it isn't loaded or doesn't say.
    pub fn build_of(&self, name: &str) -> Option<String> {
        let mods = self.mods.borrow();
        let m = mods.iter().find(|m| &*m.name == name)?;
        unsafe {
            let build = *m.lib.get::<BuildFn>(BUILD_SYMBOL).ok()?;
            let mut len = 0;
            let bytes = build(&mut len);
            Some(String::from_utf8_lossy(std::slice::from_raw_parts(bytes, len)).into_owned())
        }
    }

    /// Where control requests go: the control socket's thread sends each one
    /// here, and the engine serves it at the next pump.
    pub fn requests(&self) -> Sender<Pending> {
        self.requests.clone()
    }

    /// Runs the session: the bootstrap mod's `Bootstrap::run`, which loops until it
    /// is asked to quit and returns `Status::QUIT`. Its build must be resident,
    /// since it's on the stack whenever the loader swaps builds.
    pub fn run_bootstrap(&self) -> Result<Status, String> {
        let name = self.bootstrap.as_deref().ok_or("the game has no bootstrap mod")?;
        // Copied out, so no borrow of the mod list is held while it runs:
        // serving a load takes the list mutably.
        let (ctx, main) = {
            let mods = self.mods.borrow();
            let m = mods.iter().find(|m| &*m.name == name).ok_or_else(|| format!("{name} is not loaded"))?;
            if !m.info.bootstrap {
                return Err(bootstrap_note(name));
            }
            if !m.resident {
                return Err(format!(
                    "{name} runs the frame loop, so it must be resident: engine_mod(resident = True)"
                ));
            }
            if m.failed.get() {
                return Err(format!("{name} has failed"));
            }
            m.running.set(true);
            (m.ctx, m.main)
        };
        // The build is resident, so `main` stays mapped for the whole call.
        let status = unsafe { main(ctx, Op::RUN) };
        if let Some(m) = self.mods.borrow().iter().find(|m| m.ctx == ctx) {
            m.running.set(false);
            if status == Status::ERROR {
                m.failed.set(true);
                eprintln!("[engine] {name} failed while running the frame loop");
            }
        }
        Ok(status)
    }

    /// Runs one frame without a bootstrap, for tests and tools that drive an
    /// `Engine` directly.
    pub fn step_all(&self) {
        self.run_frame();
    }

    /// The order systems run in, one line per phase.
    pub fn schedule(&self) -> Result<String, String> {
        Ok(self.plan(&self.mods.borrow())?.describe())
    }

    fn plan(&self, mods: &[Loaded]) -> Result<Rc<Plan>, String> {
        if let Some(plan) = &*self.plan.borrow() {
            return Ok(plan.clone());
        }
        let decls: Vec<ModDecls> =
            mods.iter().map(|m| ModDecls { name: &m.name, systems: &m.systems, phases: &m.phases }).collect();
        let plan = Rc::new(schedule::plan(&decls)?);
        *self.plan.borrow_mut() = Some(plan.clone());
        Ok(plan)
    }

    /// The loader's own frame: every node in plan order. What `step_mods`
    /// runs, and what a sequential scheduler mod does through the frame
    /// primitives below.
    fn run_frame(&self) -> Status {
        if self.begin_frame().is_none() {
            return Status::ERROR;
        }
        let count = self.frame.borrow().as_ref().map_or(0, |f| f.nodes.len());
        for id in 0..count {
            self.run_node(id);
        }
        self.end_frame();
        Status::OK
    }

    /// Opens a frame: its nodes, in plan order, each system followed by its
    /// apply node if it can change the world or send events. `None` if a
    /// frame is already open, or the mods are being changed.
    fn begin_frame(&self) -> Option<Vec<(usize, String)>> {
        if self.frame.borrow().is_some() {
            return None;
        }
        let mods = self.mods.try_borrow().ok()?;
        let plan = match self.plan(&mods) {
            Ok(plan) => plan,
            Err(e) => {
                // Loads are checked against this, so only a bug gets here.
                eprintln!("[engine] no frame: {e}");
                return None;
            }
        };
        let frame_time = self.frame_time.replace(DEFAULT_FRAME);
        let mut nodes = Vec::new();
        let mut i = 0;
        while i < plan.phases.len() {
            // A group: consecutive phases at one rate, or one phase that
            // runs once a frame.
            let hz = plan.phases[i].fixed_hz;
            let end = if hz.is_some() { i + plan.phases[i..].iter().take_while(|p| p.fixed_hz == hz).count() } else { i + 1 };
            let (steps, dt) = match hz {
                None => (1, frame_time),
                Some(hz) => (self.steps(&plan.phases[i].name, hz, frame_time), 1.0 / hz),
            };
            for _ in 0..steps {
                for phase in &plan.phases[i..end] {
                    for s in &phase.systems {
                        let at = nodes.len();
                        nodes.push((FrameNode::System(s.id, dt), s.name.clone()));
                        let changes = mods
                            .iter()
                            .find(|m| *m.name == s.module)
                            .is_some_and(|m| m.systems[s.index].params.iter().any(|p| p.changes()));
                        if changes {
                            nodes.push((FrameNode::Apply(s.id, at), format!("apply({})", s.name)));
                        }
                    }
                }
            }
            i = end;
        }
        drop(mods);
        self.world.begin_frame();
        let listed = nodes.iter().enumerate().map(|(i, (_, name))| (i, name.clone())).collect();
        *self.frame.borrow_mut() = Some(OpenFrame { plan, nodes, logs: HashMap::new() });
        Some(listed)
    }

    /// Runs node `id` of the open frame. A system of a mod that is already
    /// running (the bootstrap, the scheduler, a caller up the stack) is
    /// skipped: its state is borrowed.
    /// How many steps of `hz` a group starting at `phase` takes in a frame of
    /// `seconds`, carrying the remainder to the next frame.
    fn steps(&self, phase: &str, hz: f32, seconds: f32) -> u32 {
        let mut accumulated = self.accumulated.borrow_mut();
        let acc = accumulated.entry(phase.to_string()).or_insert(0.0);
        *acc += seconds as f64;
        let step = 1.0 / hz as f64;
        // A hair of slack: sixty frames of 1/60 must make sixty steps, not
        // fifty-nine and a rounding error.
        let steps = ((*acc / step) + 1e-6).floor() as u32;
        let taken = steps.min(MAX_STEPS);
        *acc = (*acc - taken as f64 * step).max(0.0);
        if steps > MAX_STEPS {
            // Behind by more than the cap allows: drop the whole steps owed,
            // keeping only the part of one, so the next frame runs as normal.
            *acc = acc.rem_euclid(step);
        }
        taken
    }

    /// The time the next frame covers: what fixed-rate phases step by.
    pub fn set_frame_time(&self, seconds: f32) {
        self.frame_time.set(seconds.max(0.0));
    }

    fn run_node(&self, id: usize) -> Ran {
        let (node, plan) = {
            let frame = self.frame.borrow();
            let Some(frame) = frame.as_ref() else { return Ran::Refused };
            let Some(&(node, _)) = frame.nodes.get(id) else { return Ran::Refused };
            (node, frame.plan.clone())
        };
        match node {
            FrameNode::System(s, dt) => {
                let Some(planned) = plan.system(s) else { return Ran::Refused };
                let Ok(mods) = self.mods.try_borrow() else { return Ran::Refused };
                let Some(m) = mods.iter().find(|m| *m.name == planned.module) else { return Ran::Skipped };
                if m.failed.get() || m.running.get() {
                    return Ran::Skipped;
                }
                match self.run_system(m, planned.index, &planned.name, dt) {
                    Some(log) => {
                        if let Some(frame) = self.frame.borrow_mut().as_mut() {
                            frame.logs.insert(id, log);
                        }
                        Ran::Ran
                    }
                    None => Ran::Failed,
                }
            }
            FrameNode::Apply(s, ran) => {
                let log = self.frame.borrow_mut().as_mut().and_then(|f| f.logs.remove(&ran));
                let Some(log) = log else { return Ran::Skipped };
                self.apply(log, &plan.system(s).map(|p| p.module.clone()).unwrap_or_default());
                Ran::Ran
            }
        }
    }

    fn end_frame(&self) {
        if self.frame.borrow_mut().take().is_some() {
            self.world.end_frame();
        }
    }

    /// Runs one system; its log, or `None` if it failed, in which case its
    /// changes are discarded with it.
    fn run_system(&self, m: &Loaded, index: usize, name: &str, dt: f32) -> Option<Vec<Change>> {
        let desc = &m.systems[index];
        let log = Log::default();
        let frame = FrameCx { world: &self.world, log: &log, system: name, dt };
        let was_running = m.running.replace(true);
        let status = unsafe { (desc.run)(m.ctx, &frame, &desc.params) };
        m.running.set(was_running);
        if status == Status::ERROR {
            m.failed.set(true);
            eprintln!("[engine] {} failed in {name}; it won't run again until it is reloaded", m.name);
            return None;
        }
        Some(log.into_inner())
    }

    /// Lands a system's changes and publishes its events, in the order it
    /// made them.
    fn apply(&self, log: Vec<Change>, module: &str) {
        let applied = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
            let mut s = Structural::new(&self.world);
            for change in log {
                change.apply(&mut s);
            }
        }));
        if applied.is_err() {
            eprintln!("[engine] applying {module}'s changes panicked; it won't run again until it is reloaded");
            if let Ok(mods) = self.mods.try_borrow() {
                if let Some(m) = mods.iter().find(|m| *m.name == *module) {
                    m.failed.set(true);
                }
            }
        }
    }

    /// Serves control requests, as a bootstrap's `Cx::pump_loader` does, for
    /// an engine with no bootstrap. Must not be called while any mod runs.
    pub fn pump(&self, timeout: Duration) -> Pumped {
        self.serve(None, timeout, &mut |_| Err("no mod is pumping".into()))
    }

    /// Whether the loader may swap builds with `caller` pumping: nothing is
    /// on the stack but resident mods, and no load or delivery is under way.
    fn safe_point(&self, caller: *const ModContext) -> bool {
        if self.frame.borrow().is_some() {
            return false;
        }
        let Ok(mods) = self.mods.try_borrow_mut() else { return false };
        mods.iter().any(|m| std::ptr::eq(m.ctx, caller) && m.resident)
            && mods.iter().all(|m| m.resident || !m.running.get())
    }

    /// Serves every queued request, waiting up to `timeout` for the first.
    /// Messages for `caller`, which is running, go to `handler`.
    fn serve(
        &self,
        caller: Option<&str>,
        timeout: Duration,
        handler: &mut dyn FnMut(&str) -> Result<String, String>,
    ) -> Pumped {
        let first = if timeout.is_zero() { self.inbox.try_recv().ok() } else { self.inbox.recv_timeout(timeout).ok() };
        for pending in first.into_iter().chain(std::iter::from_fn(|| self.inbox.try_recv().ok())) {
            let reply = self.handle(caller, &pending.text, handler);
            (pending.reply)(reply);
        }
        if self.quitting.get() { Pumped::Quit } else { Pumped::Continue }
    }

    /// Handles one control request, returning the reply to send.
    fn handle(
        &self,
        caller: Option<&str>,
        text: &str,
        handler: &mut dyn FnMut(&str) -> Result<String, String>,
    ) -> String {
        let request = Request::parse(text);
        // Messages aren't logged, request or reply: a game played over
        // messages would flood the log.
        let quiet = matches!(request, Ok(Request::Send { .. }));
        let result = request.and_then(|request| {
            match &request {
                Request::Batch { mods } => println!("[engine] batch of {} mod(s)", mods.len()),
                Request::Send { .. } => {}
                other => println!("[engine] {}", other.encode().trim_end()),
            }
            match request {
                Request::Load { name, path } => self.load(&name, &path),
                Request::Batch { mods } => self.load_batch(&mods),
                Request::Unload { name } => self.unload(&name),
                Request::Send { name, message } if caller == Some(name.as_str()) => handler(&message),
                Request::Send { name, message } => self.send(&name, &message),
                Request::List => Ok(self.list()),
                Request::Schedule => self.schedule(),
                Request::Quit => {
                    self.quitting.set(true);
                    Ok("quitting".into())
                }
            }
        });
        match result {
            Ok(msg) => {
                if !quiet {
                    println!("[engine] {}", msg.lines().next().unwrap_or(""));
                }
                format!("ok {msg}\n")
            }
            Err(msg) => {
                if !quiet {
                    eprintln!("[engine] error: {msg}");
                }
                format!("err {msg}\n")
            }
        }
    }

    /// Copies the library to a unique file before opening it. `dlopen` hands back
    /// the already-loaded image for a path, or an inode, it has seen, so opening
    /// the Bazel output again (or a link to it) would reload nothing. See
    /// docs/lore/dlopen-returns-the-loaded-image-for-a-file-it-has-seen.md.
    fn open_staged(&self, name: &str, path: &Path) -> Result<(Library, Vec<(usize, usize)>), String> {
        std::fs::create_dir_all(&self.staging_dir)
            .map_err(|e| format!("creating {}: {e}", self.staging_dir.display()))?;
        let n = self.staged_count.get();
        self.staged_count.set(n + 1);
        let staged = self
            .staging_dir
            .join(format!("{name}-{}-{n}.so", std::process::id()));
        std::fs::copy(path, &staged).map_err(|e| format!("copying {}: {e}", path.display()))?;
        // `engine_mod` links with `-z now`, so a missing symbol fails here, while
        // the old build is still running.
        let lib = unsafe { Library::new(&staged) }.map_err(describe);
        // Read while the file still has its name: the maps name it by path.
        let image = std::fs::canonicalize(&staged).map(|path| mapped_ranges(&path)).unwrap_or_default();
        // The mapping outlives the file, so nothing is left behind to clean up.
        let _ = std::fs::remove_file(&staged);
        Ok((lib?, image))
    }
}

/// Closes every mod, so each build drops its transient part and state while
/// its code is still mapped. Without it, a resident mod's threads would
/// outlive its library.
impl Drop for Engine {
    fn drop(&mut self) {
        self.shutdown();
    }
}

impl Loaded {
    fn call(&self, op: Op) -> Status {
        let was_running = self.running.replace(true);
        let status = unsafe { (self.main)(self.ctx, op) };
        self.running.set(was_running);
        if status == Status::ERROR {
            self.failed.set(true);
            eprintln!(
                "[engine] {} failed during {op:?}; it won't run again until it is reloaded",
                self.name
            );
        }
        status
    }

    /// Lets the current build drop its transient part and its state.
    fn close_state(&self) {
        self.call(Op::CLOSE);
    }

    /// Replaces the (already dropped) state with `new`'s `Default`.
    fn fresh_state(&mut self, new: &StateSchema) {
        unsafe {
            std::alloc::dealloc((*self.ctx).state as *mut u8, self.state.layout);
            (*self.ctx).state = new_state(new);
        }
        self.state.layout = new.layout;
    }

    /// Moves the state into `new`'s layout, field by field, dropping what
    /// doesn't carry over with the old build's code. Returns the report.
    fn migrate_state(&mut self, new: &StateSchema) -> String {
        let old = &self.state;
        unsafe {
            let to = alloc_state(new.layout);
            schema::migrate(
                ((*self.ctx).state as *mut u8, &old.fields, &old.drops),
                (to as *mut u8, &new.fields, &new.drops, new.default),
            );
            std::alloc::dealloc((*self.ctx).state as *mut u8, old.layout);
            (*self.ctx).state = to;
        }
        let report = schema::report(&self.state.fields, &new.fields);
        self.state.layout = new.layout;
        report
    }

    /// Whether any pointer-sized word of the state holds an address inside
    /// this build's library: a vtable, a function, a string literal. A shallow
    /// net for state that bypassed `mod_state!` (an `unsafe impl ModState`);
    /// it can't see pointers behind other pointers, and an integer can happen
    /// to look like an address.
    fn state_points_into_its_build(&self) -> bool {
        let word = size_of::<usize>();
        let state = unsafe { (*self.ctx).state as *const u8 };
        (0..self.state.layout.size() / word).any(|i| {
            let value = unsafe { (state.add(i * word) as *const usize).read_unaligned() };
            self.image.iter().any(|&(start, end)| (start..end).contains(&value))
        })
    }
}

fn bootstrap_note(name: &str) -> String {
    format!("{name} is the game's bootstrap, but isn't one: implement Bootstrap and export_mod!(.., bootstrap)")
}

fn resident_note(name: &str) -> String {
    format!("{name} is resident: restart the engine to load its new build")
}

/// Checks that the systems of the mods after a batch (`opened` swapped in,
/// `removed` gone) can be put in order.
fn check_plan(mods: &[Loaded], opened: &[Opened], removed: Option<&str>) -> Result<(), String> {
    let mut decls: Vec<ModDecls> = mods
        .iter()
        .filter(|m| Some(&*m.name) != removed)
        .map(|m| match opened.iter().find(|o| *o.name == *m.name) {
            Some(o) => ModDecls { name: &m.name, systems: &o.decls.systems, phases: &o.decls.phases },
            None => ModDecls { name: &m.name, systems: &m.systems, phases: &m.phases },
        })
        .collect();
    for o in opened.iter().filter(|o| !mods.iter().any(|m| *m.name == *o.name)) {
        decls.push(ModDecls { name: &o.name, systems: &o.decls.systems, phases: &o.decls.phases });
    }
    schedule::plan(&decls).map(|_| ())
}

/// Orders builds so each comes after the builds in the same batch it depends
/// on. Dependencies outside the batch are already loaded, which
/// `check_links` has established.
fn in_dependency_order(mut pending: Vec<Opened>) -> Result<Vec<Opened>, String> {
    let mut ordered: Vec<Opened> = Vec::new();
    while !pending.is_empty() {
        let ready = pending.iter().position(|o| {
            o.deps.iter().all(|(dep, _)| !pending.iter().any(|p| p.name == *dep))
        });
        let Some(ready) = ready else {
            let names: Vec<&str> = pending.iter().map(|o| o.name.as_str()).collect();
            return Err(format!("dependency cycle among {}", names.join(", ")));
        };
        ordered.push(pending.remove(ready));
    }
    Ok(ordered)
}

/// The `rustc version ...` line from an ELF file's `.comment` section, where
/// rustc records itself in everything it compiles. `None` if the file isn't
/// 64-bit ELF or has no such line.
fn rustc_version(path: &Path) -> Option<String> {
    let bytes = std::fs::read(path).ok()?;
    let u16_at = |at: usize| Some(u16::from_le_bytes(bytes.get(at..at + 2)?.try_into().ok()?) as usize);
    let u32_at = |at: usize| Some(u32::from_le_bytes(bytes.get(at..at + 4)?.try_into().ok()?) as usize);
    let u64_at = |at: usize| Some(u64::from_le_bytes(bytes.get(at..at + 8)?.try_into().ok()?) as usize);
    // 64-bit, little-endian ELF.
    if bytes.get(..6)? != b"\x7fELF\x02\x01" {
        return None;
    }
    let (sections, entry_size, count, names) = (u64_at(0x28)?, u16_at(0x3a)?, u16_at(0x3c)?, u16_at(0x3e)?);
    let section = |i: usize| -> Option<(usize, usize, usize)> {
        let at = sections + i * entry_size;
        Some((u32_at(at)?, u64_at(at + 0x18)?, u64_at(at + 0x20)?))
    };
    let (_, names_at, _) = section(names)?;
    let comment = (0..count).find_map(|i| {
        let (name, at, size) = section(i)?;
        let name = bytes.get(names_at + name..)?.split(|&b| b == 0).next()?;
        (name == b".comment").then_some((at, size))
    })?;
    let (at, size) = comment;
    bytes
        .get(at..at + size)?
        .split(|&b| b == 0)
        .filter_map(|s| std::str::from_utf8(s).ok())
        .find(|s| s.starts_with("rustc version"))
        .map(str::to_string)
}

fn content_hash(path: &Path) -> Result<u64, String> {
    use std::hash::{Hash, Hasher};
    let bytes = std::fs::read(path).map_err(|e| format!("reading {}: {e}", path.display()))?;
    // Only ever compared within this process, so `DefaultHasher` is stable enough.
    let mut hasher = std::collections::hash_map::DefaultHasher::new();
    bytes.hash(&mut hasher);
    Ok(hasher.finish())
}

/// # Safety
/// `ptr` must point to `len` bytes of UTF-8, or be null with `len` 0.
unsafe fn copy_str(ptr: *const u8, len: usize) -> String {
    if len == 0 {
        return String::new();
    }
    String::from_utf8_lossy(unsafe { std::slice::from_raw_parts(ptr, len) }).into_owned()
}

/// libloading's `Display` omits the `dlerror` text, which lives in `source()`.
fn describe(e: libloading::Error) -> String {
    match std::error::Error::source(&e) {
        Some(source) => format!("{e}: {source}"),
        None => e.to_string(),
    }
}

/// A new state value: `schema`'s `Default`, in memory the loader owns.
fn new_state(schema: &StateSchema) -> *mut c_void {
    let state = alloc_state(schema.layout);
    unsafe { (schema.default)(state as *mut u8) };
    state
}

/// The services a build's `ModInfo` lists, copied.
///
/// # Safety
/// `info` must come from a loaded build (its pointers point into it).
unsafe fn read_services(info: &ModInfo) -> Result<Vec<Service>, String> {
    let services = match info.service_count {
        0 => &[][..],
        n => unsafe { std::slice::from_raw_parts(info.services, n) },
    };
    services
        .iter()
        .map(|s| {
            let name = unsafe { copy_str(s.name, s.name_len) };
            let methods = match s.method_count {
                0 => &[][..],
                n => unsafe { std::slice::from_raw_parts(s.methods, n) },
            };
            let methods = methods.iter().map(|m| (unsafe { copy_str(m.name, m.name_len) }, m.call)).collect();
            if services.iter().filter(|o| unsafe { copy_str(o.name, o.name_len) } == name).count() > 1 {
                return Err(format!("provides {name} twice"));
            }
            Ok(Service { name, methods })
        })
        .collect()
}

/// The address ranges `/proc/self/maps` shows `path` mapped at.
fn mapped_ranges(path: &Path) -> Vec<(usize, usize)> {
    let Ok(maps) = std::fs::read_to_string("/proc/self/maps") else { return Vec::new() };
    let path = path.to_string_lossy();
    maps.lines()
        .filter_map(|line| {
            // start-end perms offset dev inode      path
            let mut parts = line.splitn(6, ' ');
            let range = parts.next()?;
            parts.nth(3)?;
            if parts.next()?.trim_start() != path {
                return None;
            }
            let (start, end) = range.split_once('-')?;
            Some((usize::from_str_radix(start, 16).ok()?, usize::from_str_radix(end, 16).ok()?))
        })
        .collect()
}

fn alloc_state(layout: Layout) -> *mut c_void {
    let ptr = unsafe { std::alloc::alloc_zeroed(layout) };
    if ptr.is_null() {
        std::alloc::handle_alloc_error(layout);
    }
    ptr as *mut c_void
}

pub(crate) unsafe fn engine_of<'a>(ctx: *const ModContext) -> &'a Engine {
    unsafe { &*((*(*ctx).host).userdata as *const Engine) }
}

pub(crate) unsafe fn name_of<'a>(ctx: *const ModContext) -> &'a str {
    unsafe {
        let ctx = &*ctx;
        std::str::from_utf8_unchecked(std::slice::from_raw_parts(ctx.name, ctx.name_len))
    }
}

unsafe extern "C" fn host_log(ctx: *const ModContext, msg: *const u8, len: usize) {
    let msg = unsafe { std::slice::from_raw_parts(msg, len) };
    let name = unsafe { name_of(ctx) };
    println!("[{name}] {}", String::from_utf8_lossy(msg));
}

unsafe extern "C" fn host_reply(ctx: *const ModContext, msg: *const u8, len: usize) {
    let engine = unsafe { engine_of(ctx) };
    let msg = unsafe { std::slice::from_raw_parts(msg, len) };
    // Can't fail: nothing else holds the buffer while a mod runs.
    if let Ok(mut reply) = engine.reply.try_borrow_mut() {
        reply.push_str(&String::from_utf8_lossy(msg));
    }
}

unsafe extern "C" fn host_begin_call(
    ctx: *const ModContext,
    service: *const u8,
    service_len: usize,
    method: *const u8,
    method_len: usize,
    target: *mut CallTarget,
) -> CallStatus {
    let engine = unsafe { engine_of(ctx) };
    // No panicking in here: unwinding out of an extern "C" fn aborts.
    let Ok(mods) = engine.mods.try_borrow() else {
        // Mods are being retired or swapped (an `unload` or `close` calling out).
        return CallStatus::UNAVAILABLE;
    };
    let (service, method) = unsafe {
        let bytes = |p, n| std::str::from_utf8_unchecked(std::slice::from_raw_parts(p, n));
        (bytes(service, service_len), bytes(method, method_len))
    };
    let Some((provider, s)) = mods.iter().find_map(|m| Some((m, m.services.iter().find(|s| s.name == service)?)))
    else {
        return CallStatus::NOT_PROVIDED;
    };
    let Some(&(_, call)) = s.methods.iter().find(|(name, _)| name == method) else {
        return CallStatus::NO_SUCH_METHOD;
    };
    if provider.failed.get() {
        return CallStatus::PROVIDER_FAILED;
    }
    if provider.running.replace(true) {
        return CallStatus::REENTRANT;
    }
    unsafe { *target = CallTarget { call: Some(call), provider: provider.ctx } };
    CallStatus::OK
}

unsafe extern "C" fn host_end_call(ctx: *const ModContext, target: *const CallTarget, panicked: bool) {
    let engine = unsafe { engine_of(ctx) };
    let Ok(mods) = engine.mods.try_borrow() else { return };
    let provider = unsafe { (*target).provider };
    if let Some(m) = mods.iter().find(|m| m.ctx == provider) {
        m.running.set(false);
        if panicked {
            m.failed.set(true);
            eprintln!(
                "[engine] {} panicked in a call from {}; it won't run again until it is reloaded",
                m.name,
                unsafe { name_of(ctx) }
            );
        }
    }
}

unsafe fn host_pump(
    ctx: *const ModContext,
    timeout: Duration,
    handler: &mut dyn FnMut(&str) -> Result<String, String>,
) -> Pumped {
    let engine = unsafe { engine_of(ctx) };
    if !engine.safe_point(ctx) {
        return Pumped::Refused;
    }
    engine.serve(Some(unsafe { name_of(ctx) }), timeout, handler)
}

unsafe extern "C" fn host_step_mods(ctx: *const ModContext) -> Status {
    let engine = unsafe { engine_of(ctx) };
    // No panicking in here: unwinding out of an extern "C" fn aborts.
    let status = engine.run_frame();
    if status != Status::OK {
        eprintln!(
            "[engine] {} asked for a frame inside a frame, or while the mods are being changed",
            unsafe { name_of(ctx) }
        );
    }
    status
}

unsafe fn host_begin_frame(ctx: *const ModContext) -> Option<FramePlan> {
    let nodes = unsafe { engine_of(ctx) }.begin_frame()?;
    Some(FramePlan { nodes: nodes.into_iter().map(|(id, name)| PlannedNode { id, name }).collect() })
}

unsafe fn host_set_frame_time(ctx: *const ModContext, seconds: f32) {
    unsafe { engine_of(ctx) }.set_frame_time(seconds)
}

unsafe fn host_run_node(ctx: *const ModContext, node: usize) -> Ran {
    unsafe { engine_of(ctx) }.run_node(node)
}

unsafe fn host_keepalive(ctx: *const ModContext) -> Option<Keepalive> {
    unsafe { engine_of(ctx) }.library_of(ctx)
}

unsafe fn host_end_frame(ctx: *const ModContext) {
    unsafe { engine_of(ctx) }.end_frame()
}
