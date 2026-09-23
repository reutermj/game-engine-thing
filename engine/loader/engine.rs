//! The set of loaded mods and the host services they call back into.
//!
//! Single-threaded. Mod code runs only while the mod list is shared-borrowed
//! (stepping); loading and unloading take it mutably between steps, so a reload
//! never swaps out code that is on the stack.

use std::alloc::Layout;
use std::cell::{Cell, RefCell};
use std::ffi::c_void;
use std::path::{Path, PathBuf};

use engine_api::{
    API_VERSION, Host, INFO_SYMBOL, InfoFn, MAIN_SYMBOL, MainFn, ModContext, ModInfo, Op, Status,
    WorldApi,
};
use libloading::Library;

use crate::world::{self, WorldStorage};

pub struct Engine {
    host: Host,
    mods: RefCell<Vec<Loaded>>,
    bootstrap: Option<String>,
    /// Set while `step_mods` runs, so a stepped mod can't recurse into it.
    stepping: Cell<bool>,
    staging_dir: PathBuf,
    staged_count: Cell<u64>,
    /// Outlives every mod build; see `world.rs`.
    world: RefCell<WorldStorage>,
    /// Source of `ModContext::loaded_at`.
    load_count: Cell<u64>,
    reload_hint: RefCell<Option<String>>,
    /// The reply to the message being delivered, built up by `host_reply`.
    reply: RefCell<String>,
}

struct Loaded {
    /// `ctx.name` points into this.
    name: Box<str>,
    ctx: *mut ModContext,
    state_layout: Layout,
    info: ModInfo,
    main: MainFn,
    lib: Library,
    source: PathBuf,
    /// Hash of the library file, to recognize an unchanged build.
    content: u64,
    /// This build's interface digest, and the digests it was built against.
    interface: String,
    deps: Vec<(String, String)>,
    /// Set when the mod returned `Status::ERROR` (e.g. it panicked). Cleared by a reload.
    failed: Cell<bool>,
}

/// A new build, opened and validated but not yet swapped in.
struct Opened {
    name: String,
    path: PathBuf,
    content: u64,
    lib: Library,
    info: ModInfo,
    main: MainFn,
    state_layout: Layout,
    interface: String,
    deps: Vec<(String, String)>,
}

impl Drop for Loaded {
    fn drop(&mut self) {
        unsafe {
            let ctx = Box::from_raw(self.ctx);
            std::alloc::dealloc(ctx.state as *mut u8, self.state_layout);
        }
    }
}

impl Engine {
    pub fn new(bootstrap: Option<String>, staging_dir: PathBuf) -> Box<Engine> {
        let mut engine = Box::new(Engine {
            host: Host {
                userdata: std::ptr::null_mut(),
                log: host_log,
                step_mods: host_step_mods,
                reply: host_reply,
                world: WorldApi {
                    register: world::register,
                    spawn: world::spawn,
                    despawn: world::despawn,
                    insert: world::insert,
                    remove: world::remove,
                    get: world::get,
                    column: world::column,
                },
            },
            mods: RefCell::new(Vec::new()),
            bootstrap,
            stepping: Cell::new(false),
            staging_dir,
            staged_count: Cell::new(0),
            world: RefCell::new(WorldStorage::default()),
            load_count: Cell::new(0),
            reload_hint: RefCell::new(None),
            reply: RefCell::new(String::new()),
        });
        engine.host.userdata = &*engine as *const Engine as *mut c_void;
        engine
    }

    /// Loads `path` as mod `name`, or hot-reloads it if `name` is already loaded.
    /// A batch of one: see [`Engine::load_batch`].
    pub fn load(&self, name: &str, path: &Path) -> Result<String, String> {
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
        for (i, (name, path)) in batch.iter().enumerate() {
            if batch[..i].iter().any(|(n, _)| n == name) {
                return Err(format!("{name} is in the batch twice"));
            }
            let content = content_hash(path)?;
            if mods.iter().any(|m| &*m.name == name && m.content == content) {
                unchanged.push(name.as_str());
                continue;
            }
            opened.push(self.open(name, path, content)?);
        }
        self.check_links(&mods, &opened)?;
        let opened = in_dependency_order(opened)?;

        // Retire the builds being replaced, dependents before what they depend on.
        let mut resets = Vec::new();
        for o in opened.iter().rev() {
            let Some(m) = mods.iter_mut().find(|m| *m.name == *o.name) else { continue };
            if m.info.state_compatible(&o.info) {
                m.call(Op::UNLOAD);
            } else {
                m.close_state();
                unsafe {
                    std::alloc::dealloc((*m.ctx).state as *mut u8, m.state_layout);
                    (*m.ctx).state = alloc_state(o.state_layout);
                    (*m.ctx).state_fresh = true;
                }
                m.state_layout = o.state_layout;
                resets.push(o.name.clone());
            }
        }

        // Swap in and load the new builds, dependencies first.
        let mut report = Vec::new();
        for o in opened {
            let loaded_at = self.next_load();
            match mods.iter_mut().find(|m| *m.name == *o.name) {
                Some(m) => {
                    let generation = unsafe {
                        (*m.ctx).generation += 1;
                        (*m.ctx).loaded_at = loaded_at;
                        (*m.ctx).generation
                    };
                    // Dropping the old library here unmaps the previous build.
                    m.lib = o.lib;
                    (m.main, m.info, m.source, m.content) = (o.main, o.info, o.path, o.content);
                    (m.interface, m.deps) = (o.interface, o.deps);
                    m.failed.set(false);
                    m.call(Op::LOAD);
                    let reset = if resets.contains(&o.name) {
                        ", state layout changed so state was reset"
                    } else {
                        ""
                    };
                    report.push(format!("reloaded {} (generation {generation}{reset})", o.name));
                }
                None => {
                    let name: Box<str> = o.name.as_str().into();
                    let ctx = Box::into_raw(Box::new(ModContext {
                        host: &self.host,
                        name: name.as_ptr(),
                        name_len: name.len(),
                        generation: 0,
                        loaded_at,
                        state: alloc_state(o.state_layout),
                        state_fresh: true,
                        message: std::ptr::null(),
                        message_len: 0,
                    }));
                    mods.push(Loaded {
                        name,
                        ctx,
                        state_layout: o.state_layout,
                        info: o.info,
                        main: o.main,
                        lib: o.lib,
                        source: o.path,
                        content: o.content,
                        interface: o.interface,
                        deps: o.deps,
                        failed: Cell::new(false),
                    });
                    mods.last().unwrap().call(Op::LOAD);
                    report.push(format!("loaded {}", o.name));
                }
            }
        }
        report.extend(unchanged.iter().map(|name| format!("{name} unchanged")));
        Ok(report.join("; "))
    }

    /// Opens a build and reads what it says about itself, without running any
    /// of its mod code.
    fn open(&self, name: &str, path: &Path, content: u64) -> Result<Opened, String> {
        let lib = self.open_staged(name, path).map_err(|e| format!("{name}: {e}"))?;
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
        let state_layout = Layout::from_size_align(info.state_size.max(1), info.state_align)
            .map_err(|e| format!("{name}: bad state layout: {e}"))?;
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
        Ok(Opened { name: name.into(), path: path.into(), content, lib, info, main, state_layout, interface, deps })
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

        let mut problems = Vec::new();
        // Dependency -> running dependents it would leave on its old interface.
        let mut stranded: Vec<(&str, Vec<&str>)> = Vec::new();
        for &(name, _, deps, in_batch) in &after {
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
                            problems.push(format!(
                                "{name} was built against a different interface of {dep} than the {} one",
                                if dep_in_batch { "new" } else { "running" }
                            ));
                        }
                    }
                    Some(_) => {}
                }
            }
        }
        for (dep, names) in stranded {
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
        let dependents: Vec<&str> =
            mods.iter().filter(|m| m.deps.iter().any(|(d, _)| d == name)).map(|m| &*m.name).collect();
        if !dependents.is_empty() {
            return Err(format!("{name} is needed by {}; unload them first", dependents.join(", ")));
        }
        let m = mods.remove(index);
        m.close_state();
        Ok(format!("unloaded {name}"))
    }

    pub fn world(&self) -> &RefCell<WorldStorage> {
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
            let generation = unsafe { (*m.ctx).generation };
            let needs: Vec<&str> = m.deps.iter().map(|(d, _)| d.as_str()).collect();
            let needs = if needs.is_empty() { String::new() } else { format!(" needs {}", needs.join(",")) };
            out += &format!(
                "\n  {} gen {generation}{bootstrap}{failed}{needs}  {}",
                m.name,
                m.source.display()
            );
        }
        out += &format!("\n{}", self.world.borrow().summary());
        out
    }

    /// Runs one step of the bootstrap mod, which drives everything else. `None`
    /// when there's no runnable bootstrap mod yet.
    pub fn step_bootstrap(&self) -> Option<Status> {
        let mods = self.mods.borrow();
        let bootstrap = self.bootstrap.as_deref()?;
        let m = mods.iter().find(|m| &*m.name == bootstrap)?;
        if m.failed.get() {
            return None;
        }
        Some(m.call(Op::STEP))
    }

    /// Copies the library to a unique file before opening it. `dlopen` hands back
    /// the already-loaded image for a path, or an inode, it has seen, so opening
    /// the Bazel output again (or a link to it) would reload nothing. See
    /// docs/lore/dlopen-returns-the-loaded-image-for-a-file-it-has-seen.md.
    fn open_staged(&self, name: &str, path: &Path) -> Result<Library, String> {
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
        // The mapping outlives the file, so nothing is left behind to clean up.
        let _ = std::fs::remove_file(&staged);
        lib
    }
}

impl Loaded {
    fn call(&self, op: Op) -> Status {
        let status = unsafe { (self.main)(self.ctx, op) };
        if status == Status::ERROR {
            self.failed.set(true);
            eprintln!(
                "[engine] {} failed during {op:?}; it won't run again until it is reloaded",
                self.name
            );
        }
        status
    }

    /// Lets the current build drop its state, if the state was ever initialized.
    fn close_state(&self) {
        if unsafe { !(*self.ctx).state_fresh } {
            self.call(Op::CLOSE);
        }
    }
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

unsafe extern "C" fn host_step_mods(ctx: *const ModContext) -> Status {
    let engine = unsafe { engine_of(ctx) };
    // No panicking in here: unwinding out of an extern "C" fn aborts.
    let Ok(mods) = engine.mods.try_borrow() else {
        eprintln!("[engine] step_mods can't be called while mods are being loaded");
        return Status::ERROR;
    };
    if engine.stepping.replace(true) {
        eprintln!("[engine] step_mods called recursively by {}", unsafe { name_of(ctx) });
        return Status::ERROR;
    }
    for m in mods.iter() {
        if !std::ptr::eq(m.ctx, ctx) && !m.failed.get() {
            m.call(Op::STEP);
        }
    }
    engine.stepping.set(false);
    Status::OK
}
