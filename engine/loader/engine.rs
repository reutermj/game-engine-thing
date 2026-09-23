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
    /// Set when the mod returned `Status::ERROR` (e.g. it panicked). Cleared by a reload.
    failed: Cell<bool>,
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
        });
        engine.host.userdata = &*engine as *const Engine as *mut c_void;
        engine
    }

    /// Loads `path` as mod `name`, or hot-reloads it if `name` is already loaded.
    pub fn load(&self, name: &str, path: &Path) -> Result<String, String> {
        // Open and validate the new build before touching the running one, so a
        // bad build leaves the old code in place.
        let lib = self.open_staged(name, path)?;
        let (info, main) = unsafe {
            let info_fn = *lib.get::<InfoFn>(INFO_SYMBOL).map_err(describe)?;
            let main = *lib.get::<MainFn>(MAIN_SYMBOL).map_err(describe)?;
            (info_fn(), main)
        };
        if info.api_version != API_VERSION {
            return Err(format!(
                "{name} was built against mod API v{}, engine is v{API_VERSION}",
                info.api_version
            ));
        }
        // A zero-sized mod still gets a real allocation: `alloc_zeroed` with a
        // zero-size layout is undefined behavior.
        let state_layout = Layout::from_size_align(info.state_size.max(1), info.state_align)
            .map_err(|e| format!("{name}: bad state layout: {e}"))?;

        let mut mods = self.mods.borrow_mut();
        let Some(m) = mods.iter_mut().find(|m| &*m.name == name) else {
            let name: Box<str> = name.into();
            let ctx = Box::into_raw(Box::new(ModContext {
                host: &self.host,
                name: name.as_ptr(),
                name_len: name.len(),
                generation: 0,
                loaded_at: self.next_load(),
                state: alloc_state(state_layout),
                state_fresh: true,
            }));
            mods.push(Loaded {
                name,
                ctx,
                state_layout,
                info,
                main,
                lib,
                source: path.into(),
                failed: Cell::new(false),
            });
            let m = mods.last().unwrap();
            m.call(Op::LOAD);
            return Ok(format!("loaded {}", m.name));
        };

        let reset = !m.info.state_compatible(&info);
        unsafe {
            if reset {
                m.close_state();
                std::alloc::dealloc((*m.ctx).state as *mut u8, m.state_layout);
                m.state_layout = state_layout;
                (*m.ctx).state = alloc_state(state_layout);
                (*m.ctx).state_fresh = true;
            } else {
                m.call(Op::UNLOAD);
            }
            (*m.ctx).generation += 1;
            (*m.ctx).loaded_at = self.next_load();
        }
        // Dropping the old library here unmaps the previous build.
        m.lib = lib;
        m.main = main;
        m.info = info;
        m.source = path.into();
        m.failed.set(false);
        m.call(Op::LOAD);
        let generation = unsafe { (*m.ctx).generation };
        let reset = if reset { ", state layout changed so state was reset" } else { "" };
        Ok(format!("reloaded {name} (generation {generation}{reset})"))
    }

    pub fn unload(&self, name: &str) -> Result<String, String> {
        let mut mods = self.mods.borrow_mut();
        let index = mods
            .iter()
            .position(|m| &*m.name == name)
            .ok_or_else(|| format!("{name} is not loaded"))?;
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
            out += &format!(
                "\n  {} gen {generation}{bootstrap}{failed}  {}",
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
