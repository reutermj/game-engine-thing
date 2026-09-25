//! The loader's poison mode (`engine/loader/poison.rs`), run with
//! `ENGINE_POISON_UNLOADED=1` set by the target: an unloaded build's span is
//! left unreadable, a stale call into it faults and names the build, and a
//! build `dlclose` keeps mapped is reported instead.

use std::os::unix::process::ExitStatusExt;
use std::path::PathBuf;
use std::process::Command;
use std::sync::atomic::Ordering;

use engine_loader::engine::Engine;
use engine_loader::poison::{self, ModLibrary};
use runfiles::Runfiles;

fn lib(var: &str) -> PathBuf {
    let rlocation = std::env::var(var).unwrap_or_else(|_| panic!("${var} is not set"));
    Runfiles::create().expect("runfiles").rlocation(&rlocation).expect("in runfiles")
}

fn tmp(name: &str) -> PathBuf {
    let dir = PathBuf::from(std::env::var("TEST_TMPDIR").expect("TEST_TMPDIR")).join(name);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

/// Opens a fresh copy of counter_v1 (a copy, as the engine stages one, so
/// this test's image is its own) and returns its info function's address.
fn open_counter(test: &str) -> (ModLibrary, usize) {
    let source = lib("COUNTER_V1");
    let staged = tmp(test).join("counter.so");
    std::fs::copy(&source, &staged).unwrap();
    let lib = unsafe { ModLibrary::open(&staged, &source) }.expect("dlopen");
    let info = unsafe { *lib.get::<unsafe extern "C" fn()>(engine_api::INFO_SYMBOL).expect("info symbol") };
    (lib, info as usize)
}

/// The `/proc/self/maps` line covering `addr`: its permissions and path.
fn mapping_at(addr: usize) -> Option<(String, String)> {
    let maps = std::fs::read_to_string("/proc/self/maps").unwrap();
    maps.lines().find_map(|line| {
        let mut parts = line.split_whitespace();
        let (start, end) = parts.next()?.split_once('-')?;
        let (start, end) = (usize::from_str_radix(start, 16).ok()?, usize::from_str_radix(end, 16).ok()?);
        let perms = parts.next()?.to_string();
        let path = parts.nth(3).unwrap_or("").to_string();
        (start..end).contains(&addr).then_some((perms, path))
    })
}

#[test]
fn an_unloaded_build_is_left_unreadable_and_reserved() {
    assert_eq!(poison::mode(), poison::Mode::Unmapped, "the target sets ENGINE_POISON_UNLOADED=1");
    let (lib, info) = open_counter("unreadable");
    let (perms, path) = mapping_at(info).expect("mapped while open");
    assert!(perms.starts_with("r-x") && path.ends_with("counter.so"), "{perms} {path}");
    let guarded = poison::GUARDED.load(Ordering::Relaxed);
    drop(lib);
    // Anonymous and inaccessible: not the library, and not free for the
    // next dlopen to reuse.
    assert_eq!(mapping_at(info), Some(("---p".to_string(), String::new())));
    assert!(poison::GUARDED.load(Ordering::Relaxed) > guarded);
}

/// Run by the next test in a child process, since it dies.
#[test]
#[ignore = "calls into an unloaded build; run by a_stale_call_faults_and_names_the_build"]
fn stale_call() {
    let (lib, info) = open_counter("stale_call");
    drop(lib);
    let info: unsafe extern "C" fn() = unsafe { std::mem::transmute(info) };
    unsafe { info() };
    println!("the stale call returned");
}

#[test]
fn a_stale_call_faults_and_names_the_build() {
    let out = Command::new(std::env::current_exe().unwrap())
        .args(["--ignored", "--exact", "stale_call", "--nocapture"])
        .output()
        .unwrap();
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert_eq!(out.status.signal(), Some(11), "expected SIGSEGV; stderr:\n{stderr}");
    let named = format!("inside an unloaded build of {}", std::fs::canonicalize(lib("COUNTER_V1")).unwrap().display());
    assert!(stderr.contains(&named), "stderr:\n{stderr}");
    // The caller is found from the return address, since the faulting frame
    // is in the guard and can't be unwound.
    assert!(stderr.contains("called from:\n  "), "stderr:\n{stderr}");
}

/// The resident mod `vault` spawns a thread, which registers a TLS
/// destructor in its copy of std on the spawning thread (see
/// docs/lore/a-mod-that-spawns-a-thread-is-never-unmapped.md), so `dlclose`
/// leaves it mapped: counted and reported, not guarded.
#[test]
fn a_build_dlclose_keeps_mapped_is_reported_not_guarded() {
    let kept = poison::KEPT_MAPPED.load(Ordering::Relaxed);
    let e = Engine::new(None, tmp("kept"));
    e.load("vault", &lib("VAULT_V1")).expect("load vault");
    drop(e);
    assert!(poison::KEPT_MAPPED.load(Ordering::Relaxed) > kept);
}
