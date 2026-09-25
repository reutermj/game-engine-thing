//! A mod's opened library, and a test-only mode that makes an unloaded
//! build's addresses fault.
//!
//! A pointer into a build that outlives it (a function pointer, a vtable, a
//! `&'static str`) is a use-after-unmap. Natively it may not show: `dlclose`
//! can leave the image mapped (a TLS destructor registered in it; see
//! docs/lore/a-thread-local-a-mod-touches-keeps-its-build-mapped.md), and once
//! it is unmapped a later `dlopen` or allocation may map something else at
//! the same address, so the stale call runs someone else's bytes. With
//! `ENGINE_POISON_UNLOADED` set, dropping the last keepalive of a build
//! closes it and then maps the span it occupied `PROT_NONE`, so any later
//! read or call into it faults at once, at the first bad access.
//!
//! - `ENGINE_POISON_UNLOADED=1`: guard spans `dlclose` really unmapped; warn
//!   about builds it kept mapped.
//! - `ENGINE_POISON_UNLOADED=strict`: also `mprotect` a kept build `PROT_NONE`.
//!   glibc still runs that build's TLS destructors at thread exit, so this
//!   faults on the (known, leaky but legal) thread-local case too.
//! - `ENGINE_POISON_UNLOADED=keep`: the opposite, for the sanitizers: never
//!   close a build, and keep its staged file, so a report at exit (a leak
//!   LeakSanitizer finds) can still name the frames in unloaded builds
//!   instead of printing `<unknown module>`.
//!
//! Unset, a `ModLibrary` is a `Library` and nothing more: no span is looked
//! up and `drop` is `dlclose`.

use std::cell::UnsafeCell;
use std::ffi::{CStr, c_char, c_int, c_void};
use std::mem::ManuallyDrop;
use std::ops::Deref;
use std::path::Path;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering};
use std::sync::{Mutex, Once, OnceLock};

use libloading::Library;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Mode {
    Off,
    Unmapped,
    Strict,
    Keep,
}

/// Read once: a mode that changed mid-process would guard some builds and
/// not others, and `keep` must hold from the first build to the last.
pub fn mode() -> Mode {
    static MODE: OnceLock<Mode> = OnceLock::new();
    *MODE.get_or_init(|| match std::env::var("ENGINE_POISON_UNLOADED").as_deref() {
        Ok("strict") => Mode::Strict,
        Ok("keep") => Mode::Keep,
        Ok("" | "0") | Err(_) => Mode::Off,
        Ok(_) => Mode::Unmapped,
    })
}

/// Builds guarded so far, and builds `dlclose` kept mapped: what a test can
/// assert on to know the mode did something.
pub static GUARDED: AtomicUsize = AtomicUsize::new(0);
pub static KEPT_MAPPED: AtomicUsize = AtomicUsize::new(0);

pub struct ModLibrary {
    lib: ManuallyDrop<Library>,
    /// `None` unless poisoning.
    guard: Option<Guard>,
}

struct Guard {
    /// The path it was opened from, as the link map names it.
    name: String,
    /// The library it's a copy of, still on disk to symbolize a fault with.
    source: String,
    /// The page span its `PT_LOAD` segments cover.
    span: (usize, usize),
}

/// Held across every `dlopen` and every close-and-guard while poisoning, so
/// one test thread's `dlopen` can't land in the span another has just
/// unmapped and not yet guarded. Other mappings (a large `malloc`) still can;
/// that is reported, not prevented.
static MAPPING: Mutex<()> = Mutex::new(());

impl ModLibrary {
    /// `dlopen`s `staged`, a copy of `source`.
    ///
    /// # Safety
    ///
    /// As `libloading::Library::new`: loading runs the library's
    /// initializers, so `staged` must be a mod build (one `engine_mod` made).
    /// Nothing may call into it once the `ModLibrary` is dropped; in poison
    /// mode its span then faults.
    pub unsafe fn open(staged: &Path, source: &Path) -> Result<ModLibrary, libloading::Error> {
        let guarding = matches!(mode(), Mode::Unmapped | Mode::Strict);
        if guarding {
            static HANDLER: Once = Once::new();
            HANDLER.call_once(|| unsafe { install_handler() });
        }
        let _mapping = guarding.then(|| MAPPING.lock().unwrap_or_else(|e| e.into_inner()));
        let lib = unsafe { Library::new(staged) }?;
        let guard = guarding.then(|| {
            let name = staged.to_string_lossy().into_owned();
            let source = std::fs::canonicalize(source).unwrap_or_else(|_| source.into());
            span_of(&name).map(|span| Guard { name, source: source.to_string_lossy().into_owned(), span })
        });
        let guard = guard.flatten();
        Ok(ModLibrary { lib: ManuallyDrop::new(lib), guard })
    }
}

impl Deref for ModLibrary {
    type Target = Library;
    fn deref(&self) -> &Library {
        &self.lib
    }
}

impl Drop for ModLibrary {
    fn drop(&mut self) {
        // The handle is leaked on purpose, so the build stays in the link
        // map for a sanitizer's report at exit.
        if mode() == Mode::Keep {
            return;
        }
        let lib = unsafe { ManuallyDrop::take(&mut self.lib) };
        let Some(Guard { name, source, span: (start, end) }) = self.guard.take() else {
            drop(lib);
            return;
        };
        let _mapping = MAPPING.lock().unwrap_or_else(|e| e.into_inner());
        if let Err(e) = lib.close() {
            eprintln!("[poison] dlclose {name}: {e}");
            return;
        }
        let len = end - start;
        if span_of(&name).is_some() {
            KEPT_MAPPED.fetch_add(1, Ordering::Relaxed);
            let strict = mode() == Mode::Strict;
            eprintln!(
                "[poison] {name} stayed mapped after dlclose (a TLS destructor registered in it?){}",
                if strict { "; protecting it anyway" } else { "; a stale pointer into it still works" }
            );
            if strict {
                if unsafe { mprotect(start as *mut c_void, len, PROT_NONE) } != 0 {
                    eprintln!("[poison] mprotect {name}: {}", std::io::Error::last_os_error());
                } else {
                    record(&source, start, end, true);
                }
            }
            return;
        }
        // Reserved, not just protected: nothing else may be mapped here for
        // the rest of the process, so a stale pointer can't land in a newer
        // build's code and run it.
        let at = unsafe {
            mmap(
                start as *mut c_void,
                len,
                PROT_NONE,
                MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE | MAP_FIXED_NOREPLACE,
                -1,
                0,
            )
        };
        if at as usize != start {
            // Another thread mapped into the span between the unmap and here.
            eprintln!(
                "[poison] {name}: couldn't guard {start:#x}..{end:#x}: {}",
                std::io::Error::last_os_error()
            );
            return;
        }
        GUARDED.fetch_add(1, Ordering::Relaxed);
        record(&source, start, end, false);
    }
}

/// Guarded spans and the builds they were, for the fault handler: a crash
/// under `bazel test` otherwise leaves a log that just stops (libtest's
/// captured output dies with the process), with nothing to say which build
/// the pointer was into. Written under `MAPPING`; `end` is published last,
/// so the handler, which can't lock, reads only complete entries.
struct Guarded {
    start: AtomicUsize,
    end: AtomicUsize,
    name: UnsafeCell<[u8; NAME]>,
    name_len: AtomicUsize,
    /// Protected in place while still in the link map (`strict`): a
    /// backtrace would walk its protected headers and fault again.
    kept: AtomicBool,
}
unsafe impl Sync for Guarded {}
const NAME: usize = 240;
const MAX_GUARDED: usize = 4096;
static SPANS: [Guarded; MAX_GUARDED] = [const {
    Guarded {
        start: AtomicUsize::new(0),
        end: AtomicUsize::new(0),
        name: UnsafeCell::new([0; NAME]),
        name_len: AtomicUsize::new(0),
        kept: AtomicBool::new(false),
    }
}; MAX_GUARDED];
static RECORDED: AtomicUsize = AtomicUsize::new(0);

fn record(name: &str, start: usize, end: usize, kept: bool) {
    let i = RECORDED.load(Ordering::Relaxed);
    let Some(slot) = SPANS.get(i) else { return };
    // The tail of the path, if it's long: the end is what identifies it.
    let bytes = &name.as_bytes()[name.len().saturating_sub(NAME)..];
    unsafe { (&mut *slot.name.get())[..bytes.len()].copy_from_slice(bytes) };
    slot.name_len.store(bytes.len(), Ordering::Relaxed);
    slot.kept.store(kept, Ordering::Relaxed);
    slot.start.store(start, Ordering::Relaxed);
    slot.end.store(end, Ordering::Release);
    RECORDED.store(i + 1, Ordering::Release);
}

static mut PREVIOUS: SigAction = SigAction { handler: 0, mask: [0; 16], flags: 0, restorer: 0 };

/// Says which unloaded build a fault landed in, then puts back the handler
/// it replaced (std's, which tells a stack overflow from other faults) and
/// returns, so the access faults again and dies the way it would have.
unsafe fn install_handler() {
    let action = SigAction {
        handler: on_fault as *const () as usize,
        mask: [0; 16],
        // On std's alternate stack, as its own handler runs, so a stack
        // overflow still reaches it.
        flags: SA_SIGINFO | SA_ONSTACK,
        restorer: 0,
    };
    unsafe { sigaction(SIGSEGV, &action, &raw mut PREVIOUS) };
}

unsafe extern "C" fn on_fault(_signal: c_int, info: *mut SigInfo, context: *mut c_void) {
    let addr = unsafe { (*info).addr };
    let n = RECORDED.load(Ordering::Acquire);
    // The newest match: a span can be guarded again after `strict` protects
    // a build that later really unmaps.
    for slot in SPANS[..n].iter().rev() {
        let (start, end) = (slot.start.load(Ordering::Relaxed), slot.end.load(Ordering::Acquire));
        if (start..end).contains(&addr) {
            let name = unsafe { &(&*slot.name.get())[..slot.name_len.load(Ordering::Relaxed)] };
            let mut line = [0u8; 512];
            let mut w = Writer { buf: &mut line, at: 0 };
            w.put(b"\n[poison] fault at ");
            w.hex(addr);
            w.put(b", inside an unloaded build of ");
            w.put(name);
            w.put(b"; llvm-symbolizer --obj=<that> ");
            w.hex(addr - start);
            w.put(b" names it\n");
            let at = w.at;
            unsafe { write(2, line.as_ptr() as *const c_void, at) };
            if slot.kept.load(Ordering::Relaxed) {
                break;
            }
            // A call through a stale pointer faults with the instruction
            // pointer in the guard, where there is nothing to unwind from;
            // its caller is the return address the call just pushed. x86_64
            // `ucontext_t`: `gregs` start at byte 40, RSP is 15, RIP 16.
            let gregs = unsafe { (context as *const u8).add(40) as *const usize };
            let (rsp, rip) = unsafe { (*gregs.add(15), *gregs.add(16)) };
            if rip == addr {
                unsafe { write(2, b"  called from:\n".as_ptr() as *const c_void, 15) };
                unsafe { frame_at(*(rsp as *const usize)) };
            }
            // Raw frames, each as its object and offset for llvm-symbolizer:
            // `std::backtrace` resolves symbols, which overflows the small
            // signal stack std gives each thread. The unwinder stops at a
            // frame in the guard, so after a stale call this shows only the
            // handler's own frames.
            unsafe { write(2, b"  backtrace:\n".as_ptr() as *const c_void, 13) };
            unsafe { _Unwind_Backtrace(frame, std::ptr::null_mut()) };
            break;
        }
    }
    unsafe { sigaction(SIGSEGV, &raw const PREVIOUS, std::ptr::null_mut()) };
}

unsafe extern "C" fn frame(ctx: *mut c_void, _arg: *mut c_void) -> c_int {
    unsafe { frame_at(_Unwind_GetIP(ctx)) };
    0
}

unsafe fn frame_at(ip: usize) {
    let mut info = DlInfo { fname: std::ptr::null(), fbase: 0, sname: std::ptr::null(), saddr: 0 };
    let mut line = [0u8; 512];
    let mut w = Writer { buf: &mut line, at: 0 };
    w.put(b"  ");
    // The link map's `l_addr` (its first field) is the load bias, which is
    // what turns `ip` into the object's own address, as llvm-symbolizer
    // takes it: `fbase` isn't, for an executable not linked at zero.
    let mut map: *const usize = std::ptr::null();
    if unsafe { dladdr1(ip as *const c_void, &mut info, &mut map, RTLD_DL_LINKMAP) } != 0
        && !info.fname.is_null()
        && !map.is_null()
    {
        w.put(unsafe { CStr::from_ptr(info.fname) }.to_bytes());
        w.put(b" ");
        w.hex(ip - unsafe { *map });
    } else {
        w.hex(ip);
    }
    w.put(b"\n");
    let at = w.at;
    unsafe { write(2, line.as_ptr() as *const c_void, at) };
}

struct Writer<'a> {
    buf: &'a mut [u8],
    at: usize,
}

impl Writer<'_> {
    fn put(&mut self, bytes: &[u8]) {
        let n = bytes.len().min(self.buf.len() - self.at);
        self.buf[self.at..self.at + n].copy_from_slice(&bytes[..n]);
        self.at += n;
    }
    fn hex(&mut self, value: usize) {
        self.put(b"0x");
        let digits = (usize::BITS as usize / 4 - value.leading_zeros() as usize / 4).max(1);
        for i in (0..digits).rev() {
            self.put(&[b"0123456789abcdef"[(value >> (i * 4)) & 0xf]]);
        }
    }
}

/// The page span covering every `PT_LOAD` segment of the loaded object named
/// `name`, if it is still in the link map. One span, not a list: ld.so
/// reserves it whole and fills the gaps between segments with `PROT_NONE`,
/// and `dlclose` unmaps it whole.
fn span_of(name: &str) -> Option<(usize, usize)> {
    struct Search<'a> {
        name: &'a str,
        span: Option<(usize, usize)>,
    }
    unsafe extern "C" fn visit(info: *mut DlPhdrInfo, _size: usize, data: *mut c_void) -> c_int {
        unsafe {
            let search = &mut *(data as *mut Search);
            let info = &*info;
            if info.name.is_null() || CStr::from_ptr(info.name).to_bytes() != search.name.as_bytes() {
                return 0;
            }
            let page = page_size();
            let (mut lo, mut hi) = (usize::MAX, 0);
            for i in 0..info.phnum as usize {
                let ph = &*info.phdr.add(i);
                if ph.p_type == PT_LOAD {
                    let at = info.addr + ph.p_vaddr as usize;
                    lo = lo.min(at & !(page - 1));
                    hi = hi.max((at + ph.p_memsz as usize).next_multiple_of(page));
                }
            }
            if lo < hi {
                search.span = Some((lo, hi));
            }
            1
        }
    }
    let mut search = Search { name, span: None };
    unsafe { dl_iterate_phdr(visit, &mut search as *mut Search as *mut c_void) };
    search.span
}

fn page_size() -> usize {
    unsafe { sysconf(SC_PAGESIZE) as usize }
}

// glibc's x86_64 definitions, declared here rather than taking on the libc
// crate for a handful of symbols.
#[repr(C)]
struct DlPhdrInfo {
    addr: usize,
    name: *const c_char,
    phdr: *const Elf64Phdr,
    phnum: u16,
}

#[repr(C)]
struct Elf64Phdr {
    p_type: u32,
    p_flags: u32,
    p_offset: u64,
    p_vaddr: u64,
    p_paddr: u64,
    p_filesz: u64,
    p_memsz: u64,
    p_align: u64,
}

const PT_LOAD: u32 = 1;
const PROT_NONE: c_int = 0;
const MAP_PRIVATE: c_int = 0x02;
const MAP_ANONYMOUS: c_int = 0x20;
const MAP_NORESERVE: c_int = 0x4000;
const MAP_FIXED_NOREPLACE: c_int = 0x100000;
const SC_PAGESIZE: c_int = 30;
const SIGSEGV: c_int = 11;
const RTLD_DL_LINKMAP: c_int = 2;
const SA_SIGINFO: c_int = 4;
const SA_ONSTACK: c_int = 0x0800_0000;

#[repr(C)]
struct DlInfo {
    fname: *const c_char,
    fbase: usize,
    sname: *const c_char,
    saddr: usize,
}

#[repr(C)]
struct SigAction {
    handler: usize,
    mask: [u64; 16],
    flags: c_int,
    restorer: usize,
}

/// Only as far as `si_addr`, which is all a `SIGSEGV` needs.
#[repr(C)]
struct SigInfo {
    signo: c_int,
    errno: c_int,
    code: c_int,
    _pad: c_int,
    addr: usize,
}

unsafe extern "C" {
    fn dl_iterate_phdr(
        callback: unsafe extern "C" fn(*mut DlPhdrInfo, usize, *mut c_void) -> c_int,
        data: *mut c_void,
    ) -> c_int;
    fn mmap(addr: *mut c_void, len: usize, prot: c_int, flags: c_int, fd: c_int, off: i64) -> *mut c_void;
    fn mprotect(addr: *mut c_void, len: usize, prot: c_int) -> c_int;
    fn sysconf(name: c_int) -> i64;
    fn sigaction(signal: c_int, action: *const SigAction, old: *mut SigAction) -> c_int;
    fn write(fd: c_int, buf: *const c_void, len: usize) -> isize;
    fn dladdr1(addr: *const c_void, info: *mut DlInfo, extra: *mut *const usize, flags: c_int) -> c_int;
    fn _Unwind_Backtrace(trace: unsafe extern "C" fn(*mut c_void, *mut c_void) -> c_int, arg: *mut c_void) -> c_int;
    fn _Unwind_GetIP(ctx: *mut c_void) -> usize;
}
