//! Minimal `dlopen` wrapper.

use std::ffi::{CStr, CString, c_char, c_int, c_void};
use std::os::unix::ffi::OsStrExt;
use std::path::Path;

#[link(name = "dl")]
unsafe extern "C" {
    fn dlopen(filename: *const c_char, flags: c_int) -> *mut c_void;
    fn dlsym(handle: *mut c_void, symbol: *const c_char) -> *mut c_void;
    fn dlclose(handle: *mut c_void) -> c_int;
    fn dlerror() -> *mut c_char;
}

const RTLD_NOW: c_int = 0x2;
const RTLD_LOCAL: c_int = 0x0;

pub struct Library(*mut c_void);

impl Library {
    pub fn open(path: &Path) -> Result<Library, String> {
        let path = CString::new(path.as_os_str().as_bytes()).map_err(|e| e.to_string())?;
        let handle = unsafe { dlopen(path.as_ptr(), RTLD_NOW | RTLD_LOCAL) };
        if handle.is_null() {
            return Err(last_error());
        }
        Ok(Library(handle))
    }

    /// Looks up `name` (NUL-terminated) and reinterprets it as `T`.
    ///
    /// # Safety
    /// `T` must be a function pointer type matching the symbol's real signature.
    pub unsafe fn symbol<T: Copy>(&self, name: &[u8]) -> Result<T, String> {
        assert_eq!(size_of::<T>(), size_of::<*mut c_void>());
        let name = CStr::from_bytes_with_nul(name).map_err(|e| e.to_string())?;
        let sym = unsafe { dlsym(self.0, name.as_ptr()) };
        if sym.is_null() {
            return Err(format!("missing symbol {}", name.to_string_lossy()));
        }
        Ok(unsafe { std::mem::transmute_copy(&sym) })
    }
}

impl Drop for Library {
    fn drop(&mut self) {
        unsafe { dlclose(self.0) };
    }
}

fn last_error() -> String {
    let err = unsafe { dlerror() };
    if err.is_null() {
        return "unknown dlopen error".into();
    }
    unsafe { CStr::from_ptr(err) }.to_string_lossy().into_owned()
}
