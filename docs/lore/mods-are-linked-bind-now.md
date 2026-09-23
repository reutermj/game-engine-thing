# Mods are linked BIND_NOW

`libloading::Library::new` opens with `RTLD_LAZY` on Unix, and the portable
API has no way to ask for `RTLD_NOW`. With lazy binding, a library with an
unresolved function symbol loads fine and crashes at the first call. For the
loader that is the worst case, because by then the old build is gone.

It doesn't happen, because the library itself asks for eager binding. The
hermetic `llvm` toolchain links every Linux target with
`-Wl,-z,relro,-z,now` (read in source:
`external/llvm+/toolchain/args/linux/BUILD.bazel`), and `rules_rust` passes
the C++ toolchain's link flags through, so it shows up in `./bazel aquery` for
every mod. The result is in the dynamic section:

```
$ readelf -d libcounter.so | grep FLAGS
 (FLAGS)    BIND_NOW
 (FLAGS_1)  Flags: NOW
```

glibc binds every symbol at `dlopen` time for a library carrying `BIND_NOW`,
whatever mode the caller passed. So `Library::new` fails early on its own,
and no `#[cfg(unix)]` call to `os::unix::Library::open` with `RTLD_NOW` is
needed.

## Resolution

`engine_mod` passes `-Clink-arg=-Wl,-z,now` explicitly anyway. The loader's
guarantee then rests on a line in `engine/defs.bzl` rather than on a default
of a toolchain this repo doesn't control.

*(History: this entry and the `engine_mod` comment first attributed the flag
to rustc's full-RELRO default. That was never checked; the `aquery` line
traces to the `llvm` toolchain's args, corrected 2026-09-23.)*
