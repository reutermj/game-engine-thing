# rust_shared_library links the C++ runtime dynamically

With the hermetic `llvm` toolchain, a `rust_shared_library` gets libc++,
libc++abi and libunwind as **dynamic** dependencies, found through an
`$ORIGIN`-relative RUNPATH into Bazel's `_solib_` directory:

```
NEEDED   libc++.so.1  libc++abi.so.1  libunwind.so.1
RUNPATH  $ORIGIN/../../_solib__llvm+_Atoolchain_...
```

A `rust_binary` from the same toolchain links them statically, so the engine
binary never shows this. A mod only breaks once it is loaded from somewhere
other than `bazel-out`, and the loader always does that (see
[dlopen-returns-the-loaded-image-for-a-file-it-has-seen.md](dlopen-returns-the-loaded-image-for-a-file-it-has-seen.md)).
From the staging directory, `$ORIGIN` points nowhere useful and every load
fails with:

```
libc++.so.1: cannot open shared object file: No such file or directory
```

## Resolution

`engine_mod` sets `cc_runtime_linkage = "static"` on the `rust_shared_library`.
It is an attribute of the **patched** `rules_rust` that `rules_rs` provides
(`rust/private/rust.bzl`, `_CC_RUNTIME_LINKAGE_ATTRS`), not upstream
`rules_rust`, and the `rules_rs` README doesn't mention it. It was found by
reading the fetched source under
`$(./bazel info output_base)/external/rules_rs++rules_rust+rules_rust/`. With
it, the mod's only dynamic dependencies are libc, libdl and libpthread.

## What didn't work

`--dynamic_mode=off`, which the `llvm` README says forces the static runtime
path, changed nothing. `rules_rust` picks the runtime libraries for `cdylib`
crates itself (`get_cc_toolchain_runtime_libs` in `rust/private/rustc.bzl`),
and only `cc_runtime_linkage` feeds that choice.

The static build still carries a (now unused) RUNPATH into `bazel-out`. It is
harmless: nothing is loaded through it.
