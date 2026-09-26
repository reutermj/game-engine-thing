# A cc_library under -c opt is -O2, not a CMake Release's -O3

Found bringing Jolt Physics and Box3D in for `//bench/physics3d`
(2026-09-25). With the hermetic `llvm` module (0.8.23, clang 23), `./bazel
aquery -c opt 'mnemonic("CppCompile", @jolt//:jolt)'` shows the toolchain
compiling every C and C++ file with, among others:

```
-g0 -O2 -D_FORTIFY_SOURCE=1 -DNDEBUG -ffunction-sections -fdata-sections
-fstack-protector -Wall -Wthread-safety ... -fno-omit-frame-pointer -fPIC
```

CMake's Release, which is how Jolt and Box3D are built and benchmarked
upstream, is `-O3` (Jolt sets `CMAKE_CXX_FLAGS_RELEASE "-O3"`; CMake's own
default for C is `-O3 -DNDEBUG`). A library brought in with plain `-c opt`
is therefore compiled less aggressively than its authors measure it, and
also keeps frame pointers and stack protectors. The Rust side is not
affected: rustc gets `-C opt-level=3` under `-c opt`.

## Resolution

`bench/physics3d/copts.bzl` adds `-O3` under a `compilation_mode = opt`
`config_setting`. A target's copts come after the toolchain's, and the last
`-O` wins, which the aquery line confirms (`... -O2 ... -O3 -c file.cpp`).
The other toolchain flags are left as they are.

The same `-Wall -Wthread-safety` floods Jolt's build with warnings about its
lock helpers, which take a mutex in one function and release it in
another; `-Wno-thread-safety-analysis` on Jolt silences them. Nothing else
in either library needed a change: both built with the hermetic toolchain on
the first try, from a glob of their sources minus the optional GPU and
compute-shader backends (Jolt) and nothing (Box3D).
