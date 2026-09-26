"""Compile flags for the C and C++ engines and the shims that include their headers.

Jolt's headers pick their SIMD code paths from the compiler's target macros
(__AVX2__ and so on, Jolt/Core/Core.h), and RegisterTypes refuses a caller
whose feature bits differ from the library's. So every translation unit that
includes a Jolt header needs the same -m flags, and a cc_library's copts
don't propagate: both targets load them from here.
"""

# Jolt's CMake default on x86-64 (Build/CMakeLists.txt: USE_AVX2, USE_LZCNT,
# USE_TZCNT, USE_F16C, USE_FMADD on; Jolt/Jolt.cmake turns those into these
# flags). The comparison's Rust code builds for baseline x86-64, so this is a
# caveat on every Jolt number; --define=jolt_simd=sse2 builds Jolt at the
# same baseline instead.
_AVX2 = ["-mavx2", "-mbmi", "-mpopcnt", "-mlzcnt", "-mf16c", "-mfma", "-mfpmath=sse", "-ffp-contract=fast"]

# SSE2 is what x86-64 guarantees, and what rustc targets by default. Jolt's
# CMake adds -ffp-contract=off when FMA is unavailable.
_SSE2 = ["-msse2", "-mfpmath=sse", "-ffp-contract=off"]

JOLT_COPTS = [
    "-std=c++17",
    # Jolt's CMake builds without RTTI and exceptions; its code assumes neither.
    "-fno-rtti",
    "-fno-exceptions",
    # The toolchain turns on -Wthread-safety, and Jolt's lock helpers take and
    # release mutexes across functions, which the analysis can't follow.
    "-Wno-thread-safety-analysis",
] + select({
    Label("//bench/physics3d:jolt_simd_sse2"): _SSE2,
    "//conditions:default": _AVX2,
}) + select({
    # Jolt's CMake Release is -O3; the toolchain's opt is -O2.
    Label("//bench/physics3d:opt"): ["-O3"],
    "//conditions:default": [],
})

# Release as Jolt's CMake builds it, minus the profiler and debug renderer:
# no JPH_PROFILE_ENABLED, JPH_DEBUG_RENDERER, JPH_ENABLE_ASSERTS or
# JPH_FLOATING_POINT_EXCEPTIONS_ENABLED (the last is MSVC-only in CMake
# anyway), and no JPH_OBJECT_STREAM, since nothing here serializes. So no
# defines at all: the toolchain's opt already passes -DNDEBUG.

# Box3D's CMake: C17 with extensions (anonymous unions), and -ffp-contract=off
# for its cross-platform determinism. Its SIMD is SSE2 on every x86-64 build
# (src/core.h), the same baseline as the Rust code, so no -m flags.
BOX3D_COPTS = ["-std=gnu17", "-ffp-contract=off"] + select({
    # CMake's Release for C is -O3 -DNDEBUG.
    Label("//bench/physics3d:opt"): ["-O3"],
    "//conditions:default": [],
})
