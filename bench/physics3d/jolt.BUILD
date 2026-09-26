# Jolt Physics, built from its sources with the hermetic llvm toolchain.
# Jolt only ships CMake; this mirrors Jolt/Jolt.cmake with every optional
# backend (GPU compute, CPU compute shaders) left out.
load("@@//bench/physics3d:copts.bzl", "JOLT_COPTS")
load("@rules_cc//cc:cc_library.bzl", "cc_library")

cc_library(
    name = "jolt",
    srcs = glob(
        ["Jolt/**/*.cpp"],
        exclude = [
            # GPU compute backends and the CPU compute shaders: all off
            # without JPH_USE_{DX12,VK,MTL,CPU_COMPUTE}.
            "Jolt/Compute/*/**",
            "Jolt/Shaders/**",
        ],
    ),
    hdrs = glob([
        "Jolt/**/*.h",
        "Jolt/**/*.inl",
    ]),
    copts = JOLT_COPTS,
    includes = ["."],
    linkopts = ["-pthread"],
    visibility = ["//visibility:public"],
)

# The MIT notice travels with anything that links the library.
exports_files(["LICENSE"])
