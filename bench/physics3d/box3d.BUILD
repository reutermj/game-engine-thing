# Box3D, built from its sources with the hermetic llvm toolchain, as its
# src/CMakeLists.txt builds the library (no Tracy, no validation).
load("@@//bench/physics3d:copts.bzl", "BOX3D_COPTS")
load("@rules_cc//cc:cc_library.bzl", "cc_library")

cc_library(
    name = "box3d",
    srcs = glob([
        "src/*.c",
        "src/*.h",
        "src/*.inl",
    ]),
    hdrs = glob(["include/box3d/*.h"]),
    copts = BOX3D_COPTS,
    includes = ["include"],
    linkopts = [
        "-lm",
        "-pthread",
    ],
    visibility = ["//visibility:public"],
)

# The MIT notice travels with anything that links the library.
exports_files(["LICENSE"])
