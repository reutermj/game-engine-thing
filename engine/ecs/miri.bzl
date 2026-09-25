"""Miri tests for the ECS's unsafe core: one target per aliasing model.

Miri is Miri's own nightly toolchain (`experimental_miri` in MODULE.bazel),
registered for its own toolchain types, so these build with it and
nothing else does. Manual, since they take minutes; see
docs/runbooks/002-run-miri-and-fuzz-the-core.md.
"""

load("@rules_rs//rs/experimental/miri:miri_test.bzl", "miri_test")

MIRI_FLAGS = [
    # Alignment from what the pointer was derived from, not from the
    # address it happens to have, so a misaligned read that lands on an
    # aligned address still fails.
    "-Zmiri-symbolic-alignment-check",
    # Integers cast to pointers carry no provenance: what the columns'
    # dangling pointers for zero-sized values rely on being fine.
    "-Zmiri-strict-provenance",
]

# Stacked Borrows (Miri's default) and Tree Borrows: the aliasing rules are
# not settled, and code sound under one can be UB under the other.
MIRI_MODELS = {
    "sb": [],
    "tb": ["-Zmiri-tree-borrows"],
}

def ecs_miri_test(name, srcs, crate_root, crate_name, deps = [], data = [], miri_flags = [], tags = [], shard_count = None):
    """`<name>_sb` and `<name>_tb`: `srcs` as a test crate under Miri.

    Args:
      name: the targets' prefix.
      srcs: the test crate's sources.
      crate_root: its root.
      crate_name: its name.
      deps: libraries, compiled by Miri-as-rustc through an aspect.
      data: runfiles, such as a corpus.
      miri_flags: flags beyond `MIRI_FLAGS` and the model's.
      tags: tags beyond `manual` and `miri`.
      shard_count: for a test that shards itself (libtest doesn't).

    Returns:
      The targets' labels, for a test_suite.
    """
    tests = []
    for model, flags in MIRI_MODELS.items():
        miri_test(
            name = "{}_{}".format(name, model),
            srcs = srcs,
            crate_name = crate_name,
            crate_root = crate_root,
            data = data,
            edition = "2024",
            miri_flags = MIRI_FLAGS + flags + miri_flags,
            shard_count = shard_count,
            tags = ["manual", "miri"] + tags,
            timeout = "eternal",
            deps = deps,
        )
        tests.append(":{}_{}".format(name, model))
    return tests
