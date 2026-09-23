"""Rules for building engine mods and the games that bundle them.

    engine_mod(name = "counter", srcs = ["lib.rs"])

builds `libcounter.so`. `bazel run //mods/counter` sends it to the running
engine, which loads it, or hot-reloads it if it's already running.

    engine_game(name = "game", bootstrap = "//mods/bootstrap", mods = [...])

writes a manifest of the baked-in mods; `bazel run //game` starts the engine with it.
"""

load("@rules_rs//rs:rust_shared_library.bzl", "rust_shared_library")

EngineModInfo = provider(
    doc = "A mod the engine can load.",
    fields = {
        "mod_name": "Name the engine knows the mod by. Reloads are matched by name.",
        "library": "The mod's shared library File.",
    },
)

def _rlocationpath(ctx, file):
    # Matches `$(rlocationpath)`: external files have short paths like `../repo/pkg/f`.
    if file.short_path.startswith("../"):
        return file.short_path[3:]
    return ctx.workspace_name + "/" + file.short_path

def _launcher(ctx, executable_attr):
    """Makes the target's executable a symlink to another binary, keeping its runfiles."""
    exe = ctx.actions.declare_file(ctx.label.name)
    ctx.actions.symlink(
        output = exe,
        target_file = executable_attr.files_to_run.executable,
        is_executable = True,
    )
    return exe, executable_attr[DefaultInfo].default_runfiles

def _engine_mod_impl(ctx):
    libraries = [f for f in ctx.files.library if f.extension == "so"]
    if len(libraries) != 1:
        fail("expected exactly one .so from %s, got %s" % (ctx.attr.library.label, libraries))
    library = libraries[0]

    exe, modctl_runfiles = _launcher(ctx, ctx.attr._modctl)
    return [
        DefaultInfo(
            executable = exe,
            files = depset([library]),
            runfiles = ctx.runfiles(files = [library]).merge(modctl_runfiles),
        ),
        RunEnvironmentInfo(environment = {
            "ENGINE_MOD_NAME": ctx.attr.mod_name,
            "ENGINE_MOD_RLOCATION": _rlocationpath(ctx, library),
        }),
        EngineModInfo(mod_name = ctx.attr.mod_name, library = library),
    ]

_engine_mod = rule(
    implementation = _engine_mod_impl,
    executable = True,
    attrs = {
        "library": attr.label(mandatory = True),
        "mod_name": attr.string(mandatory = True),
        "_modctl": attr.label(
            default = "//engine/modctl",
            executable = True,
            cfg = "target",
        ),
    },
)

def engine_mod(name, srcs, deps = [], mod_name = None, visibility = None, **kwargs):
    """A hot-reloadable mod.

    Args:
      name: Target name. `bazel run` on it (re)loads the mod into the running engine.
      srcs: Rust sources; the crate root is `lib.rs`.
      deps: Extra Rust deps. `//engine/api` is always included.
      mod_name: Name the engine uses for the mod. Defaults to `name`.
      visibility: Visibility of the mod target.
      **kwargs: Passed to the underlying `rust_shared_library`.
    """
    rust_shared_library(
        name = name + "_lib",
        crate_name = name.replace("-", "_"),
        crate_root = "lib.rs",
        srcs = srcs,
        deps = deps + ["//engine/api"],
        # The engine copies each .so before dlopen, which breaks the $ORIGIN
        # RUNPATH a dynamically linked C++/unwind runtime would need.
        cc_runtime_linkage = "static",
        # Bind every symbol at dlopen time so a build with unresolved symbols is
        # rejected while the previous build is still running. The llvm toolchain
        # already links with -z now; pin it so the loader can rely on it. See
        # docs/lore/mods-are-linked-bind-now.md.
        rustc_flags = kwargs.pop("rustc_flags", []) + ["-Clink-arg=-Wl,-z,now"],
        visibility = ["//visibility:private"],
        **kwargs
    )
    _engine_mod(
        name = name,
        library = ":" + name + "_lib",
        mod_name = mod_name or name,
        visibility = visibility,
    )

def _engine_game_impl(ctx):
    bootstrap = ctx.attr.bootstrap[EngineModInfo]
    mods = [m[EngineModInfo] for m in ctx.attr.mods]

    lines = ["bootstrap %s %s" % (bootstrap.mod_name, _rlocationpath(ctx, bootstrap.library))]
    lines += ["mod %s %s" % (m.mod_name, _rlocationpath(ctx, m.library)) for m in mods]
    manifest = ctx.actions.declare_file(ctx.label.name + ".manifest")
    ctx.actions.write(manifest, "\n".join(lines) + "\n")

    exe, engine_runfiles = _launcher(ctx, ctx.attr._engine)
    libraries = [bootstrap.library] + [m.library for m in mods]
    return [
        DefaultInfo(
            executable = exe,
            files = depset([manifest]),
            runfiles = ctx.runfiles(files = [manifest] + libraries).merge(engine_runfiles),
        ),
        RunEnvironmentInfo(environment = {
            "ENGINE_MANIFEST": _rlocationpath(ctx, manifest),
        }),
    ]

engine_game = rule(
    implementation = _engine_game_impl,
    executable = True,
    doc = "The engine plus a baked-in list of mods loaded at startup.",
    attrs = {
        "bootstrap": attr.label(
            mandatory = True,
            providers = [EngineModInfo],
            doc = "The mod that owns the frame loop and steps the others.",
        ),
        "mods": attr.label_list(
            providers = [EngineModInfo],
            doc = "Mods loaded after the bootstrap mod, in order.",
        ),
        "_engine": attr.label(
            default = "//engine/loader:engine",
            executable = True,
            cfg = "target",
        ),
    },
)
