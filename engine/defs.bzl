"""Rules for building engine mods and the games that bundle them.

    engine_mod(
        name = "physics",
        srcs = ["lib.rs"],
        interface = ["components.rs"],
        mod_deps = ["//mods/transform"],
    )

builds `libphysics_mod.so`. `bazel run //mods/physics` sends it to the running
engine, which loads it, or hot-reloads it if it's already running.

A mod's `interface` is what other mods may use (its components); `mod_deps`
are the mods whose interfaces this one uses. Other mods depend on the
interface, never the implementation, so Bazel rebuilds a dependent only when
the interface it compiled against changes. See docs/architecture/mod-deps.md.

    engine_game(name = "game", bootstrap = "//mods/bootstrap", mods = [...])

writes the manifest of mods loaded at startup; `bazel run //game` starts the
engine with it, and `bazel run //game:reload` reloads every mod whose build
changed, in one batch.
"""

load("@rules_rs//rs:rust_library.bzl", "rust_library")
load("@rules_rs//rs:rust_shared_library.bzl", "rust_shared_library")

EngineModInfo = provider(
    doc = "A mod the engine can load.",
    fields = {
        "mod_name": "Name the engine knows the mod by. Reloads are matched by name.",
        "library": "The mod's shared library File.",
        "closure": "depset of struct(mod_name, library) for this mod and every " +
                   "mod it depends on, postorder: dependencies first.",
    },
)

ModLinksInfo = provider(
    doc = "A mod's interface digest, for mods that depend on it.",
    fields = {
        "mod_name": "The mod's name.",
        "digest": "File holding the interface digest; empty if it has no interface.",
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

def _mod_links_impl(ctx):
    env = ctx.actions.declare_file(ctx.label.name + ".env")
    digest = ctx.actions.declare_file(ctx.label.name + ".digest")
    args = ctx.actions.args()
    args.add("--out-env", env)
    args.add("--out-digest", digest)

    # Sorted, so the digest doesn't depend on the order srcs were listed in.
    for src in sorted(ctx.files.srcs, key = lambda f: f.short_path):
        args.add("--src", src)
    for dep in ctx.attr.deps:
        args.add("--dep", "%s=%s" % (dep[ModLinksInfo].mod_name, dep[ModLinksInfo].digest.path))
    ctx.actions.run(
        executable = ctx.executable._tool,
        arguments = [args],
        inputs = ctx.files.srcs + [dep[ModLinksInfo].digest for dep in ctx.attr.deps],
        outputs = [env, digest],
        mnemonic = "ModLinks",
    )
    return [
        DefaultInfo(files = depset([env])),
        ModLinksInfo(mod_name = ctx.attr.mod_name, digest = digest),
    ]

_mod_links = rule(
    implementation = _mod_links_impl,
    attrs = {
        "deps": attr.label_list(providers = [ModLinksInfo]),
        "mod_name": attr.string(mandatory = True),
        "srcs": attr.label_list(allow_files = [".rs"]),
        "_tool": attr.label(
            default = "//engine/tools:mod_links",
            executable = True,
            cfg = "exec",
        ),
    },
)

def _engine_mod_impl(ctx):
    libraries = [f for f in ctx.files.library if f.extension == "so"]
    if len(libraries) != 1:
        fail("expected exactly one .so from %s, got %s" % (ctx.attr.library.label, libraries))
    library = libraries[0]

    exe, modctl_runfiles = _launcher(ctx, ctx.attr._modctl)
    closure = depset(
        [struct(mod_name = ctx.attr.mod_name, library = library)],
        transitive = [dep[EngineModInfo].closure for dep in ctx.attr.mod_deps],
        order = "postorder",
    )
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
        EngineModInfo(mod_name = ctx.attr.mod_name, library = library, closure = closure),
    ]

_engine_mod = rule(
    implementation = _engine_mod_impl,
    executable = True,
    attrs = {
        "library": attr.label(mandatory = True),
        "mod_deps": attr.label_list(providers = [EngineModInfo]),
        "mod_name": attr.string(mandatory = True),
        "_modctl": attr.label(
            default = "//engine/modctl",
            executable = True,
            cfg = "target",
        ),
    },
)

def engine_mod(
        name,
        srcs,
        interface = [],
        mod_deps = [],
        deps = [],
        mod_name = None,
        visibility = None,
        **kwargs):
    """A hot-reloadable mod.

    Args:
      name: Target name. `bazel run` on it (re)loads the mod into the running engine.
      srcs: The implementation's Rust sources. The crate root is `lib.rs` unless
        `crate_root` is passed, which lets one source file build several
        variants of a mod (with different `crate_features`), as the tests do.
      interface: Rust sources other mods may depend on: the components this mod
        declares. The first file is the crate root, and the crate is named after
        the mod, so dependents write `use physics::Velocity`.
      mod_deps: `engine_mod` targets whose interfaces this mod uses. The engine
        loads them first and refuses to strand this mod by reloading one of them
        with a different interface.
      deps: Extra Rust deps. `//engine/api` is always included.
      mod_name: Name the engine uses for the mod. Defaults to `name`.
      visibility: Visibility of the mod target and its interface.
      **kwargs: Passed to the underlying `rust_shared_library`.
    """
    mod_name = mod_name or name
    # Every target this macro declares is part of the mod, so all of them
    # share its testonly-ness.
    testonly = kwargs.pop("testonly", False)
    dep_labels = [native.package_relative_label(d) for d in mod_deps]
    interface_deps = [dep.same_package_label(dep.name + "_interface") for dep in dep_labels]

    if interface:
        rust_library(
            name = name + "_interface",
            crate_name = mod_name.replace("-", "_"),
            crate_root = interface[0],
            srcs = interface,
            deps = interface_deps + ["//engine/api"],
            testonly = testonly,
            visibility = visibility,
        )
    _mod_links(
        name = name + "_links",
        srcs = interface,
        deps = [dep.same_package_label(dep.name + "_links") for dep in dep_labels],
        mod_name = mod_name,
        testonly = testonly,
        visibility = visibility,
    )
    rust_shared_library(
        name = name + "_lib",
        # Not the mod's name: that is the interface crate's, and the
        # implementation links it.
        crate_name = name.replace("-", "_") + "_mod",
        crate_root = kwargs.pop("crate_root", "lib.rs"),
        srcs = srcs,
        deps = deps + interface_deps + ([":" + name + "_interface"] if interface else []) + ["//engine/api"],
        # Bakes the interface digests into the library; see engine/tools/mod_links.rs.
        rustc_env_files = [":" + name + "_links"],
        # The engine copies each .so before dlopen, which breaks the $ORIGIN
        # RUNPATH a dynamically linked C++/unwind runtime would need.
        cc_runtime_linkage = "static",
        # Bind every symbol at dlopen time so a build with unresolved symbols is
        # rejected while the previous build is still running. The llvm toolchain
        # already links with -z now; pin it so the loader can rely on it. See
        # docs/lore/mods-are-linked-bind-now.md.
        rustc_flags = kwargs.pop("rustc_flags", []) + ["-Clink-arg=-Wl,-z,now"],
        testonly = testonly,
        visibility = ["//visibility:private"],
        **kwargs
    )
    _engine_mod(
        name = name,
        library = ":" + name + "_lib",
        mod_deps = mod_deps,
        mod_name = mod_name,
        testonly = testonly,
        visibility = visibility,
    )

GameInfo = provider(
    doc = "A game's manifest and everything it names.",
    fields = {
        "manifest": "The manifest File.",
        "runfiles": "runfiles with the manifest and every mod library.",
    },
)

def _engine_game_impl(ctx):
    bootstrap = ctx.attr.bootstrap[EngineModInfo]

    # Dependencies first, and each mod once, including dependencies the game
    # didn't list.
    closure = depset(
        transitive = [bootstrap.closure] + [m[EngineModInfo].closure for m in ctx.attr.mods],
        order = "postorder",
    ).to_list()
    lines = ["reload %s" % ctx.attr.reload_label]
    for m in closure:
        kind = "bootstrap" if m.mod_name == bootstrap.mod_name else "mod"
        lines.append("%s %s %s" % (kind, m.mod_name, _rlocationpath(ctx, m.library)))
    manifest = ctx.actions.declare_file(ctx.label.name + ".manifest")
    ctx.actions.write(manifest, "\n".join(lines) + "\n")

    game_runfiles = ctx.runfiles(files = [manifest] + [m.library for m in closure])
    exe, engine_runfiles = _launcher(ctx, ctx.attr._engine)
    return [
        DefaultInfo(
            executable = exe,
            files = depset([manifest]),
            runfiles = game_runfiles.merge(engine_runfiles),
        ),
        RunEnvironmentInfo(environment = {
            "ENGINE_MANIFEST": _rlocationpath(ctx, manifest),
        }),
        GameInfo(manifest = manifest, runfiles = game_runfiles),
    ]

_engine_game = rule(
    implementation = _engine_game_impl,
    executable = True,
    attrs = {
        "bootstrap": attr.label(mandatory = True, providers = [EngineModInfo]),
        "mods": attr.label_list(providers = [EngineModInfo]),
        "reload_label": attr.string(mandatory = True),
        "_engine": attr.label(
            default = "//engine/loader:engine",
            executable = True,
            cfg = "target",
        ),
    },
)

def _engine_game_reload_impl(ctx):
    game = ctx.attr.game[GameInfo]
    exe, modctl_runfiles = _launcher(ctx, ctx.attr._modctl)
    return [
        DefaultInfo(executable = exe, runfiles = game.runfiles.merge(modctl_runfiles)),
        RunEnvironmentInfo(environment = {
            "ENGINE_BATCH_MANIFEST": _rlocationpath(ctx, game.manifest),
        }),
    ]

_engine_game_reload = rule(
    implementation = _engine_game_reload_impl,
    executable = True,
    attrs = {
        "game": attr.label(mandatory = True, providers = [GameInfo]),
        "_modctl": attr.label(
            default = "//engine/modctl",
            executable = True,
            cfg = "target",
        ),
    },
)

def engine_game(name, bootstrap, mods = [], visibility = None):
    """The engine plus the mods loaded at startup, and a target to reload them.

    Args:
      name: `bazel run` on it starts the engine.
      bootstrap: The mod that owns the frame loop and steps the others.
      mods: Mods to load. Their `mod_deps` are included too, and every mod is
        loaded after the mods it depends on.
      visibility: Visibility of both targets.

    Also declares a reload target: `bazel run` on it sends every mod's current
    build to the running engine as one batch, and the engine reloads the ones
    that changed. It is named `reload` when `name` matches the package
    (`//game:reload`), and `<name>_reload` otherwise.
    """
    package = native.package_name().split("/")[-1]
    reload = "reload" if name == package else name + "_reload"
    _engine_game(
        name = name,
        bootstrap = bootstrap,
        mods = mods,
        reload_label = "//%s:%s" % (native.package_name(), reload),
        visibility = visibility,
    )
    _engine_game_reload(
        name = reload,
        game = ":" + name,
        visibility = visibility,
    )
