"""`sanitized_tests`: a test target, and everything it loads, under a sanitizer.

    sanitized_tests(tests = [":reload_test"])

declares `reload_test_asan`: the same test, built with every Rust crate
below it (the loader, the ECS, the mods in its `data`, the engine binary an
e2e test runs) compiled with `-Zsanitizer=address`, which brings
LeakSanitizer with it. A transition sets the flags, so those crates are
built once more in the test's own configuration and the default build never
sees them. Manual: see docs/runbooks/004-run-the-loader-under-sanitizers.md.

The compiler is the stable toolchain everything else builds with, unlocked
with `RUSTC_BOOTSTRAP=1`: `-Zsanitizer` is nightly-only, but its runtimes
ship in the stable `rust-std` (`librustc-stable_rt.{asan,lsan,tsan}.a`), and
the one-compiler rule the loader checks (every build from the engine's
rustc) holds only if the mods and the engine come from the same compiler.
"""

_FLAGS = str(Label("@rules_rust//rust/settings:extra_rustc_flags"))
_ENV = str(Label("@rules_rust//rust/settings:extra_rustc_env"))

# Every test sanitized_tests may wrap, for the suites in //engine/tests,
# which list them from here so a sanitized test can't be left out of them.
SANITIZED_TESTS = [
    "//engine/std/physics:physics_test",
    "//engine/tests:e2e_test",
    "//engine/tests:reload_test",
    "//platformer:platformer_test",
    "//pong:pong_test",
]

# One so far. ThreadSanitizer needs a std built from source (it can't see
# the synchronization in the precompiled one), so a second toolchain; that
# recipe was deferred (bead get-lm3).
SANITIZERS = {
    "asan": struct(
        # The precompiled std: AddressSanitizer intercepts its allocations
        # all the same, and misses only overflows inside std itself.
        flags = ["-Zsanitizer=address"],
        env = {
            # halt_on_error: the first report is the finding; later ones are
            # often its consequences. LeakSanitizer runs at exit.
            "ASAN_OPTIONS": "detect_leaks=1:halt_on_error=1:detect_stack_use_after_return=1:strict_init_order=1",
        },
    ),
}

def _sanitize_transition_impl(settings, attr):
    return {
        _FLAGS: settings[_FLAGS] + SANITIZERS[attr.sanitizer].flags + [
            # Stacks a report can walk without unwind tables, as the
            # runtimes' fast unwinder does.
            "-Cforce-frame-pointers=yes",
            # rustc links a sanitizer's runtime into executables only, so a
            # mod's calls into it must resolve against the executable at
            # dlopen: without this, every mod fails to load with "undefined
            # symbol: __asan_...". No effect on the mods themselves, whose
            # exports rustc's version script decides.
            "-Clink-arg=-Wl,--export-dynamic",
            # File and line in a report, not just the function.
            "-Cdebuginfo=line-tables-only",
        ],
        _ENV: settings[_ENV] + ["RUSTC_BOOTSTRAP=1"],
    }

_sanitize_transition = transition(
    implementation = _sanitize_transition_impl,
    inputs = [_FLAGS, _ENV],
    outputs = [_FLAGS, _ENV],
)

def _sanitized_test_impl(ctx):
    test = ctx.attr.test[0]
    exe = ctx.actions.declare_file(ctx.label.name)
    ctx.actions.symlink(
        output = exe,
        target_file = test[DefaultInfo].files_to_run.executable,
        is_executable = True,
    )
    env = dict(test[RunEnvironmentInfo].environment) if RunEnvironmentInfo in test else {}
    env.update(SANITIZERS[ctx.attr.sanitizer].env)

    # Builds stay mapped, and their staged files on disk, so a report at
    # exit (every leak report) can name frames in builds unloaded by then.
    # The poison mode the test has otherwise is for stale code pointers,
    # which the sanitizers don't track. See engine/loader/poison.rs.
    env["ENGINE_POISON_UNLOADED"] = "keep"

    # Relative to the test's working directory, the runfiles' main repo,
    # which is where an external file's `../repo/...` short path starts.
    symbolizer = ctx.file._symbolizer
    env["ASAN_SYMBOLIZER_PATH"] = symbolizer.short_path
    runfiles = ctx.runfiles(files = [exe, symbolizer]).merge(test[DefaultInfo].default_runfiles)
    return [
        DefaultInfo(executable = exe, runfiles = runfiles),
        RunEnvironmentInfo(environment = env),
    ]

_sanitized_test = rule(
    implementation = _sanitized_test_impl,
    test = True,
    attrs = {
        "sanitizer": attr.string(mandatory = True, values = SANITIZERS.keys()),
        "test": attr.label(mandatory = True, cfg = _sanitize_transition, executable = True),
        "_allowlist_function_transition": attr.label(
            default = "@bazel_tools//tools/allowlists/function_transition_allowlist",
        ),
        "_symbolizer": attr.label(
            default = "@llvm//tools:llvm-symbolizer",
            allow_single_file = True,
        ),
    },
)

def sanitized_tests(tests, sanitizers = SANITIZERS.keys(), skip = []):
    """`<test>_<sanitizer>` for each test and sanitizer, tagged manual.

    Args:
      tests: the test targets, in this package.
      sanitizers: which of `SANITIZERS`.
      skip: libtest filters to skip: tests that count the builds still
        mapped, which the sanitized run keeps mapped on purpose.
    """
    for test in tests:
        name = test.lstrip(":")
        label = "//{}:{}".format(native.package_name(), name)
        if label not in SANITIZED_TESTS:
            fail("add {} to SANITIZED_TESTS in //engine:sanitize.bzl".format(label))
        for sanitizer in sanitizers:
            _sanitized_test(
                name = "{}_{}".format(name, sanitizer),
                test = test,
                sanitizer = sanitizer,
                args = [a for f in skip for a in ["--skip", f]],
                tags = ["manual", sanitizer],
                # For the suites in //engine/tests.
                visibility = ["//engine/tests:__pkg__"],
                timeout = "long",
            )
