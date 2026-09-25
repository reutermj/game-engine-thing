"""`fuzz_launcher`: runs a fuzz binary with mod libraries built in the
default configuration.

A `fuzz_binary` is built through a transition that instruments every Rust
crate it contains, and its runfiles would be built the same way. The mods
it loads must not be: libFuzzer keeps each instrumented module's counters
for the whole run, and a mod's are unmapped when it's unloaded. So the
libraries are this rule's runfiles instead, and their paths reach the
binary in the environment, as `engine_mod` hands `modctl` its library.
"""

def _rlocationpath(ctx, file):
    # Matches `$(rlocationpath)`: external files have short paths like `../repo/pkg/f`.
    if file.short_path.startswith("../"):
        return file.short_path[3:]
    return ctx.workspace_name + "/" + file.short_path

def _fuzz_launcher_impl(ctx):
    exe = ctx.actions.declare_file(ctx.label.name)
    ctx.actions.symlink(output = exe, target_file = ctx.executable.binary, is_executable = True)
    env = {}
    files = []
    for var, target in ctx.attr.libs.items():
        libs = target[DefaultInfo].files.to_list()
        if len(libs) != 1:
            fail("expected one file from %s, got %s" % (target.label, libs))
        env[var] = _rlocationpath(ctx, libs[0])
        files.append(libs[0])
    return [
        DefaultInfo(
            executable = exe,
            runfiles = ctx.runfiles(files = files).merge(ctx.attr.binary[DefaultInfo].default_runfiles),
        ),
        RunEnvironmentInfo(environment = env),
    ]

fuzz_launcher = rule(
    implementation = _fuzz_launcher_impl,
    executable = True,
    attrs = {
        "binary": attr.label(mandatory = True, executable = True, cfg = "target"),
        "libs": attr.string_keyed_label_dict(allow_files = True, mandatory = True),
    },
)
