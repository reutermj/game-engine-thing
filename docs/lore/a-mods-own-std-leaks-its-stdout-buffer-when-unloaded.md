# A mod's own std leaks its stdout buffer when unloaded

Every mod links its own copy of std, statics included, and a static is
never dropped: when the build is unmapped, whatever heap memory its
statics held is leaked. The one LeakSanitizer found (2026-09-25, the first
ASan run of `reload_test`, with builds really unloaded) is std's stdout:
the first `println!` a build makes allocates the 1 KiB `LineWriter` behind
its `STDOUT` `OnceLock`, and that build's copy is never freed.

```
Direct leak of 1024 byte(s) in 1 object(s) allocated from:
    ... <std::io::buffered::linewriter::LineWriter<StdoutRaw>>::new
    ... std::io::stdio::stdout::{closure#0}
    ... <engine_ecs::between::WorldMut>::install
    ... <engine_ecs::between::WorldMut>::id::<mover_v2_mod::Pos>
    ... <mover_v2_mod::Mover as engine_api::Mod>::load
```

The `println!` isn't the mod's: it is `engine_ecs`'s migration report in
`between.rs`, compiled into the mod because mods link the ECS, and run in
the mod's copy of std. So 1 KiB leaks per build that migrates a component
from a hook, and per build that prints anything itself. Output a build
prints without a newline is lost too: nothing flushes its `LineWriter`
before it is unmapped. The other mods print through the host
(`cx.log`), in the loader's std, which is why only `mover` showed.

Finding it took two runs, since LeakSanitizer can't name frames in a
library unmapped by exit (`<unknown module>`, see
[the sanitizers entry](the-stable-rustc-sanitizes-with-rustc-bootstrap.md)),
and keeping the builds mapped (`ENGINE_POISON_UNLOADED=keep`) keeps the
static that points at the buffer, so there is no leak to report. What named
it: keep the builds, and tell LeakSanitizer not to count globals as roots
(`--test_env=LSAN_OPTIONS=use_globals=0`), which reports every
allocation only a static holds; the one in a mod's std was this.

Not fixed: whether mods print through the host, or the ECS returns its
reports instead of printing them, is a design question.
