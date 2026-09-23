# dlclose unmaps a mod nothing holds

Earlier docs here said glibc often keeps a closed library mapped (it does,
when TLS destructors are registered in it), and it was tempting to treat
"code stays mapped after `dlclose`" as the normal case. For our mods it
isn't. Measured in an integration test, counting the staged library's lines
in `/proc/self/maps`:

| moment | mappings of `bag-<pid>-<n>.so` |
|---|---|
| after loading the `bag` test mod | 4 |
| after unloading it (the world holding its code) | 4 |
| after dropping the engine | 0 |
| after unloading it, with the world *not* holding its code | 0 |

In the last case, dropping the engine then dropped the world's `Vec<String>`
values through the unmapped library's drop function, and the test binary
died without printing another line.

## Why it matters

It is the reason the world holds a reference to each library whose code it
keeps (see [ecs.md](../architecture/ecs.md#heap-data-and-whose-code-runs)):
without it, a component value outliving its mod is a use-after-unmap. It
also means "it didn't crash" proves nothing unless the library would really
have been unmapped. It would, so the integration test
`heap_data_outlives_its_mod_and_is_freed_with_its_code` can fail.

A crash like this shows up in `bazel test` as the target `FAILED` with a log
that simply stops, not as a failing test: there is no `... FAILED` line to
grep for.
