# An unloaded build's addresses are mapped again within microseconds

After `dlclose` unmaps a build, its addresses are free, and the next
mapping anywhere in the process may land there: another build's `dlopen`,
a thread's stack, a large allocation. A pointer into the old build then
doesn't fault; it reads, or calls, whatever is there now. Measured
2026-09-25 with the loader's poison mode (engine/loader/poison.rs), which
maps an unloaded build's span `PROT_NONE` right after `dlclose`, with
`MAP_FIXED_NOREPLACE`: in 20 runs of `reload_test` (about 2 800 builds
unloaded, tests on parallel threads), 10 spans had already been taken in
the microseconds between the two calls, although every `dlopen` in the
loader was serialized against them. The rest of the process took them.

## What it means

"It didn't crash" says little about a stale pointer into a build, even
when the build really was unmapped
([dlclose unmaps a mod nothing holds](dlclose-unmaps-a-mod-nothing-holds.md)):
the crash needs the span to be unmapped *still*. The poison mode is what
makes it deterministic, for the spans it wins; the ones it loses it
reports (`couldn't guard ... File exists`), not prevents.
