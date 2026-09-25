# A state's padding can look like a pointer into its build

The loader's net for mod state that points into its build (hot-reload.md,
"Who owns state") read every pointer-sized word of the state, and a
struct's words include its padding, which holds whatever bytes were last
there. Physics's state has a `u32` (`Sleepers::islands`) beside four bytes
of padding, and the padding had been left holding the upper half of some
pointer. With `islands` small, the word read as `0x7fXX_0000000N`: an
address in physics's own build whenever that build's mapping straddled a
4 GiB boundary. The state was then reset mid-game, and the replays saw it.

Measured 2026-09-25, printing the words that hit: 10 of 24 runs of
`//pong:reload_test` and `//platformer:reload_test` (in poison mode, 24
runs in parallel) reset physics at least once, always at word 7 of its
168-byte state, with values like `0x7f0b00000000` against a build mapped
at `0x7f0b00000000..0x7f0b0032f000`, or `0x7f2900000002` against
`0x7f28ffe00000..0x7f290012f000`. Run alone, the same tests passed.

## What it means

- A scan of raw struct memory reads padding, which is garbage (and reading
  it as an integer is undefined behavior besides). The net now scans only
  states without a schema, the hand-written `unsafe impl ModState` it
  exists for; a `mod_state!` state is made of `FieldType`s, which can't
  point into the build. `an_address_kept_as_a_number_is_data`
  (`//engine/tests:reload_test`) pins it.
- A heuristic that "can in principle be fooled by an integer that looks
  like an address" is fooled far more often than that sounds, because
  padding and the upper halves of pointers are everywhere.
