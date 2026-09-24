# f32::floor is a libm call on the x86-64 baseline

`f32::floor` looks like an instruction, and isn't one here. The toolchain
targets plain `x86_64-unknown-linux-gnu`, whose baseline is SSE2, and the
rounding instruction (`roundss`) is SSE4.1. So LLVM lowers `v.floor()` to
a call. Read in the emitted assembly (rustc 1.98, `-O`):

```
floor_it:
	callq	*floorf@GOTPCREL(%rip)
```

An indirect call through the GOT into libm, per value. The spatial
re-sort made two for every row it re-bounded (the Morton cell of a box's
center); replacing them with truncation plus a correction for negative
fractions (`floor_i64` in `engine/ecs/spatial.rs`, checked equal to
`floor` over the edge cases in its unit test) took the re-bound of 10 000
rows from 166 to 139 µs (measured with `Instant` around the loop, `-c
opt`, the physics pile).

## Resolution

In hot loops, don't call `floor`, `ceil`, `round` or `trunc` on floats and
expect an instruction; `as` casts (`cvttss2si`) are baseline. Raising the
target CPU would also fix it, at the cost of every machine without
SSE4.1, and of a build setting every mod would have to share.
