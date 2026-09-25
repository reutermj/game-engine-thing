# Copying a value as bytes drops its pointers

`erased.rs` moves values whose type it only knows as a size, and for small
sizes it copied them as fixed-size arrays, since a copy of a runtime size
is a call to `memcpy` (`copy_value`). The arrays were `[u8; N]`:

```rust
(dst as *mut [u8; N]).write_unaligned((src as *const [u8; N]).read_unaligned())
```

That is undefined behavior for most components, in two ways, and nothing
native shows either:

- **A pointer copied as integers loses its provenance.** A `String` moved
  this way (24 bytes) holds a pointer with no provenance, and the first
  read through it is UB. Miri, on `values_are_pushed_read_replaced_and_removed`:

  ```
  error: Undefined Behavior: constructing invalid value of type &[u8]:
  encountered a dangling reference (0x2707e2[noalloc] has no provenance)
  ```

- **Padding read as `u8` is a read of uninitialized memory.** A component
  with padding (`{ a: u8, b: u32 }`, 8 bytes) fails the copy itself:

  ```
  error: Undefined Behavior: constructing invalid value of type
  std::ptr::Unaligned<[u8; 8]>: at .0[5], encountered uninitialized memory,
  but expected an integer
  ```

Both pass natively and in every test, because the machine code is right:
UB here licenses the optimizer, not the CPU.

## Resolution

Copy as `MaybeUninit<[u8; N]>`, which may hold any bytes, uninitialized
and pointer bytes included, and carries provenance through the copy. Only
`copy_nonoverlapping` and `MaybeUninit` copies are untyped; any integer or
integer-array type is not.

It costs nothing: with the build's rustc 1.98.1 at `-C opt-level=3`, the
two forms compile to the same instructions, and LLVM folds them into one
function (`as_uninit_24 = as_bytes_24` in the emitted assembly; measured
2026-09-25 on 12 and 24 bytes).

Found by `//engine/ecs:miri_unit_sb` the first time it ran;
`values_with_padding_and_pointers_move_whole` pins both cases (it fails
under Miri with `[u8; N]`, checked by reverting).
