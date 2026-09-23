# abi_stable never unloads a library

`abi_stable` looks like the obvious choice for a Rust plugin ABI, but its
loader is built around loading each library once, forever. Read in the
0.11.3 source (`src/library/root_mod_trait.rs`, `RootModule::load_from`):

```rust
let statics = Self::root_module_statics();
statics.root_mod.try_init(|| {
    let lib = statics.raw_lib.try_init(|| -> Result<_, LibraryError> {
        let raw_library = load_raw_library::<Self>(where_)?;
        // if the library isn't leaked
        // it would cause any use of the module to be a use after free.
        ...
        Ok(leak_value(raw_library))
    })?;
```

Two properties, both fatal to hot reload:

- **The library is leaked** on purpose, so nothing ever unloads it. The crate
  docs (`src/library.rs`) say so: "The library is leaked so that the root
  module loader can do anything incompatible with library unloading."
- **The root module is a once-initialized static per type.** A second
  `load_from` for the same root module type returns the first result, so
  loading a new build of a mod hands back the old one.

## Resolution

The mod ABI is a plain C ABI (`engine/api`), loaded through libloading.
`abi_stable`'s FFI-safe types (`RString`, `RVec`, ...) don't depend on its
loader and remain an option for richer data across the boundary. That is an
open question in [hot-reload.md](../architecture/hot-reload.md).
