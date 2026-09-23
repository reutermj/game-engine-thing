# libloading hides the dlerror text in source()

`libloading::Error`'s `Display` prints only the operation: a bad library
reports as

```
dlopen failed
```

The actual `dlerror()` message (the path, and *why*) is the error's
`std::error::Error::source()`. Printing `e.to_string()` therefore throws away
the only useful part, and makes every load failure look identical.

## Resolution

`engine/loader/engine.rs` formats errors through `describe`, which appends
the source:

```
dlopen failed: /run/user/1000/game-engine-thing/libs/counter-11061-2.so: file too short
```

Any new libloading call site should go through `describe` as well.
