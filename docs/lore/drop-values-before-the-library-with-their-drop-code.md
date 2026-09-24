# Drop values before the library with their drop code

Holding a reference to a mod's library keeps its code mapped
([dlclose-unmaps-a-mod-nothing-holds](dlclose-unmaps-a-mod-nothing-holds.md)),
but only for as long as the reference lives. Rust drops a struct's fields
in declaration order. So a struct that holds both values and the keepalive
for the code that drops them must declare the values first. Declared the
other way, dropping the struct unmaps the library and then calls into it.

That happened when the ECS landed (2026-09-23). `World` declared its
component registry, which holds each build's keepalive, above its tables
and event queues. `ComponentInfo` had the same mistake, declaring
`installed` (the keepalive) above `sparse` (the values), and so did
`EventQueue`. Every test that unloaded a mod and kept its values passed
until the engine was dropped. Then `heap_data_outlives_its_mod_and_is_freed_with_its_code`
segfaulted with no output after `running 1 test`. It was the same silent
death the dlclose entry describes.

The drop order, recorded by a stand-in library whose `Drop` logs, with the
registry declared first:

```
["table lib", "sparse value", "sparse lib", "table value", "event", "event lib"]
```

## Why it matters

Nothing in the type system connects a value to the library its drop
function lives in: the keepalive is an `Arc<dyn Any>`, and the column holds
a plain function pointer. A comment at each struct explains the order.
`world_test`'s `values_drop_before_the_code_that_drops_them_is_unmapped`
pins it for tables, sparse sets and events: moving any of the three
keepalives above its values fails it.

A struct that later gains values from a mod's build (a resource store, a
cache of per-build data) needs the same ordering.
