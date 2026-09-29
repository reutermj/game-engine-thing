# A generated source moves a crate's sources, and a path into another package breaks

Found 2026-09-29 building the parallel-relations spike
(`//engine/std/physics/compare:colors_spike`), with rules_rust as rules_rs
ships it.

The comparison's benches build over physics's files with
`#[path = "../solver.rs"] mod solver;`, listing
`//engine/std/physics:solver.rs` among their `srcs`. That works while
every source is a file in the tree: rustc reads them where they are.
Adding one generated file (a genrule's output) to the same `srcs` makes
rules_rust copy the crate's sources into `bazel-out/<config>/bin/<package>/`
so that they sit together. The files from the crate's own package land
there; the one from `//engine/std/physics` doesn't land at
`../solver.rs` from it, and the build fails:

```
error: couldn't read `bazel-out/k8-opt/bin/engine/std/physics/compare/../tests/arrays.rs`
```

Putting the generated file in `compile_data` and reading it with
`include!(env!(...))` does the same, since compile data that is generated
moves the sources too.

What worked: a genrule that copies every file from the other package into
the crate's own package (`colors_spike_srcs`), and `#[path]`s to those
copies, so nothing the crate reads is outside its package. A file shared
this way is copied at build time, so it can't drift from the original.
