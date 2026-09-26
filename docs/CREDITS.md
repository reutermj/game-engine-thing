# Credits

Third-party physics engines this repo builds against. None of them is part
of the engine or of any game: each is linked only into a comparison bench,
so our own solvers can be measured against established ones on identical
scenes. One section per library. Authors and licences are as stated in the
fetched source's own files, not from memory.

Where the notices live: every library's licence text is in its fetched
source (Bazel's external repository for it), and the bench that links it
declares the licence files as `data`, so they sit in the binary's runfiles
beside it (`//bench/physics3d:licenses`). That meets the MIT and Apache-2.0
conditions for the one form in which the code is copied, the bench binary
with its runfiles. Nothing here is distributed as a binary; a release that
shipped one would have to ship that filegroup with it.

## Rapier 3D

- **Project:** Rapier, the 3D crate `rapier3d`, version 0.36.0, from
  crates.io: <https://github.com/dimforge/rapier>, <https://rapier.rs>.
- **Authors:** Sébastien Crozet (the crate's `authors`, and the copyright
  line of the licence: "Copyright 2020 Sébastien Crozet"), Dimforge.
- **Licence:** Apache-2.0 (the crate's `license` field). The crates.io
  package ships no LICENSE file, so the text is fetched pinned by sha256 from
  the v0.36.0 tag (`@rapier_license`, MODULE.bazel). Upstream has no NOTICE
  file (checked at the tag), so the licence text is the whole notice.
- **Dependencies it brings:** parry3d, nalgebra, simba and approx
  (Apache-2.0, also Dimforge), and glam, glamx, wide, arrayvec and others
  under MIT, Apache-2.0, Zlib or a choice of them, per each crate's
  `license` field.
- **What we use it for:** comparison only. `//bench/physics3d` runs its
  scenes through Rapier's `PhysicsWorld`, single-threaded, rotations locked.

## Jolt Physics

- **Project:** Jolt Physics, release v5.6.0:
  <https://github.com/jrouwe/JoltPhysics>.
- **Authors:** Jorrit Rouwe ("Copyright 2021 Jorrit Rouwe", LICENSE) and
  the project's contributors.
- **Licence:** MIT (LICENSE at the root of the release archive, exported as
  `@jolt//:LICENSE`).
- **What we use it for:** comparison only. `//bench/physics3d` builds it from
  source (`bench/physics3d/jolt.BUILD`) behind a small C shim and runs the
  same scenes on `JobSystemSingleThreaded` with translation-only bodies.

## Box3D

- **Project:** Box3D, release v0.1.0: <https://github.com/erincatto/box3d>.
- **Authors:** Erin Catto ("Copyright (c) 2026 Erin Catto", LICENSE).
- **Licence:** MIT (LICENSE at the root of the release archive, exported as
  `@box3d//:LICENSE`).
- **What we use it for:** comparison only. `//bench/physics3d` builds it from
  source (`bench/physics3d/box3d.BUILD`) behind a small C shim and runs the
  same scenes with one worker and all three angular motion locks.
