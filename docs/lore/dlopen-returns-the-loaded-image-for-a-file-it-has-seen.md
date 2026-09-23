# dlopen returns the loaded image for a file it has seen

glibc's `dlopen` doesn't load a library twice. If the path matches one
already loaded, or the file behind it is the same inode, it returns the
existing handle and maps nothing new. That includes a path whose file has
since been **replaced**, which is exactly what rebuilding the mod does to its
Bazel output.

Measured with a mod library (`ctypes.CDLL(path, mode=RTLD_NOW)._handle`):

| opened after `a.so` was loaded | same handle? |
|---|---|
| `a.so` again | yes |
| a hard link to `a.so` | yes (same inode) |
| a symlink to `a.so` | yes (same inode) |
| `a.so` after `os.replace`-ing a *different* library onto it | **yes** |
| a byte-for-byte copy of `a.so` | no |

So a hot reload that re-opens the Bazel output path loads nothing, and does so
silently: the old code keeps running, and nothing in the log says why. A hard
link or symlink doesn't help either.

## Resolution

The loader copies each build to a unique file
(`$XDG_RUNTIME_DIR/game-engine-thing/libs/<name>-<pid>-<n>.so`) and opens the
copy. It deletes the copy immediately after `dlopen`, which is safe (see
[overwriting-a-mapped-library-crashes-deleting-it-does-not.md](overwriting-a-mapped-library-crashes-deleting-it-does-not.md)).

Unloading the old build first and re-opening the same path is not an
alternative. It would lose the "bad build keeps the old one running" property,
and `dlclose` doesn't guarantee an unmap anyway: glibc keeps a library mapped
while TLS destructors are registered in it (from glibc's `dlclose`, not
measured here).

Staging has one cost: a staged copy's `$ORIGIN` is the staging directory,
not `bazel-out`, so anything the library finds relative to itself stops
resolving. That is what broke the dynamically linked C++ runtime (see
[rust-shared-libraries-link-the-cxx-runtime-dynamically.md](rust-shared-libraries-link-the-cxx-runtime-dynamically.md)).
