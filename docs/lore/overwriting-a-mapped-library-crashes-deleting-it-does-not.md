# Overwriting a mapped library crashes; deleting it does not

A loaded library's code is a mapping of its file. What happens when the file
changes depends on *how* it changes. Measured by loading a mod library, then
calling into it again after each change:

| change to the file after `dlopen` | result |
|---|---|
| deleted (`os.remove`) | call succeeds |
| truncated to 0 bytes | `SIGBUS` (exit 135) |
| overwritten in place with zeros | `SIGSEGV` (exit 139) |

Deleting (or replacing via rename) only removes a *name*. The mapping keeps
the old inode alive, and while the engine runs its staged libraries show up in
`/proc/<pid>/maps` marked `(deleted)`.

Writing into the same inode is different. Pages the process hasn't touched yet
are read from the file on first access, so they see the new bytes: new code
under old addresses, or a `SIGBUS` past the new end of file. Linux doesn't
prevent this for shared libraries. `ETXTBSY` only protects a running
executable, and `MAP_DENYWRITE` has been ignored since 5.15.

## Why it matters here

It is the reason the loader deletes its staged copy right after `dlopen`, and
why that is safe. It is also why nothing but the loader should ever write to a
staged file. Bazel replaces its outputs rather than rewriting them (a rebuild
of `//mods/counter` gave `libcounter.so` a new inode), so the Bazel output
itself wouldn't hurt a running engine, but staging means the engine never has
to rely on that.
