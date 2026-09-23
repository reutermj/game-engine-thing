# A dropped listener can keep accepting in a threaded test

The e2e test for stale sockets makes one by binding a `UnixListener` and
dropping it, which leaves the socket file with nothing listening. Then it
starts an engine, which is meant to see a refused `connect` and replace the
file.

About one run in twenty, the engine instead reported "another engine is
already listening". Measured over 100 runs with a wait added: in 11 of them,
`connect` to the just-dropped listener still succeeded when first tried, and
began failing a few polls later.

The test passed 60 of 60 times when run alone, and 60 of 60 with
`--test-threads=1`, so the cause is the other tests running on other threads.
The likely mechanism, inferred rather than traced: those tests spawn
processes, and a child forked while the listener existed holds a copy of its
fd until it calls `exec`, when `CLOEXEC` closes it. Until then the socket is
still listening.

## Resolution

The test waits until `connect` is refused before starting the engine, which
is the precondition it is about, and prints how long it waited. A test that
creates an OS resource and then relies on it being gone should check that it
is gone, when other threads in the same process can spawn children.
