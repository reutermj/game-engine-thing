//! End-to-end tests: the engine binary as a process, driven over its control
//! socket by `modctl`. The only tier that covers the manifest, runfiles,
//! the socket and the `engine_game`/`engine_mod` wiring.
//!
//! What stays untested is `bazel run` itself as the trigger: `modctl` here
//! gets the environment an `engine_mod` target sets (`ENGINE_MOD_NAME`,
//! `ENGINE_MOD_RLOCATION`) directly, rather than through Bazel.

use std::path::{Path, PathBuf};
use std::process::{Child, Command, Output, Stdio};
use std::time::{Duration, Instant};

use runfiles::Runfiles;

fn rlocation(var: &str) -> String {
    std::env::var(var).unwrap_or_else(|_| panic!("${var} is not set"))
}

fn runfile(var: &str) -> PathBuf {
    let rlocation = rlocation(var);
    Runfiles::create()
        .expect("runfiles")
        .rlocation(&rlocation)
        .unwrap_or_else(|| panic!("{rlocation} is not in runfiles"))
}

/// A fresh runtime directory (socket and staged libraries) for one test.
///
/// Not under `$TEST_TMPDIR`, which is long enough to push the socket past the
/// 108-byte limit on Unix socket paths, but named after a hash of it: that is
/// unique per test run, while `/tmp` can be shared between concurrent runs and
/// the pid repeats across them (each sandbox has its own pid namespace). See
/// docs/lore/bazel-test-paths-are-too-long-for-a-unix-socket.md.
fn runtime_dir(test: &str) -> PathBuf {
    use std::hash::{Hash, Hasher};
    let mut hasher = std::collections::hash_map::DefaultHasher::new();
    std::env::var("TEST_TMPDIR").expect("TEST_TMPDIR").hash(&mut hasher);
    let dir = PathBuf::from(format!("/tmp/engine-e2e-{:016x}-{test}", hasher.finish()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(&dir).unwrap();
    dir
}

/// Kills the engine if a test fails before asking it to quit.
struct Engine(Option<Child>);

impl Engine {
    fn start(runtime: &Path) -> Engine {
        let child = Command::new(runfile("ENGINE"))
            .env("ENGINE_MANIFEST", rlocation("MANIFEST"))
            .env("ENGINE_RUNTIME_DIR", runtime)
            .env_remove("ENGINE_SOCKET")
            .stdout(Stdio::piped())
            .stderr(Stdio::piped())
            .spawn()
            .expect("starting the engine");
        Engine(Some(child))
    }

    /// Waits for the engine to exit and returns what it printed. Polls rather
    /// than blocking, so an engine that never exits fails the test here
    /// instead of hanging it until Bazel's timeout.
    fn wait(mut self) -> Output {
        let deadline = Instant::now() + Duration::from_secs(10);
        let child = self.0.as_mut().unwrap();
        while child.try_wait().expect("polling the engine").is_none() {
            // Dropping `self` on this panic kills the engine.
            assert!(Instant::now() < deadline, "the engine didn't exit within 10s");
            std::thread::sleep(Duration::from_millis(20));
        }
        self.0.take().unwrap().wait_with_output().expect("collecting the engine's output")
    }
}

impl Drop for Engine {
    fn drop(&mut self) {
        if let Some(mut child) = self.0.take() {
            let _ = child.kill();
            let _ = child.wait();
        }
    }
}

fn modctl(runtime: &Path, args: &[&str], env: &[(&str, &str)]) -> Output {
    Command::new(runfile("MODCTL"))
        .args(args)
        .envs(env.iter().copied())
        .env("ENGINE_RUNTIME_DIR", runtime)
        .env_remove("ENGINE_SOCKET")
        .output()
        .expect("running modctl")
}

fn stdout(output: &Output) -> String {
    String::from_utf8_lossy(&output.stdout).into_owned()
}

fn describe(output: &Output) -> String {
    format!(
        "status {}\n--- stdout\n{}--- stderr\n{}",
        output.status,
        stdout(output),
        String::from_utf8_lossy(&output.stderr)
    )
}

/// `modctl list` once the engine is up and has loaded its manifest.
fn wait_until_listening(engine: &mut Engine, runtime: &Path) -> String {
    let deadline = Instant::now() + Duration::from_secs(10);
    loop {
        let out = modctl(runtime, &["list"], &[]);
        if out.status.success() {
            return stdout(&out);
        }
        let child = engine.0.as_mut().unwrap();
        if child.try_wait().expect("polling the engine").is_some() {
            let output = engine.0.take().unwrap().wait_with_output().unwrap();
            panic!("the engine exited before listening:\n{}", describe(&output));
        }
        assert!(Instant::now() < deadline, "the engine never started listening:\n{}", describe(&out));
        std::thread::sleep(Duration::from_millis(20));
    }
}

#[test]
fn a_game_starts_hot_reloads_a_mod_and_quits() {
    let runtime = runtime_dir("workflow");
    let mut engine = Engine::start(&runtime);

    let list = wait_until_listening(&mut engine, &runtime);
    assert!(list.contains("bootstrap gen 0 [bootstrap]"), "{list}");
    assert!(list.contains("counter gen 0"), "{list}");
    // Not listed by the game, but loaded as user's dependency, and first.
    let (base, user) = (list.find("  base gen 0").expect(&list), list.find("  user gen 0").expect(&list));
    assert!(base < user, "{list}");

    // Exactly what `./bazel run //mods/counter` runs: modctl with no
    // arguments, told which mod and library through its environment.
    let reload = modctl(
        &runtime,
        &[],
        &[("ENGINE_MOD_NAME", "counter"), ("ENGINE_MOD_RLOCATION", &rlocation("COUNTER_V2"))],
    );
    assert!(reload.status.success(), "{}", describe(&reload));
    assert_eq!(stdout(&reload).trim(), "reloaded counter (generation 1)");
    let list = stdout(&modctl(&runtime, &["list"], &[]));
    assert!(list.contains("counter gen 1"), "{list}");

    // A second engine must not steal the socket from the first.
    let second = Engine::start(&runtime).wait();
    assert!(!second.status.success(), "{}", describe(&second));
    assert!(String::from_utf8_lossy(&second.stderr).contains("already listening"), "{}", describe(&second));

    // A message and its reply, round trip over the socket.
    let get = modctl(&runtime, &["send", "counter", "get"], &[]);
    assert!(get.status.success(), "{}", describe(&get));
    let declined = modctl(&runtime, &["send", "counter", "jump"], &[]);
    assert!(!declined.status.success(), "{}", describe(&declined));
    assert!(stdout(&declined).contains("doesn't understand"), "{}", describe(&declined));

    let quit = modctl(&runtime, &["quit"], &[]);
    assert!(quit.status.success(), "{}", describe(&quit));
    let output = engine.wait();
    assert!(output.status.success(), "{}", describe(&output));
    assert!(stdout(&output).contains("[engine] shut down"), "{}", describe(&output));
    assert!(!runtime.join("control.sock").exists(), "the socket should be removed on exit");
}

#[test]
fn an_engine_replaces_a_socket_left_by_one_that_crashed() {
    let runtime = runtime_dir("stale_socket");
    // Binding and dropping a listener leaves the socket file behind with
    // nothing listening, as a killed engine does.
    let socket = runtime.join("control.sock");
    drop(std::os::unix::net::UnixListener::bind(&socket).unwrap());
    assert!(socket.exists());
    // Other tests spawn processes on other threads, and a child forked while
    // the listener existed holds a copy of it until it execs, so the socket
    // can briefly still accept. Wait for the state the test is about.
    let mut waited = 0;
    while std::os::unix::net::UnixStream::connect(&socket).is_ok() {
        waited += 1;
        assert!(waited < 250, "the dropped listener is still accepting after 5s");
        std::thread::sleep(Duration::from_millis(20));
    }
    if waited > 0 {
        println!("waited {waited} poll(s) for the dropped listener to close");
    }

    let mut engine = Engine::start(&runtime);
    wait_until_listening(&mut engine, &runtime);
    assert!(modctl(&runtime, &["quit"], &[]).status.success());
    let output = engine.wait();
    assert!(output.status.success(), "{}", describe(&output));
}

#[test]
fn modctl_without_an_engine_says_how_to_start_one() {
    let runtime = runtime_dir("no_engine");
    let out = modctl(&runtime, &["list"], &[]);
    assert!(!out.status.success(), "{}", describe(&out));
    let stderr = String::from_utf8_lossy(&out.stderr);
    assert!(stderr.contains("can't reach the engine") && stderr.contains("bazel run //game"), "{stderr}");
}

#[test]
fn a_failed_load_is_reported_to_modctl_and_keeps_the_engine_running() {
    let runtime = runtime_dir("failed_load");
    let mut engine = Engine::start(&runtime);
    wait_until_listening(&mut engine, &runtime);

    let bogus = runtime.join("bogus.so");
    std::fs::write(&bogus, "not an ELF file").unwrap();
    let out = modctl(&runtime, &["load", "counter", bogus.to_str().unwrap()], &[]);
    assert!(!out.status.success(), "{}", describe(&out));
    assert!(stdout(&out).contains("dlopen failed"), "{}", describe(&out));

    let list = stdout(&modctl(&runtime, &["list"], &[]));
    assert!(list.contains("counter gen 0"), "the old build must still be loaded:\n{list}");
    assert!(modctl(&runtime, &["quit"], &[]).status.success());
    assert!(engine.wait().status.success());
}

#[test]
fn a_game_reload_reloads_only_the_mods_that_changed() {
    let runtime = runtime_dir("game_reload");
    let mut engine = Engine::start(&runtime);
    wait_until_listening(&mut engine, &runtime);

    // What `./bazel run //engine/tests:test_game_reload` runs, after counter's
    // source changed from v1 to v2 (test_game_v2's manifest is that state).
    let reload = modctl(&runtime, &[], &[("ENGINE_BATCH_MANIFEST", &rlocation("MANIFEST_V2"))]);
    assert!(reload.status.success(), "{}", describe(&reload));
    assert_eq!(
        stdout(&reload).trim(),
        "reloaded counter (generation 1); clock unchanged; bootstrap unchanged; base unchanged; user unchanged"
    );

    // A per-mod reload that would strand a dependent is refused, naming the
    // game's reload target, which only the manifest could have told it.
    let strand = modctl(
        &runtime,
        &[],
        &[("ENGINE_MOD_NAME", "base"), ("ENGINE_MOD_RLOCATION", &rlocation("BASE_V2"))],
    );
    assert!(!strand.status.success(), "{}", describe(&strand));
    assert!(stdout(&strand).contains("`./bazel run //engine/tests:test_game_reload`"), "{}", describe(&strand));

    assert!(modctl(&runtime, &["quit"], &[]).status.success());
    assert!(engine.wait().status.success());
}
