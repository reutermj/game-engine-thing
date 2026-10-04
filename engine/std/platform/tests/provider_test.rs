//! `engine_game(platform = ...)`: the provider a game names is in its
//! manifest, before the bootstrap that pumps it, and answers the calls;
//! a game that names none leaves the service unprovided, which a caller
//! is told as a value.

use std::path::PathBuf;

use engine_loader::engine::Engine;

fn engine(manifest: &str, test: &str) -> (Box<Engine>, Vec<String>) {
    let manifest = engine_control::read_manifest(&std::env::var(manifest).unwrap()).unwrap();
    let names = manifest.mods.iter().map(|(name, _)| name.clone()).collect();
    let dir = PathBuf::from(std::env::var("TEST_TMPDIR").unwrap()).join(test);
    let e = Engine::new(manifest.bootstrap, dir);
    e.load_batch(&manifest.mods).expect("loading the game");
    (e, names)
}

#[test]
fn the_named_provider_is_loaded_before_the_bootstrap_and_answers() {
    let (e, names) = engine("WITH_PLATFORM", "with");
    let at = |name: &str| names.iter().position(|n| n == name).unwrap_or_else(|| panic!("{name} isn't in {names:?}"));
    assert!(at("platform") < at("fake_platform"), "{names:?}");
    assert!(at("fake_platform") < at("lockstep"), "{names:?}");
    assert_eq!(e.send("pumper", "pump"), Ok("pump 1 inputs [1:KeyW=1] window 0".into()));
    assert_eq!(e.send("pumper", "pump"), Ok("pump 2 inputs [1:KeyW=1] window 0".into()));
    e.shutdown();
}

#[test]
fn with_no_provider_the_platform_is_not_provided() {
    let (e, names) = engine("WITHOUT_PLATFORM", "without");
    assert!(!names.iter().any(|n| n == "fake_platform"), "{names:?}");
    assert_eq!(e.send("pumper", "pump"), Ok("NotProvided".into()));
    e.shutdown();
}
