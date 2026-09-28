//! The 2D baseline's tool (physics-testing.md, "The baseline"; runbook
//! 005): runs every scene the baseline records (`record.rs`) and prints,
//! for every value that moved past its band from `baseline.txt`, the old
//! value, the new, the band and which way it went.
//!
//!     ./bazel run //engine/std/physics/compare:baseline
//!     ./bazel run //engine/std/physics/compare:baseline -- --write
//!     ./bazel run -c opt //engine/std/physics/compare:baseline -- --long --write
//!
//! `--write` writes the file into the source tree (`bazel run` sets
//! `BUILD_WORKSPACE_DIRECTORY`), after printing what it changes;
//! `--long` is the long suite's, `baseline_long.txt`, minutes at -c opt.
//! `--all` prints every value, moved or not; `--offset=<percent>` runs
//! every pile family at its sizes moved by that share, recorded under the
//! sizes' names, to measure the spread the bands are set outside of (it
//! never writes).
//! `SOLVER=<variant>` compares a variant of `variants.rs` against the
//! baseline, as the tests take it.

#[allow(dead_code)] // `Arrays::snapshot`, which only `:tax` uses.
#[path = "../tests/arrays.rs"]
mod arrays;
#[allow(dead_code)] // The tests' check, `assert_holds`.
mod baseline;
#[allow(dead_code)] // The label, which the comparison prints.
mod behave;
#[allow(dead_code)] // The timings and rain, which only the comparison reads.
mod ecs;
mod family;
#[path = "../narrow.rs"]
mod narrow;
#[allow(dead_code)]
mod quality;
mod record;
mod runs;
#[allow(dead_code)]
mod scene;
mod settle;
#[allow(dead_code)]
mod sim;
#[path = "../solver.rs"]
mod solver;
#[allow(dead_code)] // The constants and tests only the experiments use.
#[path = "../tests/split_impulse.rs"]
mod split_impulse;
mod variants;

pub use sim::{Dyn, Sim};

const GROUPS: [&str; 2] = ["quality", "behaviour"];

/// The file's first lines, as comments.
fn header(long: bool) -> String {
    let (suite, write) = if long { ("long suite's", record::WRITE_LONG) } else { ("default suite's", record::WRITE) };
    let lines = [
        format!("The 2D baseline, the {suite} scenes: our accepted results, which the"),
        "tests hold every change to within each band, both ways".to_string(),
        "(docs/architecture/physics-testing.md, \"The baseline\"). Group quality".to_string(),
        "is checked by the quality tests, behaviour by the behaviour tests.".to_string(),
        "Written by".to_string(),
        format!("    {write}"),
        "never by hand: a change that moves a value writes it again in the same".to_string(),
        "commit. Columns: group, scene, measure, value, band.".to_string(),
    ];
    lines.iter().map(|l| format!("# {l}\n")).collect()
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let has = |flag: &str| args.iter().any(|a| a == flag);
    let (long, write, all) = (has("--long"), has("--write"), has("--all"));
    let offset: i32 = args.iter().find_map(|a| a.strip_prefix("--offset=")).map_or(0, |n| n.parse().expect("--offset=<percent>"));
    for a in &args {
        assert!(
            ["--long", "--write", "--all"].contains(&a.as_str()) || a.starts_with("--offset="),
            "unknown flag {a}: --long, --write, --all, --offset=<percent>"
        );
    }
    record::set_offset(offset);
    let name = if long { "baseline_long.txt" } else { "baseline.txt" };
    let workspace = std::env::var("BUILD_WORKSPACE_DIRECTORY").ok();
    let path = workspace.as_ref().map(|w| std::path::Path::new(w).join("engine/std/physics/compare").join(name));
    // The source tree's file where there is one, which may be newer than
    // the one built in.
    let built_in = if long { record::LONG } else { record::DEFAULT };
    let file = path.as_ref().and_then(|p| std::fs::read_to_string(p).ok()).unwrap_or_else(|| built_in.to_string());

    let t = std::time::Instant::now();
    let entries: Vec<baseline::Entry> = std::thread::scope(|s| {
        let q = s.spawn(|| record::quality(long));
        let b = s.spawn(|| record::behaviour(long));
        [q.join().expect("quality"), b.join().expect("behaviour")].concat()
    });
    eprintln!("{} values in {:.0} s", entries.len(), t.elapsed().as_secs_f64());

    let rows = baseline::compare(&file, &GROUPS, &entries, all);
    let moved = rows.iter().filter(|r| r.verdict != "same").count();
    if moved == 0 {
        println!("Nothing moved past its band from {name}.");
    } else {
        println!("{moved} values moved past their band from {name}.");
    }
    if !rows.is_empty() {
        print!("\n{}", baseline::table(&rows));
    }
    if write {
        assert!(offset == 0, "--offset measures spread; it never writes");
        assert!(runs::solver().is_none(), "the baseline is ours as built: unset SOLVER to write it");
        let path = path.expect("--write writes into the source tree: run it with ./bazel run");
        std::fs::write(&path, baseline::write(&header(long), &entries)).unwrap_or_else(|e| panic!("writing {}: {e}", path.display()));
        println!("\nwrote {}", path.display());
    }
}
