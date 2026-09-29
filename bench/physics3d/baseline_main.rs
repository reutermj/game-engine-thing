//! The 3D baseline's tool (physics-testing.md, "The baseline"; runbook
//! 005), as 2D's: runs every scene the baseline records (`record.rs`) and
//! prints every value that moved past its band from `baseline.txt`.
//!
//!     ./bazel run //bench/physics3d:baseline
//!     ./bazel run //bench/physics3d:baseline -- --write
//!     ./bazel run //bench/physics3d:baseline -- --long --write
//!
//! `--write` writes the file into the source tree; `--long` is the long
//! suite's, `baseline_long.txt`; `--all` prints every value, moved or
//! not; `--offset=<n>` runs each pile at its size and n (another seeded
//! drop), under the size's name, to measure spread (it never writes).
//! `TUNE=<variant>` compares a tuning of ours against the baseline.

use physics3d_bench::{baseline, record, runs};

const GROUPS: [&str; 2] = ["quality", "behaviour"];

/// The file's first lines, as comments.
fn header(long: bool) -> String {
    let (suite, write) = if long { ("long suite's", record::WRITE_LONG) } else { ("default suite's", record::WRITE) };
    let lines = [
        format!("The 3D baseline, the {suite} scenes: our accepted results, which the"),
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
    let offset: usize = args.iter().find_map(|a| a.strip_prefix("--offset=")).map_or(0, |n| n.parse().expect("--offset=<n>"));
    for a in &args {
        assert!(
            ["--long", "--write", "--all"].contains(&a.as_str()) || a.starts_with("--offset="),
            "unknown flag {a}: --long, --write, --all, --offset=<n>"
        );
    }
    record::set_offset(offset);
    let name = if long { "baseline_long.txt" } else { "baseline.txt" };
    let workspace = std::env::var("BUILD_WORKSPACE_DIRECTORY").ok();
    let path = workspace.as_ref().map(|w| std::path::Path::new(w).join("bench/physics3d").join(name));
    let built_in = if long { record::LONG } else { record::DEFAULT };
    let file = path.as_ref().and_then(|p| std::fs::read_to_string(p).ok()).unwrap_or_else(|| built_in.to_string());

    let t = std::time::Instant::now();
    let mut entries = record::quality(long);
    entries.extend(record::behaviour(long));
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
        assert!(runs::tune().is_empty(), "the baseline is ours as built: unset TUNE to write it");
        let path = path.expect("--write writes into the source tree: run it with ./bazel run");
        std::fs::write(&path, baseline::write(&header(long), &entries)).unwrap_or_else(|e| panic!("writing {}: {e}", path.display()));
        println!("\nwrote {}", path.display());
    }
}
