//! The baseline's tool, the part 2D's and 3D's share
//! (physics-testing.md, "The baseline"; runbook 005): runs every scene a
//! harness's baseline records and prints, for every value that moved past
//! its band from the file, the old value, the new, the band and which way
//! it went. Each harness's `baseline_main.rs` says what it records and
//! where; its flags:
//!
//! - `--write` writes the file into the source tree (`bazel run` sets
//!   `BUILD_WORKSPACE_DIRECTORY`), after printing what it changes;
//! - `--long` is the long suite's, `baseline_long.txt`;
//! - `--all` prints every value, moved or not;
//! - `--offset=<...>` runs every pile family at other sizes or seeds,
//!   recorded under the sizes' names, to measure the spread the bands are
//!   set outside of (it never writes).

use crate::baseline::{self, Entry};

/// A harness's baseline, as its tool writes it.
pub struct Tool<'a> {
    /// "2D" or "3D", for the file's header.
    pub dim: &'a str,
    /// The files' package, from the workspace root.
    pub dir: &'a str,
    /// The files as built in, the default suite's and the long one's.
    pub default: &'a str,
    pub long: &'a str,
    /// The commands that write them, which the header and a failing test
    /// name.
    pub write: &'a str,
    pub write_long: &'a str,
    /// What `--offset=` takes, for the usage line (`<percent>`, `<n>`).
    pub offset: &'a str,
    /// Sets the offset from its text, and says what it is: 0 for none.
    pub set_offset: &'a dyn Fn(&str) -> i64,
    /// Every entry of both groups, quality first, with `long` the long
    /// suite's.
    pub record: &'a dyn Fn(bool) -> Vec<Entry>,
    /// The environment variable that runs a variant of ours in place of
    /// ours as built (`SOLVER`, `TUNE`), and whether it is set: the
    /// baseline is ours as built, so it is never written from a variant.
    pub variant: (&'a str, bool),
}

const GROUPS: [&str; 2] = ["quality", "behaviour"];

/// The file's first lines, as comments.
fn header(t: &Tool, long: bool) -> String {
    let (suite, write) = if long { ("long suite's", t.write_long) } else { ("default suite's", t.write) };
    let lines = [
        format!("The {} baseline, the {suite} scenes: our accepted results, which the", t.dim),
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

/// The tool, on this process's arguments.
pub fn main(t: &Tool) {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let has = |flag: &str| args.iter().any(|a| a == flag);
    let (long, write, all) = (has("--long"), has("--write"), has("--all"));
    let offset = (t.set_offset)(args.iter().find_map(|a| a.strip_prefix("--offset=")).unwrap_or("0"));
    for a in &args {
        assert!(
            ["--long", "--write", "--all"].contains(&a.as_str()) || a.starts_with("--offset="),
            "unknown flag {a}: --long, --write, --all, --offset={}",
            t.offset
        );
    }
    let name = if long { "baseline_long.txt" } else { "baseline.txt" };
    let workspace = std::env::var("BUILD_WORKSPACE_DIRECTORY").ok();
    let path = workspace.as_ref().map(|w| std::path::Path::new(w).join(t.dir).join(name));
    // The source tree's file where there is one, which may be newer than
    // the one built in.
    let built_in = if long { t.long } else { t.default };
    let file = path.as_ref().and_then(|p| std::fs::read_to_string(p).ok()).unwrap_or_else(|| built_in.to_string());

    let started = std::time::Instant::now();
    let entries = (t.record)(long);
    eprintln!("{} values in {:.0} s", entries.len(), started.elapsed().as_secs_f64());

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
        assert!(!t.variant.1, "the baseline is ours as built: unset {} to write it", t.variant.0);
        let path = path.expect("--write writes into the source tree: run it with ./bazel run");
        std::fs::write(&path, baseline::write(&header(t, long), &entries)).unwrap_or_else(|e| panic!("writing {}: {e}", path.display()));
        println!("\nwrote {}", path.display());
    }
}
