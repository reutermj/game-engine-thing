//! The baseline: our own accepted results, a line a scene and a measure,
//! checked in beside the scenes and compared against in both directions,
//! each within a band set from measured noise
//! (docs/architecture/physics-testing.md, "The baseline"). A value past its
//! band fails, better or worse, until the file is written again in the
//! same commit, so the file's diff is the record of what a change did to
//! physics. This is the format and the comparison; each harness's
//! `record.rs` (`//engine/std/physics2d/compare`, `//engine/std/physics3d/compare`)
//! says what it records and with which bands, and `tool` writes the files.
//!
//! A line is a group (the test binary that checks it), a scene, a measure,
//! the value and its band, separated by two spaces or more, since a
//! scene's name has single spaces in it:
//!
//! ```text
//! quality  piles 400-1200 41 turning  rest_median  210  steps 30
//! ```

/// How far a value may move and still be the same result.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Band {
    /// Steps: `n` either way, or the fraction of the old value where that
    /// is more. Rest is looked at every 10 steps, so it moves in jumps.
    Steps(f64, f64),
    /// A fraction of the old value either way, and never less than an
    /// absolute amount: a depth of 0.0006 that moves by 0.00001 hasn't
    /// moved.
    Rel(f64, f64),
    /// A factor either way, values under the floor all equal: energies
    /// span ten decades, and under `STILL` they are rounding.
    Log(f64, f64),
    /// Counts and flags: a heavy box through the floor is never noise.
    Exact,
    /// Within an amount either way: an edge family's count of runs that
    /// did what they should, one of which a rounding flips.
    Abs(f64),
}

/// Which way is better, for the table. The band holds both ways alike;
/// this only says which way a value that moved went.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Better {
    Lower,
    Higher,
    /// Closer to a hand calculation.
    Toward(f64),
    /// Neither: what a scene is (contacts a body), not how well it did.
    Neither,
}

/// One recorded value.
#[derive(Clone, Debug, PartialEq)]
pub struct Entry {
    pub group: &'static str,
    pub scene: String,
    pub measure: String,
    pub value: f64,
    pub band: Band,
    pub better: Better,
}

impl Entry {
    pub fn new(group: &'static str, scene: impl Into<String>, measure: impl Into<String>, value: f64, band: Band, better: Better) -> Entry {
        Entry { group, scene: scene.into(), measure: measure.into(), value, band, better }
    }

    fn key(&self) -> (&str, &str, &str) {
        (self.group, &self.scene, &self.measure)
    }
}

/// A value as the file has it: whole numbers whole, others to four
/// significant digits (far inside any band), "never" for a scene that
/// never came to rest.
pub fn show(v: f64) -> String {
    if v.is_infinite() {
        return "never".to_string();
    }
    let a = v.abs();
    if v == v.trunc() && a < 1e9 {
        return format!("{v:.0}");
    }
    if (1e-3..1e4).contains(&a) {
        let decimals = (3 - a.log10().floor() as i32).max(0) as usize;
        let s = format!("{v:.decimals$}");
        return trim(&s).to_string();
    }
    let s = format!("{v:.3e}");
    let (mantissa, exponent) = s.split_once('e').expect("an exponent");
    format!("{}e{exponent}", trim(mantissa))
}

/// A decimal without its trailing zeros, which say nothing a band could use.
fn trim(s: &str) -> &str {
    if s.contains('.') { s.trim_end_matches('0').trim_end_matches('.') } else { s }
}

fn parse_value(s: &str) -> Option<f64> {
    if s == "never" { Some(f64::INFINITY) } else { s.parse().ok() }
}

impl Band {
    pub fn text(&self) -> String {
        match *self {
            Band::Steps(n, 0.0) => format!("steps {}", show(n)),
            Band::Steps(n, f) => format!("steps {} or {}%", show(n), show(100.0 * f)),
            Band::Rel(r, 0.0) => format!("rel {}", show(r)),
            Band::Rel(r, a) => format!("rel {} or {}", show(r), show(a)),
            Band::Log(f, floor) => format!("log {} under {}", show(f), show(floor)),
            Band::Exact => "exact".to_string(),
            Band::Abs(a) => format!("within {}", show(a)),
        }
    }

    /// Whether `new` is the same result as `old`.
    pub fn holds(&self, old: f64, new: f64) -> bool {
        if old.is_infinite() || new.is_infinite() {
            return old == new;
        }
        let d = (new - old).abs();
        match *self {
            Band::Steps(n, f) => d <= n.max(f * old.abs()),
            Band::Rel(r, a) => d <= (r * old.abs()).max(a),
            Band::Log(f, floor) => {
                let (o, w) = (old.max(floor), new.max(floor));
                (o / w).max(w / o) <= f
            }
            Band::Exact => show(old) == show(new),
            Band::Abs(a) => d <= a,
        }
    }
}

impl Better {
    fn verdict(&self, old: f64, new: f64) -> &'static str {
        let (o, n) = match *self {
            Better::Lower => (old, new),
            Better::Higher => (-old, -new),
            Better::Toward(t) => ((old - t).abs(), (new - t).abs()),
            Better::Neither => return "moved",
        };
        if n < o {
            "better"
        } else if n > o {
            "worse"
        } else {
            "moved"
        }
    }
}

/// A line of the file.
struct Line<'a> {
    group: &'a str,
    scene: &'a str,
    measure: &'a str,
    value: f64,
    band: &'a str,
}

/// The fields of a line: separated by two spaces or more.
fn fields(line: &str) -> Vec<&str> {
    line.split("  ").map(str::trim).filter(|f| !f.is_empty()).collect()
}

fn lines(file: &str) -> Vec<Line<'_>> {
    let mut v = Vec::new();
    for (i, line) in file.lines().enumerate() {
        if line.trim().is_empty() || line.starts_with('#') {
            continue;
        }
        let f = fields(line);
        let value = f.get(3).and_then(|s| parse_value(s));
        match (f.len(), value) {
            (5, Some(value)) => v.push(Line { group: f[0], scene: f[1], measure: f[2], value, band: f[4] }),
            _ => panic!("the baseline's line {} is not group, scene, measure, value and band: {line:?}", i + 1),
        }
    }
    v
}

/// A value past its band, or one only the run or only the file has.
#[derive(Debug)]
pub struct Row {
    pub group: String,
    pub scene: String,
    pub measure: String,
    pub old: Option<f64>,
    pub new: Option<f64>,
    pub band: String,
    /// "better", "worse" or "moved" past the band; "new", "gone"; "band"
    /// where the band itself changed; "same" within it (`compare`'s `all`).
    pub verdict: String,
}

/// Every entry of `groups` that isn't what the file has, and every line of
/// the file in `groups` that no entry has. `all` keeps the entries that
/// held too, for measuring spread.
pub fn compare(file: &str, groups: &[&str], entries: &[Entry], all: bool) -> Vec<Row> {
    let old: Vec<Line> = lines(file).into_iter().filter(|l| groups.contains(&l.group)).collect();
    let mut seen = std::collections::HashSet::new();
    for e in entries {
        assert!(seen.insert(e.key()), "recorded twice: {} {} {}", e.group, e.scene, e.measure);
    }
    let mut rows = Vec::new();
    for e in entries.iter().filter(|e| groups.contains(&e.group)) {
        let band = e.band.text();
        let row = |old: Option<f64>, verdict: &str| Row {
            group: e.group.to_string(),
            scene: e.scene.clone(),
            measure: e.measure.clone(),
            old,
            new: Some(e.value),
            band: band.clone(),
            verdict: verdict.to_string(),
        };
        match old.iter().find(|l| (l.group, l.scene, l.measure) == e.key()) {
            None => rows.push(row(None, "new")),
            Some(l) if l.band != band => rows.push(row(Some(l.value), "band")),
            Some(l) if !e.band.holds(l.value, e.value) => rows.push(row(Some(l.value), e.better.verdict(l.value, e.value))),
            Some(l) if all => rows.push(row(Some(l.value), "same")),
            Some(_) => {}
        }
    }
    for l in &old {
        if !seen.contains(&(l.group, l.scene, l.measure)) {
            rows.push(Row {
                group: l.group.to_string(),
                scene: l.scene.to_string(),
                measure: l.measure.to_string(),
                old: Some(l.value),
                new: None,
                band: l.band.to_string(),
                verdict: "gone".to_string(),
            });
        }
    }
    rows
}

/// The rows as a table: old, new, band, and which way each moved.
pub fn table(rows: &[Row]) -> String {
    let cell = |v: Option<f64>| v.map_or("–".to_string(), show);
    let mut s = String::from("| group | scene | measure | old | new | band | |\n|---|---|---|---|---|---|---|\n");
    for r in rows {
        s += &format!("| {} | {} | {} | {} | {} | {} | {} |\n", r.group, r.scene, r.measure, cell(r.old), cell(r.new), r.band, r.verdict);
    }
    s
}

/// Panics listing every value of `groups` that isn't what `file` has, with
/// `regenerate`, the command that writes the file again.
pub fn assert_holds(file: &str, groups: &[&str], entries: &[Entry], regenerate: &str) {
    let rows = compare(file, groups, entries, false);
    assert!(
        rows.is_empty(),
        "{} values moved past their band from the baseline (physics-testing.md, \"The baseline\"); if that is accepted, \
         write it again in this commit with\n    {regenerate}\n\n{}",
        rows.len(),
        table(&rows)
    );
}

/// The file for `entries`, in their order, `header` (comment lines) first.
pub fn write(header: &str, entries: &[Entry]) -> String {
    let rows: Vec<[String; 5]> =
        entries.iter().map(|e| [e.group.to_string(), e.scene.clone(), e.measure.clone(), show(e.value), e.band.text()]).collect();
    let mut widths = [0; 5];
    for r in &rows {
        for (w, c) in widths.iter_mut().zip(r) {
            *w = (*w).max(c.chars().count());
        }
    }
    let mut s = header.to_string();
    for r in &rows {
        let mut line = String::new();
        for (k, (c, w)) in r.iter().zip(widths).enumerate() {
            line += c;
            if k < 4 {
                line += &" ".repeat(w - c.chars().count() + 2);
            }
        }
        s += line.trim_end();
        s.push('\n');
    }
    s
}

#[cfg(test)]
mod tests {
    use super::*;

    fn entry(value: f64, band: Band) -> Entry {
        Entry::new("g", "pile 400 41", "rest", value, band, Better::Lower)
    }

    #[test]
    fn a_written_file_reads_back_as_the_same_values() {
        let entries = [
            entry(210.0, Band::Steps(30.0, 0.0)),
            Entry::new("g", "pile 400 41", "depth_end", 0.013_71, Band::Rel(0.1, 1e-4), Better::Lower),
            Entry::new("g", "pile 400 41", "energy_end", 1.6e-8, Band::Log(10.0, 1e-8), Better::Lower),
            Entry::new("g", "pile 400 41 turning", "rest", f64::INFINITY, Band::Steps(30.0, 0.1), Better::Lower),
            Entry::new("g", "cards", "stand", 11.0, Band::Exact, Better::Higher),
            Entry::new("g", "cards", "good", 105.0, Band::Abs(2.0), Better::Higher),
        ];
        let file = write("# header\n", &entries);
        assert!(compare(&file, &["g"], &entries, false).is_empty(), "{file}");
        assert_eq!(compare(&file, &["g"], &entries, true).len(), entries.len());
    }

    #[test]
    fn each_band_holds_inside_it_and_not_past_it() {
        assert!(Band::Steps(30.0, 0.0).holds(200.0, 230.0) && !Band::Steps(30.0, 0.0).holds(200.0, 240.0));
        assert!(Band::Steps(30.0, 0.2).holds(1000.0, 1200.0) && !Band::Steps(30.0, 0.2).holds(1000.0, 1210.0));
        assert!(Band::Steps(30.0, 0.0).holds(f64::INFINITY, f64::INFINITY) && !Band::Steps(30.0, 0.0).holds(700.0, f64::INFINITY));
        assert!(Band::Rel(0.1, 0.0).holds(0.02, 0.0219) && !Band::Rel(0.1, 0.0).holds(0.02, 0.0221));
        assert!(Band::Rel(0.1, 0.001).holds(0.0001, 0.001) && !Band::Rel(0.1, 0.001).holds(0.0001, 0.0012));
        assert!(Band::Log(10.0, 1e-8).holds(1e-6, 9e-6) && !Band::Log(10.0, 1e-8).holds(1e-6, 1.1e-5));
        assert!(Band::Log(10.0, 1e-8).holds(1e-12, 5e-9), "under the floor, equal");
        assert!(Band::Exact.holds(3.0, 3.0) && !Band::Exact.holds(3.0, 4.0));
        assert!(Band::Abs(1.0).holds(11.0, 10.0) && !Band::Abs(1.0).holds(11.0, 9.0));
    }

    #[test]
    fn a_value_past_its_band_fails_better_or_worse_and_says_which() {
        let file = write("", &[entry(210.0, Band::Steps(30.0, 0.0))]);
        for (new, verdict) in [(300.0, "worse"), (100.0, "better")] {
            let rows = compare(&file, &["g"], &[entry(new, Band::Steps(30.0, 0.0))], false);
            assert_eq!(rows.len(), 1);
            assert_eq!(rows[0].verdict, verdict);
        }
        let rows = compare(&file, &["g"], &[entry(210.0, Band::Steps(40.0, 0.0))], false);
        assert_eq!(rows[0].verdict, "band", "a band that changed fails until written");
    }

    #[test]
    fn a_line_no_run_records_is_gone_and_one_the_file_lacks_is_new() {
        let file = write("", &[entry(210.0, Band::Exact)]);
        let other = Entry::new("g", "pile 600 41", "rest", 200.0, Band::Exact, Better::Lower);
        let rows = compare(&file, &["g"], &[other], false);
        let verdicts: Vec<&str> = rows.iter().map(|r| r.verdict.as_str()).collect();
        assert_eq!(verdicts, ["new", "gone"]);
        assert!(compare(&file, &["h"], &[], false).is_empty(), "another group's lines are another binary's");
    }
}
