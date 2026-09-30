//! The bounce families' statistics: the same rules over 2D's and 3D's
//! grids (each harness's `bounces.rs` has its grids and says why a family,
//! physics-testing.md, "Families of a law"), from what each bounce is
//! (`Bounce`) and what its run measured (`Behaviour`, by the names both
//! `behave.rs` put), so a statistic means the same in both.

use crate::baseline::{Band, Better};
use crate::behaviour::Behaviour;
use crate::stats::{median, most};

/// What the statistics read of a bounce: the bounce as set up, not as run.
pub trait Bounce {
    /// Restitution.
    fn e(&self) -> f32;
    /// The impact speed along the normal.
    fn speed(&self) -> f32;
    fn mu(&self) -> f32;
    /// A ball: its bounces are each apex against the last (`series`).
    fn round(&self) -> bool;
    /// A box landing flat, which without friction meets square on as a
    /// ball does.
    fn flat(&self) -> bool;
    /// A box landing on a corner (or in 3D an edge), which tips as it
    /// bounces, and whose energy then turns between height, speed and spin:
    /// bounded apart.
    fn tips(&self) -> bool;
}

/// A family's statistics over its runs, by name: what the tests bound and
/// the baseline records. `family` is its name, which picks what applies;
/// `threshold` the closing speed below which nothing should bounce.
///
/// Of every family but the series and the pairs: the most energy a bounce
/// left with past what it came in with and what the push-out of its
/// deepest overlap lifts it by (`excess worst`, a share of what it came in
/// with; `behave::hit`), and apart the same of those that tip (`excess
/// tipping`); of the bodies that meet square on (a ball, a box flat without
/// friction), the most any bounce returned past what restitution gives and
/// that lift (`gain worst`, a share of the energy it came in with along the
/// normal), the median of that over the bounces above the threshold with
/// restitution (`gain median`; a loss is negative), their greatest loss,
/// how many under the threshold bounced and how many above it at e ≥ 0.25
/// didn't. Oblique: how far along the floor a frictionless bounce changed
/// its speed (`slip worst`), and with friction the most and the median of
/// what it kept. Pairs: the most returned past restitution, and the most
/// momentum lost. Series: the most a lossless ball rose (its highest apex
/// over its drop), and each bounce's apex over the one before against e²:
/// the most, and the median of each run's median below e = 1.
pub fn stats<H: Bounce>(family: &str, hits: &[H], runs: &[Behaviour], threshold: f32) -> Vec<(&'static str, f64)> {
    let both = |keep: &dyn Fn(&H, &Behaviour) -> bool, value: &dyn Fn(&Behaviour) -> f64| -> Vec<f64> {
        hits.iter().zip(runs).filter(|(h, b)| keep(h, b)).map(|(_, b)| value(b)).collect()
    };
    let mut v = Vec::new();
    if family == "series" {
        v.push(("rise worst", most(both(&|h, _| h.e() >= 1.0, &|b| b.get("rise most")).into_iter())));
        // A box that tips as it lands turns in flight, trading its turn for
        // height at the next bounce: each apex against the last is a
        // round body's.
        v.push(("decay worst", most(both(&|h, _| h.round(), &|b| b.get("decay most")).into_iter())));
        v.push(("decay median", median(both(&|h, _| h.round() && h.e() < 1.0, &|b| b.get("decay median")))));
        return v;
    }
    if family == "pairs" {
        v.push(("gain worst", most(both(&|_, _| true, &|b| b.get("gain")).into_iter())));
        v.push(("momentum worst", most(both(&|_, _| true, &|b| b.get("momentum")).into_iter())));
        return v;
    }
    v.push(("excess worst", most(both(&|h, _| !h.tips(), &|b| b.get("excess")).into_iter())));
    if hits.iter().any(H::tips) {
        v.push(("excess tipping", most(both(&|h, _| h.tips(), &|b| b.get("excess")).into_iter())));
    }
    // Restitution's own measures on the bodies that meet square on: a
    // corner that tips flat while it touches loses what its centre falls,
    // which is no bounce's, and dwarfs a slow impact's energy, and a box
    // that friction tips as it lands turns some of its speed along the
    // floor into speed off it (their energy is bounded all the same).
    let square = |h: &H, _: &Behaviour| h.round() || (h.flat() && h.mu() == 0.0);
    let bounces = |h: &H, b: &Behaviour| square(h, b) && h.speed() > threshold && h.e() > 0.0;
    let gains = both(&bounces, &|b| b.get("gain"));
    v.push(("gain worst", most(both(&square, &|b| b.get("gain")).into_iter())));
    v.push(("gain median", median(gains.clone())));
    v.push(("loss worst", most(gains.iter().map(|g| -g))));
    v.push(("bounced below", both(&|h, b| square(h, b) && h.speed() < threshold && b.get("bounced") == 1.0, &|_| 1.0).len() as f64));
    let flat = |h: &H, b: &Behaviour| bounces(h, b) && h.e() >= 0.25 && b.get("bounced") == 0.0;
    v.push(("flat above", both(&flat, &|_| 1.0).len() as f64));
    if family == "oblique" {
        v.push(("slip worst", most(both(&|h, _| h.mu() == 0.0, &|b| (b.get("tangent") - 1.0).abs()).into_iter())));
        v.push(("tangent most", most(both(&|h, _| h.mu() > 0.0, &|b| b.get("tangent")).into_iter())));
        v.push(("tangent median", median(both(&|h, _| h.mu() > 0.0, &|b| b.get("tangent")))));
    }
    v
}

/// A statistic by name.
pub fn stat(stats: &[(&'static str, f64)], name: &str) -> f64 {
    stats.iter().find(|(k, _)| *k == name).unwrap_or_else(|| panic!("no {name} in {stats:?}")).1
}

/// A statistic's band in the baseline, and which way is better: shares of
/// energy move by 5% of themselves or a thousandth; a count (`count`) by
/// what one run flips, which each harness sets by its grids; what is kept
/// of a speed or a height by 1%; the momentum lost, rounding in the fourth
/// decimal, by half of itself.
pub fn band(name: &str, count: Band) -> (Band, Better) {
    match name {
        "gain median" => (Band::Rel(0.05, 1e-3), Better::Toward(0.0)),
        "bounced below" | "flat above" => (count, Better::Lower),
        "tangent most" | "tangent median" => (Band::Rel(0.01, 1e-4), Better::Neither),
        "rise worst" | "decay worst" | "decay median" => (Band::Rel(0.01, 1e-3), Better::Toward(1.0)),
        "momentum worst" => (Band::Rel(0.5, 1e-4), Better::Lower),
        _ => (Band::Rel(0.05, 1e-3), Better::Lower),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Clone, Copy)]
    struct Hit {
        e: f32,
        v: f32,
        ball: bool,
    }

    impl Bounce for Hit {
        fn e(&self) -> f32 {
            self.e
        }
        fn speed(&self) -> f32 {
            self.v
        }
        fn mu(&self) -> f32 {
            0.0
        }
        fn round(&self) -> bool {
            self.ball
        }
        fn flat(&self) -> bool {
            !self.ball
        }
        fn tips(&self) -> bool {
            false
        }
    }

    fn run(gain: f64, bounced: bool) -> Behaviour {
        let mut b = Behaviour::new("exact".into());
        b.put("excess", gain);
        b.put("gain", gain);
        b.put("bounced", bounced as u8 as f64);
        b
    }

    #[test]
    fn a_bounce_below_the_threshold_that_bounced_counts_and_one_above_that_didn_t() {
        let hits = [Hit { e: 0.5, v: 0.5, ball: true }, Hit { e: 0.5, v: 5.0, ball: false }, Hit { e: 0.5, v: 5.0, ball: true }];
        let runs = [run(0.0, true), run(-0.1, false), run(0.02, true)];
        let s = stats("drops", &hits, &runs, 1.0);
        assert_eq!(stat(&s, "bounced below"), 1.0);
        assert_eq!(stat(&s, "flat above"), 1.0);
        assert_eq!(stat(&s, "gain worst"), 0.02);
        assert_eq!(stat(&s, "gain median"), 0.02, "the upper of the two above the threshold");
        assert!((stat(&s, "loss worst") - 0.1).abs() < 1e-12);
        assert!(s.iter().all(|(k, _)| *k != "excess tipping" && *k != "slip worst"), "nothing tips; not oblique");
    }

    #[test]
    fn a_count_takes_the_band_its_harness_gives() {
        assert_eq!(band("flat above", Band::Abs(3.0)), (Band::Abs(3.0), Better::Lower));
        assert_eq!(band("excess worst", Band::Abs(3.0)), (Band::Rel(0.05, 1e-3), Better::Lower));
    }
}
