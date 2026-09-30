//! A run of a behaviour scene: its named values, which each harness's
//! `behave.rs` measures from its engines' states and the tests bound; and
//! the two measures of a bouncing body that are the same arithmetic in 2D
//! and 3D once each says how high the body is and how fast it rises
//! (physics.md, "Quality beyond settling", and "Bounces").

use crate::broken::Values;

/// One engine on one scene: named values, in the order they were found.
#[derive(Clone, Debug, Default)]
pub struct Behaviour {
    /// The engine, as the comparison prints it.
    pub label: String,
    pub values: Vec<(&'static str, f64)>,
}

impl Behaviour {
    pub fn new(label: String) -> Behaviour {
        Behaviour { label, values: Vec::new() }
    }

    pub fn get(&self, name: &str) -> f64 {
        self.values.iter().find(|(k, _)| *k == name).map(|(_, v)| *v).unwrap_or_else(|| panic!("no {name} in {:?}", self.values))
    }

    pub fn put(&mut self, name: &'static str, v: f64) {
        self.values.push((name, v));
    }
}

impl Values for Behaviour {
    fn value(&self, name: &str) -> f64 {
        self.get(name)
    }
}

/// Never at rest, or never toppled: past any bound.
pub const NEVER: f64 = f64::INFINITY;

/// A ball dropped from `drop` (`Scene::Bounce`, `Kind::Bounce`), from its
/// states after the first: each step's height of its bottom and its speed
/// up. Each apex after a bounce, as a share of the drop: the first against
/// e², and over many bounces the most and the last (a lossless ball gaining
/// height gains energy). An apex is the highest step from when it starts
/// to rise to when it starts to fall.
pub fn bounce(b: &mut Behaviour, e: f32, drop: f32, steps: impl Iterator<Item = (f32, f32)>) {
    let (mut apexes, mut rising, mut top) = (Vec::new(), false, 0f32);
    for (height, up) in steps {
        if !rising && up > 0.0 {
            (rising, top) = (true, height);
        } else if rising {
            top = top.max(height);
            if up < 0.0 {
                apexes.push(top / drop);
                rising = false;
            }
        }
    }
    b.put("expected", (e * e) as f64);
    b.put("first apex", apexes.first().copied().unwrap_or(0.0) as f64);
    b.put("bounces", apexes.len() as f64);
    b.put("most apex", apexes.iter().copied().fold(0.0, f32::max) as f64);
    b.put("last apex", apexes.last().copied().unwrap_or(0.0) as f64);
}

/// Many bounces (a bounce family's `series`), from each step's height
/// and speed up after the first, `drop` the height it fell from: each apex
/// over `drop`, the most, and each apex over the one before against e²,
/// while the bounce is well above the threshold (a rebound faster than 2):
/// the median and the most. A lossless ball's never rises; one of
/// restitution e keeps e² of its height a bounce. A height is the
/// energy's, its height and its speed up over 2g, which free flight keeps:
/// the highest step would read an apex low by up to g (dt / 2)² / 2 (3% of
/// a low bounce at gravity 80), and their ratios so pass e². Unlike
/// `bounce`, an apex is the highest step while rising: the step it turns
/// in is falling already.
pub fn series(b: &mut Behaviour, e: f32, g: f32, drop: f32, steps: impl Iterator<Item = (f32, f32)>) {
    let (mut apexes, mut rising, mut top) = (Vec::new(), false, 0f32);
    for (height, up) in steps {
        if !rising && up > 0.0 {
            (rising, top) = (true, height);
        } else if rising {
            if up > 0.0 {
                top = top.max(height);
            }
            if up < 0.0 {
                apexes.push(top / drop);
                rising = false;
            }
        }
    }
    let e2 = e * e;
    let fast = |a: f32| (2.0 * g * a * drop).sqrt() > 2.0;
    let with_drop: Vec<f32> = std::iter::once(1.0).chain(apexes.iter().copied()).collect();
    let mut ratios: Vec<f64> = with_drop.windows(2).filter(|w| fast(w[0]) && fast(w[1])).map(|w| (w[1] / w[0] / e2) as f64).collect();
    b.put("apexes", apexes.len() as f64);
    b.put("rise most", apexes.iter().copied().fold(0.0, f32::max) as f64);
    ratios.sort_by(f64::total_cmp);
    b.put("decay median", ratios.get(ratios.len() / 2).copied().unwrap_or(f64::NAN));
    b.put("decay most", ratios.last().copied().unwrap_or(f64::NAN));
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Heights and speeds up of a ball dropped from `h0`, bouncing back at
    /// `e` of its speed, stepped at `dt`.
    fn flight(h0: f32, e: f32, g: f32, dt: f32, steps: usize) -> Vec<(f32, f32)> {
        let (mut h, mut v, mut t) = (h0, 0.0f32, Vec::new());
        for _ in 0..steps {
            v -= g * dt;
            h += v * dt;
            if h < 0.0 {
                (h, v) = (-h, -e * v);
            }
            t.push((h, v));
        }
        t
    }

    #[test]
    fn a_ball_that_keeps_half_its_speed_rises_a_quarter_as_high() {
        let mut b = Behaviour::new("exact".into());
        let (g, drop) = (10.0, 5.0);
        let steps = flight(drop, 0.5, g, 1e-4, 60_000).into_iter().map(|(h, v)| (h + 0.5 * v * v / g, v));
        series(&mut b, 0.5, g, drop, steps);
        assert!(b.get("apexes") >= 3.0, "{:?}", b.values);
        assert!((b.get("decay median") - 1.0).abs() < 1e-2, "{:?}", b.values);
        assert!((b.get("rise most") - 0.25).abs() < 1e-2, "{:?}", b.values);
    }

    #[test]
    fn a_bounce_apex_is_a_share_of_the_drop() {
        let mut b = Behaviour::new("exact".into());
        // Dropped from 2: falls, rises to 0.5, falls, rises to 0.125.
        let steps = [(2.0, -1.0), (0.0, 1.0), (0.4, 1.0), (0.5, -1.0), (0.0, 1.0), (0.125, -1.0)];
        bounce(&mut b, 0.5, 2.0, steps.into_iter());
        assert_eq!(b.values, [("expected", 0.25), ("first apex", 0.25), ("bounces", 2.0), ("most apex", 0.25), ("last apex", 0.0625)]);
    }
}
