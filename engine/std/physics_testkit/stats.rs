//! The statistics a family of runs is judged by: its median, its worst
//! and its least. A family's median, never one run's value, is what a
//! chaotic scene is bounded by (physics-testing.md, "What the spread
//! says").

/// The middle value, the upper of the two for an even count; NaN for
/// none, so an empty selection (no bounce above the threshold, say) shows
/// as a value no bound or band holds rather than a panic.
pub fn median(mut v: Vec<f64>) -> f64 {
    if v.is_empty() {
        return f64::NAN;
    }
    // total_cmp, not partial_cmp: the order of a -0.0 and a 0.0 decides
    // which the median is, and so what the baseline shows.
    v.sort_by(f64::total_cmp);
    v[v.len() / 2]
}

/// The greatest, or -inf for none.
pub fn most(v: impl Iterator<Item = f64>) -> f64 {
    v.fold(f64::NEG_INFINITY, f64::max)
}

/// The least, or inf for none.
pub fn least(v: impl Iterator<Item = f64>) -> f64 {
    v.fold(f64::INFINITY, f64::min)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_median_is_the_upper_middle_and_none_is_nan() {
        assert_eq!(median(vec![3.0, 1.0, 2.0]), 2.0);
        assert_eq!(median(vec![4.0, 1.0, 3.0, 2.0]), 3.0);
        assert_eq!(median(vec![f64::INFINITY, 1.0, f64::INFINITY]), f64::INFINITY, "never is past every value");
        assert!(median(Vec::new()).is_nan());
    }

    #[test]
    fn most_and_least_of_none_are_past_any_bound() {
        assert_eq!(most([1.0, 5.0, 2.0].into_iter()), 5.0);
        assert_eq!(least([1.0, 5.0, 2.0].into_iter()), 1.0);
        assert_eq!(most(std::iter::empty()), f64::NEG_INFINITY);
        assert_eq!(least(std::iter::empty()), f64::INFINITY);
    }
}
