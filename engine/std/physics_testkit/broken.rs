//! Every bound a test's runs broke, not the first, so a failure (or a
//! planted bug) says everything it changed (CLAUDE.md, "green has to be
//! earned"). A test checks each bound into a `Broken` and asserts once at
//! the end.

/// What a broken bound's line starts with: the scene, or the family by its
/// first scene.
pub trait Named {
    fn named(&self) -> String;
}

impl<T: Named + ?Sized> Named for &T {
    fn named(&self) -> String {
        (**self).named()
    }
}

/// A scene of a kind at a size (3D's `(Kind, n)`): the kind's name and the
/// size.
impl<K: Named> Named for (K, usize) {
    fn named(&self) -> String {
        format!("{} {}", self.0.named(), self.1)
    }
}

/// A run's values by name: a `Behaviour`, or a family's statistics.
pub trait Values {
    fn value(&self, name: &str) -> f64;
}

impl<T: Values + ?Sized> Values for &T {
    fn value(&self, name: &str) -> f64 {
        (**self).value(name)
    }
}

/// A run kept once a test binary (`runs::Runs`).
impl<T: Values + ?Sized> Values for std::sync::Arc<T> {
    fn value(&self, name: &str) -> f64 {
        (**self).value(name)
    }
}

/// A family's statistics, by name (`bounces::stats`).
impl Values for Vec<(&'static str, f64)> {
    fn value(&self, name: &str) -> f64 {
        self.iter().find(|(k, _)| *k == name).unwrap_or_else(|| panic!("no {name} in {self:?}")).1
    }
}

#[derive(Default)]
pub struct Broken(Vec<String>);

impl Broken {
    /// A bound, `what` saying how it broke. With `MATRIX` set in the
    /// environment every bound's line is printed, met or not, for a decision
    /// matrix (physics.md, "Why colors let the pyramid fall").
    pub fn check(&mut self, ok: bool, what: impl FnOnce() -> String) {
        let matrix = std::env::var_os("MATRIX").is_some();
        if !ok || matrix {
            let line = what();
            if matrix {
                println!("CHECK {} | {} | {line}", std::thread::current().name().unwrap_or("?"), if ok { "ok" } else { "FAIL" });
            }
            if !ok {
                self.0.push(line);
            }
        }
    }

    /// A bound on the run `at` names, its line starting with the name.
    pub fn check_at(&mut self, at: impl Named, ok: bool, what: impl FnOnce() -> String) {
        self.check(ok, || format!("{}: {}", at.named(), what()));
    }

    /// `name` at most `bound`.
    pub fn most(&mut self, at: impl Named, r: &impl Values, name: &str, bound: f64) {
        let v = r.value(name);
        self.check_at(at, v <= bound, || format!("{name} {v}, bound {bound}"));
    }

    /// `name` at least `bound`.
    pub fn least(&mut self, at: impl Named, r: &impl Values, name: &str, bound: f64) {
        let v = r.value(name);
        self.check_at(at, v >= bound, || format!("{name} {v}, at least {bound}"));
    }

    /// A family's statistic `name` at most `bound`: the family by its first
    /// scene, and its statistics.
    pub fn stat_most<N: Named, V: Values>(&mut self, f: &(N, V), name: &str, bound: f64) {
        self.most(&f.0, &f.1, name, bound);
    }

    /// ... at least `bound`.
    pub fn stat_least<N: Named, V: Values>(&mut self, f: &(N, V), name: &str, bound: f64) {
        self.least(&f.0, &f.1, name, bound);
    }

    pub fn assert(self) {
        assert!(self.0.is_empty(), "{} bounds broken:\n{}", self.0.len(), self.0.join("\n"));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    struct Scene;

    impl Named for Scene {
        fn named(&self) -> String {
            "pile 400".to_string()
        }
    }

    #[test]
    fn every_broken_bound_is_listed_by_its_scene() {
        let r = vec![("rest", 300.0), ("depth", 0.01)];
        let mut b = Broken::default();
        b.most(&Scene, &r, "rest", 250.0);
        b.least(&Scene, &r, "depth", 0.02);
        b.most(&(Scene, 7), &r, "depth", 0.02);
        b.check(false, || "by hand".to_string());
        assert_eq!(b.0, ["pile 400: rest 300, bound 250", "pile 400: depth 0.01, at least 0.02", "by hand"], "a bound met is no line");
        let err = std::panic::catch_unwind(|| b.assert()).unwrap_err();
        let want = "3 bounds broken:\npile 400: rest 300, bound 250\npile 400: depth 0.01, at least 0.02\nby hand";
        assert_eq!(err.downcast_ref::<String>().unwrap(), want);
    }

    #[test]
    fn a_kind_at_a_size_is_named_by_both() {
        let mut b = Broken::default();
        b.most(&(Scene, 7), &vec![("depth", 0.03)], "depth", 0.02);
        assert_eq!(b.0, ["pile 400 7: depth 0.03, bound 0.02"]);
    }

    #[test]
    fn none_broken_passes() {
        let mut b = Broken::default();
        b.check(true, || unreachable!("a bound met says nothing"));
        b.assert();
    }
}
