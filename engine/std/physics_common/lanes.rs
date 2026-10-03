//! The lane array a solver's kernels compute `N` contacts at once in
//! (physics.md, "The solver's speed"): what a lanes kernel shares apart
//! from the dimension. Box2D's `b2FloatW` (Erin Catto's, docs/CREDITS.md),
//! written as plain arrays rather than intrinsics, so no unsafe code.

use std::ops::{Add, Mul, Neg, Sub};

/// `N` lanes of `f32`: each operation a loop over the lanes, which LLVM
/// makes one SIMD instruction a register (SSE2 on x86-64's baseline).
#[derive(Clone, Copy, Debug)]
pub struct F<const N: usize>(pub [f32; N]);

impl<const N: usize> F<N> {
    pub const ZERO: F<N> = F([0.0; N]);

    #[inline(always)]
    pub fn splat(x: f32) -> F<N> {
        F([x; N])
    }

    #[inline(always)]
    fn zip(self, o: F<N>, f: impl Fn(f32, f32) -> f32) -> F<N> {
        let mut r = self.0;
        for l in 0..N {
            r[l] = f(self.0[l], o.0[l]);
        }
        F(r)
    }

    /// x86's `maxps`, which SSE2 has one instruction for: where neither
    /// is NaN, `f32::max`, so the lanes are the scalar solve to the bit.
    #[inline(always)]
    pub fn max(self, o: F<N>) -> F<N> {
        self.zip(o, |a, b| if a > b { a } else { b })
    }

    /// `f32::clamp`, as it is written.
    #[inline(always)]
    pub fn clamp(self, lo: F<N>, hi: F<N>) -> F<N> {
        let low = self.zip(lo, |x, lo| if x < lo { lo } else { x });
        low.zip(hi, |x, hi| if x > hi { hi } else { x })
    }

    /// `a` where `self` is positive, else `b`.
    #[inline(always)]
    pub fn positive_then(self, a: F<N>, b: F<N>) -> F<N> {
        let mut r = b.0;
        for l in 0..N {
            if self.0[l] > 0.0 {
                r[l] = a.0[l];
            }
        }
        F(r)
    }
}

impl<const N: usize> Add for F<N> {
    type Output = F<N>;
    #[inline(always)]
    fn add(self, o: F<N>) -> F<N> {
        self.zip(o, |a, b| a + b)
    }
}

impl<const N: usize> Sub for F<N> {
    type Output = F<N>;
    #[inline(always)]
    fn sub(self, o: F<N>) -> F<N> {
        self.zip(o, |a, b| a - b)
    }
}

impl<const N: usize> Mul for F<N> {
    type Output = F<N>;
    #[inline(always)]
    fn mul(self, o: F<N>) -> F<N> {
        self.zip(o, |a, b| a * b)
    }
}

impl<const N: usize> Neg for F<N> {
    type Output = F<N>;
    #[inline(always)]
    fn neg(self) -> F<N> {
        let mut r = self.0;
        for x in r.iter_mut() {
            *x = -*x;
        }
        F(r)
    }
}

#[cfg(test)]
mod tests {
    use super::F;

    /// Each lane is the scalar operation to the bit, the claim the lanes
    /// solve's equivalence with the one-contact loop rests on: the ties
    /// and signed zeros where a hand-written select could differ from
    /// `f32::max` and `f32::clamp` included. (`f32::max` returns either
    /// zero for `max(0.0, -0.0)`, so only the order the select is written
    /// in is pinned there, against its scalar spelling.)
    #[test]
    fn each_lane_is_the_scalar_operation_bit_for_bit() {
        let xs = [0.0, -0.0, 1.5, -2.25, 3.0e-39, f32::MAX, -f32::MAX, 0.1];
        let bits = |f: F<8>| f.0.map(f32::to_bits);
        let lanes = |f: fn(f32, f32) -> f32, o: f32| F(xs.map(|x| f(x, o)));
        for o in xs {
            let (a, b) = (F(xs), F::splat(o));
            assert_eq!(bits(a + b), bits(lanes(|x, o| x + o, o)));
            assert_eq!(bits(a - b), bits(lanes(|x, o| x - o, o)));
            assert_eq!(bits(a * b), bits(lanes(|x, o| x * o, o)));
            assert_eq!(bits(-a), bits(F(xs.map(|x| -x))));
            assert_eq!(bits(a.max(b)), bits(lanes(|x, o| if x > o { x } else { o }, o)));
            let (lo, hi) = (o.min(1.0), o.max(1.0));
            assert_eq!(bits(a.clamp(F::splat(lo), F::splat(hi))), bits(F(xs.map(|x| x.clamp(lo, hi)))));
            assert_eq!(bits(b.positive_then(a, F::ZERO)), bits(F(xs.map(|x| if o > 0.0 { x } else { 0.0 }))));
        }
    }
}
