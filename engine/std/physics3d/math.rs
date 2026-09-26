//! Vectors, quaternions and 3x3 matrices: just what the step needs, in
//! `f32`, with no crate to pin.

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vec3 {
    pub const ZERO: Vec3 = Vec3 { x: 0.0, y: 0.0, z: 0.0 };
    pub const X: Vec3 = Vec3 { x: 1.0, y: 0.0, z: 0.0 };
    pub const Y: Vec3 = Vec3 { x: 0.0, y: 1.0, z: 0.0 };
    pub const Z: Vec3 = Vec3 { x: 0.0, y: 0.0, z: 1.0 };

    pub const fn new(x: f32, y: f32, z: f32) -> Vec3 {
        Vec3 { x, y, z }
    }

    pub const fn splat(v: f32) -> Vec3 {
        Vec3 { x: v, y: v, z: v }
    }

    pub fn dot(self, o: Vec3) -> f32 {
        self.x * o.x + self.y * o.y + self.z * o.z
    }

    pub fn cross(self, o: Vec3) -> Vec3 {
        Vec3::new(self.y * o.z - self.z * o.y, self.z * o.x - self.x * o.z, self.x * o.y - self.y * o.x)
    }

    pub fn len(self) -> f32 {
        self.dot(self).sqrt()
    }

    /// Unit, or zero for zero.
    pub fn normalize(self) -> Vec3 {
        let l = self.len();
        if l > 0.0 { self * (1.0 / l) } else { Vec3::ZERO }
    }

    pub fn abs(self) -> Vec3 {
        Vec3::new(self.x.abs(), self.y.abs(), self.z.abs())
    }

    pub fn times(self, o: Vec3) -> Vec3 {
        Vec3::new(self.x * o.x, self.y * o.y, self.z * o.z)
    }

    pub fn clamp(self, lo: Vec3, hi: Vec3) -> Vec3 {
        Vec3::new(self.x.clamp(lo.x, hi.x), self.y.clamp(lo.y, hi.y), self.z.clamp(lo.z, hi.z))
    }

    pub fn get(self, axis: usize) -> f32 {
        [self.x, self.y, self.z][axis]
    }

    pub fn set(&mut self, axis: usize, v: f32) {
        match axis {
            0 => self.x = v,
            1 => self.y = v,
            _ => self.z = v,
        }
    }

    /// A unit vector at right angles to this unit one.
    pub fn perp(self) -> Vec3 {
        // Crossed with the axis it leans on least, so never near zero.
        let a = if self.x.abs() < 0.57735 { Vec3::X } else { Vec3::Y };
        self.cross(a).normalize()
    }
}

impl std::ops::Add for Vec3 {
    type Output = Vec3;
    fn add(self, o: Vec3) -> Vec3 {
        Vec3::new(self.x + o.x, self.y + o.y, self.z + o.z)
    }
}

impl std::ops::Sub for Vec3 {
    type Output = Vec3;
    fn sub(self, o: Vec3) -> Vec3 {
        Vec3::new(self.x - o.x, self.y - o.y, self.z - o.z)
    }
}

impl std::ops::Mul<f32> for Vec3 {
    type Output = Vec3;
    fn mul(self, s: f32) -> Vec3 {
        Vec3::new(self.x * s, self.y * s, self.z * s)
    }
}

impl std::ops::Neg for Vec3 {
    type Output = Vec3;
    fn neg(self) -> Vec3 {
        Vec3::new(-self.x, -self.y, -self.z)
    }
}

impl std::ops::AddAssign for Vec3 {
    fn add_assign(&mut self, o: Vec3) {
        *self = *self + o;
    }
}

impl std::ops::SubAssign for Vec3 {
    fn sub_assign(&mut self, o: Vec3) {
        *self = *self - o;
    }
}

/// A rotation, unit length: `v` the vector part, `w` the scalar.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Quat {
    pub v: Vec3,
    pub w: f32,
}

impl Default for Quat {
    fn default() -> Quat {
        Quat::IDENTITY
    }
}

impl Quat {
    pub const IDENTITY: Quat = Quat { v: Vec3::ZERO, w: 1.0 };

    pub fn axis_angle(axis: Vec3, angle: f32) -> Quat {
        let (s, c) = (angle * 0.5).sin_cos();
        Quat { v: axis.normalize() * s, w: c }
    }

    pub fn times(self, o: Quat) -> Quat {
        Quat { v: o.v * self.w + self.v * o.w + self.v.cross(o.v), w: self.w * o.w - self.v.dot(o.v) }
    }

    pub fn conj(self) -> Quat {
        Quat { v: -self.v, w: self.w }
    }

    pub fn rotate(self, p: Vec3) -> Vec3 {
        // p + 2w (v x p) + 2 v x (v x p), two crosses and no matrix.
        let t = self.v.cross(p) * 2.0;
        p + t * self.w + self.v.cross(t)
    }

    pub fn normalize(self) -> Quat {
        let l = (self.v.dot(self.v) + self.w * self.w).sqrt();
        if l > 0.0 { Quat { v: self.v * (1.0 / l), w: self.w / l } } else { Quat::IDENTITY }
    }

    /// Turned by angular velocity `w` for `h`, to first order, not
    /// normalized: q + h/2 (w, 0) q (Box3D's `b3IntegrateRotation`).
    pub fn integrate(self, w: Vec3, h: f32) -> Quat {
        let d = Quat { v: w * (0.5 * h), w: 0.0 }.times(self);
        Quat { v: self.v + d.v, w: self.w + d.w }
    }

    /// Turned by `w` for `h` exactly: the rotation by |w| h about w, as
    /// Jolt does (`Body::AddRotationStep`).
    pub fn integrate_exact(self, w: Vec3, h: f32) -> Quat {
        let angle = w.len() * h;
        if angle <= 1e-6 {
            return self;
        }
        Quat::axis_angle(w, angle).times(self)
    }

    pub fn matrix(self) -> Mat3 {
        Mat3 { cols: [self.rotate(Vec3::X), self.rotate(Vec3::Y), self.rotate(Vec3::Z)] }
    }
}

/// Columns: for a rotation, the body's axes in the world.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Mat3 {
    pub cols: [Vec3; 3],
}

impl Mat3 {
    pub const ZERO: Mat3 = Mat3 { cols: [Vec3::ZERO; 3] };

    pub fn apply(&self, v: Vec3) -> Vec3 {
        self.cols[0] * v.x + self.cols[1] * v.y + self.cols[2] * v.z
    }

    /// R diag(d) R^T, for R a rotation: a body's inverse inertia in the
    /// world from its own.
    pub fn rotated_diagonal(r: &Mat3, d: Vec3) -> Mat3 {
        let c = r.cols;
        // Column j: the sum over k of d_k c_k (c_k)_j.
        let col = |j: usize| c[0] * (d.x * c[0].get(j)) + c[1] * (d.y * c[1].get(j)) + c[2] * (d.z * c[2].get(j));
        Mat3 { cols: [col(0), col(1), col(2)] }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn near(a: Vec3, b: Vec3) -> bool {
        (a - b).len() < 1e-5
    }

    #[test]
    fn a_quarter_turn_about_y_takes_x_to_minus_z() {
        let q = Quat::axis_angle(Vec3::Y, std::f32::consts::FRAC_PI_2);
        assert!(near(q.rotate(Vec3::X), -Vec3::Z), "{:?}", q.rotate(Vec3::X));
        assert!(near(q.matrix().apply(Vec3::X), -Vec3::Z));
        assert!(near(q.conj().rotate(q.rotate(Vec3::new(1.0, 2.0, 3.0))), Vec3::new(1.0, 2.0, 3.0)));
    }

    #[test]
    fn integrating_agrees_with_the_exact_turn_for_small_steps() {
        let w = Vec3::new(0.3, -2.0, 1.0);
        let (a, b) = (Quat::IDENTITY.integrate(w, 0.001).normalize(), Quat::IDENTITY.integrate_exact(w, 0.001));
        assert!(near(a.rotate(Vec3::X), b.rotate(Vec3::X)));
        assert!(near(b.rotate(Vec3::X), Quat::axis_angle(w, w.len() * 0.001).rotate(Vec3::X)));
    }

    #[test]
    fn a_world_inertia_is_the_local_one_turned() {
        let q = Quat::axis_angle(Vec3::new(1.0, 1.0, 0.0), 0.7);
        let (r, d) = (q.matrix(), Vec3::new(1.0, 2.0, 3.0));
        let m = Mat3::rotated_diagonal(&r, d);
        let v = Vec3::new(0.2, -0.4, 0.9);
        // R d R^T v, the long way.
        let expect = q.rotate(d.times(q.conj().rotate(v)));
        assert!(near(m.apply(v), expect), "{:?} {:?}", m.apply(v), expect);
    }
}
