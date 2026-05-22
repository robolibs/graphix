use std::f64::consts::PI;
use std::ops::Mul;

use datapod::Point;

pub type Vec3d = nalgebra::Vector3<f64>;
pub type DynVec = nalgebra::DVector<f64>;
pub type DynMat = nalgebra::DMatrix<f64>;

pub fn se2_sigmas(translation: Point, rotation: f64) -> Vec3d {
    Vec3d::from([translation.x, translation.y, rotation])
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SE2d {
    theta: f64,
    translation: Point,
}

impl SE2d {
    pub fn identity() -> Self {
        Self::new(0.0, 0.0, 0.0)
    }

    pub fn new(theta: f64, x: f64, y: f64) -> Self {
        Self {
            theta,
            translation: Point::new(x, y, 0.0),
        }
    }

    pub fn from_translation_angle(translation: Point, theta: f64) -> Self {
        Self {
            theta,
            translation: Point::new(translation.x, translation.y, 0.0),
        }
    }

    pub fn angle(&self) -> f64 {
        self.theta
    }

    pub fn translation(&self) -> Point {
        self.translation
    }

    pub fn x(&self) -> f64 {
        self.translation.x
    }

    pub fn y(&self) -> f64 {
        self.translation.y
    }

    pub fn retract(&self, delta: Vec3d) -> Self {
        Self::new(
            wrap_angle(self.theta + delta[2]),
            self.translation.x + delta[0],
            self.translation.y + delta[1],
        )
    }

    pub fn inverse(&self) -> Self {
        let c = self.theta.cos();
        let s = self.theta.sin();
        let x = -(c * self.translation.x + s * self.translation.y);
        let y = s * self.translation.x - c * self.translation.y;
        Self::new(-self.theta, x, y)
    }

    pub fn log(&self) -> Vec3d {
        Vec3d::from([
            self.translation.x,
            self.translation.y,
            wrap_angle(self.theta),
        ])
    }

    pub fn compose(&self, rhs: Self) -> Self {
        *self * rhs
    }

    pub fn between(&self, other: Self) -> Self {
        self.inverse() * other
    }

    pub fn transform_point(&self, point: Point) -> Point {
        let c = self.theta.cos();
        let s = self.theta.sin();
        Point::new(
            self.translation.x + c * point.x - s * point.y,
            self.translation.y + s * point.x + c * point.y,
            point.z,
        )
    }

    pub fn inverse_transform_point(&self, point: Point) -> Point {
        self.inverse().transform_point(point)
    }
}

impl Mul for SE2d {
    type Output = SE2d;

    fn mul(self, rhs: Self) -> Self::Output {
        let c = self.theta.cos();
        let s = self.theta.sin();
        let x = self.translation.x + c * rhs.translation.x - s * rhs.translation.y;
        let y = self.translation.y + s * rhs.translation.x + c * rhs.translation.y;
        SE2d::new(wrap_angle(self.theta + rhs.theta), x, y)
    }
}

fn wrap_angle(mut angle: f64) -> f64 {
    while angle > PI {
        angle -= 2.0 * PI;
    }
    while angle < -PI {
        angle += 2.0 * PI;
    }
    angle
}
