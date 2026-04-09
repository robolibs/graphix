use std::collections::BTreeMap;

use graphix::X;
use graphix::factor::{
    GaussianFactor, Matrix, NonlinearFactor, SE2BetweenFactor, SE2PriorFactor, SE2d, Values, Vec3d,
    Vector,
};

#[test]
fn gaussian_factor_constructs_and_exposes_components() {
    let mut j = Matrix::new(1, 1);
    j[(0, 0)] = 2.0;
    let mut b = Vector::new(1);
    b[0] = 3.0;

    let gf = GaussianFactor::new(vec![1], vec![j.clone()], b.clone()).unwrap();
    assert_eq!(gf.size(), 1);
    assert_eq!(gf.dim(), 1);
    assert_eq!(gf.b()[0], 3.0);
    assert_eq!(gf.jacobians().len(), 1);
    assert_eq!(gf.jacobian(1).unwrap()[(0, 0)], 2.0);
}

#[test]
fn gaussian_factor_validates_dimensions() {
    let j1 = Matrix::new(2, 3);
    let j2 = Matrix::new(3, 3);
    let b = Vector::new(2);

    assert!(GaussianFactor::new(vec![1, 2], vec![j1, j2], b).is_err());
}

#[test]
fn gaussian_factor_computes_error() {
    let mut j = Matrix::new(1, 1);
    j[(0, 0)] = 2.0;
    let mut b = Vector::new(1);
    b[0] = 3.0;
    let gf = GaussianFactor::new(vec![1], vec![j], b).unwrap();

    let mut deltas = BTreeMap::new();
    let mut delta = Vector::new(1);
    delta[0] = 1.0;
    deltas.insert(1, delta);

    assert!((gf.error(&deltas).unwrap() - 12.5).abs() < 1e-9);
}

#[test]
fn gaussian_factor_handles_multiple_variables() {
    let mut j1 = Matrix::new(1, 1);
    j1[(0, 0)] = 2.0;
    let mut j2 = Matrix::new(1, 1);
    j2[(0, 0)] = 3.0;
    let mut b = Vector::new(1);
    b[0] = 1.0;
    let gf = GaussianFactor::new(vec![1, 2], vec![j1, j2], b).unwrap();

    let mut deltas = BTreeMap::new();
    let mut d1 = Vector::new(1);
    d1[0] = 1.0;
    let mut d2 = Vector::new(1);
    d2[0] = 1.0;
    deltas.insert(1, d1);
    deltas.insert(2, d2);

    assert!((gf.error(&deltas).unwrap() - 18.0).abs() < 1e-9);
}

#[test]
fn gaussian_factor_scaling_preserves_quadratic_error_equivalence() {
    let mut j = Matrix::new(1, 1);
    j[(0, 0)] = 4.0;
    let mut b = Vector::new(1);
    b[0] = 6.0;
    let mut gf = GaussianFactor::new(vec![1], vec![j], b).unwrap();

    let mut deltas = BTreeMap::new();
    let mut delta = Vector::new(1);
    delta[0] = 0.5;
    deltas.insert(1, delta);

    let original = gf.error(&deltas).unwrap();
    gf.scale(0.5);
    let scaled = gf.error(&deltas).unwrap();

    assert!((scaled - original * 0.25).abs() < 1e-9);
}

#[test]
fn se2_between_linearization_handles_exact_measurement_and_error_cases() {
    let factor = SE2BetweenFactor::new(
        X(0).into(),
        X(1).into(),
        SE2d::new(0.0, 1.0, 0.0),
        Vec3d::from_array([0.1, 0.1, 0.1]),
    )
    .unwrap();

    let mut exact = Values::new();
    exact.insert(X(0).into(), SE2d::new(0.0, 0.0, 0.0)).unwrap();
    exact.insert(X(1).into(), SE2d::new(0.0, 1.0, 0.0)).unwrap();
    let exact_linear = factor.linearize(&exact).unwrap();
    assert!(exact_linear.b().iter().all(|value| value.abs() < 0.1));
    assert_eq!(exact_linear.jacobian(X(0).into()).unwrap().rows(), 3);
    assert_eq!(exact_linear.jacobian(X(1).into()).unwrap().cols(), 3);

    let mut offset = Values::new();
    offset
        .insert(X(0).into(), SE2d::new(0.0, 0.0, 0.0))
        .unwrap();
    offset
        .insert(X(1).into(), SE2d::new(0.1, 1.5, 0.2))
        .unwrap();
    let offset_linear = factor.linearize(&offset).unwrap();
    assert!(offset_linear.b()[0].abs() > 1.0);
    assert!(offset_linear.b()[1].abs() > 1.0);
}

#[test]
fn se2_linearization_is_numerically_stable_for_small_sigmas_and_large_values() {
    let small_sigma = SE2PriorFactor::new(
        X(0).into(),
        SE2d::new(0.0, 0.0, 0.0),
        Vec3d::from_array([0.001, 0.001, 0.001]),
    )
    .unwrap();
    let mut near_zero = Values::new();
    near_zero
        .insert(X(0).into(), SE2d::new(0.0001, 0.0001, 0.0001))
        .unwrap();
    let small_linear = small_sigma.linearize(&near_zero).unwrap();
    assert_eq!(small_linear.dim(), 3);
    assert!(small_linear.b().iter().all(|value| value.is_finite()));

    let large_value = SE2PriorFactor::new(
        X(0).into(),
        SE2d::new(3.14, 1000.0, 2000.0),
        Vec3d::from_array([10.0, 10.0, 0.1]),
    )
    .unwrap();
    let mut far = Values::new();
    far.insert(X(0).into(), SE2d::new(3.24, 1050.0, 2050.0))
        .unwrap();
    let large_linear = large_value.linearize(&far).unwrap();
    assert_eq!(large_linear.dim(), 3);
    assert!(large_linear.b().iter().all(|value| value.is_finite()));
}

#[test]
fn se2_prior_linearization_matches_expected_analytical_scaling() {
    let factor = SE2PriorFactor::new(
        X(0).into(),
        SE2d::new(0.0, 1.0, 2.0),
        Vec3d::from_array([0.5, 0.5, 0.5]),
    )
    .unwrap();

    let mut values = Values::new();
    values
        .insert(X(0).into(), SE2d::new(0.1, 1.1, 2.1))
        .unwrap();

    let linear = factor.linearize(&values).unwrap();
    let jacobian = linear.jacobian(X(0).into()).unwrap();
    assert!((jacobian[(0, 0)] - 2.0).abs() < 0.4);
    assert!((jacobian[(1, 1)] - 2.0).abs() < 0.4);
    assert!((jacobian[(2, 2)] - 2.0).abs() < 0.4);
}
