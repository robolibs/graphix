use std::f64::consts::PI;
use std::rc::Rc;

use glam::DVec2;

use graphix::X;
use graphix::factor::{
    Graph, NonlinearFactor, SE2BetweenFactor, SE2PriorFactor, SE2d, Values, Vec3d, cauchy_loss,
};

#[test]
fn se2_prior_factor_has_zero_error_at_prior() {
    let prior = SE2d::new(PI / 4.0, 5.0, 10.0);
    let factor =
        SE2PriorFactor::new(X(0).into(), prior, Vec3d::from_array([1.0, 1.0, 1.0])).unwrap();

    let mut values = Values::new();
    values.insert(X(0).into(), prior).unwrap();

    assert!(factor.error(&values).abs() < 1e-9);
}

#[test]
fn se2_prior_factor_respects_sigma_weighting() {
    let factor = SE2PriorFactor::new(
        X(0).into(),
        SE2d::new(0.0, 0.0, 0.0),
        Vec3d::from_array([0.1, 0.1, 0.1]),
    )
    .unwrap();

    let mut values = Values::new();
    values
        .insert(X(0).into(), SE2d::new(0.0, 1.0, 0.0))
        .unwrap();

    assert!((factor.error(&values) - 50.0).abs() < 1.0);
}

#[test]
fn se2_between_factor_uses_local_frame_translation() {
    let factor = SE2BetweenFactor::new(
        X(0).into(),
        X(1).into(),
        SE2d::new(0.0, 1.0, 0.0),
        Vec3d::from_array([1.0, 1.0, 1.0]),
    )
    .unwrap();

    let mut values = Values::new();
    values
        .insert(X(0).into(), SE2d::new(PI / 2.0, 0.0, 0.0))
        .unwrap();
    values
        .insert(X(1).into(), SE2d::new(PI / 2.0, 0.0, 1.0))
        .unwrap();

    assert!(factor.error(&values) < 1e-6);
}

#[test]
fn se2_nonlinear_factor_graph_accumulates_error() {
    enum AnyFactor {
        Prior(SE2PriorFactor),
        Between(SE2BetweenFactor),
    }

    impl graphix::factor::FactorLike for AnyFactor {
        fn keys(&self) -> &[graphix::Key] {
            match self {
                AnyFactor::Prior(f) => f.keys(),
                AnyFactor::Between(f) => f.keys(),
            }
        }
    }

    impl NonlinearFactor for AnyFactor {
        fn error(&self, values: &Values) -> f64 {
            match self {
                AnyFactor::Prior(f) => f.error(values),
                AnyFactor::Between(f) => f.error(values),
            }
        }
    }

    let mut graph = Graph::<AnyFactor>::new();
    graph.add(Rc::new(AnyFactor::Prior(
        SE2PriorFactor::new(
            X(0).into(),
            SE2d::new(0.0, 0.0, 0.0),
            Vec3d::from_array([0.1, 0.1, 0.1]),
        )
        .unwrap(),
    )));
    graph.add(Rc::new(AnyFactor::Between(
        SE2BetweenFactor::new(
            X(0).into(),
            X(1).into(),
            SE2d::new(0.0, 1.0, 0.0),
            Vec3d::from_array([0.1, 0.1, 0.1]),
        )
        .unwrap(),
    )));

    let mut values = Values::new();
    values
        .insert(X(0).into(), SE2d::new(0.0, 0.0, 0.0))
        .unwrap();
    values
        .insert(X(1).into(), SE2d::new(0.0, 1.0, 0.0))
        .unwrap();

    let total_error: f64 = graph.iter().map(|factor| factor.error(&values)).sum();
    assert!(total_error < 1e-6);
}

#[test]
fn se2_factor_construction_rejects_nonpositive_sigmas() {
    assert!(
        SE2PriorFactor::new(
            X(0).into(),
            SE2d::new(0.0, 0.0, 0.0),
            Vec3d::from_array([0.0, 1.0, 1.0]),
        )
        .is_err()
    );
    assert!(
        SE2PriorFactor::new(
            X(0).into(),
            SE2d::new(0.0, 0.0, 0.0),
            Vec3d::from_array([1.0, -1.0, 1.0]),
        )
        .is_err()
    );
    assert!(
        SE2BetweenFactor::new(
            X(0).into(),
            X(1).into(),
            SE2d::new(0.0, 0.0, 0.0),
            Vec3d::from_array([1.0, 1.0, 0.0]),
        )
        .is_err()
    );
}

#[test]
fn se2_factors_expose_loss_function_state() {
    let mut prior = SE2PriorFactor::new(
        X(0).into(),
        SE2d::new(0.0, 0.0, 0.0),
        Vec3d::from_array([1.0, 1.0, 1.0]),
    )
    .unwrap();
    let mut between = SE2BetweenFactor::new(
        X(0).into(),
        X(1).into(),
        SE2d::new(0.0, 1.0, 0.0),
        Vec3d::from_array([1.0, 1.0, 1.0]),
    )
    .unwrap();

    assert!(!prior.has_loss_function());
    assert!(prior.loss_function().is_none());
    assert!(!between.has_loss_function());
    assert!(between.loss_function().is_none());

    let loss = cauchy_loss(2.0);
    prior.set_loss_function(loss.clone());
    between.set_loss_function(loss);

    assert!(prior.has_loss_function());
    assert!(prior.loss_function().is_some());
    assert!(between.has_loss_function());
    assert!(between.loss_function().is_some());
}

#[test]
fn se2_factor_convenience_constructors_match_explicit_sigmas() {
    let prior = SE2PriorFactor::from_translation_sigmas(
        X(0).into(),
        SE2d::identity(),
        DVec2::new(0.1, 0.2),
        0.3,
    )
    .unwrap();
    let between = SE2BetweenFactor::from_translation_sigmas(
        X(0).into(),
        X(1).into(),
        SE2d::from_translation_angle(DVec2::new(1.0, 0.0), PI / 2.0),
        DVec2::new(0.4, 0.5),
        0.6,
    )
    .unwrap();

    assert_eq!(prior.sigmas(), Vec3d::new(0.1, 0.2, 0.3));
    assert_eq!(between.sigmas(), Vec3d::new(0.4, 0.5, 0.6));
}
