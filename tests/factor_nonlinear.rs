use graphix::X;
use graphix::factor::{
    BetweenFactor, FactorGraphAdapter, GaussNewtonOptimizer, GaussNewtonParameters,
    GaussNewtonResult, GradientDescentOptimizer, GradientDescentResult, Graph,
    LevenbergMarquardtOptimizer, LevenbergMarquardtParameters, LevenbergMarquardtResult,
    NonlinearFactor, OptinumGaussNewton, OptinumGradientDescent, OptinumLevenbergMarquardt,
    Parameters, PriorFactor, SE2BetweenFactor, SE2PriorFactor, SE2d, Values, Vec3d, cauchy_loss,
    huber_loss, tukey_loss,
};
use std::rc::Rc;

#[test]
fn prior_and_between_factors_compute_expected_error() {
    let prior = PriorFactor::new(X(0).into(), 10.0, 1.0).unwrap();
    let between = BetweenFactor::new(X(0).into(), X(1).into(), 5.0, 1.0).unwrap();

    let mut values = Values::new();
    values.insert(X(0).into(), 11.0).unwrap();
    values.insert(X(1).into(), 16.0).unwrap();

    assert_eq!(prior.prior(), 10.0);
    assert_eq!(prior.sigma(), 1.0);
    assert!((prior.error(&values) - 0.5).abs() < 1e-9);
    assert_eq!(between.measured(), 5.0);
    assert_eq!(between.sigma(), 1.0);
    assert!((between.error(&values) - 0.0).abs() < 1e-9);
}

#[test]
fn scalar_factors_expose_loss_function_state() {
    let mut prior = PriorFactor::new(X(0).into(), 10.0, 1.0).unwrap();
    let mut between = BetweenFactor::new(X(0).into(), X(1).into(), 5.0, 1.0).unwrap();

    assert!(!prior.has_loss_function());
    assert!(prior.loss_function().is_none());
    assert!(!between.has_loss_function());
    assert!(between.loss_function().is_none());

    let loss = huber_loss(1.5);
    prior.set_loss_function(loss.clone());
    between.set_loss_function(loss);

    assert!(prior.has_loss_function());
    assert!(prior.loss_function().is_some());
    assert!(between.has_loss_function());
    assert!(between.loss_function().is_some());
}

#[test]
fn scalar_factor_construction_rejects_nonpositive_sigma() {
    assert!(PriorFactor::new(X(0).into(), 0.0, 0.0).is_err());
    assert!(PriorFactor::new(X(0).into(), 0.0, -1.0).is_err());
    assert!(BetweenFactor::new(X(0).into(), X(1).into(), 1.0, 0.0).is_err());
    assert!(BetweenFactor::new(X(0).into(), X(1).into(), 1.0, -1.0).is_err());
}

#[test]
fn nonlinear_factor_graph_holds_factors() {
    let mut graph = Graph::<PriorFactor>::new();
    graph.add(Rc::new(PriorFactor::new(X(0).into(), 0.0, 1.0).unwrap()));
    assert_eq!(graph.size(), 1);
}

#[test]
fn gradient_descent_solves_single_prior() {
    let mut graph = Graph::<PriorFactor>::new();
    graph.add(Rc::new(PriorFactor::new(X(0).into(), 5.0, 0.1).unwrap()));

    let mut initial = Values::new();
    initial.insert(X(0).into(), 0.0).unwrap();

    let optimizer = GradientDescentOptimizer::with_parameters(Parameters {
        step_size: 0.1,
        max_iterations: 200,
        ..Parameters::default()
    });

    let result = optimizer.optimize(&graph, &initial);
    assert!(result.converged);
    assert!((*result.values.at::<f64>(X(0).into()).unwrap() - 5.0).abs() < 1e-2);
    assert!(result.final_error < 1e-4);
}

#[test]
fn gradient_descent_solves_small_chain() {
    enum AnyFactor {
        Prior(PriorFactor),
        Between(BetweenFactor),
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
        PriorFactor::new(X(0).into(), 0.0, 0.1).unwrap(),
    )));
    graph.add(Rc::new(AnyFactor::Between(
        BetweenFactor::new(X(0).into(), X(1).into(), 1.0, 0.1).unwrap(),
    )));
    graph.add(Rc::new(AnyFactor::Between(
        BetweenFactor::new(X(1).into(), X(2).into(), 1.0, 0.1).unwrap(),
    )));

    let mut initial = Values::new();
    initial.insert(X(0).into(), 0.5).unwrap();
    initial.insert(X(1).into(), 0.5).unwrap();
    initial.insert(X(2).into(), 0.5).unwrap();

    let optimizer = GradientDescentOptimizer::with_parameters(Parameters {
        step_size: 0.05,
        max_iterations: 1000,
        ..Parameters::default()
    });
    let result = optimizer.optimize(&graph, &initial);

    assert!(result.converged);
    assert!((*result.values.at::<f64>(X(0).into()).unwrap() - 0.0).abs() < 1e-2);
    assert!((*result.values.at::<f64>(X(1).into()).unwrap() - 1.0).abs() < 1e-2);
    assert!((*result.values.at::<f64>(X(2).into()).unwrap() - 2.0).abs() < 1e-2);
}

#[test]
fn gradient_descent_handles_loop_closure_and_longer_chains() {
    let mut chain: Graph<dyn NonlinearFactor> = Graph::new();
    chain.add(Rc::new(PriorFactor::new(X(0).into(), 0.0, 0.1).unwrap()) as Rc<dyn NonlinearFactor>);
    for i in 0..4 {
        chain.add(
            Rc::new(BetweenFactor::new(X(i).into(), X(i + 1).into(), 1.0, 0.1).unwrap())
                as Rc<dyn NonlinearFactor>,
        );
    }

    let mut initial = Values::new();
    for i in 0..=4 {
        initial.insert(X(i).into(), 10.0).unwrap();
    }

    let chain_result = GradientDescentOptimizer::with_parameters(Parameters {
        step_size: 0.05,
        max_iterations: 1000,
        ..Parameters::default()
    })
    .optimize(&chain, &initial);
    assert!(chain_result.converged);
    for i in 0..=4 {
        assert!((*chain_result.values.at::<f64>(X(i).into()).unwrap() - i as f64).abs() < 1e-2);
    }

    let mut loop_graph: Graph<dyn NonlinearFactor> = Graph::new();
    loop_graph
        .add(Rc::new(PriorFactor::new(X(0).into(), 0.0, 0.1).unwrap()) as Rc<dyn NonlinearFactor>);
    loop_graph.add(
        Rc::new(BetweenFactor::new(X(0).into(), X(1).into(), 1.0, 0.1).unwrap())
            as Rc<dyn NonlinearFactor>,
    );
    loop_graph.add(
        Rc::new(BetweenFactor::new(X(1).into(), X(2).into(), 1.0, 0.1).unwrap())
            as Rc<dyn NonlinearFactor>,
    );
    loop_graph.add(
        Rc::new(BetweenFactor::new(X(2).into(), X(0).into(), -2.0, 0.1).unwrap())
            as Rc<dyn NonlinearFactor>,
    );

    let mut loop_initial = Values::new();
    loop_initial.insert(X(0).into(), 0.1).unwrap();
    loop_initial.insert(X(1).into(), 0.9).unwrap();
    loop_initial.insert(X(2).into(), 2.1).unwrap();

    let loop_result = GradientDescentOptimizer::with_parameters(Parameters {
        step_size: 0.05,
        max_iterations: 500,
        ..Parameters::default()
    })
    .optimize(&loop_graph, &loop_initial);
    assert!(loop_result.converged);
    assert!((*loop_result.values.at::<f64>(X(0).into()).unwrap() - 0.0).abs() < 1e-2);
    assert!((*loop_result.values.at::<f64>(X(1).into()).unwrap() - 1.0).abs() < 1e-2);
    assert!((*loop_result.values.at::<f64>(X(2).into()).unwrap() - 2.0).abs() < 1e-2);
}

#[test]
fn gradient_descent_handles_overdetermined_weighted_and_negative_systems() {
    let mut overdetermined: Graph<dyn NonlinearFactor> = Graph::new();
    overdetermined
        .add(Rc::new(PriorFactor::new(X(0).into(), 5.0, 0.1).unwrap()) as Rc<dyn NonlinearFactor>);
    overdetermined
        .add(Rc::new(PriorFactor::new(X(0).into(), 6.0, 0.1).unwrap()) as Rc<dyn NonlinearFactor>);

    let mut initial = Values::new();
    initial.insert(X(0).into(), 0.0).unwrap();
    let over_result = GradientDescentOptimizer::with_parameters(Parameters {
        step_size: 0.1,
        max_iterations: 200,
        ..Parameters::default()
    })
    .optimize(&overdetermined, &initial);
    assert!(over_result.converged);
    assert!((*over_result.values.at::<f64>(X(0).into()).unwrap() - 5.5).abs() < 1e-2);

    let mut weighted: Graph<dyn NonlinearFactor> = Graph::new();
    weighted
        .add(Rc::new(PriorFactor::new(X(0).into(), 5.0, 0.1).unwrap()) as Rc<dyn NonlinearFactor>);
    weighted
        .add(Rc::new(PriorFactor::new(X(0).into(), 10.0, 1.0).unwrap()) as Rc<dyn NonlinearFactor>);
    let weighted_result = GradientDescentOptimizer::with_parameters(Parameters {
        step_size: 0.1,
        max_iterations: 300,
        ..Parameters::default()
    })
    .optimize(&weighted, &initial);
    let weighted_x = *weighted_result.values.at::<f64>(X(0).into()).unwrap();
    assert!(weighted_result.converged);
    assert!(weighted_x > 5.0);
    assert!(weighted_x < 7.0);

    let mut negative: Graph<dyn NonlinearFactor> = Graph::new();
    negative
        .add(Rc::new(PriorFactor::new(X(0).into(), -5.0, 0.1).unwrap()) as Rc<dyn NonlinearFactor>);
    negative.add(
        Rc::new(BetweenFactor::new(X(0).into(), X(1).into(), -2.0, 0.1).unwrap())
            as Rc<dyn NonlinearFactor>,
    );
    let mut negative_initial = Values::new();
    negative_initial.insert(X(0).into(), 0.0).unwrap();
    negative_initial.insert(X(1).into(), 0.0).unwrap();
    let negative_result = GradientDescentOptimizer::with_parameters(Parameters {
        step_size: 0.05,
        max_iterations: 500,
        ..Parameters::default()
    })
    .optimize(&negative, &negative_initial);
    assert!(negative_result.converged);
    assert!((*negative_result.values.at::<f64>(X(0).into()).unwrap() + 5.0).abs() < 1e-2);
    assert!((*negative_result.values.at::<f64>(X(1).into()).unwrap() + 7.0).abs() < 1e-2);
}

#[test]
fn gradient_descent_reports_result_fields_and_limits() {
    let mut graph: Graph<dyn NonlinearFactor> = Graph::new();
    graph.add(Rc::new(PriorFactor::new(X(0).into(), 5.0, 0.1).unwrap()) as Rc<dyn NonlinearFactor>);

    let mut optimal = Values::new();
    optimal.insert(X(0).into(), 5.0).unwrap();
    let optimal_result = GradientDescentOptimizer::new().optimize(&graph, &optimal);
    assert!(optimal_result.iterations <= 1);
    assert!(optimal_result.converged);
    assert!(optimal_result.final_error < 1e-6);
    assert!(optimal_result.values.exists(X(0).into()));
    assert!(optimal_result.gradient_norm >= 0.0);

    let mut initial = Values::new();
    initial.insert(X(0).into(), 0.0).unwrap();
    let initial_error: f64 = graph.iter().map(|factor| factor.error(&initial)).sum();
    let result = GradientDescentOptimizer::new().optimize(&graph, &initial);
    assert!(result.final_error < initial_error);
    assert!(result.values.exists(X(0).into()));
    assert!(result.iterations <= GradientDescentOptimizer::new().parameters().max_iterations);

    let mut hard: Graph<dyn NonlinearFactor> = Graph::new();
    hard.add(Rc::new(PriorFactor::new(X(0).into(), 100.0, 0.1).unwrap()) as Rc<dyn NonlinearFactor>);
    let limited = GradientDescentOptimizer::with_parameters(Parameters {
        max_iterations: 5,
        step_size: 0.01,
        ..Parameters::default()
    })
    .optimize(&hard, &initial);
    assert!(limited.iterations <= 5);
    assert!(*limited.values.at::<f64>(X(0).into()).unwrap() > 0.0);
}

#[test]
fn trait_object_graph_accepts_mixed_nonlinear_factors() {
    let mut graph: Graph<dyn NonlinearFactor> = Graph::new();
    graph.add(Rc::new(PriorFactor::new(X(0).into(), 0.0, 1.0).unwrap()) as Rc<dyn NonlinearFactor>);
    graph.add(
        Rc::new(BetweenFactor::new(X(0).into(), X(1).into(), 1.0, 1.0).unwrap())
            as Rc<dyn NonlinearFactor>,
    );

    let mut values = Values::new();
    values.insert(X(0).into(), 0.0).unwrap();
    values.insert(X(1).into(), 1.0).unwrap();

    let total_error: f64 = graph.iter().map(|factor| factor.error(&values)).sum();
    assert!(total_error.abs() < 1e-9);
}

#[test]
fn prior_linearization_matches_expected_scalar_jacobian() {
    let factor = PriorFactor::new(X(0).into(), 5.0, 2.0).unwrap();

    let mut values = Values::new();
    values.insert(X(0).into(), 9.0).unwrap();

    let linear = factor.linearize(&values).unwrap();
    assert_eq!(linear.dim(), 1);
    assert!((linear.b()[0] - 2.0).abs() < 1e-9);
    assert!((linear.jacobian(X(0).into()).unwrap()[(0, 0)] - 0.5).abs() < 1e-6);
}

#[test]
fn se2_linearization_matches_expected_shape_and_scaling() {
    let factor = SE2PriorFactor::new(
        X(0).into(),
        SE2d::new(0.0, 1.0, 2.0),
        Vec3d::from_array([0.1, 0.1, 0.1]),
    )
    .unwrap();

    let mut values = Values::new();
    values
        .insert(X(0).into(), SE2d::new(0.1, 1.5, 2.5))
        .unwrap();

    let linear = factor.linearize(&values).unwrap();
    assert_eq!(linear.size(), 1);
    assert_eq!(linear.dim(), 3);

    let b = linear.b();
    assert!(b[0].abs() > 1.0);
    assert!(b[1].abs() > 1.0);
    assert!(b[2].abs() > 0.5);

    let jacobian = linear.jacobian(X(0).into()).unwrap();
    assert_eq!(jacobian.rows(), 3);
    assert_eq!(jacobian.cols(), 3);
    assert!((jacobian[(0, 0)] - 10.0).abs() < 1.0);
    assert!((jacobian[(1, 1)] - 10.0).abs() < 1.0);
    assert!((jacobian[(2, 2)] - 10.0).abs() < 1.0);
}

#[test]
fn gauss_newton_solves_scalar_chain() {
    let mut graph: Graph<dyn NonlinearFactor> = Graph::new();
    graph.add(Rc::new(PriorFactor::new(X(0).into(), 0.0, 0.1).unwrap()) as Rc<dyn NonlinearFactor>);
    graph.add(
        Rc::new(BetweenFactor::new(X(0).into(), X(1).into(), 1.0, 0.1).unwrap())
            as Rc<dyn NonlinearFactor>,
    );
    graph.add(
        Rc::new(BetweenFactor::new(X(1).into(), X(2).into(), 1.0, 0.1).unwrap())
            as Rc<dyn NonlinearFactor>,
    );

    let mut initial = Values::new();
    initial.insert(X(0).into(), 10.0).unwrap();
    initial.insert(X(1).into(), 10.0).unwrap();
    initial.insert(X(2).into(), 10.0).unwrap();

    let optimizer = GaussNewtonOptimizer::with_parameters(GaussNewtonParameters {
        max_iterations: 50,
        ..GaussNewtonParameters::default()
    });
    let result = optimizer.optimize(&graph, &initial);

    assert!(result.converged);
    assert!((*result.values.at::<f64>(X(0).into()).unwrap() - 0.0).abs() < 1e-6);
    assert!((*result.values.at::<f64>(X(1).into()).unwrap() - 1.0).abs() < 1e-6);
    assert!((*result.values.at::<f64>(X(2).into()).unwrap() - 2.0).abs() < 1e-6);
}

#[test]
fn levenberg_marquardt_solves_scalar_chain() {
    let mut graph: Graph<dyn NonlinearFactor> = Graph::new();
    graph.add(Rc::new(PriorFactor::new(X(0).into(), 0.0, 0.1).unwrap()) as Rc<dyn NonlinearFactor>);
    graph.add(
        Rc::new(BetweenFactor::new(X(0).into(), X(1).into(), 1.0, 0.1).unwrap())
            as Rc<dyn NonlinearFactor>,
    );
    graph.add(
        Rc::new(BetweenFactor::new(X(1).into(), X(2).into(), 1.0, 0.1).unwrap())
            as Rc<dyn NonlinearFactor>,
    );

    let mut initial = Values::new();
    initial.insert(X(0).into(), -5.0).unwrap();
    initial.insert(X(1).into(), -5.0).unwrap();
    initial.insert(X(2).into(), -5.0).unwrap();

    let optimizer = LevenbergMarquardtOptimizer::with_parameters(LevenbergMarquardtParameters {
        max_iterations: 50,
        initial_lambda: 1e-2,
        ..LevenbergMarquardtParameters::default()
    });
    let result = optimizer.optimize(&graph, &initial);

    assert!(result.converged);
    assert!((*result.values.at::<f64>(X(0).into()).unwrap() - 0.0).abs() < 1e-5);
    assert!((*result.values.at::<f64>(X(1).into()).unwrap() - 1.0).abs() < 1e-5);
    assert!((*result.values.at::<f64>(X(2).into()).unwrap() - 2.0).abs() < 1e-5);
}

#[test]
fn gauss_newton_reduces_se2_graph_error() {
    let mut graph: Graph<dyn NonlinearFactor> = Graph::new();
    graph.add(Rc::new(
        SE2PriorFactor::new(
            X(0).into(),
            SE2d::new(0.0, 0.0, 0.0),
            Vec3d::from_array([0.1, 0.1, 0.1]),
        )
        .unwrap(),
    ) as Rc<dyn NonlinearFactor>);
    graph.add(Rc::new(
        SE2BetweenFactor::new(
            X(0).into(),
            X(1).into(),
            SE2d::new(0.0, 1.0, 0.0),
            Vec3d::from_array([0.1, 0.1, 0.1]),
        )
        .unwrap(),
    ) as Rc<dyn NonlinearFactor>);

    let mut initial = Values::new();
    initial
        .insert(X(0).into(), SE2d::new(0.2, -0.5, 0.1))
        .unwrap();
    initial
        .insert(X(1).into(), SE2d::new(-0.1, 1.8, -0.2))
        .unwrap();
    let initial_error: f64 = graph.iter().map(|factor| factor.error(&initial)).sum();

    let optimizer = GaussNewtonOptimizer::new();
    let result = optimizer.optimize(&graph, &initial);
    let final_error: f64 = graph
        .iter()
        .map(|factor| factor.error(&result.values))
        .sum();

    assert!(result.converged);
    assert!(final_error < initial_error);
    assert!(final_error < 1e-6);
}

#[test]
fn factor_graph_adapter_flattens_values_and_builds_jacobian() {
    let mut graph: Graph<dyn NonlinearFactor> = Graph::new();
    graph.add(Rc::new(PriorFactor::new(X(0).into(), 5.0, 2.0).unwrap()) as Rc<dyn NonlinearFactor>);
    graph.add(
        Rc::new(BetweenFactor::new(X(0).into(), X(1).into(), 1.0, 4.0).unwrap())
            as Rc<dyn NonlinearFactor>,
    );

    let mut values = Values::new();
    values.insert(X(0).into(), 7.0).unwrap();
    values.insert(X(1).into(), 10.0).unwrap();

    let adapter = FactorGraphAdapter::new(&graph, &values).unwrap();
    assert_eq!(adapter.param_dim(), 2);
    assert_eq!(adapter.residual_dim(), 2);
    assert_eq!(adapter.ordering().len(), 2);

    let params = adapter.values_to_params(&values).unwrap();
    assert_eq!(params.size(), 2);
    assert!((params[0] - 7.0).abs() < 1e-9);
    assert!((params[1] - 10.0).abs() < 1e-9);

    let residuals = adapter.residuals(&params).unwrap();
    assert!((residuals[0] - 1.0).abs() < 1e-9);
    assert!((residuals[1] - 0.5).abs() < 1e-6);

    let jacobian = adapter.jacobian(&params).unwrap();
    assert_eq!(jacobian.rows(), 2);
    assert_eq!(jacobian.cols(), 2);
    assert!((jacobian[(0, 0)] - 0.5).abs() < 1e-6);
    assert!((jacobian[(0, 1)] - 0.0).abs() < 1e-9);
    assert!((jacobian[(1, 0)] + 0.25).abs() < 1e-6);
    assert!((jacobian[(1, 1)] - 0.25).abs() < 1e-6);

    let reconstructed = adapter.params_to_values(&params).unwrap();
    assert!((*reconstructed.at::<f64>(X(0).into()).unwrap() - 7.0).abs() < 1e-9);
    assert!((*reconstructed.at::<f64>(X(1).into()).unwrap() - 10.0).abs() < 1e-9);
}

#[test]
fn optimizers_expose_and_apply_parameters() {
    let mut gd = GradientDescentOptimizer::new();
    assert_eq!(gd.parameters().max_iterations, 100);
    assert!((gd.parameters().step_size - 0.01).abs() < 1e-12);
    let custom_gd = Parameters {
        max_iterations: 25,
        step_size: 0.25,
        tolerance: 1e-5,
        h: 1e-6,
        verbose: true,
    };
    gd.set_parameters(custom_gd.clone());
    assert_eq!(gd.parameters().max_iterations, 25);
    assert!((gd.parameters().step_size - 0.25).abs() < 1e-12);
    assert_eq!(gd.parameters().verbose, custom_gd.verbose);

    let mut gn = GaussNewtonOptimizer::new();
    assert_eq!(gn.parameters().max_iterations, 100);
    let custom_gn = GaussNewtonParameters {
        max_iterations: 10,
        tolerance: 1e-5,
        min_step_norm: 1e-8,
        verbose: true,
    };
    gn.set_parameters(custom_gn.clone());
    assert_eq!(gn.parameters().max_iterations, 10);
    assert_eq!(gn.parameters().verbose, custom_gn.verbose);

    let mut lm = LevenbergMarquardtOptimizer::new();
    assert_eq!(lm.parameters().max_iterations, 100);
    let custom_lm = LevenbergMarquardtParameters {
        max_iterations: 12,
        tolerance: 1e-5,
        min_step_norm: 1e-8,
        initial_lambda: 1e-2,
        lambda_factor: 5.0,
        min_lambda: 1e-8,
        max_lambda: 1e6,
        verbose: true,
    };
    lm.set_parameters(custom_lm.clone());
    assert_eq!(lm.parameters().max_iterations, 12);
    assert!((lm.parameters().initial_lambda - 1e-2).abs() < 1e-12);
    assert_eq!(lm.parameters().verbose, custom_lm.verbose);
}

#[test]
fn optimizer_result_aliases_match_shared_result_shape() {
    let mut values = Values::new();
    values.insert(1, 1.0f64).unwrap();

    let gd: GradientDescentResult = graphix::factor::OptimizationResult {
        values: values.clone(),
        final_error: 1.0,
        iterations: 2,
        converged: false,
        gradient_norm: 3.0,
    };
    let gn: GaussNewtonResult = graphix::factor::OptimizationResult {
        values: values.clone(),
        final_error: 0.5,
        iterations: 4,
        converged: true,
        gradient_norm: 0.1,
    };
    let lm: LevenbergMarquardtResult = graphix::factor::OptimizationResult {
        values,
        final_error: 0.25,
        iterations: 5,
        converged: true,
        gradient_norm: 0.01,
    };

    assert_eq!(gd.iterations, 2);
    assert!(gn.converged);
    assert!((lm.final_error - 0.25).abs() < 1e-12);
}

#[test]
fn optinum_adapters_match_native_optimizers_on_scalar_chain() {
    let mut graph: Graph<dyn NonlinearFactor> = Graph::new();
    graph.add(Rc::new(PriorFactor::new(X(0).into(), 0.0, 0.1).unwrap()) as Rc<dyn NonlinearFactor>);
    graph.add(
        Rc::new(BetweenFactor::new(X(0).into(), X(1).into(), 1.0, 0.1).unwrap())
            as Rc<dyn NonlinearFactor>,
    );
    graph.add(
        Rc::new(BetweenFactor::new(X(1).into(), X(2).into(), 1.0, 0.1).unwrap())
            as Rc<dyn NonlinearFactor>,
    );

    let mut initial = Values::new();
    initial.insert(X(0).into(), 3.0).unwrap();
    initial.insert(X(1).into(), 3.0).unwrap();
    initial.insert(X(2).into(), 3.0).unwrap();

    let gd_params = Parameters {
        step_size: 0.05,
        max_iterations: 500,
        ..Parameters::default()
    };
    let gd_native =
        GradientDescentOptimizer::with_parameters(gd_params.clone()).optimize(&graph, &initial);
    let gd_adapter = OptinumGradientDescent {
        max_iterations: gd_params.max_iterations,
        step_size: gd_params.step_size,
        tolerance: gd_params.tolerance,
        h: gd_params.h,
        use_adam: true,
        verbose: gd_params.verbose,
    }
    .optimize(&graph, &initial);
    assert!(gd_adapter.converged);
    assert!((gd_native.final_error - gd_adapter.final_error).abs() < 1e-9);

    let gn_params = GaussNewtonParameters {
        max_iterations: 50,
        ..GaussNewtonParameters::default()
    };
    let gn_native =
        GaussNewtonOptimizer::with_parameters(gn_params.clone()).optimize(&graph, &initial);
    let gn_adapter = OptinumGaussNewton {
        max_iterations: gn_params.max_iterations,
        tolerance: gn_params.tolerance,
        min_step_norm: gn_params.min_step_norm,
        verbose: gn_params.verbose,
    }
    .optimize(&graph, &initial);
    assert!(gn_adapter.converged);
    assert!((gn_native.final_error - gn_adapter.final_error).abs() < 1e-9);

    let lm_params = LevenbergMarquardtParameters {
        max_iterations: 50,
        initial_lambda: 1e-2,
        ..LevenbergMarquardtParameters::default()
    };
    let lm_native =
        LevenbergMarquardtOptimizer::with_parameters(lm_params.clone()).optimize(&graph, &initial);
    let lm_adapter = OptinumLevenbergMarquardt {
        max_iterations: lm_params.max_iterations,
        tolerance: lm_params.tolerance,
        min_step_norm: lm_params.min_step_norm,
        initial_lambda: lm_params.initial_lambda,
        lambda_factor: lm_params.lambda_factor,
        min_lambda: lm_params.min_lambda,
        max_lambda: lm_params.max_lambda,
        verbose: lm_params.verbose,
    }
    .optimize(&graph, &initial);
    assert!(lm_adapter.converged);
    assert!((lm_native.final_error - lm_adapter.final_error).abs() < 1e-9);
}

fn build_robust_slam_graph(
    robust_loss: Option<std::sync::Arc<dyn graphix::factor::LossFunction>>,
) -> (Graph<dyn NonlinearFactor>, Values) {
    use std::f64::consts::PI;

    let mut graph: Graph<dyn NonlinearFactor> = Graph::new();
    graph.add(Rc::new(
        SE2PriorFactor::new(
            X(0).into(),
            SE2d::new(0.0, 0.0, 0.0),
            Vec3d::from_array([0.01, 0.01, 0.01]),
        )
        .unwrap(),
    ) as Rc<dyn NonlinearFactor>);

    let odom_sigma = Vec3d::from_array([0.1, 0.1, 0.05]);
    let loop_sigma = Vec3d::from_array([0.05, 0.05, 0.05]);
    let edges = [
        (X(0), X(1), SE2d::new(0.0, 2.0, 0.0)),
        (X(1), X(2), SE2d::new(PI / 2.0, 0.0, 2.0)),
        (X(2), X(3), SE2d::new(PI / 2.0, -2.0, 0.0)),
        (X(3), X(4), SE2d::new(PI / 2.0, 0.0, -2.0)),
    ];
    for (a, b, measurement) in edges {
        graph.add(Rc::new(
            SE2BetweenFactor::new(a.into(), b.into(), measurement, odom_sigma).unwrap(),
        ) as Rc<dyn NonlinearFactor>);
    }
    graph.add(Rc::new(
        SE2BetweenFactor::new(
            X(4).into(),
            X(0).into(),
            SE2d::new(PI / 2.0, 0.0, 0.0),
            loop_sigma,
        )
        .unwrap(),
    ) as Rc<dyn NonlinearFactor>);

    let mut outlier = SE2BetweenFactor::new(
        X(2).into(),
        X(0).into(),
        SE2d::new(-PI, -0.5, -0.5),
        loop_sigma,
    )
    .unwrap();
    if let Some(loss) = robust_loss {
        outlier.set_loss_function(loss);
    }
    graph.add(Rc::new(outlier) as Rc<dyn NonlinearFactor>);

    let mut initial = Values::new();
    initial
        .insert(X(0).into(), SE2d::new(0.0, 0.0, 0.0))
        .unwrap();
    initial
        .insert(X(1).into(), SE2d::new(0.0, 2.0, 0.0))
        .unwrap();
    initial
        .insert(X(2).into(), SE2d::new(std::f64::consts::PI / 2.0, 2.0, 2.0))
        .unwrap();
    initial
        .insert(X(3).into(), SE2d::new(std::f64::consts::PI, 0.0, 2.0))
        .unwrap();
    initial
        .insert(
            X(4).into(),
            SE2d::new(3.0 * std::f64::consts::PI / 2.0, 0.0, 0.0),
        )
        .unwrap();

    (graph, initial)
}

#[test]
fn robust_losses_reduce_outlier_impact_in_se2_slam() {
    let optimizer = LevenbergMarquardtOptimizer::with_parameters(LevenbergMarquardtParameters {
        max_iterations: 100,
        initial_lambda: 1e-3,
        ..LevenbergMarquardtParameters::default()
    });

    let (plain_graph, initial) = build_robust_slam_graph(None);
    let plain = optimizer.optimize(&plain_graph, &initial);
    let plain_pose = *plain.values.at::<SE2d>(X(2).into()).unwrap();
    let plain_dist = ((plain_pose.x() - 2.0).powi(2) + (plain_pose.y() - 2.0).powi(2)).sqrt();

    let robust_losses = [huber_loss(1.345), cauchy_loss(2.3849), tukey_loss(4.6851)];
    let mut best_robust_dist = f64::INFINITY;
    for loss in robust_losses {
        let (graph, initial) = build_robust_slam_graph(Some(loss));
        let result = optimizer.optimize(&graph, &initial);
        assert!(result.final_error.is_finite());
        let pose = *result.values.at::<SE2d>(X(2).into()).unwrap();
        let dist = ((pose.x() - 2.0).powi(2) + (pose.y() - 2.0).powi(2)).sqrt();
        best_robust_dist = best_robust_dist.min(dist);
    }

    assert!(best_robust_dist <= plain_dist);
}

#[test]
fn robust_linearization_downweights_large_outliers() {
    let plain = BetweenFactor::new(X(0).into(), X(1).into(), 1.0, 0.1).unwrap();
    let mut robust = plain.clone();
    robust.set_loss_function(huber_loss(1.345));

    let mut values = Values::new();
    values.insert(X(0).into(), 0.0).unwrap();
    values.insert(X(1).into(), 10.0).unwrap();

    let plain_linear = plain.linearize(&values).unwrap();
    let robust_linear = robust.linearize(&values).unwrap();

    let plain_norm = plain_linear.b().iter().map(|v| v * v).sum::<f64>().sqrt();
    let robust_norm = robust_linear.b().iter().map(|v| v * v).sum::<f64>().sqrt();
    assert!(robust_norm < plain_norm);
    assert!(
        robust_linear.jacobian(X(0).into()).unwrap()[(0, 0)].abs()
            < plain_linear.jacobian(X(0).into()).unwrap()[(0, 0)].abs()
    );
}
