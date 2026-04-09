use std::f64::consts::PI;
use std::rc::Rc;

use graphix::X;
use graphix::factor::{
    GaussNewtonOptimizer, GradientDescentOptimizer, Graph, LevenbergMarquardtOptimizer,
    NonlinearFactor, OptinumGaussNewton, OptinumGradientDescent, OptinumLevenbergMarquardt,
    SE2BetweenFactor, SE2PriorFactor, SE2d, Values, Vec3d,
};

fn build_problem() -> (Graph<dyn NonlinearFactor>, Values) {
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
        (X(0), X(1), SE2d::new(0.0, 2.05, 0.0)),
        (X(1), X(2), SE2d::new(PI / 2.0, 0.0, 2.03)),
        (X(2), X(3), SE2d::new(PI / 2.0, -1.98, 0.0)),
        (X(3), X(4), SE2d::new(PI / 2.0, 0.0, -2.02)),
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

    let mut initial = Values::new();
    initial
        .insert(X(0).into(), SE2d::new(0.0, 0.0, 0.0))
        .unwrap();
    initial
        .insert(X(1).into(), SE2d::new(0.0, 2.05, 0.0))
        .unwrap();
    initial
        .insert(X(2).into(), SE2d::new(PI / 2.0, 2.05, 2.03))
        .unwrap();
    initial
        .insert(X(3).into(), SE2d::new(PI, 0.07, 2.03))
        .unwrap();
    initial
        .insert(X(4).into(), SE2d::new(3.0 * PI / 2.0, 0.07, 0.01))
        .unwrap();

    (graph, initial)
}

fn pose_error_mm(pose: SE2d, expected_x: f64, expected_y: f64) -> f64 {
    ((pose.x() - expected_x).powi(2) + (pose.y() - expected_y).powi(2)).sqrt() * 1000.0
}

fn print_result(label: &str, iterations: usize, error: f64, pose: SE2d) {
    println!(
        "{label:28} {:>6} iters  error={error:>10.5}  x4=({:>7.3}, {:>7.3})  drift={:>8.2} mm",
        iterations,
        pose.x(),
        pose.y(),
        pose_error_mm(pose, 0.0, 0.0),
    );
}

fn main() {
    let (graph, initial) = build_problem();

    let gd = GradientDescentOptimizer::with_parameters(graphix::factor::Parameters {
        step_size: 0.05,
        max_iterations: 200,
        ..Default::default()
    })
    .optimize(&graph, &initial);
    let gn = GaussNewtonOptimizer::new().optimize(&graph, &initial);
    let lm = LevenbergMarquardtOptimizer::new().optimize(&graph, &initial);
    let ogd = OptinumGradientDescent {
        step_size: 0.05,
        max_iterations: 200,
        use_adam: true,
        ..Default::default()
    }
    .optimize(&graph, &initial);
    let ogn = OptinumGaussNewton::default().optimize(&graph, &initial);
    let olm = OptinumLevenbergMarquardt::default().optimize(&graph, &initial);

    println!("SE2 optimizer comparison");
    println!("method                        iterations  result");
    print_result(
        "gradient descent",
        gd.iterations,
        gd.final_error,
        *gd.values.at::<SE2d>(X(4).into()).unwrap(),
    );
    print_result(
        "gauss-newton",
        gn.iterations,
        gn.final_error,
        *gn.values.at::<SE2d>(X(4).into()).unwrap(),
    );
    print_result(
        "levenberg-marquardt",
        lm.iterations,
        lm.final_error,
        *lm.values.at::<SE2d>(X(4).into()).unwrap(),
    );
    print_result(
        "optinum gradient descent",
        ogd.iterations,
        ogd.final_error,
        *ogd.values.at::<SE2d>(X(4).into()).unwrap(),
    );
    print_result(
        "optinum gauss-newton",
        ogn.iterations,
        ogn.final_error,
        *ogn.values.at::<SE2d>(X(4).into()).unwrap(),
    );
    print_result(
        "optinum levenberg-marquardt",
        olm.iterations,
        olm.final_error,
        *olm.values.at::<SE2d>(X(4).into()).unwrap(),
    );
}
