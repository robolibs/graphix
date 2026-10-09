use std::f64::consts::PI;

use datapod::Point;

use graphix::X;
use graphix::factor::{
    LevenbergMarquardtOptimizer, PoseGraph2d, SE2d, cauchy_loss, huber_loss, tukey_loss,
};

fn p(x: f64, y: f64) -> Point {
    Point::new(x, y, 0.0)
}

fn distance_mm(pose: SE2d, x: f64, y: f64) -> f64 {
    ((pose.x() - x).powi(2) + (pose.y() - y).powi(2)).sqrt() * 1000.0
}

fn build_base_problem() -> PoseGraph2d {
    let mut problem = PoseGraph2d::new();
    problem
        .insert_pose(X(0).into(), SE2d::identity())
        .unwrap()
        .insert_pose(X(1).into(), SE2d::from_translation_angle(p(2.0, 0.0), 0.0))
        .unwrap()
        .insert_pose(
            X(2).into(),
            SE2d::from_translation_angle(p(2.0, 2.0), PI / 2.0),
        )
        .unwrap()
        .insert_pose(X(3).into(), SE2d::from_translation_angle(p(0.0, 2.0), PI))
        .unwrap()
        .insert_pose(
            X(4).into(),
            SE2d::from_translation_angle(Point::default(), 3.0 * PI / 2.0),
        )
        .unwrap()
        .add_prior(X(0).into(), SE2d::identity(), p(0.01, 0.01), 0.01)
        .unwrap()
        .add_between(
            X(0).into(),
            X(1).into(),
            SE2d::from_translation_angle(p(2.0, 0.0), 0.0),
            p(0.1, 0.1),
            0.05,
        )
        .unwrap()
        .add_between(
            X(1).into(),
            X(2).into(),
            SE2d::from_translation_angle(p(0.0, 2.0), PI / 2.0),
            p(0.1, 0.1),
            0.05,
        )
        .unwrap()
        .add_between(
            X(2).into(),
            X(3).into(),
            SE2d::from_translation_angle(p(-2.0, 0.0), PI / 2.0),
            p(0.1, 0.1),
            0.05,
        )
        .unwrap()
        .add_between(
            X(3).into(),
            X(4).into(),
            SE2d::from_translation_angle(p(0.0, -2.0), PI / 2.0),
            p(0.1, 0.1),
            0.05,
        )
        .unwrap()
        .add_between(
            X(4).into(),
            X(0).into(),
            SE2d::from_translation_angle(Point::default(), PI / 2.0),
            p(0.05, 0.05),
            0.05,
        )
        .unwrap();
    problem
}

fn main() {
    let optimizer = LevenbergMarquardtOptimizer::new();
    let base = build_base_problem();
    let mut plain = base.clone();
    plain
        .add_between(
            X(2).into(),
            X(0).into(),
            SE2d::from_translation_angle(p(-0.5, -0.5), -PI),
            p(0.05, 0.05),
            0.05,
        )
        .unwrap();

    let mut huber_graph = base.clone();
    huber_graph
        .add_between_with_loss(
            X(2).into(),
            X(0).into(),
            SE2d::from_translation_angle(p(-0.5, -0.5), -PI),
            p(0.05, 0.05),
            0.05,
            huber_loss(1.345),
        )
        .unwrap();

    let mut cauchy_graph = base.clone();
    cauchy_graph
        .add_between_with_loss(
            X(2).into(),
            X(0).into(),
            SE2d::from_translation_angle(p(-0.5, -0.5), -PI),
            p(0.05, 0.05),
            0.05,
            cauchy_loss(2.3849),
        )
        .unwrap();

    let mut tukey_graph = base;
    tukey_graph
        .add_between_with_loss(
            X(2).into(),
            X(0).into(),
            SE2d::from_translation_angle(p(-0.5, -0.5), -PI),
            p(0.05, 0.05),
            0.05,
            tukey_loss(4.6851),
        )
        .unwrap();

    let plain_result = optimizer.optimize(plain.graph(), plain.initial_values());
    let huber_result = optimizer.optimize(huber_graph.graph(), huber_graph.initial_values());
    let cauchy_result = optimizer.optimize(cauchy_graph.graph(), cauchy_graph.initial_values());
    let tukey_result = optimizer.optimize(tukey_graph.graph(), tukey_graph.initial_values());

    let plain_pose = *plain_result.values.at::<SE2d>(X(2).into()).unwrap();
    let huber_pose = *huber_result.values.at::<SE2d>(X(2).into()).unwrap();
    let cauchy_pose = *cauchy_result.values.at::<SE2d>(X(2).into()).unwrap();
    let tukey_pose = *tukey_result.values.at::<SE2d>(X(2).into()).unwrap();

    println!("method               x2 error (mm)   final cost");
    println!(
        "plain              {:>12.2}   {:>10.4}",
        distance_mm(plain_pose, 2.0, 2.0),
        plain_result.final_error
    );
    println!(
        "huber              {:>12.2}   {:>10.4}",
        distance_mm(huber_pose, 2.0, 2.0),
        huber_result.final_error
    );
    println!(
        "cauchy             {:>12.2}   {:>10.4}",
        distance_mm(cauchy_pose, 2.0, 2.0),
        cauchy_result.final_error
    );
    println!(
        "tukey              {:>12.2}   {:>10.4}",
        distance_mm(tukey_pose, 2.0, 2.0),
        tukey_result.final_error
    );
}
