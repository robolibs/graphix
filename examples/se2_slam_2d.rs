use std::f64::consts::FRAC_PI_2;
use std::rc::Rc;

use glam::DVec2;

use graphix::X;
use graphix::factor::{Graph, NonlinearFactor, SE2BetweenFactor, SE2PriorFactor, SE2d, Values};

fn main() {
    let mut graph: Graph<dyn NonlinearFactor> = Graph::new();
    graph.add(Rc::new(
        SE2PriorFactor::from_translation_sigmas(
            X(0).into(),
            SE2d::identity(),
            DVec2::splat(0.1),
            0.1,
        )
        .unwrap(),
    ) as Rc<dyn NonlinearFactor>);
    graph.add(Rc::new(
        SE2BetweenFactor::from_translation_sigmas(
            X(0).into(),
            X(1).into(),
            SE2d::from_translation_angle(DVec2::new(1.0, 0.0), 0.0),
            DVec2::splat(0.2),
            0.1,
        )
        .unwrap(),
    ) as Rc<dyn NonlinearFactor>);
    graph.add(Rc::new(
        SE2BetweenFactor::from_translation_sigmas(
            X(1).into(),
            X(2).into(),
            SE2d::from_translation_angle(DVec2::ZERO, FRAC_PI_2),
            DVec2::splat(0.2),
            0.1,
        )
        .unwrap(),
    ) as Rc<dyn NonlinearFactor>);

    let mut values = Values::new();
    values.insert(X(0).into(), SE2d::identity()).unwrap();
    values
        .insert(
            X(1).into(),
            SE2d::from_translation_angle(DVec2::new(1.0, 0.0), 0.0),
        )
        .unwrap();
    values
        .insert(
            X(2).into(),
            SE2d::from_translation_angle(DVec2::new(1.0, 0.0), FRAC_PI_2),
        )
        .unwrap();

    let total_error: f64 = graph.iter().map(|factor| factor.error(&values)).sum();
    println!("se2 slam error: {:.6}", total_error);
}
