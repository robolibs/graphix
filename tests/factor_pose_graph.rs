use std::f64::consts::PI;

use glam::DVec2;

use graphix::X;
use graphix::factor::{PoseGraph2d, SE2d, cauchy_loss};

#[test]
fn pose_graph_builder_collects_graph_and_initial_values() {
    let mut pose_graph = PoseGraph2d::new();
    pose_graph
        .insert_pose(X(0).into(), SE2d::identity())
        .unwrap()
        .insert_pose_xytheta(X(1).into(), 2.0, 0.0, 0.0)
        .unwrap()
        .add_prior(X(0).into(), SE2d::identity(), DVec2::splat(0.01), 0.01)
        .unwrap()
        .add_between_xytheta(
            X(0).into(),
            X(1).into(),
            2.0,
            0.0,
            0.0,
            DVec2::splat(0.1),
            0.05,
        )
        .unwrap()
        .add_between_xytheta_with_loss(
            X(1).into(),
            X(0).into(),
            -2.1,
            0.0,
            0.0,
            DVec2::splat(0.2),
            0.1,
            cauchy_loss(2.0),
        )
        .unwrap();

    assert_eq!(pose_graph.graph().size(), 3);
    assert_eq!(pose_graph.initial_values().size(), 2);
    assert_eq!(
        *pose_graph.initial_values().at::<SE2d>(X(1).into()).unwrap(),
        SE2d::new(0.0, 2.0, 0.0)
    );
}

#[test]
fn pose_graph_builder_supports_into_parts_for_optimizers() {
    let mut pose_graph = PoseGraph2d::new();
    pose_graph
        .insert_pose_xytheta(X(0).into(), 0.0, 0.0, 0.0)
        .unwrap()
        .insert_pose_xytheta(X(1).into(), 2.0, 0.0, 0.0)
        .unwrap()
        .add_prior_xytheta(X(0).into(), 0.0, 0.0, 0.0, DVec2::splat(0.01), 0.01)
        .unwrap()
        .add_between(
            X(0).into(),
            X(1).into(),
            SE2d::from_translation_angle(DVec2::new(2.0, 0.0), PI / 2.0),
            DVec2::splat(0.1),
            0.05,
        )
        .unwrap();

    let (graph, initial) = pose_graph.into_parts();
    assert_eq!(graph.size(), 2);
    assert_eq!(initial.size(), 2);
}
