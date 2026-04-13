use datapod::Point;

use graphix::X;
use graphix::factor::{GaussNewtonOptimizer, PoseGraph2d, SE2d};
use graphix::vertex::algorithms::dijkstra;
use graphix::vertex::spatial::{
    knn_graph_2d, mutual_nearest_correspondences_2d, nearest_vertex_2d,
};

fn p(x: f64, y: f64) -> Point {
    Point::new(x, y, 0.0)
}

fn heading(points: &[Point], index: usize) -> f64 {
    let current = points[index];
    let neighbor = if index + 1 < points.len() {
        points[index + 1]
    } else {
        points[index - 1]
    };
    let delta = neighbor - current;
    delta.y.atan2(delta.x)
}

fn mean_position_error(poses: &[SE2d], expected: &[Point]) -> f64 {
    poses
        .iter()
        .zip(expected.iter())
        .map(|(pose, point)| ((pose.x() - point.x).powi(2) + (pose.y() - point.y).powi(2)).sqrt())
        .sum::<f64>()
        / poses.len() as f64
}

#[test]
fn spatial_route_can_seed_and_optimize_pose_graph() {
    let points = vec![
        p(0.0, 0.0),
        p(1.0, 0.1),
        p(2.0, 0.0),
        p(3.0, 0.4),
        p(4.0, 0.9),
        p(5.0, 1.1),
        p(2.2, 1.5),
        p(3.5, 1.7),
    ];

    let graph = knn_graph_2d(points, 3, |p| *p).unwrap();
    let start = nearest_vertex_2d(&graph, p(-0.2, 0.0), |_, p| *p)
        .unwrap()
        .unwrap()
        .0;
    let goal = nearest_vertex_2d(&graph, p(5.1, 1.0), |_, p| *p)
        .unwrap()
        .unwrap()
        .0;

    let route = dijkstra(&graph, start, goal);
    assert!(route.found);
    assert_eq!(
        route
            .path
            .iter()
            .map(|vertex| vertex.value())
            .collect::<Vec<_>>(),
        vec![0, 2, 3, 5]
    );

    let route_points: Vec<_> = route
        .path
        .iter()
        .map(|vertex| *graph.get_vertex(*vertex).unwrap())
        .collect();
    let exact_poses: Vec<_> = route_points
        .iter()
        .enumerate()
        .map(|(index, point)| SE2d::from_translation_angle(*point, heading(&route_points, index)))
        .collect();

    let mut problem = PoseGraph2d::new();
    for (index, exact_pose) in exact_poses.iter().enumerate() {
        let noise = 0.08 * index as f64;
        let noisy = SE2d::from_translation_angle(
            p(exact_pose.x() + 0.15 * noise, exact_pose.y() - 0.10 * noise),
            exact_pose.angle() + 0.03 * noise,
        );
        problem.insert_pose(X(index as u64).into(), noisy).unwrap();
    }

    problem
        .add_prior(X(0).into(), exact_poses[0], p(0.01, 0.01), 0.01)
        .unwrap();
    for (index, window) in exact_poses.windows(2).enumerate() {
        problem
            .add_between(
                X(index as u64).into(),
                X(index as u64 + 1).into(),
                window[0].between(window[1]),
                p(0.05, 0.05),
                0.03,
            )
            .unwrap();
    }

    let before: Vec<_> = (0..exact_poses.len())
        .map(|index| {
            *problem
                .initial_values()
                .at::<SE2d>(X(index as u64).into())
                .unwrap()
        })
        .collect();
    let result = GaussNewtonOptimizer::new().optimize(problem.graph(), problem.initial_values());
    let after: Vec<_> = (0..exact_poses.len())
        .map(|index| *result.values.at::<SE2d>(X(index as u64).into()).unwrap())
        .collect();

    assert!(
        mean_position_error(&after, &route_points) < mean_position_error(&before, &route_points)
    );
    assert!(result.final_error < 1e-8);
}

#[test]
fn correspondence_matches_can_drive_pose_graph_constraints() {
    let predicted_landmarks = vec![p(0.0, 0.0), p(2.0, 0.0), p(4.0, 0.0), p(8.0, 8.0)];
    let observed_landmarks = vec![p(0.05, -0.02), p(2.04, 0.01), p(3.97, -0.03), p(20.0, 20.0)];

    let correspondences = mutual_nearest_correspondences_2d(
        &predicted_landmarks,
        &observed_landmarks,
        Some(0.2),
        |p| *p,
        |p| *p,
    )
    .unwrap();

    assert_eq!(correspondences.len(), 3);
    assert_eq!(
        correspondences
            .iter()
            .map(|corr| (corr.source_index, corr.target_index))
            .collect::<Vec<_>>(),
        vec![(0, 0), (1, 1), (2, 2)]
    );

    let mut problem = PoseGraph2d::new();
    problem
        .insert_pose(
            X(0).into(),
            SE2d::from_translation_angle(p(0.3, -0.2), 0.05),
        )
        .unwrap()
        .add_prior(X(0).into(), SE2d::identity(), p(0.5, 0.5), 0.2)
        .unwrap();

    let mut expected_translations = Vec::new();
    let mut before_errors = Vec::new();
    for corr in correspondences {
        let predicted = predicted_landmarks[corr.source_index];
        let observed = observed_landmarks[corr.target_index];
        let translation = observed - predicted;
        expected_translations.push(translation);
        let initial_pose = SE2d::from_translation_angle(predicted + p(0.2, -0.1), 0.02);
        before_errors.push((initial_pose.translation() - translation).magnitude());
        problem
            .add_between(
                X(0).into(),
                X((corr.source_index + 1) as u64).into(),
                SE2d::from_translation_angle(translation, 0.0),
                p(0.05, 0.05),
                0.1,
            )
            .unwrap()
            .insert_pose(X((corr.source_index + 1) as u64).into(), initial_pose)
            .unwrap();
    }

    let result = GaussNewtonOptimizer::new().optimize(problem.graph(), problem.initial_values());
    for corr_index in 0..3usize {
        let pose = result
            .values
            .at::<SE2d>(X((corr_index + 1) as u64).into())
            .unwrap();
        let expected = expected_translations[corr_index];
        let position_error = (pose.translation() - expected).magnitude();
        assert!(position_error < before_errors[corr_index]);
        assert!(position_error < 1e-3);
    }
}
