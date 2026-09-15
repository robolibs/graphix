use datapod::Point;

use graphix::X;
use graphix::factor::{GaussNewtonOptimizer, PoseGraph2d, SE2d};
use graphix::vertex::algorithms::dijkstra;
use graphix::vertex::spatial::{knn_graph_2d, nearest_vertex_2d};

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

fn mean_position_error_mm(poses: &[SE2d], expected: &[Point]) -> f64 {
    let total = poses
        .iter()
        .zip(expected.iter())
        .map(|(pose, point)| ((pose.x() - point.x).powi(2) + (pose.y() - point.y).powi(2)).sqrt())
        .sum::<f64>();
    1000.0 * total / poses.len() as f64
}

fn main() {
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

    let graph = knn_graph_2d(points, 3, |p| *p).expect("failed to build k-NN graph");
    let start = nearest_vertex_2d(&graph, p(-0.2, 0.0), |_, p| *p)
        .expect("nearest start failed")
        .expect("graph should not be empty")
        .0;
    let goal = nearest_vertex_2d(&graph, p(5.1, 1.0), |_, p| *p)
        .expect("nearest goal failed")
        .expect("graph should not be empty")
        .0;

    let route = dijkstra(&graph, start, goal);
    assert!(route.found, "expected a path through the spatial graph");

    let route_points: Vec<_> = route
        .path
        .iter()
        .map(|vertex| *graph.get_vertex(*vertex).expect("route vertex missing"))
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
        problem
            .insert_pose(X(index as u64).into(), noisy)
            .expect("failed to seed initial pose");
    }

    problem
        .add_prior(X(0).into(), exact_poses[0], p(0.01, 0.01), 0.01)
        .expect("failed to add prior");

    for (index, window) in exact_poses.windows(2).enumerate() {
        let measured = window[0].between(window[1]);
        problem
            .add_between(
                X(index as u64).into(),
                X(index as u64 + 1).into(),
                measured,
                p(0.05, 0.05),
                0.03,
            )
            .expect("failed to add odometry factor");
    }

    let before: Vec<_> = (0..exact_poses.len())
        .map(|index| {
            *problem
                .initial_values()
                .at::<SE2d>(X(index as u64).into())
                .expect("missing initial pose")
        })
        .collect();

    let result = GaussNewtonOptimizer::new().optimize(problem.graph(), problem.initial_values());

    let after: Vec<_> = (0..exact_poses.len())
        .map(|index| {
            *result
                .values
                .at::<SE2d>(X(index as u64).into())
                .expect("missing optimized pose")
        })
        .collect();

    println!("spatial graph -> pose graph");
    println!("spatial vertices: {}", graph.vertex_count());
    println!("spatial edges: {}", graph.edge_count());
    println!(
        "snapped route: {:?}",
        route
            .path
            .iter()
            .map(|vertex| vertex.value())
            .collect::<Vec<_>>()
    );
    println!("route distance: {:.3}", route.distance);
    println!(
        "initial mean position error: {:.2} mm",
        mean_position_error_mm(&before, &route_points)
    );
    println!(
        "optimized mean position error: {:.2} mm",
        mean_position_error_mm(&after, &route_points)
    );
    println!("final factor error: {:.6}", result.final_error);
}
