use std::env;
use std::hint::black_box;
use std::time::{Duration, Instant};

use glam::DVec2;

use graphix::X;
use graphix::factor::{GaussNewtonOptimizer, PoseGraph2d, SE2d};
use graphix::vertex::algorithms::betweenness_centrality_parallel;
use graphix::vertex::spatial::knn_graph_2d;

fn sample_points(count: usize) -> Vec<DVec2> {
    (0..count)
        .map(|index| {
            let t = index as f64 * 0.15;
            DVec2::new(
                t.cos() * (1.0 + 0.02 * index as f64),
                t.sin() * (1.0 + 0.02 * index as f64),
            )
        })
        .collect()
}

fn build_pose_chain_problem(count: usize) -> PoseGraph2d {
    let mut problem = PoseGraph2d::new();
    let sigma_xy = DVec2::splat(0.05);
    for index in 0..count {
        let x = index as f64 * 0.5;
        let y = (index as f64 * 0.03).sin() * 0.2;
        let theta = if index + 1 < count {
            (((index + 1) as f64 * 0.03).sin() * 0.2 - y).atan2(0.5)
        } else {
            0.0
        };
        let noisy_pose = SE2d::from_translation_angle(
            DVec2::new(x + 0.02 * index as f64, y - 0.01 * index as f64),
            theta + 0.005 * index as f64,
        );
        problem
            .insert_pose(X(index as u64).into(), noisy_pose)
            .unwrap();
    }

    problem
        .add_prior(X(0).into(), SE2d::identity(), DVec2::splat(0.01), 0.01)
        .unwrap();

    for index in 0..(count - 1) {
        let from = index as f64;
        let to = (index + 1) as f64;
        let p0 = DVec2::new(from * 0.5, (from * 0.03).sin() * 0.2);
        let p1 = DVec2::new(to * 0.5, (to * 0.03).sin() * 0.2);
        let delta = p1 - p0;
        let theta = delta.y.atan2(delta.x);
        problem
            .add_between(
                X(index as u64).into(),
                X(index as u64 + 1).into(),
                SE2d::from_translation_angle(delta, theta),
                sigma_xy,
                0.03,
            )
            .unwrap();
    }

    problem
}

fn benchmark(name: &str, iterations: usize, mut workload: impl FnMut()) {
    let mut total = Duration::ZERO;
    let mut best = Duration::MAX;
    for _ in 0..iterations {
        let started = Instant::now();
        workload();
        let elapsed = started.elapsed();
        total += elapsed;
        best = best.min(elapsed);
    }
    let avg = total / iterations as u32;
    println!(
        "{name:32} avg={:>8.3} ms  best={:>8.3} ms  iters={iterations}",
        avg.as_secs_f64() * 1000.0,
        best.as_secs_f64() * 1000.0,
    );
}

fn main() {
    let quick = env::args().any(|arg| arg == "--quick");
    let graph_points = if quick { 600 } else { 2_500 };
    let graph_iterations = if quick { 3 } else { 10 };
    let centrality_iterations = if quick { 2 } else { 5 };
    let pose_count = if quick { 80 } else { 300 };
    let optimizer_iterations = if quick { 3 } else { 8 };

    let points = sample_points(graph_points);
    benchmark("spatial knn graph build", graph_iterations, || {
        let graph = knn_graph_2d(points.clone(), 6, |p| *p).unwrap();
        black_box(graph.edge_count());
    });

    let centrality_graph = knn_graph_2d(points, 6, |p| *p).unwrap();
    benchmark("parallel betweenness", centrality_iterations, || {
        let centrality = betweenness_centrality_parallel(&centrality_graph);
        black_box(centrality.len());
    });

    let pose_problem = build_pose_chain_problem(pose_count);
    let optimizer = GaussNewtonOptimizer::new();
    benchmark("gauss-newton pose chain", optimizer_iterations, || {
        let result = optimizer.optimize(pose_problem.graph(), pose_problem.initial_values());
        black_box(result.final_error);
    });
}
