use glam::DVec2;

use graphix::vertex::algorithms::dijkstra;
use graphix::vertex::spatial::knn_graph_2d;

fn main() {
    let points = vec![
        DVec2::new(0.0, 0.0),
        DVec2::new(1.0, 0.2),
        DVec2::new(2.0, 0.0),
        DVec2::new(3.0, 0.6),
        DVec2::new(4.0, 1.0),
        DVec2::new(5.0, 1.1),
    ];

    let graph = knn_graph_2d(points, 2, |p| *p).expect("failed to build k-NN graph");
    let start = graphix::vertex::VertexId::new(0);
    let goal = graphix::vertex::VertexId::new(5);
    let result = dijkstra(&graph, start, goal);

    println!("spatial pathfinding");
    println!("vertices: {}", graph.vertex_count());
    println!("edges: {}", graph.edge_count());
    println!("path found: {}", result.found);
    println!("distance: {:.3}", result.distance);
    println!(
        "path: {:?}",
        result
            .path
            .iter()
            .map(|vertex| vertex.value())
            .collect::<Vec<_>>()
    );
}
