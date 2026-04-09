use glam::DVec2;

use graphix::vertex::EdgeType;
use graphix::vertex::Graph;
use graphix::vertex::algorithms::dijkstra;
use graphix::vertex::spatial::{
    connect_k_nearest_neighbors_2d, k_nearest_vertices_2d, nearest_vertex_2d,
};

fn main() {
    let mut graph = Graph::<DVec2, ()>::new();
    let a = graph.add_vertex(DVec2::new(0.0, 0.0));
    let _b = graph.add_vertex(DVec2::new(1.0, 0.0));
    let _c = graph.add_vertex(DVec2::new(2.0, 0.0));
    let _d = graph.add_vertex(DVec2::new(3.0, 1.0));
    let e = graph.add_vertex(DVec2::new(4.0, 1.0));

    let added = connect_k_nearest_neighbors_2d(
        &mut graph,
        2,
        EdgeType::Undirected,
        |_, p| *p,
        |_, _, _| (),
    )
    .unwrap();
    println!("constructed spatial graph with {added} edges");

    let query = DVec2::new(4.0, 4.0);
    let nearest = nearest_vertex_2d(&graph, query, |_, p| *p)
        .unwrap()
        .unwrap();
    println!(
        "nearest to ({:.1}, {:.1}) is vertex {} at distance {:.3}",
        query.x,
        query.y,
        nearest.0.value(),
        nearest.1
    );

    let knn = k_nearest_vertices_2d(&graph, query, 3, |_, p| *p).unwrap();
    for (rank, (vertex, distance)) in knn.into_iter().enumerate() {
        println!(
            "#{rank}: vertex {} distance {:.3}",
            vertex.value(),
            distance
        );
    }

    let path = dijkstra(&graph, a, e);
    println!("path found: {}", path.found);
    println!("path distance: {:.3}", path.distance);
    println!(
        "path vertices: {:?}",
        path.path
            .iter()
            .map(|vertex| vertex.value())
            .collect::<Vec<_>>()
    );
}
