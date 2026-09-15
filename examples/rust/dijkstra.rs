use graphix::vertex::algorithms::dijkstra;
use graphix::vertex::{EdgeType, Graph};

fn main() {
    let mut graph = Graph::<(), ()>::new();
    let a = graph.add_unit_vertex();
    let b = graph.add_unit_vertex();
    let c = graph.add_unit_vertex();

    graph.add_edge(a, b, 1.0, EdgeType::Undirected, ());
    graph.add_edge(b, c, 2.0, EdgeType::Undirected, ());
    graph.add_edge(a, c, 10.0, EdgeType::Undirected, ());

    let result = dijkstra(&graph, a, c);
    println!("dijkstra found: {}", result.found);
    println!("distance: {}", result.distance);
    println!("path: {:?}", result.path);
}
