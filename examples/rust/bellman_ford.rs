use graphix::vertex::algorithms::{bellman_ford, bellman_ford_path};
use graphix::vertex::{EdgeType, Graph};

fn main() {
    let mut graph = Graph::<(), ()>::new();
    let v0 = graph.add_unit_vertex();
    let v1 = graph.add_unit_vertex();
    let v2 = graph.add_unit_vertex();

    graph.add_edge(v0, v1, 3.0, EdgeType::Directed, ());
    graph.add_edge(v1, v2, -5.0, EdgeType::Directed, ());
    graph.add_edge(v0, v2, 10.0, EdgeType::Directed, ());

    let result = bellman_ford(&graph, v0);
    println!("negative cycle: {}", result.has_negative_cycle);
    println!("distance to v2: {}", result.distances[&v2]);
    println!("path 0->2: {:?}", bellman_ford_path(&graph, v0, v2));
}
