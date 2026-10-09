use graphix::vertex::algorithms::{graph_diameter, graph_radius, is_acyclic, is_bipartite};
use graphix::vertex::{EdgeType, Graph};

fn main() {
    let mut graph = Graph::<(), ()>::new();
    let v0 = graph.add_unit_vertex();
    let v1 = graph.add_unit_vertex();
    let v2 = graph.add_unit_vertex();
    let v3 = graph.add_unit_vertex();

    graph.add_edge(v0, v1, 1.0, EdgeType::Undirected, ());
    graph.add_edge(v1, v2, 1.0, EdgeType::Undirected, ());
    graph.add_edge(v2, v3, 1.0, EdgeType::Undirected, ());

    let bipartite = is_bipartite(&graph);
    println!("is bipartite: {}", bipartite.is_bipartite);
    println!("is acyclic: {}", is_acyclic(&graph));
    println!("diameter: {}", graph_diameter(&graph));
    println!("radius: {}", graph_radius(&graph));
}
