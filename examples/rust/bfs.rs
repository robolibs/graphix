use graphix::vertex::algorithms::{bfs, reconstruct_bfs_path};
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

    let result = bfs(&graph, v0);
    println!("bfs discovery: {:?}", result.discovery_order);
    println!("bfs path 0->3: {:?}", reconstruct_bfs_path(&result, v0, v3));
}
