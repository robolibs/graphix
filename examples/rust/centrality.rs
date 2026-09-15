use graphix::vertex::algorithms::{
    betweenness_centrality_parallel, closeness_centrality_all_parallel,
    degree_centrality_all_parallel, graph_center_parallel,
};
use graphix::vertex::{EdgeType, Graph};

fn main() {
    let mut graph = Graph::<(), ()>::new();
    let center = graph.add_unit_vertex();
    let l1 = graph.add_unit_vertex();
    let l2 = graph.add_unit_vertex();
    let l3 = graph.add_unit_vertex();

    graph.add_edge(center, l1, 1.0, EdgeType::Undirected, ());
    graph.add_edge(center, l2, 1.0, EdgeType::Undirected, ());
    graph.add_edge(center, l3, 1.0, EdgeType::Undirected, ());

    println!("degree: {:?}", degree_centrality_all_parallel(&graph));
    println!("closeness: {:?}", closeness_centrality_all_parallel(&graph));
    println!("betweenness: {:?}", betweenness_centrality_parallel(&graph));
    println!("center: {:?}", graph_center_parallel(&graph));
}
