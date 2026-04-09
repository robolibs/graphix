use std::collections::HashSet;

use graphix::vertex::transformations::{complement, induced_subgraph, transpose};
use graphix::vertex::views::{filter_vertices_view, reversed, subgraph_view};
use graphix::vertex::{EdgeType, Graph};

fn main() {
    let mut graph = Graph::<(), ()>::new();
    let v0 = graph.add_unit_vertex();
    let v1 = graph.add_unit_vertex();
    let v2 = graph.add_unit_vertex();
    graph.add_edge(v0, v1, 1.0, EdgeType::Directed, ());
    graph.add_edge(v1, v2, 1.0, EdgeType::Undirected, ());

    let reversed_view = reversed(&graph);
    let filtered = filter_vertices_view(&graph, |v| v.value() != v1.value());
    let subset: HashSet<_> = [v0, v1].into_iter().collect();
    let subgraph = subgraph_view(&graph, subset);
    let transposed = transpose(&graph);
    let induced = induced_subgraph(&graph, vec![v0, v1]);
    let comp = complement(&Graph::<(), ()>::new());

    println!("reversed edges: {}", reversed_view.edge_count());
    println!("filtered vertices: {}", filtered.vertex_count());
    println!("subgraph edges: {}", subgraph.edge_count());
    println!("transpose edges: {}", transposed.edge_count());
    println!("induced vertices: {}", induced.vertex_count());
    println!("empty complement vertices: {}", comp.vertex_count());
}
