use graphix::vertex::transformations::{
    complement, filter_edges, filter_vertices, graph_intersection, graph_union, induced_subgraph,
    transpose,
};
use graphix::vertex::{EdgeType, Graph};

fn main() {
    let mut labeled = Graph::<&'static str, ()>::new();
    let a = labeled.add_vertex("A");
    let b = labeled.add_vertex("B");
    let c = labeled.add_vertex("C");
    let d = labeled.add_vertex("D");
    labeled.add_edge(a, b, 1.0, EdgeType::Directed, ());
    labeled.add_edge(b, c, 2.0, EdgeType::Directed, ());
    labeled.add_edge(c, d, 3.0, EdgeType::Undirected, ());

    let transposed = transpose(&labeled);
    let subgraph = induced_subgraph(&labeled, [a, b, c]);
    let filtered_vertices = filter_vertices(&labeled, |vertex, graph| graph[vertex] != "D");
    let filtered_edges = filter_edges(&labeled, |edge, _| edge.weight >= 2.0);

    let mut g1 = Graph::<(), ()>::new();
    let g1a = g1.add_unit_vertex();
    let g1b = g1.add_unit_vertex();
    let g1c = g1.add_unit_vertex();
    g1.add_edge(g1a, g1b, 1.0, EdgeType::Directed, ());
    g1.add_edge(g1b, g1c, 2.0, EdgeType::Undirected, ());

    let mut g2 = Graph::<(), ()>::new();
    let g2a = g2.add_unit_vertex();
    let g2b = g2.add_unit_vertex();
    let g2c = g2.add_unit_vertex();
    g2.add_edge(g2a, g2b, 1.0, EdgeType::Directed, ());
    g2.add_edge(g2a, g2c, 4.0, EdgeType::Undirected, ());

    let unioned = graph_union(&g1, &g2);
    let intersected = graph_intersection(&g1, &g2);
    let complement_graph = complement(&g1);

    println!("graph transformations");
    println!("  transpose edge count: {}", transposed.edge_count());
    println!("  induced subgraph edge count: {}", subgraph.edge_count());
    println!("  union edge count: {}", unioned.graph.edge_count());
    println!("  intersection edge count: {}", intersected.edge_count());
    println!(
        "  filtered vertices: {:?}",
        filtered_vertices
            .vertices()
            .into_iter()
            .map(|v| labeled[v])
            .collect::<Vec<_>>()
    );
    println!("  filtered edges: {}", filtered_edges.edge_count());
    println!("  complement edge count: {}", complement_graph.edge_count());
}
