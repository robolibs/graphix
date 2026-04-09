use graphix::vertex::{EdgeType, Graph};

fn main() {
    let mut graph = Graph::<&'static str, ()>::new();

    let a = graph.add_vertex("A");
    let b = graph.add_vertex("B");
    let c = graph.add_vertex("C");

    graph.add_edge(a, b, 1.0, EdgeType::Undirected, ());
    graph.add_edge(b, c, 2.0, EdgeType::Directed, ());

    println!("vertices: {}", graph.vertex_count());
    println!("edges from B: {}", graph.edges_from(b).len());
}
