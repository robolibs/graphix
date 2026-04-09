use graphix::vertex::algorithms::strongly_connected_components;
use graphix::vertex::{EdgeType, Graph};

fn main() {
    let mut graph = Graph::<(), ()>::new();
    let a = graph.add_unit_vertex();
    let b = graph.add_unit_vertex();
    let c = graph.add_unit_vertex();
    let d = graph.add_unit_vertex();

    graph.add_edge(a, b, 1.0, EdgeType::Directed, ());
    graph.add_edge(b, a, 1.0, EdgeType::Directed, ());
    graph.add_edge(b, c, 1.0, EdgeType::Directed, ());
    graph.add_edge(c, d, 1.0, EdgeType::Directed, ());
    graph.add_edge(d, c, 1.0, EdgeType::Directed, ());

    let scc = strongly_connected_components(&graph);
    println!("num components: {}", scc.num_components);
    println!("components: {:?}", scc.components);
}
