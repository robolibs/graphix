use graphix::vertex::algorithms::{find_cycle_directed, find_cycle_undirected};
use graphix::vertex::{EdgeType, Graph};

fn main() {
    let mut directed = Graph::<(), ()>::new();
    let a = directed.add_unit_vertex();
    let b = directed.add_unit_vertex();
    let c = directed.add_unit_vertex();
    directed.add_edge(a, b, 1.0, EdgeType::Directed, ());
    directed.add_edge(b, c, 1.0, EdgeType::Directed, ());
    directed.add_edge(c, a, 1.0, EdgeType::Directed, ());

    let mut undirected = Graph::<(), ()>::new();
    let u0 = undirected.add_unit_vertex();
    let u1 = undirected.add_unit_vertex();
    let u2 = undirected.add_unit_vertex();
    undirected.add_edge(u0, u1, 1.0, EdgeType::Undirected, ());
    undirected.add_edge(u1, u2, 1.0, EdgeType::Undirected, ());
    undirected.add_edge(u2, u0, 1.0, EdgeType::Undirected, ());

    println!(
        "directed cycle: {}",
        find_cycle_directed(&directed).has_cycle
    );
    println!(
        "undirected cycle: {}",
        find_cycle_undirected(&undirected).has_cycle
    );
}
