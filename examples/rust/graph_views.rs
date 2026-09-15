use graphix::vertex::views::{filter_edges_view, filter_vertices_view, reversed, subgraph_view};
use graphix::vertex::{EdgeType, Graph};

fn main() {
    let mut graph = Graph::<&'static str, ()>::new();
    let a = graph.add_vertex("A");
    let b = graph.add_vertex("B");
    let c = graph.add_vertex("C");
    let d = graph.add_vertex("D");

    graph.add_edge(a, b, 1.0, EdgeType::Directed, ());
    graph.add_edge(b, c, 2.0, EdgeType::Directed, ());
    graph.add_edge(c, d, 3.0, EdgeType::Undirected, ());

    let reversed_graph = reversed(&graph);
    let heavy = filter_edges_view(&graph, |edge| edge.weight >= 2.0);
    let keep_abd = filter_vertices_view(&graph, |vertex| matches!(graph[vertex], "A" | "B" | "D"));
    let subset = subgraph_view(&graph, [a, b, c]);

    println!("graph views");
    println!(
        "  reversed neighbors of C: {:?}",
        reversed_graph
            .neighbors(c)
            .into_iter()
            .map(|v| graph[v])
            .collect::<Vec<_>>()
    );
    println!("  heavy-edge count: {}", heavy.edge_count());
    println!(
        "  filtered vertices: {:?}",
        keep_abd
            .vertices()
            .into_iter()
            .map(|v| graph[v])
            .collect::<Vec<_>>()
    );
    println!(
        "  subgraph vertices: {:?}",
        subset
            .vertices()
            .into_iter()
            .map(|v| graph[v])
            .collect::<Vec<_>>()
    );
}
