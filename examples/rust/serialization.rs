use graphix::vertex::{EdgeType, Graph};

#[tokio::main]
async fn main() {
    let mut graph = Graph::<&'static str, ()>::new();
    let a = graph.add_vertex("A");
    let b = graph.add_vertex("B");
    let c = graph.add_vertex("C");

    graph.add_edge(a, b, 1.0, EdgeType::Undirected, ());
    graph.add_edge(b, c, 2.0, EdgeType::Directed, ());

    let path = std::env::temp_dir().join("graphix_serialization_example.dot");
    graph
        .save_dot_async(&path, |_, label| label.to_string())
        .await
        .unwrap();

    let roundtrip = Graph::<String, ()>::load_dot_async(&path, |text| text.to_string())
        .await
        .unwrap();
    let dot = roundtrip.to_dot_string(|_, label| label.clone());
    let _ = tokio::fs::remove_file(&path).await;

    println!("serialized vertices: {}", roundtrip.vertex_count());
    println!("serialized edges: {}", roundtrip.edge_count());
    println!("dot contains digraph: {}", dot.contains("digraph G"));
}
