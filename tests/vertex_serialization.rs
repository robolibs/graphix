use std::path::PathBuf;
use std::time::{SystemTime, UNIX_EPOCH};

use graphix::vertex::{EdgeType, Graph};

#[derive(Debug, Clone, Default, PartialEq)]
struct Point2D {
    x: f64,
    y: f64,
}

fn temp_file(name: &str) -> PathBuf {
    let nanos = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    std::env::temp_dir().join(format!("graphix_rs_{name}_{nanos}.dot"))
}

#[test]
fn dot_roundtrip_preserves_mixed_graph_structure() {
    let mut graph = Graph::<(), ()>::new();
    let v0 = graph.add_unit_vertex();
    let v1 = graph.add_unit_vertex();
    let v2 = graph.add_unit_vertex();

    graph.add_edge(v0, v1, 1.5, EdgeType::Undirected, ());
    graph.add_edge(v1, v2, 2.5, EdgeType::Directed, ());

    let path = temp_file("mixed_roundtrip");
    graph.save_dot_default(&path).unwrap();
    let loaded = Graph::<(), ()>::load_dot_default(&path).unwrap();
    let _ = std::fs::remove_file(&path);

    assert_eq!(loaded.vertex_count(), 3);
    assert_eq!(loaded.edge_count(), 2);
    assert!(loaded.has_edge(v0, v1));
    assert!(loaded.has_edge(v1, v0));
    assert!(loaded.has_edge(v1, v2));
    assert!(!loaded.has_edge(v2, v1));
    assert!((loaded.get_weight(loaded.get_edge(v0, v1).unwrap()).unwrap() - 1.5).abs() < 1e-9);
    assert!((loaded.get_weight(loaded.get_edge(v1, v2).unwrap()).unwrap() - 2.5).abs() < 1e-9);
}

#[test]
fn dot_roundtrip_handles_empty_singleton_self_loop_and_isolated_vertices() {
    let empty_path = temp_file("empty_roundtrip");
    Graph::<(), ()>::new()
        .save_dot_default(&empty_path)
        .unwrap();
    let empty_loaded = Graph::<(), ()>::load_dot_default(&empty_path).unwrap();
    let _ = std::fs::remove_file(&empty_path);
    assert_eq!(empty_loaded.vertex_count(), 0);
    assert_eq!(empty_loaded.edge_count(), 0);

    let mut graph = Graph::<(), ()>::new();
    let v0 = graph.add_unit_vertex();
    let v1 = graph.add_unit_vertex();
    let v2 = graph.add_unit_vertex();
    graph.add_edge(v0, v0, 1.0, EdgeType::Undirected, ());
    graph.add_edge(v0, v1, 2.0, EdgeType::Undirected, ());

    let path = temp_file("selfloop_isolated_roundtrip");
    graph.save_dot_default(&path).unwrap();
    let loaded = Graph::<(), ()>::load_dot_default(&path).unwrap();
    let _ = std::fs::remove_file(&path);

    assert_eq!(loaded.vertex_count(), 3);
    assert_eq!(loaded.edge_count(), 2);
    assert!(loaded.has_edge(v0, v0));
    assert!(loaded.has_edge(v0, v1));
    assert!(loaded.has_vertex(v2));
    assert_eq!(loaded.degree(v2), 0);
}

#[test]
fn dot_string_uses_expected_format_markers() {
    let mut undirected = Graph::<(), ()>::new();
    let u0 = undirected.add_unit_vertex();
    let u1 = undirected.add_unit_vertex();
    undirected.add_edge(u0, u1, 1.0, EdgeType::Undirected, ());
    let undirected_dot = undirected.to_dot_string_default();
    assert!(undirected_dot.contains("graph G"));
    assert!(undirected_dot.contains("--"));
    assert!(!undirected_dot.contains("digraph G"));

    let mut mixed = Graph::<(), ()>::new();
    let m0 = mixed.add_unit_vertex();
    let m1 = mixed.add_unit_vertex();
    let m2 = mixed.add_unit_vertex();
    mixed.add_edge(m0, m1, 1.0, EdgeType::Undirected, ());
    mixed.add_edge(m1, m2, 2.0, EdgeType::Directed, ());
    let mixed_dot = mixed.to_dot_string_default();
    assert!(mixed_dot.contains("digraph G"));
    assert!(mixed_dot.contains("->"));
    assert!(mixed_dot.contains("dir=none"));
}

#[test]
fn dot_roundtrip_preserves_vertex_properties() {
    let mut graph = Graph::<Point2D, ()>::new();
    let v0 = graph.add_vertex(Point2D { x: 1.0, y: 2.0 });
    let v1 = graph.add_vertex(Point2D { x: 3.0, y: 4.0 });
    graph.add_edge(v0, v1, 1.0, EdgeType::Undirected, ());

    let path = temp_file("props_roundtrip");
    graph
        .save_dot(&path, |_, point| format!("{},{}", point.x, point.y))
        .unwrap();
    let loaded = Graph::<Point2D, ()>::load_dot(&path, |text| {
        let (x, y) = text.split_once(',').unwrap();
        Point2D {
            x: x.parse().unwrap(),
            y: y.parse().unwrap(),
        }
    })
    .unwrap();
    let _ = std::fs::remove_file(&path);

    assert_eq!(loaded.vertex_count(), 2);
    assert_eq!(loaded[v0], Point2D { x: 1.0, y: 2.0 });
    assert_eq!(loaded[v1], Point2D { x: 3.0, y: 4.0 });
}

#[test]
fn dot_roundtrip_preserves_special_characters_in_vertex_labels() {
    let mut graph = Graph::<String, ()>::new();
    let v0 = graph.add_vertex("point \"alpha\", x=1\\2".to_string());
    let v1 = graph.add_vertex("comma, quote\", slash\\ and spaces".to_string());
    graph.add_edge(v0, v1, 1.0, EdgeType::Directed, ());

    let path = temp_file("special_chars_roundtrip");
    graph.save_dot(&path, |_, text| text.clone()).unwrap();
    let loaded = Graph::<String, ()>::load_dot(&path, |text| text.to_string()).unwrap();
    let _ = std::fs::remove_file(&path);

    assert_eq!(loaded[v0], "point \"alpha\", x=1\\2");
    assert_eq!(loaded[v1], "comma, quote\", slash\\ and spaces");
}

#[test]
fn dot_operations_report_invalid_paths() {
    let graph = Graph::<(), ()>::new();
    let missing_parent = std::env::temp_dir()
        .join("graphix_rs_missing_parent")
        .join("nested")
        .join("graph.dot");
    let _ = std::fs::remove_dir_all(
        missing_parent
            .parent()
            .unwrap()
            .parent()
            .unwrap()
            .to_path_buf(),
    );

    assert!(graph.save_dot_default(&missing_parent).is_err());

    let missing_file = std::env::temp_dir()
        .join("graphix_rs_missing_file")
        .join("missing.dot");
    let _ = std::fs::remove_dir_all(missing_file.parent().unwrap().to_path_buf());
    assert!(Graph::<(), ()>::load_dot_default(&missing_file).is_err());
}

#[tokio::test]
async fn dot_async_save_and_load_work() {
    let mut graph = Graph::<(), ()>::new();
    let v0 = graph.add_unit_vertex();
    let v1 = graph.add_unit_vertex();
    graph.add_edge(v0, v1, 7.0, EdgeType::Directed, ());

    let path = temp_file("async_roundtrip");
    graph.save_dot_default_async(&path).await.unwrap();
    let loaded = Graph::<(), ()>::load_dot_default_async(&path)
        .await
        .unwrap();
    let _ = std::fs::remove_file(&path);

    assert_eq!(loaded.vertex_count(), 2);
    assert_eq!(loaded.edge_count(), 1);
    assert!(loaded.has_edge(v0, v1));
    assert!(!loaded.has_edge(v1, v0));
}
