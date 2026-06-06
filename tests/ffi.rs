use graphix::ffi::*;

#[test]
fn ffi_graph_adds_vertices_and_edges() {
    let graph = graphix_graph_new();
    assert!(!graph.is_null());

    let a = graphix_graph_add_vertex(graph);
    let b = graphix_graph_add_vertex(graph);
    assert_eq!(graphix_graph_vertex_count(graph), 2);
    assert!(graphix_graph_has_vertex(graph, a));

    let mut edge_id = usize::MAX;
    assert!(graphix_graph_add_edge(
        graph,
        a,
        b,
        2.5,
        GraphixEdgeType::Directed,
        &mut edge_id,
    ));
    assert_eq!(edge_id, 0);
    assert_eq!(graphix_graph_edge_count(graph), 1);
    assert!(graphix_graph_has_edge(graph, a, b));
    assert_eq!(graphix_graph_degree(graph, a), 1);

    graphix_graph_free(graph);
}

#[test]
fn ffi_reports_error_for_missing_vertices() {
    let graph = graphix_graph_new();
    assert!(!graph.is_null());
    let a = graphix_graph_add_vertex(graph);
    let missing = GraphixVertex { value: 999 };

    assert!(!graphix_graph_add_edge(
        graph,
        a,
        missing,
        1.0,
        GraphixEdgeType::Undirected,
        std::ptr::null_mut(),
    ));
    assert!(!graphix_last_error_message().is_null());

    graphix_graph_free(graph);
}
