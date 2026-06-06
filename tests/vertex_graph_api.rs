use graphix::vertex::{
    EdgeType, Graph, add_edge, add_edge_default, add_edge_weighted, add_edge_with_weight,
    add_vertex, add_vertex_with_property, clear as clear_fn, clear_graph, degree as degree_fn,
    edge as edge_fn, edges as edges_fn, get_edge as get_edge_fn, neighbors as neighbors_fn,
    num_edges, num_vertices, remove_edge_by_id, remove_edge_by_vertices,
    remove_vertex as remove_vertex_fn, shortest_path as shortest_path_fn, source as source_fn,
    source_or_err as source_or_err_fn, target as target_fn, target_or_err as target_or_err_fn,
    vertices as vertices_fn,
};

#[test]
fn graph_weight_mutation_and_edge_removal_work() {
    let mut g = Graph::<(), ()>::new();
    let v1 = g.add_unit_vertex();
    let v2 = g.add_unit_vertex();
    let v3 = g.add_unit_vertex();

    let e1 = g.add_edge(v1, v2, 1.5, EdgeType::Undirected, ());
    let e2 = g.add_edge(v2, v3, 2.5, EdgeType::Directed, ());

    assert_eq!(g.get_weight(e1), Some(1.5));
    assert_eq!(g.get_weight(e2), Some(2.5));

    g.set_weight(e1, 10.0).unwrap();
    g.set_weight(e2, 20.0).unwrap();
    assert_eq!(g.get_weight(e1), Some(10.0));
    assert_eq!(g.get_weight(e2), Some(20.0));
    assert_eq!(g.get_edge_type(e1), Some(EdgeType::Undirected));
    assert_eq!(g.get_edge_type(e2), Some(EdgeType::Directed));
    assert_eq!(g.source(e1), Some(v1));
    assert_eq!(g.target(e1), Some(v2));
    assert_eq!(g.out_edges(v2), vec![e1, e2]);

    assert!(g.remove_edge(e2));
    assert!(!g.has_edge(v2, v3));
    assert_eq!(g.edge_count(), 1);

    assert!(g.remove_edge_between(v1, v2));
    assert!(!g.has_edge(v1, v2));
    assert!(!g.has_edge(v2, v1));
    assert_eq!(g.edge_count(), 0);
}

#[test]
fn graph_copy_and_move_like_clone_semantics_are_independent() {
    #[derive(Debug, Clone, PartialEq)]
    struct Point {
        x: f64,
        y: f64,
    }

    let mut g1 = Graph::<Point, ()>::new();
    let v0 = g1.add_vertex(Point { x: 0.0, y: 0.0 });
    let v1 = g1.add_vertex(Point { x: 1.0, y: 1.0 });
    let v2 = g1.add_vertex(Point { x: 2.0, y: 2.0 });
    g1.add_edge(v0, v1, 1.0, EdgeType::Directed, ());
    g1.add_edge(v1, v2, 2.0, EdgeType::Undirected, ());

    let mut g2 = g1.clone();
    assert_eq!(g2.vertex_count(), 3);
    assert_eq!(g2.edge_count(), 2);
    assert!(g2.has_edge(v0, v1));
    assert!(!g2.has_edge(v1, v0));
    assert!(g2.has_edge(v1, v2));
    assert!(g2.has_edge(v2, v1));

    g2[v0].x = 99.0;
    assert_eq!(g2[v0].x, 99.0);
    assert_eq!(g1[v0].x, 0.0);
}

#[test]
fn free_function_graph_api_matches_member_api() {
    let mut g = Graph::<(), ()>::new();
    let v1 = add_vertex(&mut g);
    let v2 = add_vertex(&mut g);
    let v3 = add_vertex(&mut g);

    let e1 = add_edge_default(v1, v2, &mut g);
    let e2 = add_edge_weighted(v2, v3, 2.5, &mut g);

    assert_eq!(num_vertices(&g), 3);
    assert_eq!(num_edges(&g), 2);
    assert_eq!(degree_fn(v2, &g), 2);
    assert_eq!(neighbors_fn(v2, &g), vec![v1, v3]);
    assert_eq!(vertices_fn(&g).len(), 3);
    assert_eq!(edges_fn(&g).len(), 2);
    assert_eq!(source_fn(e1, &g), Some(v1));
    assert_eq!(target_fn(e1, &g), Some(v2));
    assert_eq!(source_fn(e2, &g), Some(v2));
    assert_eq!(target_fn(e2, &g), Some(v3));

    assert!(remove_edge_by_id(e1, &mut g));
    assert_eq!(num_edges(&g), 1);
    assert!(remove_edge_by_vertices(v2, v3, &mut g));
    assert_eq!(num_edges(&g), 0);

    clear_fn(&mut g);
    assert_eq!(num_vertices(&g), 0);
    assert_eq!(num_edges(&g), 0);
}

#[test]
fn boost_style_generic_free_add_edge_helpers_work_for_property_graphs() {
    let mut g = Graph::<i32, ()>::new();
    let v1 = add_vertex_with_property(10, &mut g);
    let v2 = add_vertex_with_property(20, &mut g);
    let v3 = add_vertex_with_property(30, &mut g);

    let e1 = add_edge(v1, v2, &mut g);
    let e2 = add_edge_with_weight(v2, v3, 4.5, &mut g);

    assert_eq!(num_vertices(&g), 3);
    assert_eq!(num_edges(&g), 2);
    assert_eq!(g.get_weight(e1), Some(1.0));
    assert_eq!(g.get_weight(e2), Some(4.5));
    assert!(g.has_edge(v1, v2));
    assert!(g.has_edge(v2, v1));
    assert!(g.has_edge(v2, v3));
    assert!(g.has_edge(v3, v2));

    clear_graph(&mut g);
    assert_eq!(num_vertices(&g), 0);
    assert_eq!(num_edges(&g), 0);
}

#[test]
fn free_function_vertex_property_addition_works() {
    let mut g = Graph::<i32, ()>::new();
    let v1 = add_vertex_with_property(100, &mut g);
    let v2 = add_vertex_with_property(200, &mut g);
    g.add_edge(v1, v2, 5.0, EdgeType::Undirected, ());

    assert_eq!(num_vertices(&g), 2);
    assert_eq!(g[v1], 100);
    assert_eq!(g[v2], 200);
    assert_eq!(g.source(g.get_edge(v1, v2).unwrap()), Some(v1));
}

#[test]
fn edge_property_accessors_work() {
    #[derive(Debug, Clone, PartialEq, Eq)]
    struct EdgeInfo {
        speed: i32,
        stop: bool,
    }

    let mut g = Graph::<i32, EdgeInfo>::new();
    let v1 = g.add_vertex(100);
    let v2 = g.add_vertex(200);
    let e = g.add_edge(
        v1,
        v2,
        1.5,
        EdgeType::Directed,
        EdgeInfo {
            speed: 50,
            stop: false,
        },
    );

    assert_eq!(
        g.edge_property(e),
        Some(&EdgeInfo {
            speed: 50,
            stop: false
        })
    );
    g.edge_property_mut(e).unwrap().stop = true;
    assert!(g.edge_property(e).unwrap().stop);
}

#[test]
fn edge_query_helpers_match_member_api() {
    let mut g = Graph::<(), ()>::new();
    let v1 = g.add_unit_vertex();
    let v2 = g.add_unit_vertex();
    let v3 = g.add_unit_vertex();

    let e = g.add_edge(v1, v2, 5.0, EdgeType::Undirected, ());

    assert_eq!(g.get_edge(v1, v2), Some(e));
    assert_eq!(g.get_edge(v2, v1), Some(e));
    assert_eq!(g.get_edge(v1, v3), None);

    assert_eq!(g.edge(v1, v2), (e, true));
    assert_eq!(g.edge(v1, v3), (0, false));

    assert_eq!(get_edge_fn(v1, v2, &g), Some(e));
    assert_eq!(get_edge_fn(v2, v1, &g), Some(e));
    assert_eq!(get_edge_fn(v1, v3, &g), None);

    assert_eq!(edge_fn(v1, v2, &g), (e, true));
    assert_eq!(edge_fn(v1, v3, &g), (0, false));
}

#[test]
fn vertex_removal_cleans_up_incident_edges() {
    let mut g = Graph::<i32, ()>::new();
    let v1 = g.add_vertex(10);
    let v2 = g.add_vertex(20);
    let v3 = g.add_vertex(30);

    let e12 = g.add_edge(v1, v2, 1.0, EdgeType::Directed, ());
    let e23 = g.add_edge(v2, v3, 2.0, EdgeType::Directed, ());
    let e13 = g.add_edge(v1, v3, 3.0, EdgeType::Directed, ());

    assert_eq!(g.vertex_count(), 3);
    assert_eq!(g.edge_count(), 3);
    assert!(g.remove_vertex(v2));

    assert_eq!(g.vertex_count(), 2);
    assert_eq!(g.edge_count(), 1);
    assert!(!g.has_vertex(v2));
    assert!(g.has_vertex(v1));
    assert!(g.has_vertex(v3));
    assert!(!g.has_edge(v1, v2));
    assert!(!g.has_edge(v2, v3));
    assert!(g.has_edge(v1, v3));
    assert_eq!(g.source(e12), None);
    assert_eq!(g.target(e23), None);
    assert_eq!(g.source(e13), Some(v1));
    assert_eq!(g.target(e13), Some(v3));
    assert!(!g.remove_vertex(v2));
}

#[test]
fn free_function_remove_vertex_matches_member_api() {
    let mut g = Graph::<(), ()>::new();
    let v1 = add_vertex(&mut g);
    let v2 = add_vertex(&mut g);
    let v3 = add_vertex(&mut g);
    add_edge_default(v1, v2, &mut g);
    add_edge_default(v2, v3, &mut g);

    assert!(remove_vertex_fn(v2, &mut g));
    assert_eq!(num_vertices(&g), 2);
    assert_eq!(num_edges(&g), 0);
    assert!(!g.has_vertex(v2));
    assert!(g.has_vertex(v1));
    assert!(g.has_vertex(v3));
}

#[test]
fn mixed_directed_and_undirected_edges_preserve_directionality_and_iteration() {
    let mut g = Graph::<&'static str, ()>::new();
    let alice = g.add_vertex("Alice");
    let bob = g.add_vertex("Bob");
    let charlie = g.add_vertex("Charlie");

    let follows = g.add_edge(alice, bob, 1.0, EdgeType::Directed, ());
    let friends = g.add_edge(bob, charlie, 2.0, EdgeType::Undirected, ());

    assert!(g.has_edge(alice, bob));
    assert!(!g.has_edge(bob, alice));
    assert!(g.has_edge(bob, charlie));
    assert!(g.has_edge(charlie, bob));

    assert_eq!(g.neighbors(alice), vec![bob]);
    assert_eq!(g.neighbors(bob), vec![charlie]);
    assert_eq!(g.degree(alice), 1);
    assert_eq!(g.degree(bob), 1);
    assert_eq!(g.degree(charlie), 1);

    let edges = g.edges();
    assert_eq!(edges.len(), 2);
    assert!(edges.iter().any(|edge| edge.id == follows
        && edge.source == alice.value()
        && edge.target == bob.value()
        && edge.edge_type == EdgeType::Directed
        && (edge.weight - 1.0).abs() < 1e-9));
    assert!(edges.iter().any(|edge| edge.id == friends
        && edge.edge_type == EdgeType::Undirected
        && ((edge.source == bob.value() && edge.target == charlie.value())
            || (edge.source == charlie.value() && edge.target == bob.value()))));
}

#[test]
fn parallel_edges_and_clone_preserve_edge_identity_and_types() {
    let mut g = Graph::<(), ()>::new();
    let v0 = g.add_unit_vertex();
    let v1 = g.add_unit_vertex();

    let directed = g.add_edge(v0, v1, 1.0, EdgeType::Directed, ());
    let undirected = g.add_edge(v0, v1, 2.0, EdgeType::Undirected, ());

    assert_ne!(directed, undirected);
    assert_eq!(g.edge_count(), 2);
    assert_eq!(g.neighbors(v0), vec![v1, v1]);
    assert_eq!(g.out_edges(v0), vec![directed, undirected]);
    assert_eq!(g.get_edge_type(directed), Some(EdgeType::Directed));
    assert_eq!(g.get_edge_type(undirected), Some(EdgeType::Undirected));

    let cloned = g.clone();
    assert_eq!(cloned.edge_count(), 2);
    assert_eq!(cloned.get_edge_type(directed), Some(EdgeType::Directed));
    assert_eq!(cloned.get_edge_type(undirected), Some(EdgeType::Undirected));
    assert!(cloned.has_edge(v0, v1));
    assert!(cloned.has_edge(v1, v0));
}

#[test]
fn graph_shortest_path_api_matches_algorithm_expectations() {
    let mut g = Graph::<i32, ()>::new();
    let v1 = g.add_vertex(100);
    let v2 = g.add_vertex(200);
    let v3 = g.add_vertex(300);
    let v4 = g.add_vertex(400);

    g.add_edge(v1, v2, 1.0, EdgeType::Undirected, ());
    g.add_edge(v2, v4, 1.0, EdgeType::Undirected, ());
    g.add_edge(v1, v3, 5.0, EdgeType::Undirected, ());
    g.add_edge(v3, v4, 5.0, EdgeType::Undirected, ());

    let member = g.shortest_path(v1, v4);
    let free = shortest_path_fn(v1, v4, &g);

    assert!(member.found);
    assert_eq!(member.distance, 2.0);
    assert_eq!(member.path, vec![v1, v2, v4]);
    assert_eq!(free.path, member.path);
    assert_eq!(g[member.path[0]], 100);
    assert_eq!(g[member.path[1]], 200);
    assert_eq!(g[member.path[2]], 400);

    let same = g.shortest_path(v1, v1);
    assert!(same.found);
    assert_eq!(same.distance, 0.0);
    assert_eq!(same.path, vec![v1]);

    let isolated = g.add_vertex(500);
    let missing = g.shortest_path(v1, isolated);
    assert!(!missing.found);
    assert!(missing.distance.is_infinite());
    assert!(missing.path.is_empty());
}

#[test]
fn strict_source_target_helpers_report_invalid_edges() {
    let mut g = Graph::<(), ()>::new();
    let v0 = g.add_unit_vertex();
    let v1 = g.add_unit_vertex();
    let e = g.add_edge(v0, v1, 1.0, EdgeType::Directed, ());

    assert_eq!(g.source_or_err(e).unwrap(), v0);
    assert_eq!(g.target_or_err(e).unwrap(), v1);
    assert_eq!(source_or_err_fn(e, &g).unwrap(), v0);
    assert_eq!(target_or_err_fn(e, &g).unwrap(), v1);

    assert!(g.source_or_err(999).is_err());
    assert!(g.target_or_err(999).is_err());
    assert!(source_or_err_fn(999, &g).is_err());
    assert!(target_or_err_fn(999, &g).is_err());
}
