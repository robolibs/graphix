use graphix::vertex::algorithms::{
    astar, astar_dijkstra, bellman_ford, bellman_ford_path, betweenness_centrality,
    betweenness_centrality_normalized, betweenness_centrality_parallel, bfs, bfs_to,
    closeness_centrality, closeness_centrality_all, closeness_centrality_all_parallel,
    connected_components, degree_centrality, degree_centrality_all, degree_centrality_all_parallel,
    dfs, dfs_iterative, dfs_to, dijkstra, find_cycle_directed, find_cycle_undirected,
    get_component_map, get_shortest_path, graph_center, graph_center_parallel, graph_diameter,
    graph_diameter_parallel, graph_radius, graph_radius_parallel, has_cycle_directed,
    has_cycle_undirected, in_same_scc, is_acyclic, is_bipartite, is_connected,
    is_strongly_connected, largest_component_size, largest_scc_size, manhattan_distance,
    most_central_vertex, prim_mst, prim_mst_from, reconstruct_bfs_path, reconstruct_dfs_path,
    same_component, strongly_connected_components, top_k_central_vertices, topological_sort,
    topological_sort_dfs,
};
use graphix::vertex::{EdgeType, Graph};

#[test]
fn bfs_finds_distances_and_path() {
    let mut g = Graph::<(), ()>::new();
    let v0 = g.add_unit_vertex();
    let v1 = g.add_unit_vertex();
    let v2 = g.add_unit_vertex();
    let v3 = g.add_unit_vertex();

    g.add_edge(v0, v1, 1.0, EdgeType::Undirected, ());
    g.add_edge(v1, v2, 1.0, EdgeType::Undirected, ());
    g.add_edge(v2, v3, 1.0, EdgeType::Undirected, ());

    let result = bfs(&g, v0);
    assert_eq!(result.distance[&v0], 0);
    assert_eq!(result.distance[&v3], 3);

    let targeted = bfs_to(&g, v0, v3);
    assert!(targeted.target_found);
    assert_eq!(
        reconstruct_bfs_path(&targeted, v0, v3),
        vec![v0, v1, v2, v3]
    );
}

#[test]
fn dfs_visits_path_and_iterative_variant_runs() {
    let mut g = Graph::<(), ()>::new();
    let v0 = g.add_unit_vertex();
    let v1 = g.add_unit_vertex();
    let v2 = g.add_unit_vertex();
    let v3 = g.add_unit_vertex();

    g.add_edge(v0, v1, 1.0, EdgeType::Undirected, ());
    g.add_edge(v1, v2, 1.0, EdgeType::Undirected, ());
    g.add_edge(v2, v3, 1.0, EdgeType::Undirected, ());

    let result = dfs(&g, v0);
    assert_eq!(result.preorder[0], v0);
    assert_eq!(result.preorder.len(), 4);
    assert_eq!(result.postorder.len(), 4);

    let targeted = dfs_to(&g, v0, v3);
    assert!(targeted.target_found);
    assert_eq!(
        reconstruct_dfs_path(&targeted, v0, v3),
        vec![v0, v1, v2, v3]
    );

    let iterative = dfs_iterative(&g, v0);
    assert_eq!(iterative.preorder[0], v0);
    assert_eq!(iterative.preorder.len(), 4);
}

#[test]
fn connected_components_reports_multiple_components() {
    let mut g = Graph::<(), ()>::new();
    let v0 = g.add_unit_vertex();
    let v1 = g.add_unit_vertex();
    let v2 = g.add_unit_vertex();
    let v3 = g.add_unit_vertex();
    let v4 = g.add_unit_vertex();

    g.add_edge(v0, v1, 1.0, EdgeType::Undirected, ());
    g.add_edge(v2, v3, 1.0, EdgeType::Undirected, ());

    let result = connected_components(&g);
    assert_eq!(result.num_components, 3);
    assert!(!is_connected(&g));
    assert_eq!(largest_component_size(&g), 2);
    assert!(same_component(&g, v0, v1));
    assert!(!same_component(&g, v0, v2));
    assert_eq!(result.component_id[&v4], 2);
}

#[test]
fn dijkstra_finds_shortest_weighted_path() {
    let mut g = Graph::<(), ()>::new();
    let a = g.add_unit_vertex();
    let b = g.add_unit_vertex();
    let c = g.add_unit_vertex();

    g.add_edge(a, b, 1.0, EdgeType::Undirected, ());
    g.add_edge(b, c, 2.0, EdgeType::Undirected, ());
    g.add_edge(a, c, 10.0, EdgeType::Undirected, ());

    let result = dijkstra(&g, a, c);
    assert!(result.found);
    assert_eq!(result.distance, 3.0);
    assert_eq!(result.path, vec![a, b, c]);
}

#[test]
fn graph_properties_and_index_access_work() {
    #[derive(Debug, Clone, PartialEq, Eq)]
    struct Node {
        name: &'static str,
    }

    let mut g = Graph::<Node, ()>::new();
    let a = g.add_vertex(Node { name: "A" });
    let b = g.add_vertex(Node { name: "B" });

    g.add_edge(a, b, 5.0, EdgeType::Undirected, ());

    assert_eq!(g[a].name, "A");
    assert_eq!(g.vertex_count(), 2);
    assert_eq!(g.edge_count(), 1);
    assert_eq!(g.degree(a), 1);
    assert_eq!(g.neighbors(a), vec![b]);
    assert_eq!(g.edges().len(), 1);
}

#[test]
fn bellman_ford_handles_negative_weights() {
    let mut g = Graph::<(), ()>::new();
    let v0 = g.add_unit_vertex();
    let v1 = g.add_unit_vertex();
    let v2 = g.add_unit_vertex();

    g.add_edge(v0, v1, 3.0, EdgeType::Directed, ());
    g.add_edge(v1, v2, -5.0, EdgeType::Directed, ());
    g.add_edge(v0, v2, 10.0, EdgeType::Directed, ());

    let result = bellman_ford(&g, v0);
    assert!(!result.has_negative_cycle);
    assert_eq!(result.distances[&v2], -2.0);
    assert_eq!(bellman_ford_path(&g, v0, v2), vec![v0, v1, v2]);
    assert_eq!(get_shortest_path(&g, v0, v2), vec![v0, v1, v2]);
}

#[test]
fn bellman_ford_detects_negative_cycle() {
    let mut g = Graph::<(), ()>::new();
    let v0 = g.add_unit_vertex();
    let v1 = g.add_unit_vertex();
    let v2 = g.add_unit_vertex();

    g.add_edge(v0, v1, 1.0, EdgeType::Directed, ());
    g.add_edge(v1, v2, -3.0, EdgeType::Directed, ());
    g.add_edge(v2, v0, 1.0, EdgeType::Directed, ());

    let result = bellman_ford(&g, v0);
    assert!(result.has_negative_cycle);
    assert!(!result.negative_cycle.is_empty());
}

#[test]
fn cycle_detection_handles_directed_and_undirected_graphs() {
    let mut directed = Graph::<(), ()>::new();
    let a = directed.add_unit_vertex();
    let b = directed.add_unit_vertex();
    directed.add_edge(a, b, 1.0, EdgeType::Directed, ());
    directed.add_edge(b, a, 1.0, EdgeType::Directed, ());

    let directed_cycle = find_cycle_directed(&directed);
    assert!(directed_cycle.has_cycle);
    assert!(has_cycle_directed(&directed));

    let mut undirected = Graph::<(), ()>::new();
    let u0 = undirected.add_unit_vertex();
    let u1 = undirected.add_unit_vertex();
    let u2 = undirected.add_unit_vertex();
    undirected.add_edge(u0, u1, 1.0, EdgeType::Undirected, ());
    undirected.add_edge(u1, u2, 1.0, EdgeType::Undirected, ());
    undirected.add_edge(u2, u0, 1.0, EdgeType::Undirected, ());

    let undirected_cycle = find_cycle_undirected(&undirected);
    assert!(undirected_cycle.has_cycle);
    assert!(has_cycle_undirected(&undirected));
}

#[test]
fn topological_sort_orders_a_dag_and_rejects_cycles() {
    let mut dag = Graph::<(), ()>::new();
    let v0 = dag.add_unit_vertex();
    let v1 = dag.add_unit_vertex();
    let v2 = dag.add_unit_vertex();
    let v3 = dag.add_unit_vertex();

    dag.add_edge(v0, v1, 1.0, EdgeType::Directed, ());
    dag.add_edge(v0, v2, 1.0, EdgeType::Directed, ());
    dag.add_edge(v1, v3, 1.0, EdgeType::Directed, ());
    dag.add_edge(v2, v3, 1.0, EdgeType::Directed, ());

    let result = topological_sort(&dag);
    assert!(result.is_dag);
    assert_eq!(result.order.len(), 4);
    let pos = |vertex| {
        result
            .order
            .iter()
            .position(|candidate| *candidate == vertex)
            .unwrap()
    };
    assert!(pos(v0) < pos(v1));
    assert!(pos(v0) < pos(v2));
    assert!(pos(v1) < pos(v3));
    assert!(pos(v2) < pos(v3));

    let mut cyclic = Graph::<(), ()>::new();
    let a = cyclic.add_unit_vertex();
    let b = cyclic.add_unit_vertex();
    cyclic.add_edge(a, b, 1.0, EdgeType::Directed, ());
    cyclic.add_edge(b, a, 1.0, EdgeType::Directed, ());

    let cycle_result = topological_sort(&cyclic);
    assert!(!cycle_result.is_dag);
    assert!(cycle_result.order.is_empty());
}

#[test]
fn astar_matches_dijkstra_on_a_grid() {
    let mut g = Graph::<(), ()>::new();
    let vertices: Vec<_> = (0..9).map(|_| g.add_unit_vertex()).collect();
    for row in 0..3 {
        for col in 0..3 {
            let idx = row * 3 + col;
            if col < 2 {
                g.add_edge(
                    vertices[idx],
                    vertices[idx + 1],
                    1.0,
                    EdgeType::Undirected,
                    (),
                );
            }
            if row < 2 {
                g.add_edge(
                    vertices[idx],
                    vertices[idx + 3],
                    1.0,
                    EdgeType::Undirected,
                    (),
                );
            }
        }
    }

    let astar_result = astar(&g, vertices[0], vertices[8], |v, target| {
        manhattan_distance(v.value() as usize, target.value() as usize, 3)
    });
    let dijkstra_like = astar_dijkstra(&g, vertices[0], vertices[8]);

    assert!(astar_result.found);
    assert_eq!(astar_result.distance, 4.0);
    assert_eq!(dijkstra_like.distance, 4.0);
    assert!(astar_result.nodes_explored <= dijkstra_like.nodes_explored);
}

#[test]
fn mst_and_strong_connectivity_cover_representative_cases() {
    let mut g = Graph::<(), ()>::new();
    let v0 = g.add_unit_vertex();
    let v1 = g.add_unit_vertex();
    let v2 = g.add_unit_vertex();
    g.add_edge(v0, v1, 1.0, EdgeType::Undirected, ());
    g.add_edge(v1, v2, 2.0, EdgeType::Undirected, ());
    g.add_edge(v0, v2, 3.0, EdgeType::Undirected, ());

    let mst = prim_mst(&g);
    assert!(mst.is_spanning_tree);
    assert_eq!(mst.edges.len(), 2);
    assert_eq!(mst.total_weight, 3.0);

    let mst_from = prim_mst_from(&g, v1);
    assert!(mst_from.is_spanning_tree);
    assert_eq!(mst_from.total_weight, 3.0);

    let mut directed = Graph::<(), ()>::new();
    let a = directed.add_unit_vertex();
    let b = directed.add_unit_vertex();
    let c = directed.add_unit_vertex();
    let d = directed.add_unit_vertex();
    directed.add_edge(a, b, 1.0, EdgeType::Directed, ());
    directed.add_edge(b, a, 1.0, EdgeType::Directed, ());
    directed.add_edge(b, c, 1.0, EdgeType::Directed, ());
    directed.add_edge(c, d, 1.0, EdgeType::Directed, ());
    directed.add_edge(d, c, 1.0, EdgeType::Directed, ());

    let scc = strongly_connected_components(&directed);
    let component_map = get_component_map(&directed);
    assert_eq!(scc.num_components, 2);
    assert_eq!(component_map[&a], component_map[&b]);
    assert_eq!(component_map[&c], component_map[&d]);
    assert_ne!(component_map[&a], component_map[&c]);
    assert!(in_same_scc(&directed, a, b));
    assert!(in_same_scc(&directed, c, d));
    assert_eq!(largest_scc_size(&directed), 2);
    assert!(!is_strongly_connected(&directed));
}

#[test]
fn graph_properties_and_centrality_behave_reasonably() {
    let mut tree = Graph::<(), ()>::new();
    let center = tree.add_unit_vertex();
    let l1 = tree.add_unit_vertex();
    let l2 = tree.add_unit_vertex();
    let l3 = tree.add_unit_vertex();
    tree.add_edge(center, l1, 1.0, EdgeType::Undirected, ());
    tree.add_edge(center, l2, 1.0, EdgeType::Undirected, ());
    tree.add_edge(center, l3, 1.0, EdgeType::Undirected, ());

    let bipartite = is_bipartite(&tree);
    assert!(bipartite.is_bipartite);
    assert!(is_acyclic(&tree));
    assert_eq!(graph_diameter(&tree), 2.0);

    let degree_center = degree_centrality(&tree, center);
    let closeness_center = closeness_centrality(&tree, center);
    let betweenness = betweenness_centrality(&tree);
    assert!(degree_center > degree_centrality(&tree, l1));
    assert!(closeness_center > closeness_centrality(&tree, l1));
    assert!(betweenness[&center] > betweenness[&l1]);
}

#[test]
fn centrality_handles_empty_and_singleton_graphs() {
    let empty = Graph::<(), ()>::new();
    assert!(graphix::vertex::algorithms::degree_centrality_all(&empty).is_empty());
    assert!(graphix::vertex::algorithms::closeness_centrality_all(&empty).is_empty());
    assert!(betweenness_centrality(&empty).is_empty());
    assert_eq!(graph_diameter(&empty), 0.0);
    assert_eq!(graph_radius(&empty), 0.0);
    assert!(graph_center(&empty).is_empty());

    let mut singleton = Graph::<(), ()>::new();
    let v0 = singleton.add_unit_vertex();
    assert_eq!(degree_centrality(&singleton, v0), 0.0);
    assert_eq!(closeness_centrality(&singleton, v0), 0.0);
    assert_eq!(betweenness_centrality(&singleton)[&v0], 0.0);
    assert_eq!(graph_diameter(&singleton), 0.0);
    assert_eq!(graph_radius(&singleton), 0.0);
    assert_eq!(graph_center(&singleton), vec![v0]);
}

#[test]
fn weighted_graph_metrics_match_expected_values() {
    let mut g = Graph::<(), ()>::new();
    let v0 = g.add_unit_vertex();
    let v1 = g.add_unit_vertex();
    let v2 = g.add_unit_vertex();
    g.add_edge(v0, v1, 5.0, EdgeType::Undirected, ());
    g.add_edge(v1, v2, 3.0, EdgeType::Undirected, ());

    assert_eq!(graph_diameter(&g), 8.0);
    assert_eq!(graph_radius(&g), 5.0);
    assert_eq!(graph_center(&g), vec![v1]);
}

#[test]
fn disconnected_graph_metrics_follow_upstream_semantics() {
    let mut g = Graph::<(), ()>::new();
    let v0 = g.add_unit_vertex();
    let v1 = g.add_unit_vertex();
    let v2 = g.add_unit_vertex();
    let v3 = g.add_unit_vertex();
    g.add_edge(v0, v1, 1.0, EdgeType::Undirected, ());
    g.add_edge(v2, v3, 1.0, EdgeType::Undirected, ());

    assert!(graph_diameter(&g).is_infinite());
    assert!(graph_radius(&g).is_infinite());
    assert!(graph_center(&g).is_empty());
    assert!(graph_diameter_parallel(&g).is_infinite());
    assert!(graph_radius_parallel(&g).is_infinite());
    assert!(graph_center_parallel(&g).is_empty());
}

#[test]
fn parallel_graph_metrics_match_sequential_versions() {
    let mut g = Graph::<(), ()>::new();
    let v0 = g.add_unit_vertex();
    let v1 = g.add_unit_vertex();
    let v2 = g.add_unit_vertex();
    let v3 = g.add_unit_vertex();
    g.add_edge(v0, v1, 1.0, EdgeType::Undirected, ());
    g.add_edge(v1, v2, 2.0, EdgeType::Undirected, ());
    g.add_edge(v2, v3, 3.0, EdgeType::Undirected, ());

    assert_eq!(graph_diameter_parallel(&g), graph_diameter(&g));
    assert_eq!(graph_radius_parallel(&g), graph_radius(&g));
    assert_eq!(graph_center_parallel(&g), graph_center(&g));
}

#[test]
fn parallel_centrality_maps_match_sequential_versions() {
    let mut g = Graph::<(), ()>::new();
    let center = g.add_unit_vertex();
    let l1 = g.add_unit_vertex();
    let l2 = g.add_unit_vertex();
    let l3 = g.add_unit_vertex();
    g.add_edge(center, l1, 1.0, EdgeType::Undirected, ());
    g.add_edge(center, l2, 1.0, EdgeType::Undirected, ());
    g.add_edge(center, l3, 1.0, EdgeType::Undirected, ());

    assert_eq!(
        degree_centrality_all_parallel(&g),
        degree_centrality_all(&g)
    );
    assert_eq!(
        closeness_centrality_all_parallel(&g),
        closeness_centrality_all(&g)
    );

    let sequential = betweenness_centrality(&g);
    let parallel = betweenness_centrality_parallel(&g);
    for vertex in g.vertices() {
        assert!((sequential[&vertex] - parallel[&vertex]).abs() < 1e-9);
    }
}

#[test]
fn normalized_betweenness_and_rank_helpers_work() {
    let mut path = Graph::<(), ()>::new();
    let v0 = path.add_unit_vertex();
    let v1 = path.add_unit_vertex();
    let v2 = path.add_unit_vertex();
    path.add_edge(v0, v1, 1.0, EdgeType::Undirected, ());
    path.add_edge(v1, v2, 1.0, EdgeType::Undirected, ());

    let normalized = betweenness_centrality_normalized(&path);
    assert_eq!(normalized[&v0], 0.0);
    assert!((normalized[&v1] - 0.5).abs() < 1e-9);
    assert_eq!(normalized[&v2], 0.0);

    let degree = degree_centrality_all(&path);
    let top_1 = top_k_central_vertices(&degree, 1);
    let top_2 = top_k_central_vertices(&degree, 2);
    assert_eq!(top_1, vec![v1]);
    assert_eq!(top_2.len(), 2);
    assert_eq!(top_2[0], v1);
    assert_eq!(most_central_vertex(&degree).unwrap(), v1);

    let empty: std::collections::HashMap<graphix::vertex::VertexId<()>, f64> =
        std::collections::HashMap::new();
    assert!(most_central_vertex(&empty).is_err());
}

#[test]
fn dfs_topological_sort_matches_dag_constraints_and_detects_cycles() {
    let mut dag = Graph::<(), ()>::new();
    let v0 = dag.add_unit_vertex();
    let v1 = dag.add_unit_vertex();
    let v2 = dag.add_unit_vertex();
    let v3 = dag.add_unit_vertex();
    dag.add_edge(v0, v1, 1.0, EdgeType::Directed, ());
    dag.add_edge(v0, v2, 1.0, EdgeType::Directed, ());
    dag.add_edge(v1, v3, 1.0, EdgeType::Directed, ());
    dag.add_edge(v2, v3, 1.0, EdgeType::Directed, ());

    let result = topological_sort_dfs(&dag);
    assert!(result.is_dag);
    assert_eq!(result.order.len(), 4);
    let pos = |vertex| {
        result
            .order
            .iter()
            .position(|candidate| *candidate == vertex)
            .unwrap()
    };
    assert!(pos(v0) < pos(v1));
    assert!(pos(v0) < pos(v2));
    assert!(pos(v1) < pos(v3));
    assert!(pos(v2) < pos(v3));

    let mut cyclic = Graph::<(), ()>::new();
    let a = cyclic.add_unit_vertex();
    let b = cyclic.add_unit_vertex();
    cyclic.add_edge(a, b, 1.0, EdgeType::Directed, ());
    cyclic.add_edge(b, a, 1.0, EdgeType::Directed, ());

    let cycle_result = topological_sort_dfs(&cyclic);
    assert!(!cycle_result.is_dag);
    assert!(cycle_result.order.is_empty());
}
