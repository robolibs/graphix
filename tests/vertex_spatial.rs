use glam::DVec2;

use graphix::vertex::EdgeType;
use graphix::vertex::Graph;
use graphix::vertex::algorithms::dijkstra;
use graphix::vertex::spatial::{
    connect_k_nearest_neighbors_2d, connect_vertices_within_radius_2d, k_nearest_vertices_2d,
    knn_graph_2d, mutual_nearest_correspondences_2d, nearest_neighbor_correspondences_2d,
    nearest_vertex_2d, radius_graph_2d, radius_limited_correspondences_2d,
    vertices_within_radius_2d,
};

#[test]
fn nearest_vertex_queries_work_for_2d_property_graphs() {
    let mut g = Graph::<DVec2, ()>::new();
    let v0 = g.add_vertex(DVec2::new(0.0, 0.0));
    let v1 = g.add_vertex(DVec2::new(10.0, 0.0));
    let v2 = g.add_vertex(DVec2::new(5.0, 5.0));

    let nearest = nearest_vertex_2d(&g, DVec2::new(4.0, 4.0), |_, p| *p)
        .unwrap()
        .unwrap();
    assert_eq!(nearest.0, v2);
    assert!(nearest.1 < 2.0);

    let knn = k_nearest_vertices_2d(&g, DVec2::new(3.0, 1.0), 2, |_, p| *p).unwrap();
    assert_eq!(knn.len(), 2);
    assert_eq!(knn[0].0, v0);
    assert!(knn[0].1 <= knn[1].1);

    let within = vertices_within_radius_2d(&g, DVec2::new(0.0, 0.0), 7.2, |_, p| *p).unwrap();
    let ids: Vec<_> = within.into_iter().map(|(vertex, _)| vertex).collect();
    assert!(ids.contains(&v0));
    assert!(ids.contains(&v2));
    assert!(!ids.contains(&v1));
}

#[test]
fn spatial_queries_handle_empty_graph_and_zero_k() {
    let g = Graph::<DVec2, ()>::new();
    assert!(
        nearest_vertex_2d(&g, DVec2::ZERO, |_, p| *p)
            .unwrap()
            .is_none()
    );
    assert!(
        k_nearest_vertices_2d(&g, DVec2::ZERO, 3, |_, p| *p)
            .unwrap()
            .is_empty()
    );
    assert!(
        vertices_within_radius_2d(&g, DVec2::ZERO, 1.0, |_, p| *p)
            .unwrap()
            .is_empty()
    );
}

#[test]
fn knn_and_radius_graph_builders_create_distance_weighted_graphs() {
    let points = vec![
        DVec2::new(0.0, 0.0),
        DVec2::new(1.0, 0.0),
        DVec2::new(2.0, 0.0),
        DVec2::new(5.0, 0.0),
    ];

    let knn = knn_graph_2d(points.clone(), 1, |p| *p).unwrap();
    assert_eq!(knn.vertex_count(), 4);
    assert!(knn.edge_count() >= 2);
    for edge in knn.edges() {
        assert!(edge.weight > 0.0);
    }

    let radius = radius_graph_2d(points, 1.1, |p| *p).unwrap();
    assert_eq!(radius.vertex_count(), 4);
    assert!(radius.edge_count() >= 2);
    assert!(!radius.has_edge(
        graphix::vertex::VertexId::new(0),
        graphix::vertex::VertexId::new(3)
    ));
}

#[test]
fn spatial_connectors_can_build_pathfinding_graphs() {
    let mut g = Graph::<DVec2, ()>::new();
    let a = g.add_vertex(DVec2::new(0.0, 0.0));
    let _b = g.add_vertex(DVec2::new(1.0, 0.0));
    let _c = g.add_vertex(DVec2::new(2.0, 0.0));
    let d = g.add_vertex(DVec2::new(3.0, 0.0));

    let added =
        connect_k_nearest_neighbors_2d(&mut g, 2, EdgeType::Undirected, |_, p| *p, |_, _, _| ())
            .unwrap();
    assert!(added > 0);

    let result = dijkstra(&g, a, d);
    assert!(result.found);
    assert_eq!(result.path.first().copied(), Some(a));
    assert_eq!(result.path.last().copied(), Some(d));
    assert!(result.distance > 0.0);
}

#[test]
fn radius_connector_respects_radius_and_custom_edge_properties() {
    let mut g = Graph::<DVec2, &'static str>::new();
    let v0 = g.add_vertex(DVec2::new(0.0, 0.0));
    let v1 = g.add_vertex(DVec2::new(0.5, 0.0));
    let v2 = g.add_vertex(DVec2::new(3.0, 0.0));

    let added = connect_vertices_within_radius_2d(
        &mut g,
        1.0,
        EdgeType::Undirected,
        |_, p| *p,
        |_, _, _| "near",
    )
    .unwrap();

    assert_eq!(added, 1);
    let edge = g.get_edge(v0, v1).unwrap();
    assert_eq!(g.edge_property(edge), Some(&"near"));
    assert!(!g.has_edge(v0, v2));
}

#[test]
fn nearest_neighbor_correspondences_match_expected_targets() {
    let sources = vec![
        DVec2::new(0.1, 0.0),
        DVec2::new(1.9, 0.1),
        DVec2::new(4.2, 0.0),
    ];
    let targets = vec![
        DVec2::new(0.0, 0.0),
        DVec2::new(2.0, 0.0),
        DVec2::new(4.0, 0.0),
    ];

    let correspondences =
        nearest_neighbor_correspondences_2d(&sources, &targets, |p| *p, |p| *p).unwrap();
    assert_eq!(correspondences.len(), 3);
    assert_eq!(correspondences[0].source_index, 0);
    assert_eq!(correspondences[0].target_index, 0);
    assert_eq!(correspondences[1].target_index, 1);
    assert_eq!(correspondences[2].target_index, 2);
    assert!(correspondences.iter().all(|corr| corr.distance < 0.25));
}

#[test]
fn radius_and_mutual_correspondence_filters_drop_weak_matches() {
    let sources = vec![
        DVec2::new(0.0, 0.0),
        DVec2::new(1.0, 0.0),
        DVec2::new(5.0, 5.0),
    ];
    let targets = vec![
        DVec2::new(0.1, 0.0),
        DVec2::new(1.1, 0.0),
        DVec2::new(1.2, 0.0),
    ];

    let limited =
        radius_limited_correspondences_2d(&sources, &targets, 0.35, |p| *p, |p| *p).unwrap();
    assert_eq!(limited.len(), 2);
    assert!(limited.iter().all(|corr| corr.distance <= 0.35));

    let mutual =
        mutual_nearest_correspondences_2d(&sources, &targets, Some(0.35), |p| *p, |p| *p).unwrap();
    assert_eq!(mutual.len(), 2);
    assert_eq!(mutual[0].source_index, 0);
    assert_eq!(mutual[0].target_index, 0);
    assert_eq!(mutual[1].source_index, 1);
    assert_eq!(mutual[1].target_index, 1);
}

#[test]
fn correspondence_helpers_handle_empty_inputs() {
    let empty: Vec<DVec2> = Vec::new();
    let targets = vec![DVec2::ZERO];

    assert!(
        nearest_neighbor_correspondences_2d(&empty, &targets, |p| *p, |p| *p)
            .unwrap()
            .is_empty()
    );
    assert!(
        radius_limited_correspondences_2d(&targets, &empty, 1.0, |p| *p, |p| *p)
            .unwrap()
            .is_empty()
    );
    assert!(
        mutual_nearest_correspondences_2d(&empty, &targets, None, |p| *p, |p| *p)
            .unwrap()
            .is_empty()
    );
}
