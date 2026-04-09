use std::collections::HashSet;

use graphix::vertex::transformations::{
    complement, disjoint_union, filter_edges, filter_vertices, graph_intersection, graph_union,
    induced_subgraph, transpose,
};
use graphix::vertex::views::{filter_edges_view, filter_vertices_view, reversed, subgraph_view};
use graphix::vertex::{EdgeType, Graph, VertexId};

#[test]
fn reversed_view_reverses_directed_edges() {
    let mut g = Graph::<(), ()>::new();
    let v0 = g.add_unit_vertex();
    let v1 = g.add_unit_vertex();
    g.add_edge(v0, v1, 1.0, EdgeType::Directed, ());

    let view = reversed(&g);
    let edges = view.edges();
    assert_eq!(edges.len(), 1);
    assert_eq!(edges[0].source, v1.value());
    assert_eq!(edges[0].target, v0.value());
    assert_eq!(view.neighbors(v1), vec![v0]);
    assert_eq!(view.out_edges(v1).len(), 1);
    assert_eq!(view.out_edges(v0).len(), 0);
    assert!(std::ptr::eq(view.base(), &g));
}

#[test]
fn reversed_view_preserves_undirected_and_self_loop_edges() {
    let mut g = Graph::<(), ()>::new();
    let v0 = g.add_unit_vertex();
    let v1 = g.add_unit_vertex();
    g.add_edge(v0, v1, 2.0, EdgeType::Undirected, ());
    g.add_edge(v0, v0, 3.0, EdgeType::Directed, ());

    let view = reversed(&g);
    let edges = view.edges();
    assert_eq!(edges.len(), 2);
    assert!(
        edges
            .iter()
            .any(|edge| edge.edge_type == EdgeType::Undirected
                && edge.source == v0.value()
                && edge.target == v1.value())
    );
    assert!(edges.iter().any(|edge| edge.edge_type == EdgeType::Directed
        && edge.source == v0.value()
        && edge.target == v0.value()));
}

#[test]
fn filtered_and_subgraph_views_respect_vertex_selection() {
    let mut g = Graph::<(), ()>::new();
    let verts: Vec<_> = (0..5).map(|_| g.add_unit_vertex()).collect();
    g.add_edge(verts[0], verts[1], 1.0, EdgeType::Undirected, ());
    g.add_edge(verts[0], verts[2], 2.0, EdgeType::Undirected, ());
    g.add_edge(verts[0], verts[3], 3.0, EdgeType::Undirected, ());

    let filtered = filter_vertices_view(&g, |v| v.value() % 2 == 0);
    assert!(filtered.has_vertex(verts[0]));
    assert!(!filtered.has_vertex(verts[1]));
    assert!(std::ptr::eq(filtered.base(), &g));

    let edge_filtered = filter_edges_view(&g, |edge| edge.weight >= 2.0);
    assert_eq!(edge_filtered.edge_count(), 2);
    assert!(std::ptr::eq(edge_filtered.base(), &g));

    let combined = graphix::vertex::views::FilteredGraphView::new(
        &g,
        |v| v.value() % 2 == 0,
        |edge| edge.weight >= 2.0,
    );
    assert_eq!(combined.vertex_count(), 3);
    assert_eq!(combined.edge_count(), 1);

    let subset: HashSet<VertexId<()>> = [verts[0], verts[2]].into_iter().collect();
    let subgraph = subgraph_view(&g, subset.clone());
    assert_eq!(subgraph.vertex_count(), 2);
    assert_eq!(subgraph.edge_count(), 1);
    assert_eq!(subgraph.neighbors(verts[0]), vec![verts[2]]);
    assert_eq!(subgraph.out_edges(verts[0]).len(), 1);
    assert_eq!(subgraph.out_edges(verts[2]).len(), 1);
    assert_eq!(subgraph.vertex_set().len(), subset.len());
    assert!(std::ptr::eq(subgraph.base(), &g));

    assert_eq!(filtered.out_edges(verts[0]).len(), 1);
    assert_eq!(edge_filtered.out_edges(verts[0]).len(), 2);
}

#[test]
fn transpose_and_induced_subgraph_preserve_structure() {
    let mut g = Graph::<(), ()>::new();
    let v0 = g.add_unit_vertex();
    let v1 = g.add_unit_vertex();
    let v2 = g.add_unit_vertex();
    g.add_edge(v0, v1, 1.0, EdgeType::Directed, ());
    g.add_edge(v1, v2, 2.0, EdgeType::Directed, ());

    let transposed = transpose(&g);
    assert_eq!(transposed.vertex_count(), 3);
    assert_eq!(transposed.edge_count(), 2);

    let subset = vec![v0, v1];
    let induced = induced_subgraph(&g, subset);
    assert_eq!(induced.vertex_count(), 2);
    assert_eq!(induced.edge_count(), 1);
}

#[test]
fn transpose_and_induced_subgraph_preserve_weights_and_edge_properties() {
    #[derive(Debug, Clone, PartialEq, Eq)]
    struct EdgeInfo {
        speed: i32,
        allow_stop: bool,
    }

    let mut g = Graph::<(), EdgeInfo>::new();
    let v0 = g.add_unit_vertex();
    let v1 = g.add_unit_vertex();
    let e = g.add_edge(
        v0,
        v1,
        42.0,
        EdgeType::Directed,
        EdgeInfo {
            speed: 30,
            allow_stop: true,
        },
    );
    g.add_edge(
        v0,
        v0,
        1.0,
        EdgeType::Directed,
        EdgeInfo {
            speed: 5,
            allow_stop: false,
        },
    );

    let transposed = transpose(&g);
    let edges = transposed.edges();
    assert_eq!(edges.len(), 2);
    let reversed = edges
        .iter()
        .find(|edge| edge.source == v1.value() && edge.target == v0.value())
        .unwrap();
    assert_eq!(reversed.weight, 42.0);
    assert_eq!(
        transposed.edge_property(reversed.id),
        Some(&EdgeInfo {
            speed: 30,
            allow_stop: true
        })
    );

    let induced = induced_subgraph(&g, vec![v0]);
    assert_eq!(induced.vertex_count(), 1);
    assert_eq!(induced.edge_count(), 1);
    assert_eq!(g.edge_property(e).unwrap().speed, 30);
}

#[test]
fn graph_set_operations_work() {
    let mut g1 = Graph::<(), ()>::new();
    let a0 = g1.add_unit_vertex();
    let a1 = g1.add_unit_vertex();
    g1.add_edge(a0, a1, 1.0, EdgeType::Undirected, ());

    let mut g2 = Graph::<(), ()>::new();
    let b0 = g2.add_unit_vertex();
    let b1 = g2.add_unit_vertex();
    g2.add_edge(b0, b1, 2.0, EdgeType::Undirected, ());

    let union = graph_union(&g1, &g2);
    assert_eq!(union.graph.vertex_count(), 4);
    assert_eq!(union.graph.edge_count(), 2);

    let disjoint = disjoint_union(&g1, &g2);
    assert_eq!(disjoint.graph.vertex_count(), 4);

    let intersection = graph_intersection(&g1, &g2);
    assert_eq!(intersection.vertex_count(), 2);
    assert_eq!(intersection.edge_count(), 1);
}

#[test]
fn graph_set_operations_preserve_vertex_and_edge_properties() {
    #[derive(Debug, Clone, PartialEq, Eq, Default)]
    struct EdgeInfo {
        lanes: i32,
    }

    let mut g1 = Graph::<&'static str, EdgeInfo>::new();
    let a0 = g1.add_vertex("A0");
    let a1 = g1.add_vertex("A1");
    g1.add_edge(a0, a1, 1.5, EdgeType::Directed, EdgeInfo { lanes: 1 });

    let mut g2 = Graph::<&'static str, EdgeInfo>::new();
    let b0 = g2.add_vertex("B0");
    let b1 = g2.add_vertex("B1");
    g2.add_edge(b0, b1, 2.5, EdgeType::Directed, EdgeInfo { lanes: 2 });

    let union = graph_union(&g1, &g2);
    assert_eq!(union.graph.vertex_count(), 4);
    let union_edges = union.graph.edges();
    assert_eq!(union_edges.len(), 2);
    assert!(union_edges.iter().any(|edge| edge.weight == 1.5
        && edge.edge_type == EdgeType::Directed
        && union.graph.edge_property(edge.id).unwrap().lanes == 1));
    assert!(union_edges.iter().any(|edge| edge.weight == 2.5
        && edge.edge_type == EdgeType::Directed
        && union.graph.edge_property(edge.id).unwrap().lanes == 2));

    let intersection = graph_intersection(&g1, &g2);
    assert_eq!(intersection.vertex_count(), 2);
    assert_eq!(intersection.edge_count(), 1);
    assert_eq!(intersection.edges()[0].weight, 1.5);
    assert_eq!(
        intersection
            .edge_property(intersection.edges()[0].id)
            .unwrap()
            .lanes,
        1
    );

    let complement_graph = complement(&g1);
    assert_eq!(complement_graph.vertex_count(), 2);
    assert_eq!(complement_graph.edge_count(), 1);
    let complement_edge = complement_graph.edges()[0].id;
    assert_eq!(
        complement_graph
            .edge_property(complement_edge)
            .unwrap()
            .lanes,
        0
    );
}

#[test]
fn complement_and_filters_produce_expected_results() {
    let mut g = Graph::<(), ()>::new();
    let v0 = g.add_unit_vertex();
    let v1 = g.add_unit_vertex();
    let _v2 = g.add_unit_vertex();
    g.add_edge(v0, v1, 1.0, EdgeType::Undirected, ());

    let comp = complement(&g);
    assert_eq!(comp.vertex_count(), 3);
    assert_eq!(comp.edge_count(), 2);

    let only_even = filter_vertices(&g, |v, _| v.value() % 2 == 0);
    assert_eq!(only_even.vertex_count(), 2);

    let only_light = filter_edges(&g, |edge, _| edge.weight <= 1.0);
    assert_eq!(only_light.edge_count(), 1);
}

#[test]
fn views_handle_nonexistent_subset_vertices_and_property_access() {
    #[derive(Debug, Clone, PartialEq, Eq)]
    struct NodeData {
        name: &'static str,
        value: i32,
    }

    let mut g = Graph::<NodeData, ()>::new();
    let v0 = g.add_vertex(NodeData {
        name: "A",
        value: 10,
    });
    let _v1 = g.add_vertex(NodeData {
        name: "B",
        value: 5,
    });
    let v2 = g.add_vertex(NodeData {
        name: "C",
        value: 20,
    });

    let filtered = filter_vertices_view(&g, |vertex| g[vertex].value >= 10);
    assert_eq!(filtered.vertex_count(), 2);
    assert_eq!(filtered[v0].name, "A");
    assert_eq!(filtered[v2].name, "C");

    let subgraph = subgraph_view(&g, [v0, v2, VertexId::new(99)]);
    assert!(subgraph.has_vertex(v0));
    assert!(subgraph.has_vertex(v2));
    assert!(!subgraph.has_vertex(VertexId::new(99)));
    assert_eq!(subgraph[v0].name, "A");
}
