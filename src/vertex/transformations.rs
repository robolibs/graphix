use std::collections::{HashMap, HashSet};

use super::{Edge, EdgeType, Graph, VertexId};

pub fn transpose<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
) -> Graph<VertexProperty, EdgeProperty>
where
    VertexProperty: Clone,
    EdgeProperty: Clone,
{
    let mut result = Graph::new();
    let mut mapping = HashMap::new();

    for vertex in graph.vertices() {
        mapping.insert(vertex, result.add_vertex(graph[vertex].clone()));
    }

    for edge in graph.edges() {
        let src = mapping[&VertexId::new(edge.source)];
        let tgt = mapping[&VertexId::new(edge.target)];
        match edge.edge_type {
            EdgeType::Directed => {
                result.add_edge(tgt, src, edge.weight, edge.edge_type, edge.property);
            }
            EdgeType::Undirected => {
                result.add_edge(src, tgt, edge.weight, edge.edge_type, edge.property);
            }
        };
    }

    result
}

pub fn induced_subgraph<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
    vertices: impl IntoIterator<Item = VertexId<VertexProperty>>,
) -> Graph<VertexProperty, EdgeProperty>
where
    VertexProperty: Clone,
    EdgeProperty: Clone,
{
    let selected: HashSet<_> = vertices.into_iter().collect();
    let mut result = Graph::new();
    let mut mapping = HashMap::new();

    for vertex in graph.vertices() {
        if selected.contains(&vertex) {
            mapping.insert(vertex, result.add_vertex(graph[vertex].clone()));
        }
    }

    for edge in graph.edges() {
        let src = VertexId::new(edge.source);
        let tgt = VertexId::new(edge.target);
        if selected.contains(&src) && selected.contains(&tgt) {
            result.add_edge(
                mapping[&src],
                mapping[&tgt],
                edge.weight,
                edge.edge_type,
                edge.property,
            );
        }
    }

    result
}

#[derive(Debug, Clone)]
pub struct UnionResult<VertexProperty = (), EdgeProperty = ()> {
    pub graph: Graph<VertexProperty, EdgeProperty>,
    pub g1_mapping: HashMap<u32, u32>,
    pub g2_mapping: HashMap<u32, u32>,
}

pub fn graph_union<VertexProperty, EdgeProperty>(
    g1: &Graph<VertexProperty, EdgeProperty>,
    g2: &Graph<VertexProperty, EdgeProperty>,
) -> UnionResult<VertexProperty, EdgeProperty>
where
    VertexProperty: Clone,
    EdgeProperty: Clone,
{
    let mut graph = Graph::new();
    let mut g1_mapping = HashMap::new();
    let mut g2_mapping = HashMap::new();

    for vertex in g1.vertices() {
        let new_vertex = graph.add_vertex(g1[vertex].clone());
        g1_mapping.insert(vertex.value(), new_vertex.value());
    }
    for vertex in g2.vertices() {
        let new_vertex = graph.add_vertex(g2[vertex].clone());
        g2_mapping.insert(vertex.value(), new_vertex.value());
    }

    for edge in g1.edges() {
        graph.add_edge(
            VertexId::new(g1_mapping[&edge.source]),
            VertexId::new(g1_mapping[&edge.target]),
            edge.weight,
            edge.edge_type,
            edge.property,
        );
    }
    for edge in g2.edges() {
        graph.add_edge(
            VertexId::new(g2_mapping[&edge.source]),
            VertexId::new(g2_mapping[&edge.target]),
            edge.weight,
            edge.edge_type,
            edge.property,
        );
    }

    UnionResult {
        graph,
        g1_mapping,
        g2_mapping,
    }
}

pub fn disjoint_union<VertexProperty, EdgeProperty>(
    g1: &Graph<VertexProperty, EdgeProperty>,
    g2: &Graph<VertexProperty, EdgeProperty>,
) -> UnionResult<VertexProperty, EdgeProperty>
where
    VertexProperty: Clone,
    EdgeProperty: Clone,
{
    graph_union(g1, g2)
}

pub fn graph_intersection<VertexProperty, EdgeProperty>(
    g1: &Graph<VertexProperty, EdgeProperty>,
    g2: &Graph<VertexProperty, EdgeProperty>,
) -> Graph<VertexProperty, EdgeProperty>
where
    VertexProperty: Clone,
    EdgeProperty: Clone,
{
    let mut result = Graph::new();
    let common_vertices: HashSet<_> = g1
        .vertices()
        .into_iter()
        .map(|vertex| vertex.value())
        .filter(|value| g2.has_vertex(VertexId::new(*value)))
        .collect();
    let mut mapping = HashMap::new();
    for vertex in &common_vertices {
        let original = VertexId::new(*vertex);
        mapping.insert(*vertex, result.add_vertex(g1[original].clone()).value());
    }

    let mut g2_edges = HashSet::new();
    for edge in g2.edges() {
        g2_edges.insert((edge.source, edge.target, edge.edge_type));
        if edge.edge_type == EdgeType::Undirected {
            g2_edges.insert((edge.target, edge.source, edge.edge_type));
        }
    }

    for edge in g1.edges() {
        if common_vertices.contains(&edge.source)
            && common_vertices.contains(&edge.target)
            && g2_edges.contains(&(edge.source, edge.target, edge.edge_type))
        {
            result.add_edge(
                VertexId::new(mapping[&edge.source]),
                VertexId::new(mapping[&edge.target]),
                edge.weight,
                edge.edge_type,
                edge.property,
            );
        }
    }

    result
}

pub fn complement<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
) -> Graph<VertexProperty, EdgeProperty>
where
    VertexProperty: Clone,
    EdgeProperty: Clone + Default,
{
    let mut result = Graph::new();
    let vertices = graph.vertices();
    let mut mapping = HashMap::new();
    for vertex in &vertices {
        mapping.insert(
            vertex.value(),
            result.add_vertex(graph[*vertex].clone()).value(),
        );
    }

    let mut existing = HashSet::new();
    for edge in graph.edges() {
        if edge.edge_type == EdgeType::Undirected {
            let key = if edge.source < edge.target {
                (edge.source, edge.target)
            } else {
                (edge.target, edge.source)
            };
            existing.insert(key);
        }
    }

    for (index, source) in vertices.iter().enumerate() {
        for target in vertices.iter().skip(index + 1) {
            let key = if source.value() < target.value() {
                (source.value(), target.value())
            } else {
                (target.value(), source.value())
            };
            if !existing.contains(&key) {
                result.add_edge(
                    VertexId::new(mapping[&source.value()]),
                    VertexId::new(mapping[&target.value()]),
                    1.0,
                    EdgeType::Undirected,
                    EdgeProperty::default(),
                );
            }
        }
    }

    result
}

pub fn filter_vertices<VertexProperty, EdgeProperty, P>(
    graph: &Graph<VertexProperty, EdgeProperty>,
    predicate: P,
) -> Graph<VertexProperty, EdgeProperty>
where
    VertexProperty: Clone,
    EdgeProperty: Clone,
    P: Fn(VertexId<VertexProperty>, &Graph<VertexProperty, EdgeProperty>) -> bool,
{
    let selected: Vec<_> = graph
        .vertices()
        .into_iter()
        .filter(|vertex| predicate(*vertex, graph))
        .collect();
    induced_subgraph(graph, selected)
}

pub fn filter_edges<VertexProperty, EdgeProperty, P>(
    graph: &Graph<VertexProperty, EdgeProperty>,
    predicate: P,
) -> Graph<VertexProperty, EdgeProperty>
where
    VertexProperty: Clone,
    EdgeProperty: Clone,
    P: Fn(&Edge<EdgeProperty>, &Graph<VertexProperty, EdgeProperty>) -> bool,
{
    let mut result = Graph::new();
    let mut mapping = HashMap::new();
    for vertex in graph.vertices() {
        mapping.insert(vertex, result.add_vertex(graph[vertex].clone()));
    }
    for edge in graph.edges() {
        if predicate(&edge, graph) {
            result.add_edge(
                mapping[&VertexId::new(edge.source)],
                mapping[&VertexId::new(edge.target)],
                edge.weight,
                edge.edge_type,
                edge.property,
            );
        }
    }
    result
}
