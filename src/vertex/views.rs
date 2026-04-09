use std::collections::HashSet;
use std::ops::Index;

use super::{Edge, EdgeType, Graph, VertexId};

#[derive(Clone, Copy)]
pub struct ReversedGraphView<'a, VertexProperty, EdgeProperty> {
    graph: &'a Graph<VertexProperty, EdgeProperty>,
}

pub fn reversed<'a, VertexProperty, EdgeProperty>(
    graph: &'a Graph<VertexProperty, EdgeProperty>,
) -> ReversedGraphView<'a, VertexProperty, EdgeProperty> {
    ReversedGraphView { graph }
}

impl<'a, VertexProperty, EdgeProperty> ReversedGraphView<'a, VertexProperty, EdgeProperty>
where
    EdgeProperty: Clone,
{
    pub fn base(&self) -> &'a Graph<VertexProperty, EdgeProperty> {
        self.graph
    }

    pub fn vertex_count(&self) -> usize {
        self.graph.vertex_count()
    }

    pub fn edge_count(&self) -> usize {
        self.graph.edge_count()
    }

    pub fn has_vertex(&self, vertex: VertexId<VertexProperty>) -> bool {
        self.graph.has_vertex(vertex)
    }

    pub fn vertices(&self) -> Vec<VertexId<VertexProperty>> {
        self.graph.vertices()
    }

    pub fn edges(&self) -> Vec<Edge<EdgeProperty>> {
        self.graph
            .edges()
            .into_iter()
            .map(|mut edge| {
                if edge.edge_type == EdgeType::Directed {
                    std::mem::swap(&mut edge.source, &mut edge.target);
                }
                edge
            })
            .collect()
    }

    pub fn neighbors(&self, vertex: VertexId<VertexProperty>) -> Vec<VertexId<VertexProperty>> {
        self.edges()
            .into_iter()
            .filter_map(|edge| {
                if edge.source == vertex.value() {
                    Some(VertexId::new(edge.target))
                } else if edge.edge_type == EdgeType::Undirected && edge.target == vertex.value() {
                    Some(VertexId::new(edge.source))
                } else {
                    None
                }
            })
            .collect()
    }

    pub fn out_edges(&self, vertex: VertexId<VertexProperty>) -> Vec<Edge<EdgeProperty>> {
        self.edges()
            .into_iter()
            .filter(|edge| {
                edge.source == vertex.value()
                    || (edge.edge_type == EdgeType::Undirected && edge.target == vertex.value())
            })
            .collect()
    }
}

impl<'a, VertexProperty, EdgeProperty> Index<VertexId<VertexProperty>>
    for ReversedGraphView<'a, VertexProperty, EdgeProperty>
{
    type Output = VertexProperty;

    fn index(&self, index: VertexId<VertexProperty>) -> &Self::Output {
        &self.graph[index]
    }
}

pub struct FilteredGraphView<'a, VertexProperty, EdgeProperty, VP, EP>
where
    VP: Fn(VertexId<VertexProperty>) -> bool,
    EP: Fn(&Edge<EdgeProperty>) -> bool,
{
    graph: &'a Graph<VertexProperty, EdgeProperty>,
    vertex_predicate: VP,
    edge_predicate: EP,
}

pub fn filter_vertices_view<'a, VertexProperty, EdgeProperty, VP>(
    graph: &'a Graph<VertexProperty, EdgeProperty>,
    vertex_predicate: VP,
) -> FilteredGraphView<'a, VertexProperty, EdgeProperty, VP, impl Fn(&Edge<EdgeProperty>) -> bool>
where
    VP: Fn(VertexId<VertexProperty>) -> bool,
{
    FilteredGraphView {
        graph,
        vertex_predicate,
        edge_predicate: |_| true,
    }
}

pub fn filter_edges_view<'a, VertexProperty, EdgeProperty, EP>(
    graph: &'a Graph<VertexProperty, EdgeProperty>,
    edge_predicate: EP,
) -> FilteredGraphView<
    'a,
    VertexProperty,
    EdgeProperty,
    impl Fn(VertexId<VertexProperty>) -> bool,
    EP,
>
where
    EP: Fn(&Edge<EdgeProperty>) -> bool,
{
    FilteredGraphView {
        graph,
        vertex_predicate: |_| true,
        edge_predicate,
    }
}

impl<'a, VertexProperty, EdgeProperty, VP, EP>
    FilteredGraphView<'a, VertexProperty, EdgeProperty, VP, EP>
where
    VP: Fn(VertexId<VertexProperty>) -> bool,
    EP: Fn(&Edge<EdgeProperty>) -> bool,
    EdgeProperty: Clone,
{
    pub fn new(
        graph: &'a Graph<VertexProperty, EdgeProperty>,
        vertex_predicate: VP,
        edge_predicate: EP,
    ) -> Self {
        Self {
            graph,
            vertex_predicate,
            edge_predicate,
        }
    }

    pub fn base(&self) -> &'a Graph<VertexProperty, EdgeProperty> {
        self.graph
    }

    pub fn vertices(&self) -> Vec<VertexId<VertexProperty>> {
        self.graph
            .vertices()
            .into_iter()
            .filter(|vertex| (self.vertex_predicate)(*vertex))
            .collect()
    }

    pub fn has_vertex(&self, vertex: VertexId<VertexProperty>) -> bool {
        self.graph.has_vertex(vertex) && (self.vertex_predicate)(vertex)
    }

    pub fn vertex_count(&self) -> usize {
        self.vertices().len()
    }

    pub fn edges(&self) -> Vec<Edge<EdgeProperty>> {
        self.graph
            .edges()
            .into_iter()
            .filter(|edge| {
                (self.vertex_predicate)(VertexId::new(edge.source))
                    && (self.vertex_predicate)(VertexId::new(edge.target))
                    && (self.edge_predicate)(edge)
            })
            .collect()
    }

    pub fn edge_count(&self) -> usize {
        self.edges().len()
    }

    pub fn neighbors(&self, vertex: VertexId<VertexProperty>) -> Vec<VertexId<VertexProperty>> {
        self.edges()
            .into_iter()
            .filter_map(|edge| {
                if edge.source == vertex.value() {
                    Some(VertexId::new(edge.target))
                } else if edge.edge_type == EdgeType::Undirected && edge.target == vertex.value() {
                    Some(VertexId::new(edge.source))
                } else {
                    None
                }
            })
            .collect()
    }

    pub fn out_edges(&self, vertex: VertexId<VertexProperty>) -> Vec<Edge<EdgeProperty>> {
        self.edges()
            .into_iter()
            .filter(|edge| {
                edge.source == vertex.value()
                    || (edge.edge_type == EdgeType::Undirected && edge.target == vertex.value())
            })
            .collect()
    }
}

impl<'a, VertexProperty, EdgeProperty, VP, EP> Index<VertexId<VertexProperty>>
    for FilteredGraphView<'a, VertexProperty, EdgeProperty, VP, EP>
where
    VP: Fn(VertexId<VertexProperty>) -> bool,
    EP: Fn(&Edge<EdgeProperty>) -> bool,
{
    type Output = VertexProperty;

    fn index(&self, index: VertexId<VertexProperty>) -> &Self::Output {
        &self.graph[index]
    }
}

pub struct SubgraphView<'a, VertexProperty, EdgeProperty> {
    graph: &'a Graph<VertexProperty, EdgeProperty>,
    vertex_set: HashSet<VertexId<VertexProperty>>,
}

pub fn subgraph_view<'a, VertexProperty, EdgeProperty>(
    graph: &'a Graph<VertexProperty, EdgeProperty>,
    vertex_set: impl IntoIterator<Item = VertexId<VertexProperty>>,
) -> SubgraphView<'a, VertexProperty, EdgeProperty> {
    SubgraphView {
        graph,
        vertex_set: vertex_set.into_iter().collect(),
    }
}

impl<'a, VertexProperty, EdgeProperty> SubgraphView<'a, VertexProperty, EdgeProperty>
where
    EdgeProperty: Clone,
{
    pub fn base(&self) -> &'a Graph<VertexProperty, EdgeProperty> {
        self.graph
    }

    pub fn vertex_set(&self) -> &HashSet<VertexId<VertexProperty>> {
        &self.vertex_set
    }

    pub fn has_vertex(&self, vertex: VertexId<VertexProperty>) -> bool {
        self.graph.has_vertex(vertex) && self.vertex_set.contains(&vertex)
    }

    pub fn vertices(&self) -> Vec<VertexId<VertexProperty>> {
        self.graph
            .vertices()
            .into_iter()
            .filter(|vertex| self.vertex_set.contains(vertex))
            .collect()
    }

    pub fn vertex_count(&self) -> usize {
        self.vertices().len()
    }

    pub fn edges(&self) -> Vec<Edge<EdgeProperty>> {
        self.graph
            .edges()
            .into_iter()
            .filter(|edge| {
                self.vertex_set.contains(&VertexId::new(edge.source))
                    && self.vertex_set.contains(&VertexId::new(edge.target))
            })
            .collect()
    }

    pub fn edge_count(&self) -> usize {
        self.edges().len()
    }

    pub fn neighbors(&self, vertex: VertexId<VertexProperty>) -> Vec<VertexId<VertexProperty>> {
        self.edges()
            .into_iter()
            .filter_map(|edge| {
                if edge.source == vertex.value() {
                    Some(VertexId::new(edge.target))
                } else if edge.edge_type == EdgeType::Undirected && edge.target == vertex.value() {
                    Some(VertexId::new(edge.source))
                } else {
                    None
                }
            })
            .collect()
    }

    pub fn out_edges(&self, vertex: VertexId<VertexProperty>) -> Vec<Edge<EdgeProperty>> {
        self.edges()
            .into_iter()
            .filter(|edge| {
                edge.source == vertex.value()
                    || (edge.edge_type == EdgeType::Undirected && edge.target == vertex.value())
            })
            .collect()
    }
}

impl<'a, VertexProperty, EdgeProperty> Index<VertexId<VertexProperty>>
    for SubgraphView<'a, VertexProperty, EdgeProperty>
{
    type Output = VertexProperty;

    fn index(&self, index: VertexId<VertexProperty>) -> &Self::Output {
        &self.graph[index]
    }
}
