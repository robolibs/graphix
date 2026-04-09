use std::collections::{HashMap, HashSet};
use std::ops::{Index, IndexMut};
use std::path::Path;

use rayon::prelude::*;
use tokio::fs;

use crate::core::{Id, Store};

use super::algorithms::{ShortestPathResult, dijkstra};

#[derive(Debug, Clone)]
pub struct VertexRecord<VertexProperty> {
    pub property: VertexProperty,
}

pub type VertexId<VertexProperty> = Id<VertexRecord<VertexProperty>>;
pub type EdgeId = usize;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum EdgeType {
    Undirected,
    Directed,
}

#[derive(Debug, Clone)]
pub struct Edge<EdgeProperty = ()> {
    pub id: EdgeId,
    pub source: u32,
    pub target: u32,
    pub weight: f64,
    pub edge_type: EdgeType,
    pub property: EdgeProperty,
}

#[derive(Debug, Clone)]
pub struct Graph<VertexProperty = (), EdgeProperty = ()> {
    vertices: Store<VertexRecord<VertexProperty>>,
    adjacency: HashMap<u32, Vec<Edge<EdgeProperty>>>,
    edge_index: HashMap<EdgeId, Edge<EdgeProperty>>,
    next_edge_id: EdgeId,
    edge_count: usize,
}

impl<VertexProperty, EdgeProperty> Default for Graph<VertexProperty, EdgeProperty> {
    fn default() -> Self {
        Self::new()
    }
}

impl<VertexProperty, EdgeProperty> Graph<VertexProperty, EdgeProperty> {
    pub fn new() -> Self {
        Self {
            vertices: Store::new(),
            adjacency: HashMap::new(),
            edge_index: HashMap::new(),
            next_edge_id: 0,
            edge_count: 0,
        }
    }

    pub fn add_vertex(&mut self, property: VertexProperty) -> VertexId<VertexProperty> {
        let id = self.vertices.add(VertexRecord { property });
        self.adjacency.entry(id.value()).or_default();
        id
    }

    pub fn has_vertex(&self, vertex: VertexId<VertexProperty>) -> bool {
        self.vertices.contains(vertex)
    }

    pub fn vertex_count(&self) -> usize {
        self.vertices.len()
    }

    pub fn edge_count(&self) -> usize {
        self.edge_count
    }

    pub fn add_edge(
        &mut self,
        source: VertexId<VertexProperty>,
        target: VertexId<VertexProperty>,
        weight: f64,
        edge_type: EdgeType,
        property: EdgeProperty,
    ) -> EdgeId
    where
        EdgeProperty: Clone,
    {
        assert!(self.has_vertex(source), "source vertex does not exist");
        assert!(self.has_vertex(target), "target vertex does not exist");

        let edge_id = self.next_edge_id;
        self.next_edge_id += 1;

        let forward = Edge {
            id: edge_id,
            source: source.value(),
            target: target.value(),
            weight,
            edge_type,
            property: property.clone(),
        };
        self.edge_index.insert(edge_id, forward.clone());
        self.adjacency
            .entry(source.value())
            .or_default()
            .push(forward);

        if matches!(edge_type, EdgeType::Undirected) {
            let reverse = Edge {
                id: edge_id,
                source: target.value(),
                target: source.value(),
                weight,
                edge_type,
                property,
            };
            self.adjacency
                .entry(target.value())
                .or_default()
                .push(reverse);
        }

        self.edge_count += 1;
        edge_id
    }

    pub fn set_weight(&mut self, edge_id: EdgeId, weight: f64) -> Result<(), String> {
        let mut updated = false;
        if let Some(edge) = self.edge_index.get_mut(&edge_id) {
            edge.weight = weight;
            updated = true;
        }
        for edges in self.adjacency.values_mut() {
            for edge in edges.iter_mut() {
                if edge.id == edge_id {
                    edge.weight = weight;
                }
            }
        }
        if updated {
            Ok(())
        } else {
            Err("edge not found".to_string())
        }
    }

    pub fn edges_from(&self, source: VertexId<VertexProperty>) -> &[Edge<EdgeProperty>] {
        self.adjacency
            .get(&source.value())
            .map(Vec::as_slice)
            .unwrap_or(&[])
    }

    pub fn neighbors(&self, source: VertexId<VertexProperty>) -> Vec<VertexId<VertexProperty>> {
        self.edges_from(source)
            .iter()
            .map(|edge| VertexId::new(edge.target))
            .collect()
    }

    pub fn degree(&self, source: VertexId<VertexProperty>) -> usize {
        self.edges_from(source).len()
    }

    pub fn has_edge(
        &self,
        source: VertexId<VertexProperty>,
        target: VertexId<VertexProperty>,
    ) -> bool {
        self.get_edge(source, target).is_some()
    }

    pub fn get_edge(
        &self,
        source: VertexId<VertexProperty>,
        target: VertexId<VertexProperty>,
    ) -> Option<EdgeId> {
        self.edges_from(source)
            .iter()
            .find(|edge| edge.target == target.value())
            .map(|edge| edge.id)
    }

    pub fn edge(
        &self,
        source: VertexId<VertexProperty>,
        target: VertexId<VertexProperty>,
    ) -> (EdgeId, bool) {
        match self.get_edge(source, target) {
            Some(edge_id) => (edge_id, true),
            None => (0, false),
        }
    }

    pub fn get_edge_type(&self, edge_id: EdgeId) -> Option<EdgeType> {
        self.edge_index.get(&edge_id).map(|edge| edge.edge_type)
    }

    pub fn get_weight(&self, edge_id: EdgeId) -> Option<f64> {
        self.edge_index.get(&edge_id).map(|edge| edge.weight)
    }

    pub fn edge_property(&self, edge_id: EdgeId) -> Option<&EdgeProperty> {
        self.edge_index.get(&edge_id).map(|edge| &edge.property)
    }

    pub fn edge_property_mut(&mut self, edge_id: EdgeId) -> Option<&mut EdgeProperty> {
        self.edge_index
            .get_mut(&edge_id)
            .map(|edge| &mut edge.property)
    }

    pub fn source(&self, edge_id: EdgeId) -> Option<VertexId<VertexProperty>> {
        self.edge_index
            .get(&edge_id)
            .map(|edge| VertexId::new(edge.source))
    }

    pub fn source_or_err(&self, edge_id: EdgeId) -> Result<VertexId<VertexProperty>, String> {
        self.source(edge_id)
            .ok_or_else(|| "edge not found".to_string())
    }

    pub fn target(&self, edge_id: EdgeId) -> Option<VertexId<VertexProperty>> {
        self.edge_index
            .get(&edge_id)
            .map(|edge| VertexId::new(edge.target))
    }

    pub fn target_or_err(&self, edge_id: EdgeId) -> Result<VertexId<VertexProperty>, String> {
        self.target(edge_id)
            .ok_or_else(|| "edge not found".to_string())
    }

    pub fn out_edges(&self, source: VertexId<VertexProperty>) -> Vec<EdgeId> {
        self.edges_from(source).iter().map(|edge| edge.id).collect()
    }

    pub fn vertices(&self) -> Vec<VertexId<VertexProperty>> {
        self.vertices.ids().collect()
    }

    pub fn edges(&self) -> Vec<Edge<EdgeProperty>>
    where
        EdgeProperty: Clone,
    {
        let mut seen = HashSet::new();
        let mut result = Vec::with_capacity(self.edge_count);
        for edge in self.edge_index.values() {
            if seen.insert(edge.id) {
                result.push(edge.clone());
            }
        }
        result.sort_by_key(|edge| edge.id);
        result
    }

    pub fn get_vertex(&self, vertex: VertexId<VertexProperty>) -> Option<&VertexProperty> {
        Some(&self.vertices.get(vertex)?.property)
    }

    pub fn get_vertex_mut(
        &mut self,
        vertex: VertexId<VertexProperty>,
    ) -> Option<&mut VertexProperty> {
        Some(&mut self.vertices.get_mut(vertex)?.property)
    }

    pub fn clear(&mut self) {
        *self = Self::new();
    }

    pub fn remove_vertex(&mut self, vertex: VertexId<VertexProperty>) -> bool {
        if !self.has_vertex(vertex) {
            return false;
        }

        let vertex_key = vertex.value();
        let incident_edges: HashSet<_> = self
            .edge_index
            .values()
            .filter(|edge| edge.source == vertex_key || edge.target == vertex_key)
            .map(|edge| edge.id)
            .collect();

        self.adjacency.remove(&vertex_key);

        if incident_edges.is_empty() {
            self.vertices.remove(vertex);
            return true;
        }

        for edges in self.adjacency.values_mut() {
            edges.retain(|edge| !incident_edges.contains(&edge.id));
        }

        for edge_id in &incident_edges {
            self.edge_index.remove(edge_id);
        }

        self.edge_count = self.edge_count.saturating_sub(incident_edges.len());
        self.vertices.remove(vertex);
        true
    }

    pub fn remove_edge(&mut self, edge_id: EdgeId) -> bool {
        let mut removed = self.edge_index.remove(&edge_id).is_some();
        for edges in self.adjacency.values_mut() {
            let before = edges.len();
            edges.retain(|edge| edge.id != edge_id);
            if edges.len() != before {
                removed = true;
            }
        }
        if removed {
            self.edge_count = self.edge_count.saturating_sub(1);
        }
        removed
    }

    pub fn remove_edge_between(
        &mut self,
        source: VertexId<VertexProperty>,
        target: VertexId<VertexProperty>,
    ) -> bool {
        let Some(edge_id) = self.get_edge(source, target) else {
            return false;
        };
        self.remove_edge(edge_id)
    }

    pub fn shortest_path(
        &self,
        source: VertexId<VertexProperty>,
        target: VertexId<VertexProperty>,
    ) -> ShortestPathResult<VertexProperty>
    where
        EdgeProperty: Clone,
    {
        dijkstra(self, source, target)
    }
}

impl<VertexProperty, EdgeProperty> Graph<VertexProperty, EdgeProperty>
where
    VertexProperty: Clone + Default + Send + Sync,
    EdgeProperty: Clone + Default + Send + Sync,
{
    pub fn to_dot_string<PropertyWriter>(&self, write_prop: PropertyWriter) -> String
    where
        PropertyWriter: Fn(VertexId<VertexProperty>, &VertexProperty) -> String + Sync + Send,
    {
        let vertices = self.vertices();
        let edges = self.edges();
        let has_directed = edges
            .iter()
            .any(|edge| matches!(edge.edge_type, EdgeType::Directed));
        let has_undirected = edges
            .iter()
            .any(|edge| matches!(edge.edge_type, EdgeType::Undirected));
        let is_mixed = has_directed && has_undirected;
        let header = if has_directed {
            "digraph G {\n"
        } else {
            "graph G {\n"
        };

        let mut vertex_lines: Vec<_> = vertices
            .par_iter()
            .map(|vertex| {
                let property = self
                    .get_vertex(*vertex)
                    .expect("vertex should exist while serializing");
                let label = escape_dot(write_prop(*vertex, property));
                if label.is_empty() {
                    format!("  {};\n", vertex.value())
                } else {
                    format!("  {} [label=\"{}\"];\n", vertex.value(), label)
                }
            })
            .collect();
        vertex_lines.par_sort_unstable();

        let mut edge_lines: Vec<_> = edges
            .par_iter()
            .map(|edge| {
                let connector = if matches!(edge.edge_type, EdgeType::Directed) || is_mixed {
                    "->"
                } else {
                    "--"
                };
                let mut attrs = vec![format!("weight=\"{}\"", edge.weight)];
                if is_mixed && matches!(edge.edge_type, EdgeType::Undirected) {
                    attrs.push("dir=none".to_string());
                }
                format!(
                    "  {} {} {} [{}];\n",
                    edge.source,
                    connector,
                    edge.target,
                    attrs.join(", ")
                )
            })
            .collect();
        edge_lines.par_sort_unstable();

        let mut dot = String::with_capacity(
            header.len()
                + vertex_lines.iter().map(String::len).sum::<usize>()
                + edge_lines.iter().map(String::len).sum::<usize>()
                + 2,
        );
        dot.push_str(header);
        for line in vertex_lines {
            dot.push_str(&line);
        }
        for line in edge_lines {
            dot.push_str(&line);
        }
        dot.push_str("}\n");
        dot
    }

    pub fn save_dot<P, PropertyWriter>(
        &self,
        path: P,
        write_prop: PropertyWriter,
    ) -> Result<(), String>
    where
        P: AsRef<Path>,
        PropertyWriter: Fn(VertexId<VertexProperty>, &VertexProperty) -> String + Sync + Send,
    {
        std::fs::write(path, self.to_dot_string(write_prop)).map_err(|err| err.to_string())
    }

    pub async fn save_dot_async<P, PropertyWriter>(
        &self,
        path: P,
        write_prop: PropertyWriter,
    ) -> Result<(), String>
    where
        P: AsRef<Path>,
        PropertyWriter: Fn(VertexId<VertexProperty>, &VertexProperty) -> String + Sync + Send,
    {
        fs::write(path, self.to_dot_string(write_prop))
            .await
            .map_err(|err| err.to_string())
    }

    pub fn load_dot<P, PropertyReader>(path: P, read_prop: PropertyReader) -> Result<Self, String>
    where
        P: AsRef<Path>,
        PropertyReader: Fn(&str) -> VertexProperty,
    {
        let content = std::fs::read_to_string(path).map_err(|err| err.to_string())?;
        Self::from_dot_str(&content, read_prop)
    }

    pub async fn load_dot_async<P, PropertyReader>(
        path: P,
        read_prop: PropertyReader,
    ) -> Result<Self, String>
    where
        P: AsRef<Path>,
        PropertyReader: Fn(&str) -> VertexProperty,
    {
        let content = fs::read_to_string(path)
            .await
            .map_err(|err| err.to_string())?;
        Self::from_dot_str(&content, read_prop)
    }

    pub fn from_dot_str<PropertyReader>(
        content: &str,
        read_prop: PropertyReader,
    ) -> Result<Self, String>
    where
        PropertyReader: Fn(&str) -> VertexProperty,
    {
        let mut graph = Self::new();
        let mut lines = content
            .lines()
            .map(str::trim)
            .filter(|line| !line.is_empty());
        let header = lines
            .next()
            .ok_or_else(|| "DOT content is empty".to_string())?;
        let is_digraph = header.starts_with("digraph ");
        let is_graph = header.starts_with("graph ");
        if !is_digraph && !is_graph {
            return Err("DOT content must start with graph or digraph".to_string());
        }

        for line in lines {
            if line == "}" {
                break;
            }
            if line.contains("->") || line.contains("--") {
                parse_edge_line(&mut graph, line, is_digraph)?;
            } else {
                parse_vertex_line(&mut graph, line, &read_prop)?;
            }
        }

        Ok(graph)
    }
}

impl<EdgeProperty> Graph<(), EdgeProperty> {
    pub fn add_unit_vertex(&mut self) -> VertexId<()> {
        self.add_vertex(())
    }

    pub fn add_edge_default(&mut self, source: VertexId<()>, target: VertexId<()>) -> EdgeId
    where
        EdgeProperty: Clone + Default,
    {
        self.add_edge(
            source,
            target,
            1.0,
            EdgeType::Undirected,
            EdgeProperty::default(),
        )
    }

    pub fn add_edge_weighted(
        &mut self,
        source: VertexId<()>,
        target: VertexId<()>,
        weight: f64,
    ) -> EdgeId
    where
        EdgeProperty: Clone + Default,
    {
        self.add_edge(
            source,
            target,
            weight,
            EdgeType::Undirected,
            EdgeProperty::default(),
        )
    }

    pub fn to_dot_string_default(&self) -> String
    where
        EdgeProperty: Clone + Default + Send + Sync,
    {
        self.to_dot_string(|_, _| String::new())
    }

    pub fn save_dot_default<P>(&self, path: P) -> Result<(), String>
    where
        P: AsRef<Path>,
        EdgeProperty: Clone + Default + Send + Sync,
    {
        self.save_dot(path, |_, _| String::new())
    }

    pub async fn save_dot_default_async<P>(&self, path: P) -> Result<(), String>
    where
        P: AsRef<Path>,
        EdgeProperty: Clone + Default + Send + Sync,
    {
        self.save_dot_async(path, |_, _| String::new()).await
    }

    pub fn load_dot_default<P>(path: P) -> Result<Self, String>
    where
        P: AsRef<Path>,
        EdgeProperty: Clone + Default + Send + Sync,
    {
        Self::load_dot(path, |_| ())
    }

    pub async fn load_dot_default_async<P>(path: P) -> Result<Self, String>
    where
        P: AsRef<Path>,
        EdgeProperty: Clone + Default + Send + Sync,
    {
        Self::load_dot_async(path, |_| ()).await
    }
}

pub fn num_vertices<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
) -> usize {
    graph.vertex_count()
}

pub fn num_edges<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
) -> usize {
    graph.edge_count()
}

pub fn add_vertex<EdgeProperty>(graph: &mut Graph<(), EdgeProperty>) -> VertexId<()> {
    graph.add_unit_vertex()
}

pub fn add_vertex_with_property<VertexProperty, EdgeProperty>(
    property: VertexProperty,
    graph: &mut Graph<VertexProperty, EdgeProperty>,
) -> VertexId<VertexProperty> {
    graph.add_vertex(property)
}

pub fn add_edge_default<EdgeProperty>(
    source: VertexId<()>,
    target: VertexId<()>,
    graph: &mut Graph<(), EdgeProperty>,
) -> EdgeId
where
    EdgeProperty: Clone + Default,
{
    graph.add_edge_default(source, target)
}

pub fn add_edge<VertexProperty, EdgeProperty>(
    source: VertexId<VertexProperty>,
    target: VertexId<VertexProperty>,
    graph: &mut Graph<VertexProperty, EdgeProperty>,
) -> EdgeId
where
    EdgeProperty: Clone + Default,
{
    graph.add_edge(
        source,
        target,
        1.0,
        EdgeType::Undirected,
        EdgeProperty::default(),
    )
}

pub fn add_edge_with_weight<VertexProperty, EdgeProperty>(
    source: VertexId<VertexProperty>,
    target: VertexId<VertexProperty>,
    weight: f64,
    graph: &mut Graph<VertexProperty, EdgeProperty>,
) -> EdgeId
where
    EdgeProperty: Clone + Default,
{
    graph.add_edge(
        source,
        target,
        weight,
        EdgeType::Undirected,
        EdgeProperty::default(),
    )
}

pub fn add_edge_weighted<EdgeProperty>(
    source: VertexId<()>,
    target: VertexId<()>,
    weight: f64,
    graph: &mut Graph<(), EdgeProperty>,
) -> EdgeId
where
    EdgeProperty: Clone + Default,
{
    graph.add_edge_weighted(source, target, weight)
}

pub fn degree<VertexProperty, EdgeProperty>(
    vertex: VertexId<VertexProperty>,
    graph: &Graph<VertexProperty, EdgeProperty>,
) -> usize {
    graph.degree(vertex)
}

pub fn neighbors<VertexProperty, EdgeProperty>(
    vertex: VertexId<VertexProperty>,
    graph: &Graph<VertexProperty, EdgeProperty>,
) -> Vec<VertexId<VertexProperty>> {
    graph.neighbors(vertex)
}

pub fn vertices<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
) -> Vec<VertexId<VertexProperty>> {
    graph.vertices()
}

pub fn get_edge<VertexProperty, EdgeProperty>(
    source: VertexId<VertexProperty>,
    target: VertexId<VertexProperty>,
    graph: &Graph<VertexProperty, EdgeProperty>,
) -> Option<EdgeId> {
    graph.get_edge(source, target)
}

pub fn edge<VertexProperty, EdgeProperty>(
    source: VertexId<VertexProperty>,
    target: VertexId<VertexProperty>,
    graph: &Graph<VertexProperty, EdgeProperty>,
) -> (EdgeId, bool) {
    graph.edge(source, target)
}

pub fn edges<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
) -> Vec<Edge<EdgeProperty>>
where
    EdgeProperty: Clone,
{
    graph.edges()
}

pub fn clear<VertexProperty, EdgeProperty>(graph: &mut Graph<VertexProperty, EdgeProperty>) {
    graph.clear();
}

pub fn clear_graph<VertexProperty, EdgeProperty>(graph: &mut Graph<VertexProperty, EdgeProperty>) {
    graph.clear();
}

pub fn remove_vertex<VertexProperty, EdgeProperty>(
    vertex: VertexId<VertexProperty>,
    graph: &mut Graph<VertexProperty, EdgeProperty>,
) -> bool {
    graph.remove_vertex(vertex)
}

pub fn remove_edge_by_id<VertexProperty, EdgeProperty>(
    edge_id: EdgeId,
    graph: &mut Graph<VertexProperty, EdgeProperty>,
) -> bool {
    graph.remove_edge(edge_id)
}

pub fn remove_edge_by_vertices<VertexProperty, EdgeProperty>(
    source: VertexId<VertexProperty>,
    target: VertexId<VertexProperty>,
    graph: &mut Graph<VertexProperty, EdgeProperty>,
) -> bool {
    graph.remove_edge_between(source, target)
}

pub fn source<VertexProperty, EdgeProperty>(
    edge_id: EdgeId,
    graph: &Graph<VertexProperty, EdgeProperty>,
) -> Option<VertexId<VertexProperty>> {
    graph.source(edge_id)
}

pub fn source_or_err<VertexProperty, EdgeProperty>(
    edge_id: EdgeId,
    graph: &Graph<VertexProperty, EdgeProperty>,
) -> Result<VertexId<VertexProperty>, String> {
    graph.source_or_err(edge_id)
}

pub fn target<VertexProperty, EdgeProperty>(
    edge_id: EdgeId,
    graph: &Graph<VertexProperty, EdgeProperty>,
) -> Option<VertexId<VertexProperty>> {
    graph.target(edge_id)
}

pub fn target_or_err<VertexProperty, EdgeProperty>(
    edge_id: EdgeId,
    graph: &Graph<VertexProperty, EdgeProperty>,
) -> Result<VertexId<VertexProperty>, String> {
    graph.target_or_err(edge_id)
}

pub fn shortest_path<VertexProperty, EdgeProperty>(
    source: VertexId<VertexProperty>,
    target: VertexId<VertexProperty>,
    graph: &Graph<VertexProperty, EdgeProperty>,
) -> ShortestPathResult<VertexProperty>
where
    EdgeProperty: Clone,
{
    graph.shortest_path(source, target)
}

impl<VertexProperty, EdgeProperty> Index<VertexId<VertexProperty>>
    for Graph<VertexProperty, EdgeProperty>
{
    type Output = VertexProperty;

    fn index(&self, index: VertexId<VertexProperty>) -> &Self::Output {
        self.get_vertex(index).expect("vertex does not exist")
    }
}

impl<VertexProperty, EdgeProperty> IndexMut<VertexId<VertexProperty>>
    for Graph<VertexProperty, EdgeProperty>
{
    fn index_mut(&mut self, index: VertexId<VertexProperty>) -> &mut Self::Output {
        self.get_vertex_mut(index).expect("vertex does not exist")
    }
}

fn escape_dot(value: String) -> String {
    value.replace('\\', "\\\\").replace('"', "\\\"")
}

fn unescape_dot(value: &str) -> String {
    let mut out = String::with_capacity(value.len());
    let mut chars = value.chars();
    while let Some(ch) = chars.next() {
        if ch == '\\' {
            if let Some(next) = chars.next() {
                out.push(next);
            }
        } else {
            out.push(ch);
        }
    }
    out
}

fn ensure_vertex_exists<VertexProperty, EdgeProperty>(
    graph: &mut Graph<VertexProperty, EdgeProperty>,
    raw_id: u32,
) -> VertexId<VertexProperty>
where
    VertexProperty: Clone + Default,
{
    while graph.vertex_count() <= raw_id as usize {
        let _ = graph.add_vertex(VertexProperty::default());
    }
    VertexId::new(raw_id)
}

fn parse_vertex_line<VertexProperty, EdgeProperty, PropertyReader>(
    graph: &mut Graph<VertexProperty, EdgeProperty>,
    line: &str,
    read_prop: &PropertyReader,
) -> Result<(), String>
where
    VertexProperty: Clone + Default,
    PropertyReader: Fn(&str) -> VertexProperty,
{
    let trimmed = line.trim_end_matches(';').trim();
    let id_part = trimmed.split_whitespace().next().unwrap_or(trimmed);
    let raw_id: u32 = id_part
        .parse()
        .map_err(|_| format!("invalid vertex id: {id_part}"))?;
    let vertex = ensure_vertex_exists(graph, raw_id);

    if let Some(label) = extract_attr(trimmed, "label") {
        let property = read_prop(&unescape_dot(&label));
        *graph
            .get_vertex_mut(vertex)
            .ok_or_else(|| "vertex should exist after ensure".to_string())? = property;
    }

    Ok(())
}

fn parse_edge_line<VertexProperty, EdgeProperty>(
    graph: &mut Graph<VertexProperty, EdgeProperty>,
    line: &str,
    is_digraph: bool,
) -> Result<(), String>
where
    VertexProperty: Clone + Default,
    EdgeProperty: Clone + Default,
{
    let trimmed = line.trim_end_matches(';').trim();
    let (connector, edge_type) = if trimmed.contains("->") {
        let undirected_in_mixed = extract_attr(trimmed, "dir")
            .as_deref()
            .map(|value| value == "none")
            .unwrap_or(false);
        (
            "->",
            if undirected_in_mixed {
                EdgeType::Undirected
            } else {
                EdgeType::Directed
            },
        )
    } else if trimmed.contains("--") {
        ("--", EdgeType::Undirected)
    } else {
        return Err(format!("invalid edge line: {trimmed}"));
    };

    let (left, right) = trimmed
        .split_once(connector)
        .ok_or_else(|| format!("invalid edge line: {trimmed}"))?;
    let source_id: u32 = left
        .trim()
        .parse()
        .map_err(|_| format!("invalid source vertex: {}", left.trim()))?;
    let rhs = right.trim();
    let target_part = rhs
        .split([' ', '['])
        .find(|part| !part.is_empty())
        .ok_or_else(|| format!("invalid edge target in line: {trimmed}"))?;
    let target_id: u32 = target_part
        .parse()
        .map_err(|_| format!("invalid target vertex: {target_part}"))?;

    let weight = extract_attr(trimmed, "weight")
        .and_then(|value| value.parse::<f64>().ok())
        .unwrap_or(1.0);

    let source = ensure_vertex_exists(graph, source_id);
    let target = ensure_vertex_exists(graph, target_id);

    let actual_type = if !is_digraph && connector == "--" {
        EdgeType::Undirected
    } else {
        edge_type
    };

    graph.add_edge(source, target, weight, actual_type, EdgeProperty::default());
    Ok(())
}

fn extract_attr(line: &str, key: &str) -> Option<String> {
    let start = line.find('[')?;
    let end = line.rfind(']')?;
    let attrs = &line[start + 1..end];
    for attr in split_dot_attrs(attrs) {
        let (name, value) = attr.split_once('=')?;
        if name.trim() == key {
            let raw = value.trim();
            if raw.starts_with('"') && raw.ends_with('"') && raw.len() >= 2 {
                return Some(raw[1..raw.len() - 1].to_string());
            }
            return Some(raw.to_string());
        }
    }
    None
}

fn split_dot_attrs(attrs: &str) -> Vec<&str> {
    let mut parts = Vec::new();
    let mut start = 0usize;
    let mut in_quotes = false;
    let mut escaped = false;

    for (index, ch) in attrs.char_indices() {
        if escaped {
            escaped = false;
            continue;
        }
        match ch {
            '\\' if in_quotes => escaped = true,
            '"' => in_quotes = !in_quotes,
            ',' if !in_quotes => {
                parts.push(attrs[start..index].trim());
                start = index + 1;
            }
            _ => {}
        }
    }

    if start < attrs.len() {
        parts.push(attrs[start..].trim());
    }

    parts
}
