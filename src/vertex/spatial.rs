use std::collections::{HashMap, HashSet};

use glam::DVec2;
use kiddo::{KdTree, SquaredEuclidean};

use super::{EdgeType, Graph, VertexId};

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Correspondence2d {
    pub source_index: usize,
    pub target_index: usize,
    pub distance: f64,
}

fn build_tree<VertexProperty, EdgeProperty, F>(
    graph: &Graph<VertexProperty, EdgeProperty>,
    position: F,
) -> KdTree<f64, 2>
where
    F: Fn(VertexId<VertexProperty>, &VertexProperty) -> DVec2,
{
    let mut tree = KdTree::new();
    for vertex in graph.vertices() {
        let Some(property) = graph.get_vertex(vertex) else {
            continue;
        };
        let point = position(vertex, property);
        tree.add(&[point.x, point.y], vertex.value().into());
    }
    tree
}

fn build_point_tree<Point, F>(points: &[Point], position: F) -> KdTree<f64, 2>
where
    F: Copy + Fn(&Point) -> DVec2,
{
    let mut tree = KdTree::new();
    for (index, point) in points.iter().enumerate() {
        let p = position(point);
        tree.add(&[p.x, p.y], index as u64);
    }
    tree
}

pub fn nearest_vertex_2d<VertexProperty, EdgeProperty, F>(
    graph: &Graph<VertexProperty, EdgeProperty>,
    query: DVec2,
    position: F,
) -> Result<Option<(VertexId<VertexProperty>, f64)>, String>
where
    F: Fn(VertexId<VertexProperty>, &VertexProperty) -> DVec2,
{
    if graph.vertex_count() == 0 {
        return Ok(None);
    }
    let tree = build_tree(graph, position);
    let nearest = tree.nearest_one::<SquaredEuclidean>(&[query.x, query.y]);
    Ok(Some((
        VertexId::new(nearest.item as u32),
        nearest.distance.sqrt(),
    )))
}

pub fn k_nearest_vertices_2d<VertexProperty, EdgeProperty, F>(
    graph: &Graph<VertexProperty, EdgeProperty>,
    query: DVec2,
    k: usize,
    position: F,
) -> Result<Vec<(VertexId<VertexProperty>, f64)>, String>
where
    F: Fn(VertexId<VertexProperty>, &VertexProperty) -> DVec2,
{
    if k == 0 || graph.vertex_count() == 0 {
        return Ok(Vec::new());
    }
    let tree = build_tree(graph, position);
    Ok(tree
        .nearest_n::<SquaredEuclidean>(&[query.x, query.y], k)
        .into_iter()
        .map(|hit| (VertexId::new(hit.item as u32), hit.distance.sqrt()))
        .collect())
}

pub fn vertices_within_radius_2d<VertexProperty, EdgeProperty, F>(
    graph: &Graph<VertexProperty, EdgeProperty>,
    query: DVec2,
    radius: f64,
    position: F,
) -> Result<Vec<(VertexId<VertexProperty>, f64)>, String>
where
    F: Fn(VertexId<VertexProperty>, &VertexProperty) -> DVec2,
{
    if radius < 0.0 || graph.vertex_count() == 0 {
        return Ok(Vec::new());
    }
    let tree = build_tree(graph, position);
    Ok(tree
        .within::<SquaredEuclidean>(&[query.x, query.y], radius * radius)
        .into_iter()
        .map(|hit| (VertexId::new(hit.item as u32), hit.distance.sqrt()))
        .collect())
}

pub fn connect_k_nearest_neighbors_2d<VertexProperty, EdgeProperty, F, P>(
    graph: &mut Graph<VertexProperty, EdgeProperty>,
    k: usize,
    edge_type: EdgeType,
    position: F,
    make_edge_property: P,
) -> Result<usize, String>
where
    F: Copy + Fn(VertexId<VertexProperty>, &VertexProperty) -> DVec2,
    P: Copy + Fn(VertexId<VertexProperty>, VertexId<VertexProperty>, f64) -> EdgeProperty,
    EdgeProperty: Clone,
{
    if k == 0 || graph.vertex_count() == 0 {
        return Ok(0);
    }

    let tree = build_tree(graph, position);
    let vertices = graph.vertices();
    let mut added = 0usize;
    let mut seen = HashSet::new();

    for source in vertices {
        let source_pos = position(
            source,
            graph
                .get_vertex(source)
                .ok_or_else(|| "source vertex missing".to_string())?,
        );
        let neighbors = tree.nearest_n::<SquaredEuclidean>(&[source_pos.x, source_pos.y], k + 1);
        for neighbor in neighbors {
            let target = VertexId::new(neighbor.item as u32);
            if target == source {
                continue;
            }
            let key = canonical_edge_key(source.value(), target.value(), edge_type);
            if !seen.insert(key) {
                continue;
            }
            if graph.has_edge(source, target) {
                continue;
            }
            let distance = neighbor.distance.sqrt();
            let property = make_edge_property(source, target, distance);
            graph.add_edge(source, target, distance, edge_type, property);
            added += 1;
        }
    }

    Ok(added)
}

pub fn connect_vertices_within_radius_2d<VertexProperty, EdgeProperty, F, P>(
    graph: &mut Graph<VertexProperty, EdgeProperty>,
    radius: f64,
    edge_type: EdgeType,
    position: F,
    make_edge_property: P,
) -> Result<usize, String>
where
    F: Copy + Fn(VertexId<VertexProperty>, &VertexProperty) -> DVec2,
    P: Copy + Fn(VertexId<VertexProperty>, VertexId<VertexProperty>, f64) -> EdgeProperty,
    EdgeProperty: Clone,
{
    if radius < 0.0 || graph.vertex_count() == 0 {
        return Ok(0);
    }

    let tree = build_tree(graph, position);
    let vertices = graph.vertices();
    let mut added = 0usize;
    let mut seen = HashSet::new();

    for source in vertices {
        let source_pos = position(
            source,
            graph
                .get_vertex(source)
                .ok_or_else(|| "source vertex missing".to_string())?,
        );
        let hits = tree.within::<SquaredEuclidean>(&[source_pos.x, source_pos.y], radius * radius);
        for hit in hits {
            let target = VertexId::new(hit.item as u32);
            if target == source {
                continue;
            }
            let key = canonical_edge_key(source.value(), target.value(), edge_type);
            if !seen.insert(key) {
                continue;
            }
            if graph.has_edge(source, target) {
                continue;
            }
            let distance = hit.distance.sqrt();
            let property = make_edge_property(source, target, distance);
            graph.add_edge(source, target, distance, edge_type, property);
            added += 1;
        }
    }

    Ok(added)
}

pub fn knn_graph_2d<VertexProperty, F>(
    points: impl IntoIterator<Item = VertexProperty>,
    k: usize,
    position: F,
) -> Result<Graph<VertexProperty, ()>, String>
where
    F: Copy + Fn(&VertexProperty) -> DVec2,
{
    let mut graph = Graph::new();
    for point in points {
        graph.add_vertex(point);
    }
    connect_k_nearest_neighbors_2d(
        &mut graph,
        k,
        EdgeType::Undirected,
        |_, p| position(p),
        |_, _, _| (),
    )?;
    Ok(graph)
}

pub fn radius_graph_2d<VertexProperty, F>(
    points: impl IntoIterator<Item = VertexProperty>,
    radius: f64,
    position: F,
) -> Result<Graph<VertexProperty, ()>, String>
where
    F: Copy + Fn(&VertexProperty) -> DVec2,
{
    let mut graph = Graph::new();
    for point in points {
        graph.add_vertex(point);
    }
    connect_vertices_within_radius_2d(
        &mut graph,
        radius,
        EdgeType::Undirected,
        |_, p| position(p),
        |_, _, _| (),
    )?;
    Ok(graph)
}

pub fn nearest_neighbor_correspondences_2d<SourcePoint, TargetPoint, FS, FT>(
    sources: &[SourcePoint],
    targets: &[TargetPoint],
    source_position: FS,
    target_position: FT,
) -> Result<Vec<Correspondence2d>, String>
where
    FS: Copy + Fn(&SourcePoint) -> DVec2,
    FT: Copy + Fn(&TargetPoint) -> DVec2,
{
    if sources.is_empty() || targets.is_empty() {
        return Ok(Vec::new());
    }

    let tree = build_point_tree(targets, target_position);
    let mut correspondences = Vec::with_capacity(sources.len());
    for (source_index, source) in sources.iter().enumerate() {
        let point = source_position(source);
        let nearest = tree.nearest_one::<SquaredEuclidean>(&[point.x, point.y]);
        correspondences.push(Correspondence2d {
            source_index,
            target_index: nearest.item as usize,
            distance: nearest.distance.sqrt(),
        });
    }
    Ok(correspondences)
}

pub fn radius_limited_correspondences_2d<SourcePoint, TargetPoint, FS, FT>(
    sources: &[SourcePoint],
    targets: &[TargetPoint],
    radius: f64,
    source_position: FS,
    target_position: FT,
) -> Result<Vec<Correspondence2d>, String>
where
    FS: Copy + Fn(&SourcePoint) -> DVec2,
    FT: Copy + Fn(&TargetPoint) -> DVec2,
{
    if radius < 0.0 || sources.is_empty() || targets.is_empty() {
        return Ok(Vec::new());
    }

    let tree = build_point_tree(targets, target_position);
    let mut correspondences = Vec::new();
    for (source_index, source) in sources.iter().enumerate() {
        let point = source_position(source);
        let nearest = tree.nearest_one::<SquaredEuclidean>(&[point.x, point.y]);
        let distance = nearest.distance.sqrt();
        if distance <= radius {
            correspondences.push(Correspondence2d {
                source_index,
                target_index: nearest.item as usize,
                distance,
            });
        }
    }
    Ok(correspondences)
}

pub fn mutual_nearest_correspondences_2d<SourcePoint, TargetPoint, FS, FT>(
    sources: &[SourcePoint],
    targets: &[TargetPoint],
    max_distance: Option<f64>,
    source_position: FS,
    target_position: FT,
) -> Result<Vec<Correspondence2d>, String>
where
    FS: Copy + Fn(&SourcePoint) -> DVec2,
    FT: Copy + Fn(&TargetPoint) -> DVec2,
{
    if sources.is_empty() || targets.is_empty() {
        return Ok(Vec::new());
    }

    let forward =
        nearest_neighbor_correspondences_2d(sources, targets, source_position, target_position)?;
    let reverse =
        nearest_neighbor_correspondences_2d(targets, sources, target_position, source_position)?;
    let reverse_map: HashMap<usize, usize> = reverse
        .into_iter()
        .map(|corr| (corr.source_index, corr.target_index))
        .collect();

    Ok(forward
        .into_iter()
        .filter(|corr| reverse_map.get(&corr.target_index) == Some(&corr.source_index))
        .filter(|corr| max_distance.is_none_or(|limit| corr.distance <= limit))
        .collect())
}

fn canonical_edge_key(source: u32, target: u32, edge_type: EdgeType) -> (u32, u32, EdgeType) {
    match edge_type {
        EdgeType::Directed => (source, target, edge_type),
        EdgeType::Undirected => {
            if source < target {
                (source, target, edge_type)
            } else {
                (target, source, edge_type)
            }
        }
    }
}
