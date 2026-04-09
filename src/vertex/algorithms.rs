use std::cmp::Ordering;
use std::collections::{BinaryHeap, HashMap, HashSet, VecDeque};

use rayon::prelude::*;

use super::{EdgeId, Graph, VertexId};

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BfsResult<VertexProperty> {
    pub parent: HashMap<VertexId<VertexProperty>, VertexId<VertexProperty>>,
    pub distance: HashMap<VertexId<VertexProperty>, usize>,
    pub discovery_order: Vec<VertexId<VertexProperty>>,
    pub target_found: bool,
}

impl<VertexProperty> Default for BfsResult<VertexProperty> {
    fn default() -> Self {
        Self {
            parent: HashMap::new(),
            distance: HashMap::new(),
            discovery_order: Vec::new(),
            target_found: false,
        }
    }
}

pub fn bfs<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
    source: VertexId<VertexProperty>,
) -> BfsResult<VertexProperty> {
    bfs_internal(graph, source, None)
}

pub fn bfs_to<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
    source: VertexId<VertexProperty>,
    target: VertexId<VertexProperty>,
) -> BfsResult<VertexProperty> {
    bfs_internal(graph, source, Some(target))
}

fn bfs_internal<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
    source: VertexId<VertexProperty>,
    target: Option<VertexId<VertexProperty>>,
) -> BfsResult<VertexProperty> {
    let mut result = BfsResult::default();
    let mut visited = HashSet::new();
    let mut queue = VecDeque::new();

    if !graph.has_vertex(source) {
        return result;
    }

    queue.push_back(source);
    visited.insert(source);
    result.distance.insert(source, 0);
    result.discovery_order.push(source);

    while let Some(current) = queue.pop_front() {
        if target == Some(current) {
            result.target_found = true;
            return result;
        }

        for neighbor in graph.neighbors(current) {
            if visited.insert(neighbor) {
                queue.push_back(neighbor);
                result.parent.insert(neighbor, current);
                result
                    .distance
                    .insert(neighbor, result.distance[&current] + 1);
                result.discovery_order.push(neighbor);
            }
        }
    }

    result
}

pub fn reconstruct_bfs_path<VertexProperty>(
    result: &BfsResult<VertexProperty>,
    source: VertexId<VertexProperty>,
    target: VertexId<VertexProperty>,
) -> Vec<VertexId<VertexProperty>> {
    reconstruct_path(&result.parent, &result.distance, source, target)
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DfsResult<VertexProperty> {
    pub parent: HashMap<VertexId<VertexProperty>, VertexId<VertexProperty>>,
    pub discovery_time: HashMap<VertexId<VertexProperty>, usize>,
    pub finish_time: HashMap<VertexId<VertexProperty>, usize>,
    pub preorder: Vec<VertexId<VertexProperty>>,
    pub postorder: Vec<VertexId<VertexProperty>>,
    pub target_found: bool,
}

impl<VertexProperty> Default for DfsResult<VertexProperty> {
    fn default() -> Self {
        Self {
            parent: HashMap::new(),
            discovery_time: HashMap::new(),
            finish_time: HashMap::new(),
            preorder: Vec::new(),
            postorder: Vec::new(),
            target_found: false,
        }
    }
}

pub fn dfs<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
    source: VertexId<VertexProperty>,
) -> DfsResult<VertexProperty> {
    let mut result = DfsResult::default();
    let mut visited = HashSet::new();
    let mut time = 0;

    if graph.has_vertex(source) {
        dfs_visit(graph, source, None, &mut visited, &mut result, &mut time);
    }

    result
}

pub fn dfs_to<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
    source: VertexId<VertexProperty>,
    target: VertexId<VertexProperty>,
) -> DfsResult<VertexProperty> {
    let mut result = DfsResult::default();
    let mut visited = HashSet::new();
    let mut time = 0;

    if graph.has_vertex(source) {
        dfs_visit(
            graph,
            source,
            Some(target),
            &mut visited,
            &mut result,
            &mut time,
        );
    }

    result
}

fn dfs_visit<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
    current: VertexId<VertexProperty>,
    target: Option<VertexId<VertexProperty>>,
    visited: &mut HashSet<VertexId<VertexProperty>>,
    result: &mut DfsResult<VertexProperty>,
    time: &mut usize,
) -> bool {
    visited.insert(current);
    result.discovery_time.insert(current, *time);
    *time += 1;
    result.preorder.push(current);

    if target == Some(current) {
        result.target_found = true;
        return true;
    }

    for neighbor in graph.neighbors(current) {
        if !visited.contains(&neighbor) {
            result.parent.insert(neighbor, current);
            if dfs_visit(graph, neighbor, target, visited, result, time) {
                return true;
            }
        }
    }

    result.finish_time.insert(current, *time);
    *time += 1;
    result.postorder.push(current);
    false
}

pub fn dfs_iterative<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
    source: VertexId<VertexProperty>,
) -> DfsResult<VertexProperty> {
    let mut result = DfsResult::default();
    let mut visited = HashSet::new();
    let mut stack = vec![source];
    let mut time = 0;

    while let Some(current) = stack.pop() {
        if !graph.has_vertex(current) || !visited.insert(current) {
            continue;
        }

        result.discovery_time.insert(current, time);
        time += 1;
        result.preorder.push(current);

        let neighbors = graph.neighbors(current);
        for neighbor in neighbors.into_iter().rev() {
            if !visited.contains(&neighbor) {
                stack.push(neighbor);
                result.parent.entry(neighbor).or_insert(current);
            }
        }
    }

    result
}

pub fn reconstruct_dfs_path<VertexProperty>(
    result: &DfsResult<VertexProperty>,
    source: VertexId<VertexProperty>,
    target: VertexId<VertexProperty>,
) -> Vec<VertexId<VertexProperty>> {
    reconstruct_path(&result.parent, &result.discovery_time, source, target)
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ComponentsResult<VertexProperty> {
    pub component_id: HashMap<VertexId<VertexProperty>, usize>,
    pub components: Vec<Vec<VertexId<VertexProperty>>>,
    pub num_components: usize,
}

impl<VertexProperty> Default for ComponentsResult<VertexProperty> {
    fn default() -> Self {
        Self {
            component_id: HashMap::new(),
            components: Vec::new(),
            num_components: 0,
        }
    }
}

pub fn connected_components<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
) -> ComponentsResult<VertexProperty> {
    let mut result = ComponentsResult::default();
    let mut visited = HashSet::new();

    for vertex in graph.vertices() {
        if visited.contains(&vertex) {
            continue;
        }

        let mut component = Vec::new();
        mark_component(graph, vertex, &mut visited, &mut component);
        let component_id = result.num_components;
        for vertex in &component {
            result.component_id.insert(*vertex, component_id);
        }
        result.components.push(component);
        result.num_components += 1;
    }

    result
}

fn mark_component<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
    vertex: VertexId<VertexProperty>,
    visited: &mut HashSet<VertexId<VertexProperty>>,
    component: &mut Vec<VertexId<VertexProperty>>,
) {
    visited.insert(vertex);
    component.push(vertex);

    for neighbor in graph.neighbors(vertex) {
        if !visited.contains(&neighbor) {
            mark_component(graph, neighbor, visited, component);
        }
    }
}

pub fn is_connected<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
) -> bool {
    connected_components(graph).num_components <= 1
}

pub fn largest_component_size<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
) -> usize {
    connected_components(graph)
        .components
        .iter()
        .map(Vec::len)
        .max()
        .unwrap_or(0)
}

pub fn same_component<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
    a: VertexId<VertexProperty>,
    b: VertexId<VertexProperty>,
) -> bool {
    let result = connected_components(graph);
    matches!(
        (result.component_id.get(&a), result.component_id.get(&b)),
        (Some(left), Some(right)) if left == right
    )
}

#[derive(Debug, Clone, PartialEq)]
pub struct ShortestPathResult<VertexProperty> {
    pub found: bool,
    pub distance: f64,
    pub path: Vec<VertexId<VertexProperty>>,
}

impl<VertexProperty> Default for ShortestPathResult<VertexProperty> {
    fn default() -> Self {
        Self {
            found: false,
            distance: f64::INFINITY,
            path: Vec::new(),
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct QueueState<VertexProperty> {
    vertex: VertexId<VertexProperty>,
    distance: f64,
}

impl<VertexProperty> PartialEq for QueueState<VertexProperty> {
    fn eq(&self, other: &Self) -> bool {
        self.distance == other.distance
    }
}

impl<VertexProperty> Eq for QueueState<VertexProperty> {}

impl<VertexProperty> PartialOrd for QueueState<VertexProperty> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl<VertexProperty> Ord for QueueState<VertexProperty> {
    fn cmp(&self, other: &Self) -> Ordering {
        other.distance.total_cmp(&self.distance)
    }
}

pub fn dijkstra<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
    source: VertexId<VertexProperty>,
    target: VertexId<VertexProperty>,
) -> ShortestPathResult<VertexProperty> {
    let mut result = ShortestPathResult::default();

    if !graph.has_vertex(source) || !graph.has_vertex(target) {
        return result;
    }

    if source == target {
        result.found = true;
        result.distance = 0.0;
        result.path.push(source);
        return result;
    }

    let mut distances = HashMap::new();
    let mut predecessors = HashMap::new();
    let mut visited = HashSet::new();
    let mut queue = BinaryHeap::new();

    for vertex in graph.vertices() {
        distances.insert(vertex, f64::INFINITY);
    }

    distances.insert(source, 0.0);
    queue.push(QueueState {
        vertex: source,
        distance: 0.0,
    });

    while let Some(QueueState { vertex, .. }) = queue.pop() {
        if !visited.insert(vertex) {
            continue;
        }

        if vertex == target {
            break;
        }

        for edge in graph.edges_from(vertex) {
            let neighbor = VertexId::new(edge.target);
            if visited.contains(&neighbor) {
                continue;
            }

            let new_distance = distances[&vertex] + edge.weight;
            if new_distance < distances[&neighbor] {
                distances.insert(neighbor, new_distance);
                predecessors.insert(neighbor, vertex);
                queue.push(QueueState {
                    vertex: neighbor,
                    distance: new_distance,
                });
            }
        }
    }

    let Some(distance) = distances.get(&target).copied() else {
        return result;
    };

    if !distance.is_finite() {
        return result;
    }

    result.found = true;
    result.distance = distance;
    result.path = reconstruct_predecessor_path(&predecessors, source, target);
    result
}

#[derive(Debug, Clone)]
pub struct BellmanFordResult<VertexProperty> {
    pub distances: HashMap<VertexId<VertexProperty>, f64>,
    pub predecessors: HashMap<VertexId<VertexProperty>, VertexId<VertexProperty>>,
    pub has_negative_cycle: bool,
    pub negative_cycle: Vec<VertexId<VertexProperty>>,
}

impl<VertexProperty> Default for BellmanFordResult<VertexProperty> {
    fn default() -> Self {
        Self {
            distances: HashMap::new(),
            predecessors: HashMap::new(),
            has_negative_cycle: false,
            negative_cycle: Vec::new(),
        }
    }
}

pub fn bellman_ford<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
    source: VertexId<VertexProperty>,
) -> BellmanFordResult<VertexProperty>
where
    EdgeProperty: Clone,
{
    let mut result = BellmanFordResult::default();
    let vertices = graph.vertices();
    if vertices.is_empty() || !graph.has_vertex(source) {
        return result;
    }

    for vertex in &vertices {
        result.distances.insert(*vertex, f64::INFINITY);
    }
    result.distances.insert(source, 0.0);

    let edges = graph.edges();
    for _ in 0..vertices.len().saturating_sub(1) {
        let mut updated = false;
        for edge in &edges {
            if edge.edge_type != super::EdgeType::Directed {
                continue;
            }
            let u = VertexId::new(edge.source);
            let v = VertexId::new(edge.target);
            let Some(distance_u) = result.distances.get(&u).copied() else {
                continue;
            };
            if distance_u.is_finite() && distance_u + edge.weight < result.distances[&v] {
                result.distances.insert(v, distance_u + edge.weight);
                result.predecessors.insert(v, u);
                updated = true;
            }
        }
        if !updated {
            break;
        }
    }

    for edge in &edges {
        if edge.edge_type != super::EdgeType::Directed {
            continue;
        }
        let u = VertexId::new(edge.source);
        let v = VertexId::new(edge.target);
        let Some(distance_u) = result.distances.get(&u).copied() else {
            continue;
        };
        if distance_u.is_finite() && distance_u + edge.weight < result.distances[&v] {
            result.has_negative_cycle = true;
            let mut current = v;
            for _ in 0..vertices.len() {
                if let Some(parent) = result.predecessors.get(&current).copied() {
                    current = parent;
                }
            }

            let cycle_start = current;
            let mut cycle = vec![cycle_start];
            let Some(mut current) = result.predecessors.get(&cycle_start).copied() else {
                result.negative_cycle = cycle;
                break;
            };
            while current != cycle_start {
                cycle.push(current);
                let Some(parent) = result.predecessors.get(&current).copied() else {
                    break;
                };
                current = parent;
            }
            cycle.push(cycle_start);
            cycle.reverse();
            result.negative_cycle = cycle;
            break;
        }
    }

    result
}

pub fn bellman_ford_path<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
    source: VertexId<VertexProperty>,
    target: VertexId<VertexProperty>,
) -> Vec<VertexId<VertexProperty>>
where
    EdgeProperty: Clone,
{
    let result = bellman_ford(graph, source);
    if result.has_negative_cycle {
        return Vec::new();
    }
    let Some(distance) = result.distances.get(&target).copied() else {
        return Vec::new();
    };
    if !distance.is_finite() {
        return Vec::new();
    }
    reconstruct_predecessor_path(&result.predecessors, source, target)
}

pub fn get_shortest_path<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
    source: VertexId<VertexProperty>,
    target: VertexId<VertexProperty>,
) -> Vec<VertexId<VertexProperty>>
where
    EdgeProperty: Clone,
{
    bellman_ford_path(graph, source, target)
}

#[derive(Debug, Clone)]
pub struct CycleResult<VertexProperty> {
    pub has_cycle: bool,
    pub cycle: Vec<VertexId<VertexProperty>>,
}

impl<VertexProperty> Default for CycleResult<VertexProperty> {
    fn default() -> Self {
        Self {
            has_cycle: false,
            cycle: Vec::new(),
        }
    }
}

pub fn find_cycle_directed<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
) -> CycleResult<VertexProperty> {
    #[derive(Clone, Copy, PartialEq, Eq)]
    enum State {
        Unvisited,
        Visiting,
        Visited,
    }

    let vertices = graph.vertices();
    let mut state = HashMap::new();
    let mut parent = HashMap::new();
    let mut cycle = Vec::new();

    for vertex in &vertices {
        state.insert(*vertex, State::Unvisited);
    }

    fn dfs<VertexProperty, EdgeProperty>(
        graph: &Graph<VertexProperty, EdgeProperty>,
        vertex: VertexId<VertexProperty>,
        state: &mut HashMap<VertexId<VertexProperty>, State>,
        parent: &mut HashMap<VertexId<VertexProperty>, VertexId<VertexProperty>>,
        cycle: &mut Vec<VertexId<VertexProperty>>,
    ) -> bool {
        state.insert(vertex, State::Visiting);

        for edge in graph.edges_from(vertex) {
            if edge.edge_type != super::EdgeType::Directed {
                continue;
            }
            let neighbor = VertexId::new(edge.target);
            match state.get(&neighbor).copied().unwrap_or(State::Unvisited) {
                State::Visiting => {
                    cycle.push(neighbor);
                    let mut current = vertex;
                    while current != neighbor {
                        cycle.push(current);
                        let Some(next) = parent.get(&current).copied() else {
                            return false;
                        };
                        current = next;
                    }
                    cycle.push(neighbor);
                    cycle.reverse();
                    return true;
                }
                State::Unvisited => {
                    parent.insert(neighbor, vertex);
                    if dfs(graph, neighbor, state, parent, cycle) {
                        return true;
                    }
                }
                State::Visited => {}
            }
        }

        state.insert(vertex, State::Visited);
        false
    }

    for vertex in vertices {
        if state[&vertex] == State::Unvisited
            && dfs(graph, vertex, &mut state, &mut parent, &mut cycle)
        {
            return CycleResult {
                has_cycle: true,
                cycle,
            };
        }
    }

    CycleResult::default()
}

pub fn has_cycle_directed<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
) -> bool {
    find_cycle_directed(graph).has_cycle
}

pub fn find_cycle_undirected<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
) -> CycleResult<VertexProperty> {
    let vertices = graph.vertices();
    let mut visited = HashSet::new();
    let mut parent = HashMap::new();
    let mut cycle = Vec::new();

    fn dfs<VertexProperty, EdgeProperty>(
        graph: &Graph<VertexProperty, EdgeProperty>,
        vertex: VertexId<VertexProperty>,
        parent_vertex: VertexId<VertexProperty>,
        visited: &mut HashSet<VertexId<VertexProperty>>,
        parent: &mut HashMap<VertexId<VertexProperty>, VertexId<VertexProperty>>,
        cycle: &mut Vec<VertexId<VertexProperty>>,
    ) -> bool {
        visited.insert(vertex);
        parent.insert(vertex, parent_vertex);

        for edge in graph.edges_from(vertex) {
            if edge.edge_type != super::EdgeType::Undirected {
                continue;
            }
            let neighbor = VertexId::new(edge.target);
            if visited.contains(&neighbor) {
                if neighbor != parent_vertex {
                    cycle.push(neighbor);
                    let mut current = vertex;
                    while current != neighbor {
                        cycle.push(current);
                        current = parent[&current];
                    }
                    cycle.push(neighbor);
                    cycle.reverse();
                    return true;
                }
            } else if dfs(graph, neighbor, vertex, visited, parent, cycle) {
                return true;
            }
        }

        false
    }

    for vertex in vertices {
        if visited.contains(&vertex) {
            continue;
        }
        if dfs(graph, vertex, vertex, &mut visited, &mut parent, &mut cycle) {
            return CycleResult {
                has_cycle: true,
                cycle,
            };
        }
    }

    CycleResult::default()
}

pub fn has_cycle_undirected<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
) -> bool {
    find_cycle_undirected(graph).has_cycle
}

#[derive(Debug, Clone, Default)]
pub struct TopologicalSortResult<VertexProperty> {
    pub order: Vec<VertexId<VertexProperty>>,
    pub is_dag: bool,
    pub cycle: Vec<VertexId<VertexProperty>>,
}

pub fn topological_sort<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
) -> TopologicalSortResult<VertexProperty>
where
    EdgeProperty: Clone,
{
    let vertices = graph.vertices();
    if vertices.is_empty() {
        return TopologicalSortResult {
            order: Vec::new(),
            is_dag: true,
            cycle: Vec::new(),
        };
    }

    let mut in_degree = HashMap::new();
    for vertex in &vertices {
        in_degree.insert(*vertex, 0usize);
    }

    let edges = graph.edges();
    for edge in &edges {
        if edge.edge_type == super::EdgeType::Directed {
            *in_degree.entry(VertexId::new(edge.target)).or_insert(0) += 1;
        }
    }

    let mut stack: Vec<_> = vertices
        .iter()
        .copied()
        .filter(|vertex| in_degree[vertex] == 0)
        .collect();
    let mut order = Vec::with_capacity(vertices.len());

    while let Some(current) = stack.pop() {
        order.push(current);
        for edge in graph.edges_from(current) {
            if edge.edge_type != super::EdgeType::Directed {
                continue;
            }
            let target = VertexId::new(edge.target);
            let degree = in_degree
                .get_mut(&target)
                .expect("target vertex should exist");
            *degree -= 1;
            if *degree == 0 {
                stack.push(target);
            }
        }
    }

    if order.len() == vertices.len() {
        return TopologicalSortResult {
            order,
            is_dag: true,
            cycle: Vec::new(),
        };
    }

    let cycle = find_cycle_directed(graph).cycle;
    TopologicalSortResult {
        order: Vec::new(),
        is_dag: false,
        cycle,
    }
}

pub fn topological_sort_dfs<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
) -> TopologicalSortResult<VertexProperty>
where
    EdgeProperty: Clone,
{
    #[derive(Clone, Copy, PartialEq, Eq)]
    enum State {
        Unvisited,
        Visiting,
        Visited,
    }

    let vertices = graph.vertices();
    if vertices.is_empty() {
        return TopologicalSortResult {
            order: Vec::new(),
            is_dag: true,
            cycle: Vec::new(),
        };
    }

    let mut state = HashMap::new();
    let mut postorder = Vec::with_capacity(vertices.len());
    let mut has_cycle = false;

    for vertex in &vertices {
        state.insert(*vertex, State::Unvisited);
    }

    fn dfs_visit<VertexProperty, EdgeProperty>(
        graph: &Graph<VertexProperty, EdgeProperty>,
        vertex: VertexId<VertexProperty>,
        state: &mut HashMap<VertexId<VertexProperty>, State>,
        postorder: &mut Vec<VertexId<VertexProperty>>,
        has_cycle: &mut bool,
    ) where
        EdgeProperty: Clone,
    {
        if *has_cycle {
            return;
        }

        state.insert(vertex, State::Visiting);

        for edge in graph.edges_from(vertex) {
            if edge.edge_type != super::EdgeType::Directed {
                continue;
            }
            let neighbor = VertexId::new(edge.target);
            match state.get(&neighbor).copied().unwrap_or(State::Unvisited) {
                State::Visiting => {
                    *has_cycle = true;
                    return;
                }
                State::Unvisited => dfs_visit(graph, neighbor, state, postorder, has_cycle),
                State::Visited => {}
            }
            if *has_cycle {
                return;
            }
        }

        state.insert(vertex, State::Visited);
        postorder.push(vertex);
    }

    for vertex in &vertices {
        if state[vertex] == State::Unvisited {
            dfs_visit(graph, *vertex, &mut state, &mut postorder, &mut has_cycle);
            if has_cycle {
                return TopologicalSortResult {
                    order: Vec::new(),
                    is_dag: false,
                    cycle: find_cycle_directed(graph).cycle,
                };
            }
        }
    }

    postorder.reverse();
    TopologicalSortResult {
        order: postorder,
        is_dag: true,
        cycle: Vec::new(),
    }
}

fn reconstruct_path<VertexProperty, Marker>(
    parents: &HashMap<VertexId<VertexProperty>, VertexId<VertexProperty>>,
    seen: &HashMap<VertexId<VertexProperty>, Marker>,
    source: VertexId<VertexProperty>,
    target: VertexId<VertexProperty>,
) -> Vec<VertexId<VertexProperty>> {
    if !seen.contains_key(&target) {
        return Vec::new();
    }

    reconstruct_predecessor_path(parents, source, target)
}

fn reconstruct_predecessor_path<VertexProperty>(
    parents: &HashMap<VertexId<VertexProperty>, VertexId<VertexProperty>>,
    source: VertexId<VertexProperty>,
    target: VertexId<VertexProperty>,
) -> Vec<VertexId<VertexProperty>> {
    let mut path = Vec::new();
    let mut current = target;

    while current != source {
        path.push(current);
        let Some(parent) = parents.get(&current).copied() else {
            return Vec::new();
        };
        current = parent;
    }

    path.push(source);
    path.reverse();
    path
}

#[derive(Debug, Clone, PartialEq)]
pub struct AStarResult<VertexProperty> {
    pub found: bool,
    pub distance: f64,
    pub path: Vec<VertexId<VertexProperty>>,
    pub nodes_explored: usize,
}

impl<VertexProperty> Default for AStarResult<VertexProperty> {
    fn default() -> Self {
        Self {
            found: false,
            distance: f64::INFINITY,
            path: Vec::new(),
            nodes_explored: 0,
        }
    }
}

pub fn astar<VertexProperty, EdgeProperty, Heuristic>(
    graph: &Graph<VertexProperty, EdgeProperty>,
    source: VertexId<VertexProperty>,
    target: VertexId<VertexProperty>,
    heuristic: Heuristic,
) -> AStarResult<VertexProperty>
where
    Heuristic: Fn(VertexId<VertexProperty>, VertexId<VertexProperty>) -> f64,
{
    let mut result = AStarResult::default();
    if !graph.has_vertex(source) || !graph.has_vertex(target) {
        return result;
    }
    if source == target {
        result.found = true;
        result.distance = 0.0;
        result.path.push(source);
        result.nodes_explored = 1;
        return result;
    }

    let mut g_score = HashMap::new();
    let mut predecessors = HashMap::new();
    let mut closed = HashSet::new();
    let mut open = BinaryHeap::new();

    for vertex in graph.vertices() {
        g_score.insert(vertex, f64::INFINITY);
    }
    g_score.insert(source, 0.0);
    open.push(QueueState {
        vertex: source,
        distance: heuristic(source, target),
    });

    while let Some(QueueState { vertex, .. }) = open.pop() {
        if !closed.insert(vertex) {
            continue;
        }
        result.nodes_explored += 1;
        if vertex == target {
            result.found = true;
            result.distance = g_score[&target];
            result.path = reconstruct_predecessor_path(&predecessors, source, target);
            return result;
        }

        for edge in graph.edges_from(vertex) {
            let neighbor = VertexId::new(edge.target);
            if closed.contains(&neighbor) {
                continue;
            }
            let tentative = g_score[&vertex] + edge.weight;
            if tentative < g_score[&neighbor] {
                g_score.insert(neighbor, tentative);
                predecessors.insert(neighbor, vertex);
                open.push(QueueState {
                    vertex: neighbor,
                    distance: tentative + heuristic(neighbor, target),
                });
            }
        }
    }

    result
}

pub fn astar_dijkstra<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
    source: VertexId<VertexProperty>,
    target: VertexId<VertexProperty>,
) -> AStarResult<VertexProperty> {
    astar(graph, source, target, |_, _| 0.0)
}

pub fn manhattan_distance(v1: usize, v2: usize, grid_width: usize) -> f64 {
    let r1 = v1 / grid_width;
    let c1 = v1 % grid_width;
    let r2 = v2 / grid_width;
    let c2 = v2 % grid_width;
    ((r1 as isize - r2 as isize).abs() + (c1 as isize - c2 as isize).abs()) as f64
}

pub fn euclidean_distance<Point>(p1: &Point, p2: &Point) -> f64
where
    Point: EuclideanPoint,
{
    let dx = p1.x() - p2.x();
    let dy = p1.y() - p2.y();
    (dx * dx + dy * dy).sqrt()
}

pub trait EuclideanPoint {
    fn x(&self) -> f64;
    fn y(&self) -> f64;
}

#[derive(Debug, Clone)]
pub struct MstResult<VertexProperty> {
    pub edges: Vec<EdgeId>,
    pub total_weight: f64,
    pub is_spanning_tree: bool,
    pub parent: HashMap<VertexId<VertexProperty>, VertexId<VertexProperty>>,
}

impl<VertexProperty> Default for MstResult<VertexProperty> {
    fn default() -> Self {
        Self {
            edges: Vec::new(),
            total_weight: 0.0,
            is_spanning_tree: false,
            parent: HashMap::new(),
        }
    }
}

pub fn prim_mst<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
) -> MstResult<VertexProperty>
where
    EdgeProperty: Clone,
{
    let vertices = graph.vertices();
    if let Some(start) = vertices.first().copied() {
        prim_mst_from(graph, start)
    } else {
        MstResult {
            is_spanning_tree: true,
            ..MstResult::default()
        }
    }
}

pub fn prim_mst_from<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
    start: VertexId<VertexProperty>,
) -> MstResult<VertexProperty>
where
    EdgeProperty: Clone,
{
    #[derive(Clone, Copy, Debug)]
    struct PrimEntry<V> {
        vertex: V,
        edge_id: EdgeId,
        weight: f64,
    }
    impl<V> PartialEq for PrimEntry<V> {
        fn eq(&self, other: &Self) -> bool {
            self.weight == other.weight
        }
    }
    impl<V> Eq for PrimEntry<V> {}
    impl<V> PartialOrd for PrimEntry<V> {
        fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
            Some(self.cmp(other))
        }
    }
    impl<V> Ord for PrimEntry<V> {
        fn cmp(&self, other: &Self) -> Ordering {
            other.weight.total_cmp(&self.weight)
        }
    }

    let mut result = MstResult::default();
    let mut in_mst = HashSet::new();
    let mut pq = BinaryHeap::new();
    let edges = graph.edges();

    if !graph.has_vertex(start) {
        return result;
    }

    in_mst.insert(start);
    for edge in &edges {
        if edge.edge_type == super::EdgeType::Undirected
            && (edge.source == start.value() || edge.target == start.value())
        {
            let neighbor = if edge.source == start.value() {
                VertexId::new(edge.target)
            } else {
                VertexId::new(edge.source)
            };
            pq.push(PrimEntry {
                vertex: neighbor,
                edge_id: edge.id,
                weight: edge.weight,
            });
        }
    }

    while let Some(PrimEntry {
        vertex,
        edge_id,
        weight,
    }) = pq.pop()
    {
        if !in_mst.insert(vertex) {
            continue;
        }
        result.edges.push(edge_id);
        result.total_weight += weight;
        if let Some(edge) = edges.iter().find(|candidate| candidate.id == edge_id) {
            let parent = if edge.source == vertex.value() {
                VertexId::new(edge.target)
            } else {
                VertexId::new(edge.source)
            };
            result.parent.insert(vertex, parent);
        }
        for edge in &edges {
            if edge.edge_type == super::EdgeType::Undirected
                && (edge.source == vertex.value() || edge.target == vertex.value())
            {
                let neighbor = if edge.source == vertex.value() {
                    VertexId::new(edge.target)
                } else {
                    VertexId::new(edge.source)
                };
                if !in_mst.contains(&neighbor) {
                    pq.push(PrimEntry {
                        vertex: neighbor,
                        edge_id: edge.id,
                        weight: edge.weight,
                    });
                }
            }
        }
    }

    result.is_spanning_tree = in_mst.len() == graph.vertex_count();
    result
}

#[derive(Debug, Clone, Default)]
pub struct SccResult<VertexProperty> {
    pub components: Vec<Vec<VertexId<VertexProperty>>>,
    pub num_components: usize,
}

pub fn strongly_connected_components<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
) -> SccResult<VertexProperty>
where
    EdgeProperty: Clone,
{
    let vertices = graph.vertices();
    let mut index = HashMap::new();
    let mut lowlink = HashMap::new();
    let mut on_stack = HashSet::new();
    let mut stack = Vec::new();
    let mut current_index = 0usize;
    let mut components = Vec::new();

    fn strongconnect<VertexProperty, EdgeProperty>(
        graph: &Graph<VertexProperty, EdgeProperty>,
        v: VertexId<VertexProperty>,
        index: &mut HashMap<VertexId<VertexProperty>, usize>,
        lowlink: &mut HashMap<VertexId<VertexProperty>, usize>,
        on_stack: &mut HashSet<VertexId<VertexProperty>>,
        stack: &mut Vec<VertexId<VertexProperty>>,
        current_index: &mut usize,
        components: &mut Vec<Vec<VertexId<VertexProperty>>>,
    ) where
        EdgeProperty: Clone,
    {
        index.insert(v, *current_index);
        lowlink.insert(v, *current_index);
        *current_index += 1;
        stack.push(v);
        on_stack.insert(v);

        for edge in graph.edges_from(v) {
            if edge.edge_type != super::EdgeType::Directed {
                continue;
            }
            let w = VertexId::new(edge.target);
            if !index.contains_key(&w) {
                strongconnect(
                    graph,
                    w,
                    index,
                    lowlink,
                    on_stack,
                    stack,
                    current_index,
                    components,
                );
                let low_v = lowlink[&v].min(lowlink[&w]);
                lowlink.insert(v, low_v);
            } else if on_stack.contains(&w) {
                let low_v = lowlink[&v].min(index[&w]);
                lowlink.insert(v, low_v);
            }
        }

        if lowlink[&v] == index[&v] {
            let mut component = Vec::new();
            while let Some(w) = stack.pop() {
                on_stack.remove(&w);
                component.push(w);
                if w == v {
                    break;
                }
            }
            component.sort();
            components.push(component);
        }
    }

    for vertex in vertices {
        if !index.contains_key(&vertex) {
            strongconnect(
                graph,
                vertex,
                &mut index,
                &mut lowlink,
                &mut on_stack,
                &mut stack,
                &mut current_index,
                &mut components,
            );
        }
    }

    components.sort_by_key(|component| component.first().copied());
    let num_components = components.len();
    SccResult {
        components,
        num_components,
    }
}

pub fn is_strongly_connected<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
) -> bool
where
    EdgeProperty: Clone,
{
    let result = strongly_connected_components(graph);
    result.num_components == 1 && !result.components.is_empty()
}

pub fn largest_scc_size<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
) -> usize
where
    EdgeProperty: Clone,
{
    strongly_connected_components(graph)
        .components
        .iter()
        .map(Vec::len)
        .max()
        .unwrap_or(0)
}

pub fn get_component_map<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
) -> HashMap<VertexId<VertexProperty>, usize>
where
    EdgeProperty: Clone,
{
    let mut component_map = HashMap::new();
    for (component_id, component) in strongly_connected_components(graph)
        .components
        .into_iter()
        .enumerate()
    {
        for vertex in component {
            component_map.insert(vertex, component_id);
        }
    }
    component_map
}

pub fn in_same_scc<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
    a: VertexId<VertexProperty>,
    b: VertexId<VertexProperty>,
) -> bool
where
    EdgeProperty: Clone,
{
    strongly_connected_components(graph)
        .components
        .iter()
        .any(|component| component.contains(&a) && component.contains(&b))
}

#[derive(Debug, Clone)]
pub struct BipartiteResult<VertexProperty> {
    pub is_bipartite: bool,
    pub coloring: HashMap<VertexId<VertexProperty>, i32>,
    pub odd_cycle: Vec<VertexId<VertexProperty>>,
}

impl<VertexProperty> Default for BipartiteResult<VertexProperty> {
    fn default() -> Self {
        Self {
            is_bipartite: true,
            coloring: HashMap::new(),
            odd_cycle: Vec::new(),
        }
    }
}

pub fn is_bipartite<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
) -> BipartiteResult<VertexProperty> {
    let mut result = BipartiteResult::default();
    for start in graph.vertices() {
        if result.coloring.contains_key(&start) {
            continue;
        }
        let mut queue = VecDeque::new();
        let mut parent = HashMap::new();
        queue.push_back(start);
        result.coloring.insert(start, 0);
        parent.insert(start, start);
        while let Some(current) = queue.pop_front() {
            let current_color = result.coloring[&current];
            for neighbor in graph.neighbors(current) {
                if let std::collections::hash_map::Entry::Vacant(entry) =
                    result.coloring.entry(neighbor)
                {
                    entry.insert(1 - current_color);
                    parent.insert(neighbor, current);
                    queue.push_back(neighbor);
                } else if result.coloring[&neighbor] == current_color {
                    result.is_bipartite = false;
                    result.odd_cycle = vec![current, neighbor];
                    return result;
                }
            }
        }
    }
    result
}

pub fn bipartite_check<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
) -> BipartiteResult<VertexProperty> {
    is_bipartite(graph)
}

pub fn is_acyclic_directed<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
) -> bool {
    !has_cycle_directed(graph)
}

pub fn is_acyclic_undirected<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
) -> bool {
    !has_cycle_undirected(graph)
}

pub fn is_acyclic<VertexProperty, EdgeProperty>(graph: &Graph<VertexProperty, EdgeProperty>) -> bool
where
    EdgeProperty: Clone,
{
    let mut has_directed = false;
    let mut has_undirected = false;
    for edge in graph.edges() {
        match edge.edge_type {
            super::EdgeType::Directed => has_directed = true,
            super::EdgeType::Undirected => has_undirected = true,
        }
    }
    if has_directed && !has_undirected {
        is_acyclic_directed(graph)
    } else {
        is_acyclic_undirected(graph)
    }
}

pub fn graph_diameter<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
) -> f64 {
    match weighted_eccentricities(graph) {
        Some(eccentricities) => eccentricities.values().copied().fold(0.0, f64::max),
        None => f64::INFINITY,
    }
}

pub fn graph_diameter_parallel<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
) -> f64
where
    VertexProperty: Sync,
    EdgeProperty: Sync,
{
    match weighted_eccentricities_parallel(graph) {
        Some(eccentricities) => eccentricities.values().copied().fold(0.0, f64::max),
        None => f64::INFINITY,
    }
}

pub fn graph_radius<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
) -> f64 {
    match weighted_eccentricities(graph) {
        Some(eccentricities) => eccentricities
            .values()
            .copied()
            .reduce(f64::min)
            .unwrap_or(0.0),
        None => f64::INFINITY,
    }
}

pub fn graph_radius_parallel<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
) -> f64
where
    VertexProperty: Sync,
    EdgeProperty: Sync,
{
    match weighted_eccentricities_parallel(graph) {
        Some(eccentricities) => eccentricities
            .values()
            .copied()
            .reduce(f64::min)
            .unwrap_or(0.0),
        None => f64::INFINITY,
    }
}

pub fn graph_center<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
) -> Vec<VertexId<VertexProperty>> {
    let Some(eccentricities) = weighted_eccentricities(graph) else {
        return Vec::new();
    };
    if eccentricities.is_empty() {
        return Vec::new();
    }
    let radius = eccentricities
        .values()
        .copied()
        .reduce(f64::min)
        .unwrap_or(0.0);
    eccentricities
        .into_iter()
        .filter_map(|(vertex, eccentricity)| {
            if (eccentricity - radius).abs() < 1e-9 {
                Some(vertex)
            } else {
                None
            }
        })
        .collect()
}

pub fn graph_center_parallel<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
) -> Vec<VertexId<VertexProperty>>
where
    VertexProperty: Sync,
    EdgeProperty: Sync,
{
    let Some(eccentricities) = weighted_eccentricities_parallel(graph) else {
        return Vec::new();
    };
    if eccentricities.is_empty() {
        return Vec::new();
    }
    let radius = eccentricities
        .values()
        .copied()
        .reduce(f64::min)
        .unwrap_or(0.0);
    let mut center: Vec<_> = eccentricities
        .into_par_iter()
        .filter_map(|(vertex, eccentricity)| {
            if (eccentricity - radius).abs() < 1e-9 {
                Some(vertex)
            } else {
                None
            }
        })
        .collect();
    center.par_sort_unstable();
    center
}

fn weighted_eccentricities<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
) -> Option<HashMap<VertexId<VertexProperty>, f64>> {
    if graph.vertex_count() == 0 {
        return Some(HashMap::new());
    }

    let vertices = graph.vertices();
    let mut eccentricities = HashMap::new();
    for source in &vertices {
        let mut farthest: f64 = 0.0;
        for target in &vertices {
            let result = dijkstra(graph, *source, *target);
            if !result.found {
                return None;
            }
            farthest = farthest.max(result.distance);
        }
        eccentricities.insert(*source, farthest);
    }
    Some(eccentricities)
}

fn weighted_eccentricities_parallel<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
) -> Option<HashMap<VertexId<VertexProperty>, f64>>
where
    VertexProperty: Sync,
    EdgeProperty: Sync,
{
    if graph.vertex_count() == 0 {
        return Some(HashMap::new());
    }

    let vertices = graph.vertices();
    vertices
        .par_iter()
        .map(|source| {
            let mut farthest: f64 = 0.0;
            for target in &vertices {
                let result = dijkstra(graph, *source, *target);
                if !result.found {
                    return None;
                }
                farthest = farthest.max(result.distance);
            }
            Some((*source, farthest))
        })
        .collect::<Option<Vec<_>>>()
        .map(|pairs| pairs.into_iter().collect())
}

pub fn degree_centrality<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
    vertex: VertexId<VertexProperty>,
) -> f64
where
    EdgeProperty: Clone,
{
    let n = graph.vertex_count();
    if n <= 1 || !graph.has_vertex(vertex) {
        return 0.0;
    }
    let has_directed = graph
        .edges()
        .iter()
        .any(|edge| edge.edge_type == super::EdgeType::Directed);
    if has_directed {
        let in_degree = graph
            .edges()
            .iter()
            .filter(|edge| {
                edge.edge_type == super::EdgeType::Directed && edge.target == vertex.value()
            })
            .count();
        (graph.degree(vertex) + in_degree) as f64 / (n - 1) as f64
    } else {
        graph.degree(vertex) as f64 / (2.0 * (n - 1) as f64)
    }
}

pub fn degree_centrality_all<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
) -> HashMap<VertexId<VertexProperty>, f64>
where
    EdgeProperty: Clone,
{
    graph
        .vertices()
        .into_iter()
        .map(|vertex| (vertex, degree_centrality(graph, vertex)))
        .collect()
}

pub fn degree_centrality_all_parallel<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
) -> HashMap<VertexId<VertexProperty>, f64>
where
    VertexProperty: Sync,
    EdgeProperty: Clone + Sync,
{
    graph
        .vertices()
        .into_par_iter()
        .map(|vertex| (vertex, degree_centrality(graph, vertex)))
        .collect()
}

pub fn closeness_centrality<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
    source: VertexId<VertexProperty>,
) -> f64 {
    let n = graph.vertex_count();
    if n <= 1 || !graph.has_vertex(source) {
        return 0.0;
    }
    let result = bfs(graph, source);
    if result.distance.len() != n {
        return 0.0;
    }
    let total: usize = result
        .distance
        .iter()
        .filter_map(|(vertex, distance)| {
            if *vertex != source {
                Some(*distance)
            } else {
                None
            }
        })
        .sum();
    if total == 0 {
        0.0
    } else {
        (n - 1) as f64 / total as f64
    }
}

pub fn closeness_centrality_all<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
) -> HashMap<VertexId<VertexProperty>, f64> {
    graph
        .vertices()
        .into_iter()
        .map(|vertex| (vertex, closeness_centrality(graph, vertex)))
        .collect()
}

pub fn closeness_centrality_all_parallel<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
) -> HashMap<VertexId<VertexProperty>, f64>
where
    VertexProperty: Sync,
    EdgeProperty: Sync,
{
    graph
        .vertices()
        .into_par_iter()
        .map(|vertex| (vertex, closeness_centrality(graph, vertex)))
        .collect()
}

pub fn betweenness_centrality<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
) -> HashMap<VertexId<VertexProperty>, f64>
where
    EdgeProperty: Clone,
{
    let vertices = graph.vertices();
    let mut betweenness = HashMap::new();
    for vertex in &vertices {
        betweenness.insert(*vertex, 0.0);
    }

    for source in &vertices {
        let mut predecessors: HashMap<VertexId<VertexProperty>, Vec<VertexId<VertexProperty>>> =
            HashMap::new();
        let mut sigma: HashMap<VertexId<VertexProperty>, f64> = HashMap::new();
        let mut distance: HashMap<VertexId<VertexProperty>, i32> = HashMap::new();
        let mut delta: HashMap<VertexId<VertexProperty>, f64> = HashMap::new();
        let mut queue = VecDeque::new();
        let mut stack = Vec::new();

        for vertex in &vertices {
            predecessors.insert(*vertex, Vec::new());
            sigma.insert(*vertex, 0.0);
            distance.insert(*vertex, -1);
            delta.insert(*vertex, 0.0);
        }

        sigma.insert(*source, 1.0);
        distance.insert(*source, 0);
        queue.push_back(*source);

        while let Some(v) = queue.pop_front() {
            stack.push(v);
            for neighbor in graph.neighbors(v) {
                if distance[&neighbor] < 0 {
                    distance.insert(neighbor, distance[&v] + 1);
                    queue.push_back(neighbor);
                }
                if distance[&neighbor] == distance[&v] + 1 {
                    sigma.insert(neighbor, sigma[&neighbor] + sigma[&v]);
                    predecessors.entry(neighbor).or_default().push(v);
                }
            }
        }

        while let Some(w) = stack.pop() {
            let preds = predecessors.get(&w).cloned().unwrap_or_default();
            for v in preds {
                let contribution = if sigma[&w] == 0.0 {
                    0.0
                } else {
                    (sigma[&v] / sigma[&w]) * (1.0 + delta[&w])
                };
                delta.insert(v, delta[&v] + contribution);
            }
            if w != *source {
                betweenness.insert(w, betweenness[&w] + delta[&w]);
            }
        }
    }

    let has_directed = graph
        .edges()
        .iter()
        .any(|edge| edge.edge_type == super::EdgeType::Directed);
    if !has_directed {
        for value in betweenness.values_mut() {
            *value /= 2.0;
        }
    }

    betweenness
}

pub fn betweenness_centrality_parallel<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
) -> HashMap<VertexId<VertexProperty>, f64>
where
    VertexProperty: Sync,
    EdgeProperty: Clone + Sync,
{
    let vertices = graph.vertices();
    let has_directed = graph
        .edges()
        .iter()
        .any(|edge| edge.edge_type == super::EdgeType::Directed);

    let reduced = vertices
        .par_iter()
        .map(|source| brandes_from_source(graph, &vertices, *source))
        .reduce(HashMap::new, |mut acc, partial| {
            for (vertex, value) in partial {
                *acc.entry(vertex).or_insert(0.0) += value;
            }
            acc
        });

    let mut betweenness = vertices
        .iter()
        .map(|vertex| (*vertex, *reduced.get(vertex).unwrap_or(&0.0)))
        .collect::<HashMap<_, _>>();

    if !has_directed {
        for value in betweenness.values_mut() {
            *value /= 2.0;
        }
    }

    betweenness
}

pub fn betweenness_centrality_normalized<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
) -> HashMap<VertexId<VertexProperty>, f64>
where
    EdgeProperty: Clone,
{
    normalize_betweenness(graph, betweenness_centrality(graph))
}

pub fn betweenness_centrality_normalized_parallel<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
) -> HashMap<VertexId<VertexProperty>, f64>
where
    VertexProperty: Sync,
    EdgeProperty: Clone + Sync,
{
    normalize_betweenness(graph, betweenness_centrality_parallel(graph))
}

pub fn top_k_central_vertices<VertexProperty>(
    centrality: &HashMap<VertexId<VertexProperty>, f64>,
    k: usize,
) -> Vec<VertexId<VertexProperty>> {
    let mut entries: Vec<_> = centrality
        .iter()
        .map(|(vertex, score)| (*vertex, *score))
        .collect();
    entries.sort_by(|(left_vertex, left_score), (right_vertex, right_score)| {
        right_score
            .total_cmp(left_score)
            .then_with(|| left_vertex.cmp(right_vertex))
    });
    entries
        .into_iter()
        .take(k)
        .map(|(vertex, _)| vertex)
        .collect()
}

pub fn most_central_vertex<VertexProperty>(
    centrality: &HashMap<VertexId<VertexProperty>, f64>,
) -> Result<VertexId<VertexProperty>, String> {
    top_k_central_vertices(centrality, 1)
        .into_iter()
        .next()
        .ok_or_else(|| "centrality map is empty".to_string())
}

fn normalize_betweenness<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
    mut centrality: HashMap<VertexId<VertexProperty>, f64>,
) -> HashMap<VertexId<VertexProperty>, f64>
where
    EdgeProperty: Clone,
{
    let n = graph.vertex_count();
    if n < 3 {
        for value in centrality.values_mut() {
            *value = 0.0;
        }
        return centrality;
    }

    let factor = ((n - 1) * (n - 2)) as f64;

    if factor > 0.0 {
        for value in centrality.values_mut() {
            *value /= factor;
        }
    }

    centrality
}

fn brandes_from_source<VertexProperty, EdgeProperty>(
    graph: &Graph<VertexProperty, EdgeProperty>,
    vertices: &[VertexId<VertexProperty>],
    source: VertexId<VertexProperty>,
) -> HashMap<VertexId<VertexProperty>, f64> {
    let mut predecessors: HashMap<VertexId<VertexProperty>, Vec<VertexId<VertexProperty>>> =
        HashMap::new();
    let mut sigma: HashMap<VertexId<VertexProperty>, f64> = HashMap::new();
    let mut distance: HashMap<VertexId<VertexProperty>, i32> = HashMap::new();
    let mut delta: HashMap<VertexId<VertexProperty>, f64> = HashMap::new();
    let mut queue = VecDeque::new();
    let mut stack = Vec::new();

    for vertex in vertices {
        predecessors.insert(*vertex, Vec::new());
        sigma.insert(*vertex, 0.0);
        distance.insert(*vertex, -1);
        delta.insert(*vertex, 0.0);
    }

    sigma.insert(source, 1.0);
    distance.insert(source, 0);
    queue.push_back(source);

    while let Some(v) = queue.pop_front() {
        stack.push(v);
        for neighbor in graph.neighbors(v) {
            if distance[&neighbor] < 0 {
                distance.insert(neighbor, distance[&v] + 1);
                queue.push_back(neighbor);
            }
            if distance[&neighbor] == distance[&v] + 1 {
                sigma.insert(neighbor, sigma[&neighbor] + sigma[&v]);
                predecessors.entry(neighbor).or_default().push(v);
            }
        }
    }

    let mut partial = HashMap::new();
    while let Some(w) = stack.pop() {
        let preds = predecessors.get(&w).cloned().unwrap_or_default();
        for v in preds {
            let contribution = if sigma[&w] == 0.0 {
                0.0
            } else {
                (sigma[&v] / sigma[&w]) * (1.0 + delta[&w])
            };
            delta.insert(v, delta[&v] + contribution);
        }
        if w != source {
            partial.insert(w, delta[&w]);
        }
    }

    partial
}
