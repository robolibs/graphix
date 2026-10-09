use std::rc::Rc;

use graphix::factor::{
    FactorGraphAdapter, GaussNewtonOptimizer, Graph as FactorGraph, PriorFactor, Values,
};
use graphix::vertex::algorithms::{
    betweenness_centrality_parallel, bfs_to, closeness_centrality_all_parallel,
    graph_center_parallel, reconstruct_bfs_path, topological_sort,
};
use graphix::vertex::property_map::{get, make_associative_property_map_with_default, put};
use graphix::vertex::views::{filter_edges_view, reversed};
use graphix::{factor, vertex};

fn main() {
    let mut vertex_graph = vertex::Graph::<(), ()>::new();
    let a = vertex_graph.add_unit_vertex();
    let b = vertex_graph.add_unit_vertex();
    let c = vertex_graph.add_unit_vertex();
    let ab = vertex_graph.add_edge(a, b, 1.0, vertex::EdgeType::Undirected, ());
    let bc = vertex_graph.add_edge(b, c, 2.0, vertex::EdgeType::Undirected, ());

    let mut factor_graph = factor::Graph::<MockFactor>::new();
    factor_graph.add(Rc::new(MockFactor::new(vec![
        graphix::X(0).into(),
        graphix::L(0).into(),
    ])));
    let bfs = bfs_to(&vertex_graph, a, c);
    let shortest = vertex_graph.shortest_path(a, c);

    let mut dag = vertex::Graph::<&'static str, ()>::new();
    let parse = dag.add_vertex("parse");
    let analyze = dag.add_vertex("analyze");
    let build = dag.add_vertex("build");
    dag.add_edge(parse, analyze, 1.0, vertex::EdgeType::Directed, ());
    dag.add_edge(analyze, build, 1.0, vertex::EdgeType::Directed, ());
    let topo = topological_sort(&dag);

    let centrality = closeness_centrality_all_parallel(&vertex_graph);
    let between = betweenness_centrality_parallel(&vertex_graph);
    let centers = graph_center_parallel(&vertex_graph);

    let colors = make_associative_property_map_with_default::<_, &'static str>("white");
    put(&*colors, &a, "gray");
    put(&*colors, &b, "black");

    let reversed_dag = reversed(&dag);
    let heavy_edges = filter_edges_view(&vertex_graph, |edge| edge.weight >= 2.0);

    let mut optimization_graph: FactorGraph<PriorFactor> = FactorGraph::new();
    optimization_graph.add(Rc::new(
        PriorFactor::new(graphix::X(0).into(), 5.0, 0.5).unwrap(),
    ));
    let mut initial = Values::new();
    initial.insert(graphix::X(0).into(), 0.0f64).unwrap();
    let optimized = GaussNewtonOptimizer::new().optimize(&optimization_graph, &initial);
    let adapter = FactorGraphAdapter::new(&optimization_graph, &initial).unwrap();

    println!("Graphix Rust library");
    println!("  - vertex graph vertices: {}", vertex_graph.vertex_count());
    println!(
        "  - bfs path a->c: {:?}",
        reconstruct_bfs_path(&bfs, a, c)
            .into_iter()
            .map(|id| id.value())
            .collect::<Vec<_>>()
    );
    println!("  - dijkstra distance a->c: {}", shortest.distance);
    println!(
        "  - shortest path a->c: {:?}",
        shortest
            .path
            .iter()
            .map(|id| id.value())
            .collect::<Vec<_>>()
    );
    println!(
        "  - edge endpoints: a-b=({},{}) b-c=({},{})",
        vertex_graph.source_or_err(ab).unwrap().value(),
        vertex_graph.target_or_err(ab).unwrap().value(),
        vertex_graph.source_or_err(bc).unwrap().value(),
        vertex_graph.target_or_err(bc).unwrap().value()
    );
    println!(
        "  - topological order: {:?}",
        topo.order.into_iter().map(|id| dag[id]).collect::<Vec<_>>()
    );
    println!("  - closeness centrality: {:?}", centrality);
    println!("  - betweenness centrality: {:?}", between);
    println!(
        "  - graph center: {:?}",
        centers.into_iter().map(|id| id.value()).collect::<Vec<_>>()
    );
    println!(
        "  - property map colors: a={}, c={}",
        get(&*colors, &a).unwrap(),
        get(&*colors, &c).unwrap()
    );
    println!(
        "  - reversed dag neighbors(build): {:?}",
        reversed_dag
            .neighbors(build)
            .into_iter()
            .map(|id| dag[id])
            .collect::<Vec<_>>()
    );
    println!(
        "  - heavy-edge view edge count: {}",
        heavy_edges.edge_count()
    );
    println!("  - factor graph factors: {}", factor_graph.len());
    println!(
        "  - optimizer result x0: {:.3}",
        optimized.values.at::<f64>(graphix::X(0).into()).unwrap()
    );
    println!("  - factor adapter parameter dim: {}", adapter.param_dim());
    println!("  - robust SLAM example: cargo run --example robust_slam");
}

#[derive(Debug, Clone)]
struct MockFactor {
    keys: Vec<graphix::Key>,
}

impl MockFactor {
    fn new(keys: Vec<graphix::Key>) -> Self {
        Self { keys }
    }
}

impl factor::FactorLike for MockFactor {
    fn keys(&self) -> &[graphix::Key] {
        &self.keys
    }
}
