use graphix::vertex::algorithms::topological_sort;
use graphix::vertex::{EdgeType, Graph};

fn main() {
    let mut dag = Graph::<&'static str, ()>::new();
    let parse = dag.add_vertex("parse");
    let analyze = dag.add_vertex("analyze");
    let build = dag.add_vertex("build");
    let package = dag.add_vertex("package");

    dag.add_edge(parse, analyze, 1.0, EdgeType::Directed, ());
    dag.add_edge(analyze, build, 1.0, EdgeType::Directed, ());
    dag.add_edge(build, package, 1.0, EdgeType::Directed, ());

    let result = topological_sort(&dag);
    let labels: Vec<_> = result.order.into_iter().map(|id| dag[id]).collect();
    println!("is dag: {}", result.is_dag);
    println!("order: {:?}", labels);
}
