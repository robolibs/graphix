use graphix::vertex::property_map::{
    CompositePropertyMap, get, make_associative_property_map, make_vector_property_map, put,
};
use graphix::vertex::{EdgeType, Graph};

fn main() {
    let mut graph = Graph::<&'static str, ()>::new();
    let start = graph.add_vertex("Start");
    let goal = graph.add_vertex("Goal");
    let edge = graph.add_edge(start, goal, 1.0, EdgeType::Directed, ());

    let names = make_associative_property_map::<_, String>();
    let costs = make_vector_property_map(4, 0.0f64);
    let capacities = make_associative_property_map::<_, i32>();
    let start_idx = start.value() as usize;

    put(&*names, &start, "start".to_string());
    put(&*names, &goal, "goal".to_string());
    put(&*costs, &start_idx, 0.0);
    put(&*capacities, &edge, 100);

    let mut composite =
        CompositePropertyMap::<graphix::vertex::VertexId<&'static str>, usize>::new();
    composite.add_vertex_property("name", names.clone());
    composite.add_edge_property("capacity", capacities.clone());

    println!("graph vertices: {}", graph.vertex_count());
    println!("name(start): {}", get(&*names, &start).unwrap());
    println!("capacity(edge): {}", get(&*capacities, &edge).unwrap());
    println!(
        "composite has name map: {}",
        composite.has_vertex_property("name")
    );
}
