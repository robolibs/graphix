use graphix::vertex::property_map::{
    AssociativePropertyMap, CompositePropertyMap, ConstantPropertyMap, IdentityPropertyMap,
    PropertyMap, VectorPropertyMap, clear, contains, get, make_associative_property_map,
    make_associative_property_map_with_default, make_constant_property_map,
    make_identity_property_map, make_vector_property_map, put, size,
};
use graphix::vertex::{EdgeType, Graph};

#[test]
fn associative_property_map_supports_basic_operations() {
    let pmap = AssociativePropertyMap::<usize, String>::new();
    pmap.put(&0, "vertex0".to_string());
    pmap.put(&1, "vertex1".to_string());

    assert_eq!(pmap.get(&0).unwrap(), "vertex0");
    assert_eq!(pmap.get(&1).unwrap(), "vertex1");
    assert_eq!(pmap.size(), 2);
    assert!(pmap.contains(&0));
    assert!(!pmap.contains(&2));

    let mut entries = pmap.entries();
    entries.sort_by_key(|(key, _)| *key);
    assert_eq!(entries.len(), 2);
    assert_eq!(entries[0].1, "vertex0");

    pmap.erase(&1);
    assert_eq!(pmap.size(), 1);
}

#[test]
fn associative_property_map_supports_iteration_and_updates() {
    let pmap = AssociativePropertyMap::<usize, i32>::new();
    pmap.put(&0, 10);
    pmap.put(&1, 20);
    pmap.put(&2, 30);
    pmap.put(&1, 25);

    let mut collected: Vec<_> = (&pmap).into_iter().collect();
    collected.sort_by_key(|(key, _)| *key);

    assert_eq!(collected, vec![(0, 10), (1, 25), (2, 30)]);
    assert_eq!(collected.iter().map(|(_, value)| *value).sum::<i32>(), 65);
}

#[test]
fn associative_property_map_defaults_and_free_functions_work() {
    let pmap = make_associative_property_map_with_default::<usize, i32>(999);
    put(&*pmap, &0, 100);
    put(&*pmap, &1, 200);

    assert_eq!(get(&*pmap, &0).unwrap(), 100);
    assert_eq!(get(&*pmap, &55).unwrap(), 999);
    assert!(contains(&*pmap, &0));
    assert!(!contains(&*pmap, &55));
    assert_eq!(size(&*pmap), 2);
    clear(&*pmap);
    assert_eq!(size(&*pmap), 0);
}

#[test]
fn vector_constant_and_identity_property_maps_work() {
    let vector = VectorPropertyMap::new(5, 0);
    vector.put(&2, 42);
    vector.put(&8, 80);
    assert_eq!(vector.get(&2).unwrap(), 42);
    assert_eq!(vector.get(&8).unwrap(), 80);
    assert_eq!(vector.get(&6).unwrap(), 0);
    assert_eq!(vector.size(), 9);
    vector.reserve(100);
    vector.resize(12);
    assert_eq!(vector.size(), 12);
    assert_eq!(vector.get(&10).unwrap(), 0);
    vector.set_default(999);
    assert_eq!(vector.get(&50).unwrap(), 999);

    let constant = ConstantPropertyMap::<usize, &str>::new("fixed");
    assert_eq!(constant.get(&0).unwrap(), "fixed");
    constant.put(&0, "ignored");
    assert_eq!(constant.get(&123).unwrap(), "fixed");
    assert!(constant.contains(&123));
    assert_eq!(constant.size(), 0);

    let identity = IdentityPropertyMap::<usize>::new();
    assert_eq!(identity.get(&42).unwrap(), 42);
    identity.put(&42, 1);
    assert_eq!(identity.get(&42).unwrap(), 42);
    assert!(identity.contains(&42));
    assert_eq!(identity.size(), 0);
}

#[test]
fn factory_functions_and_composite_registry_work() {
    let colors = make_associative_property_map::<usize, String>();
    let weights = make_vector_property_map(4, 0.0f64);
    let visited = make_associative_property_map_with_default::<usize, bool>(false);
    let constant = make_constant_property_map::<usize, i32>(7);
    let identity = make_identity_property_map::<usize>();

    put(&*colors, &0, "red".to_string());
    put(&*weights, &3, 9.5);
    assert_eq!(get(&*constant, &999).unwrap(), 7);
    assert_eq!(get(&*identity, &8).unwrap(), 8);

    let mut composite = CompositePropertyMap::<usize, usize>::new();
    composite.add_vertex_property("color", colors.clone());
    composite.add_vertex_property("weight", weights.clone());
    composite.add_vertex_property("visited", visited.clone());

    assert!(composite.has_vertex_property("color"));
    assert!(composite.has_vertex_property("weight"));
    assert!(!composite.has_edge_property("cost"));

    let retrieved = composite
        .get_vertex_property::<AssociativePropertyMap<usize, String>>("color")
        .unwrap();
    assert_eq!(retrieved.get(&0).unwrap(), "red");

    let retrieved_weights = composite
        .get_vertex_property::<VectorPropertyMap<f64>>("weight")
        .unwrap();
    assert_eq!(retrieved_weights.get(&3).unwrap(), 9.5);

    let retrieved_visited = composite
        .get_vertex_property::<AssociativePropertyMap<usize, bool>>("visited")
        .unwrap();
    assert!(!retrieved_visited.get(&10).unwrap());

    composite.remove_vertex_property("weight");
    assert!(!composite.has_vertex_property("weight"));
    composite.clear();
    assert!(!composite.has_vertex_property("color"));
}

#[test]
fn property_maps_integrate_with_graph_vertex_and_edge_ids() {
    let mut graph = Graph::<&'static str, ()>::new();
    let alice = graph.add_vertex("Alice");
    let bob = graph.add_vertex("Bob");
    let edge = graph.add_edge(alice, bob, 1.0, EdgeType::Directed, ());

    let ages = make_associative_property_map::<_, i32>();
    let cities = make_associative_property_map_with_default::<_, &'static str>("Unknown");
    let capacities = make_associative_property_map::<_, i32>();

    put(&*ages, &alice, 30);
    put(&*ages, &bob, 25);
    put(&*cities, &alice, "NYC");
    put(&*capacities, &edge, 100);

    assert_eq!(graph[alice], "Alice");
    assert_eq!(get(&*ages, &alice).unwrap(), 30);
    assert_eq!(get(&*ages, &bob).unwrap(), 25);
    assert_eq!(get(&*cities, &bob).unwrap(), "Unknown");
    assert_eq!(get(&*capacities, &edge).unwrap(), 100);
}
