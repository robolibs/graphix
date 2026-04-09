use std::rc::Rc;

use glam::DVec2;

use graphix::factor::{
    Factor, FactorLike, Graph, SE2d, Values, Vec3d, cauchy_loss, huber_loss, no_loss, se2_sigmas,
    tukey_loss,
};
use graphix::{L, X};

#[test]
fn factor_struct_exposes_keys_and_membership() {
    let factor = Factor::new(vec![1, 2, 3]);
    assert_eq!(factor.size(), 3);
    assert_eq!(factor.keys(), &[1, 2, 3]);
    assert!(factor.involves(2));
    assert!(!factor.involves(99));
}

#[test]
fn factor_supports_tuple_construction() {
    let single = Factor::from((42u64,));
    let triple = Factor::from((10u64, 20u64, 30u64));
    let large = Factor::from((1u64, 2, 3, 4, 5, 6, 7, 8));
    let dupes = Factor::from([9u64, 9, 10]);
    let from_vec = Factor::from(vec![11u64, 12, 13]);

    assert_eq!(single.keys(), &[42]);
    assert_eq!(triple.keys(), &[10, 20, 30]);
    assert_eq!(large.keys(), &[1, 2, 3, 4, 5, 6, 7, 8]);
    assert_eq!(dupes.keys(), &[9, 9, 10]);
    assert_eq!(from_vec.keys(), &[11, 12, 13]);
}

#[test]
fn factor_graph_collects_unique_keys() {
    let mut graph = Graph::<Factor>::new();
    graph.add(Rc::new(Factor::new(vec![X(0).into(), X(1).into()])));
    graph.add(Rc::new(Factor::new(vec![X(1).into(), L(0).into()])));
    graph.add_optional(None);

    assert_eq!(graph.size(), 2);
    assert!(!graph.is_empty());
    assert!(!graph.empty());
    assert!(graph.at(0).is_some());
    assert!(graph.at(10).is_none());
    assert!(graph.at_or_err(0).is_ok());
    assert!(graph.at_or_err(10).is_err());
    let keys = graph.keys();
    assert_eq!(keys.len(), 3);

    graph.clear();
    assert!(graph.is_empty());
    assert!(graph.empty());
}

#[test]
fn factor_graph_clone_keeps_shared_factors_without_aliasing_container_state() {
    let mut graph = Graph::<Factor>::new();
    graph.add(Rc::new(Factor::new(vec![X(0).into(), X(1).into()])));

    let mut cloned = graph.clone();
    cloned.add(Rc::new(Factor::new(vec![L(0).into()])));

    assert_eq!(graph.size(), 1);
    assert_eq!(cloned.size(), 2);
    assert!(graph.keys().contains(&X(0).into()));
    assert!(cloned.keys().contains(&L(0).into()));
}

#[test]
fn factor_defaults_and_large_graphs_match_expected_shape() {
    let empty = Factor::default();
    assert_eq!(empty.size(), 0);
    assert!(empty.keys().is_empty());
    assert!(!empty.involves(1));

    let mut graph = Graph::<Factor>::new();
    for i in 0..100_u64 {
        graph.add(Rc::new(Factor::new(vec![i, i + 1])));
    }
    assert_eq!(graph.size(), 100);
    let keys = graph.keys();
    assert_eq!(keys.len(), 101);
    assert!(keys.contains(&0));
    assert!(keys.contains(&100));
}

#[test]
fn values_support_type_erasure_and_copy() {
    let mut values = Values::new();
    values.insert(1, 5.0).unwrap();
    values.insert(2, String::from("hello")).unwrap();

    assert_eq!(*values.at::<f64>(1).unwrap(), 5.0);
    assert_eq!(values.at::<String>(2).unwrap(), "hello");

    let copy = values.clone();
    assert_eq!(*copy.at::<f64>(1).unwrap(), 5.0);
    assert_eq!(copy.at::<String>(2).unwrap(), "hello");
}

#[derive(Clone, Debug, PartialEq)]
struct CustomValue {
    count: i32,
    weight: f64,
}

#[test]
fn values_support_custom_and_mixed_types() {
    let mut values = Values::new();
    values.insert(1, 42_i32).unwrap();
    values.insert(2, vec![1.0, 2.0, 3.0]).unwrap();
    values
        .insert(
            3,
            CustomValue {
                count: 7,
                weight: 2.5,
            },
        )
        .unwrap();

    assert_eq!(*values.at::<i32>(1).unwrap(), 42);
    assert_eq!(values.at::<Vec<f64>>(2).unwrap(), &vec![1.0, 2.0, 3.0]);
    assert_eq!(
        values.at::<CustomValue>(3).unwrap(),
        &CustomValue {
            count: 7,
            weight: 2.5
        }
    );
}

#[test]
fn values_iteration_and_key_collection_match_ordered_map_semantics() {
    let mut values = Values::new();
    values.insert(10, 100_i32).unwrap();
    values.insert(20, 200_i32).unwrap();
    values.insert(30, 300_i32).unwrap();

    assert_eq!(values.keys(), vec![10, 20, 30]);

    let collected: Vec<_> = (&values).into_iter().map(|(key, _)| *key).collect();
    assert_eq!(collected, vec![10, 20, 30]);

    let sum: i32 = (&values)
        .into_iter()
        .map(|(key, _)| *values.at::<i32>(*key).unwrap())
        .sum();
    assert_eq!(sum, 600);
}

#[test]
fn values_mixed_type_iteration_keeps_keys_accessible() {
    let mut values = Values::new();
    values.insert(1, 100_i32).unwrap();
    values.insert(2, 2.5_f64).unwrap();
    values.insert(3, String::from("test")).unwrap();

    let mut count = 0;
    for (key, _) in &values {
        count += 1;
        assert!(values.exists(*key));
    }

    assert_eq!(count, 3);
}

#[test]
fn values_reject_duplicates_and_type_mismatches_and_iterate_in_key_order() {
    let mut values = Values::new();
    values.insert(5, 5.0f64).unwrap();
    values.insert(1, 1_i32).unwrap();
    values.insert(3, String::from("three")).unwrap();

    assert!(values.insert(5, 6.0f64).is_err());
    assert!(values.at::<f64>(99).is_err());
    assert!(values.at::<f64>(1).is_err());

    assert_eq!(values.keys(), vec![1, 3, 5]);

    let iterated: Vec<_> = values.into_iter().map(|(key, _)| *key).collect();
    assert_eq!(iterated, vec![1, 3, 5]);

    let mut clone = values.clone();
    clone.erase(3);
    assert_eq!(clone.keys(), vec![1, 5]);
    assert_eq!(values.keys(), vec![1, 3, 5]);
}

#[test]
fn robust_losses_return_reasonable_values() {
    let null = no_loss();
    let huber = huber_loss(1.345);
    let cauchy = cauchy_loss(2.3849);
    let tukey = tukey_loss(4.6851);

    assert_eq!(null.evaluate(4.0), 4.0);
    assert!(huber.evaluate(4.0) <= 4.0);
    assert!(cauchy.weight(4.0) < 1.0);
    assert_eq!(tukey.weight(1000.0), 0.0);
}

#[test]
fn robust_losses_match_expected_formulas_and_ordering() {
    let null = no_loss();
    let huber = huber_loss(1.345);
    let cauchy = cauchy_loss(2.3849);
    let tukey = tukey_loss(4.6851);

    let small = 0.5;
    assert!((null.evaluate(small) - small).abs() < 1e-12);
    assert!((huber.evaluate(small) - 0.25).abs() < 1e-12);
    assert!((huber.weight(small) - 1.0).abs() < 1e-12);

    let large = 25.0;
    assert!(null.evaluate(large) > huber.evaluate(large));
    assert!(null.evaluate(large) > cauchy.evaluate(large));
    assert!(null.evaluate(large) > tukey.evaluate(large));
    assert!(huber.weight(small) >= huber.weight(large));
    assert!(cauchy.weight(small) > cauchy.weight(large));
    assert!(tukey.weight(small) >= tukey.weight(large));
}

#[test]
fn factor_public_geometry_types_remain_small_and_useful() {
    let pose = SE2d::from_translation_angle(DVec2::new(1.0, 2.0), 0.25);
    assert!((pose.angle() - 0.25).abs() < 1e-12);
    let delta = Vec3d::from_array([0.5, -0.25, 0.1]);
    let updated = pose.retract(delta);
    assert!((updated.x() - 1.5).abs() < 1e-12);
    assert!((updated.y() - 1.75).abs() < 1e-12);
    assert_eq!(pose.translation(), DVec2::new(1.0, 2.0));
    assert_eq!(SE2d::identity(), SE2d::new(0.0, 0.0, 0.0));
    assert_eq!(
        se2_sigmas(DVec2::new(0.1, 0.2), 0.3),
        Vec3d::new(0.1, 0.2, 0.3)
    );
}

#[test]
fn se2_helpers_cover_transform_and_relative_pose_workflows() {
    let a = SE2d::from_translation_angle(DVec2::new(1.0, 0.0), 0.0);
    let b = SE2d::from_translation_angle(DVec2::new(3.0, 2.0), 0.5);

    let relative = a.between(b);
    let recomposed = a.compose(relative);

    assert_eq!(recomposed, b);
    assert_eq!(
        a.transform_point(DVec2::new(2.0, 1.0)),
        DVec2::new(3.0, 1.0)
    );
    assert_eq!(
        a.inverse_transform_point(DVec2::new(3.0, 1.0)),
        DVec2::new(2.0, 1.0)
    );
}
