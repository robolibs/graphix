use graphix::{Key, L, SmallVec, X};

#[test]
fn smallvec_default_and_initializer_like_usage() {
    let mut vec = SmallVec::<i32, 4>::new();
    assert!(vec.empty());
    assert_eq!(vec.size(), 0);

    vec.push_back(1);
    vec.push_back(2);
    vec.push_back(3);
    assert_eq!(vec.size(), 3);
    assert_eq!(vec[0], 1);
    assert_eq!(vec[1], 2);
    assert_eq!(vec[2], 3);
}

#[test]
fn smallvec_transitions_to_heap_and_preserves_values() {
    let mut vec = SmallVec::<i32, 4>::new();
    for value in 1..=6 {
        vec.push_back(value);
    }

    assert_eq!(vec.size(), 6);
    assert_eq!(vec[0], 1);
    assert_eq!(vec[4], 5);
    assert_eq!(vec[5], 6);
}

#[test]
fn smallvec_copy_clone_clear_and_iteration_work() {
    let mut vec = SmallVec::<i32, 4>::new();
    vec.push_back(10);
    vec.push_back(20);
    vec.push_back(30);

    let copy = vec.clone();
    assert_eq!(copy.size(), 3);
    assert_eq!(copy[1], 20);

    let sum: i32 = vec.iter().copied().sum();
    assert_eq!(sum, 60);

    vec.clear();
    assert!(vec.empty());
    assert_eq!(vec.size(), 0);
}

#[test]
fn smallvec_works_with_keys() {
    let mut keys = SmallVec::<Key, 6>::new();
    keys.push_back(X(0).into());
    keys.push_back(X(1).into());
    keys.push_back(L(5).into());

    assert_eq!(keys.size(), 3);
    let count = keys.iter().count();
    assert_eq!(count, 3);
}
