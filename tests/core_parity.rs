use std::collections::HashSet;

use graphix::{Id, Key, L, P, Store, Symbol, X};

#[test]
fn key_and_symbol_roundtrip() {
    let x0 = X(0);
    let x1 = X(1);
    let l5 = L(5);
    let p2 = P(2);

    assert_eq!(x0.chr(), b'x');
    assert_eq!(x1.index(), 1);
    assert_eq!(l5.chr(), b'l');
    assert_eq!(p2.chr(), b'p');

    let key: Key = x0.into();
    assert_eq!(key, x0.key());
    assert_ne!(x0.key(), x1.key());
}

#[test]
fn id_supports_order_hash_and_invalid_default() {
    #[derive(Debug)]
    struct Marker;

    let invalid = Id::<Marker>::default();
    assert!(!invalid.is_valid());

    let a = Id::<Marker>::new(1);
    let b = Id::<Marker>::new(2);
    assert!(a < b);
    assert!(a.is_valid());

    let mut set = HashSet::new();
    set.insert(a);
    set.insert(b);
    set.insert(Id::<Marker>::new(1));
    assert_eq!(set.len(), 2);
}

#[test]
fn store_supports_add_remove_reuse_and_iteration() {
    let mut store = Store::new();
    let a = store.add("alpha");
    let b = store.add("beta");

    assert_eq!(store.get(a), Some(&"alpha"));
    assert_eq!(store.get(b), Some(&"beta"));
    assert_eq!(store.len(), 2);
    assert!(store.contains(a));

    let removed = store.remove(a);
    assert_eq!(removed, Some("alpha"));
    assert!(!store.contains(a));
    assert_eq!(store.len(), 1);

    let c = store.add("gamma");
    assert_eq!(c.value(), a.value());
    assert_eq!(store.get(c), Some(&"gamma"));

    let ids: Vec<_> = store.ids().collect();
    assert_eq!(ids.len(), 2);
    assert!(ids.contains(&b));
    assert!(ids.contains(&c));
}

#[test]
fn symbol_ordering_is_stable() {
    let a = Symbol::new(b'a', 1);
    let b = Symbol::new(b'a', 2);
    let c = Symbol::new(b'b', 0);

    assert!(a < b);
    assert!(b < c);
}
