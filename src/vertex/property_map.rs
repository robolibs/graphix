use std::any::Any;
use std::collections::HashMap;
use std::hash::Hash;
use std::sync::{Arc, RwLock};

pub trait PropertyMap<K, V>: Send + Sync {
    fn get(&self, key: &K) -> Result<V, String>;
    fn put(&self, key: &K, value: V);
    fn contains(&self, key: &K) -> bool;
    fn clear(&self);
    fn size(&self) -> usize;
}

#[derive(Debug)]
pub struct AssociativePropertyMap<K, V> {
    storage: RwLock<HashMap<K, V>>,
    default: RwLock<Option<V>>,
}

pub struct AssociativeIter<K, V> {
    entries: std::vec::IntoIter<(K, V)>,
}

impl<K, V> AssociativePropertyMap<K, V> {
    pub fn new() -> Self {
        Self {
            storage: RwLock::new(HashMap::new()),
            default: RwLock::new(None),
        }
    }

    pub fn with_default(default: V) -> Self {
        Self {
            storage: RwLock::new(HashMap::new()),
            default: RwLock::new(Some(default)),
        }
    }

    pub fn erase(&self, key: &K) -> Option<V>
    where
        K: Eq + Hash,
    {
        self.storage.write().expect("lock poisoned").remove(key)
    }

    pub fn set_default(&self, value: V) {
        *self.default.write().expect("lock poisoned") = Some(value);
    }

    pub fn entries(&self) -> Vec<(K, V)>
    where
        K: Clone,
        V: Clone,
    {
        self.storage
            .read()
            .expect("lock poisoned")
            .iter()
            .map(|(key, value)| (key.clone(), value.clone()))
            .collect()
    }

    pub fn iter(&self) -> AssociativeIter<K, V>
    where
        K: Clone,
        V: Clone,
    {
        AssociativeIter {
            entries: self.entries().into_iter(),
        }
    }
}

impl<K, V> Default for AssociativePropertyMap<K, V> {
    fn default() -> Self {
        Self::new()
    }
}

impl<K, V> Iterator for AssociativeIter<K, V> {
    type Item = (K, V);

    fn next(&mut self) -> Option<Self::Item> {
        self.entries.next()
    }
}

impl<K, V> IntoIterator for &AssociativePropertyMap<K, V>
where
    K: Clone,
    V: Clone,
{
    type Item = (K, V);
    type IntoIter = AssociativeIter<K, V>;

    fn into_iter(self) -> Self::IntoIter {
        self.iter()
    }
}

impl<K, V> PropertyMap<K, V> for AssociativePropertyMap<K, V>
where
    K: Eq + Hash + Clone + Send + Sync,
    V: Clone + Send + Sync,
{
    fn get(&self, key: &K) -> Result<V, String> {
        if let Some(value) = self.storage.read().expect("lock poisoned").get(key) {
            return Ok(value.clone());
        }
        if let Some(default) = self.default.read().expect("lock poisoned").as_ref() {
            return Ok(default.clone());
        }
        Err("Key not found in property map".to_string())
    }

    fn put(&self, key: &K, value: V) {
        self.storage
            .write()
            .expect("lock poisoned")
            .insert(key.clone(), value);
    }

    fn contains(&self, key: &K) -> bool {
        self.storage
            .read()
            .expect("lock poisoned")
            .contains_key(key)
    }

    fn clear(&self) {
        self.storage.write().expect("lock poisoned").clear();
    }

    fn size(&self) -> usize {
        self.storage.read().expect("lock poisoned").len()
    }
}

#[derive(Debug)]
pub struct VectorPropertyMap<V> {
    storage: RwLock<Vec<V>>,
    default: RwLock<V>,
}

impl<V> VectorPropertyMap<V>
where
    V: Clone,
{
    pub fn new(initial_size: usize, default: V) -> Self {
        Self {
            storage: RwLock::new(vec![default.clone(); initial_size]),
            default: RwLock::new(default),
        }
    }

    pub fn reserve(&self, capacity: usize) {
        self.storage
            .write()
            .expect("lock poisoned")
            .reserve(capacity);
    }

    pub fn resize(&self, new_size: usize) {
        let default = self.default.read().expect("lock poisoned").clone();
        self.storage
            .write()
            .expect("lock poisoned")
            .resize(new_size, default);
    }

    pub fn set_default(&self, value: V) {
        *self.default.write().expect("lock poisoned") = value;
    }
}

impl<V> Default for VectorPropertyMap<V>
where
    V: Default + Clone,
{
    fn default() -> Self {
        Self::new(0, V::default())
    }
}

impl<V> PropertyMap<usize, V> for VectorPropertyMap<V>
where
    V: Clone + Send + Sync,
{
    fn get(&self, key: &usize) -> Result<V, String> {
        let storage = self.storage.read().expect("lock poisoned");
        if let Some(value) = storage.get(*key) {
            Ok(value.clone())
        } else {
            Ok(self.default.read().expect("lock poisoned").clone())
        }
    }

    fn put(&self, key: &usize, value: V) {
        let default = self.default.read().expect("lock poisoned").clone();
        let mut storage = self.storage.write().expect("lock poisoned");
        if *key >= storage.len() {
            storage.resize(*key + 1, default);
        }
        storage[*key] = value;
    }

    fn contains(&self, key: &usize) -> bool {
        *key < self.storage.read().expect("lock poisoned").len()
    }

    fn clear(&self) {
        self.storage.write().expect("lock poisoned").clear();
    }

    fn size(&self) -> usize {
        self.storage.read().expect("lock poisoned").len()
    }
}

#[derive(Debug)]
pub struct ConstantPropertyMap<K, V> {
    value: V,
    _marker: std::marker::PhantomData<fn(K)>,
}

impl<K, V> ConstantPropertyMap<K, V> {
    pub fn new(value: V) -> Self {
        Self {
            value,
            _marker: std::marker::PhantomData,
        }
    }
}

impl<K, V> PropertyMap<K, V> for ConstantPropertyMap<K, V>
where
    K: Send + Sync,
    V: Clone + Send + Sync,
{
    fn get(&self, _key: &K) -> Result<V, String> {
        Ok(self.value.clone())
    }

    fn put(&self, _key: &K, _value: V) {}

    fn contains(&self, _key: &K) -> bool {
        true
    }

    fn clear(&self) {}

    fn size(&self) -> usize {
        0
    }
}

#[derive(Debug, Default)]
pub struct IdentityPropertyMap<K> {
    _marker: std::marker::PhantomData<fn(K)>,
}

impl<K> IdentityPropertyMap<K> {
    pub fn new() -> Self {
        Self {
            _marker: std::marker::PhantomData,
        }
    }
}

impl<K> PropertyMap<K, K> for IdentityPropertyMap<K>
where
    K: Clone + Send + Sync,
{
    fn get(&self, key: &K) -> Result<K, String> {
        Ok(key.clone())
    }

    fn put(&self, _key: &K, _value: K) {}

    fn contains(&self, _key: &K) -> bool {
        true
    }

    fn clear(&self) {}

    fn size(&self) -> usize {
        0
    }
}

pub fn get<K, V, P>(pmap: &P, key: &K) -> Result<V, String>
where
    P: PropertyMap<K, V> + ?Sized,
{
    pmap.get(key)
}

pub fn put<K, V, P>(pmap: &P, key: &K, value: V)
where
    P: PropertyMap<K, V> + ?Sized,
{
    pmap.put(key, value);
}

pub fn contains<K, V, P>(pmap: &P, key: &K) -> bool
where
    P: PropertyMap<K, V> + ?Sized,
{
    pmap.contains(key)
}

pub fn clear<K, V, P>(pmap: &P)
where
    P: PropertyMap<K, V> + ?Sized,
{
    pmap.clear();
}

pub fn size<K, V, P>(pmap: &P) -> usize
where
    P: PropertyMap<K, V> + ?Sized,
{
    pmap.size()
}

pub fn make_associative_property_map<K, V>() -> Arc<AssociativePropertyMap<K, V>>
where
    K: Eq + Hash,
{
    Arc::new(AssociativePropertyMap::new())
}

pub fn make_associative_property_map_with_default<K, V>(
    default: V,
) -> Arc<AssociativePropertyMap<K, V>>
where
    K: Eq + Hash,
{
    Arc::new(AssociativePropertyMap::with_default(default))
}

pub fn make_vector_property_map<V>(size: usize, default: V) -> Arc<VectorPropertyMap<V>>
where
    V: Clone,
{
    Arc::new(VectorPropertyMap::new(size, default))
}

pub fn make_constant_property_map<K, V>(value: V) -> Arc<ConstantPropertyMap<K, V>> {
    Arc::new(ConstantPropertyMap::new(value))
}

pub fn make_identity_property_map<K>() -> Arc<IdentityPropertyMap<K>> {
    Arc::new(IdentityPropertyMap::new())
}

pub struct CompositePropertyMap<VertexKey = usize, EdgeKey = usize> {
    vertex_properties: HashMap<String, Box<dyn Any + Send + Sync>>,
    edge_properties: HashMap<String, Box<dyn Any + Send + Sync>>,
    _vertex_marker: std::marker::PhantomData<fn(VertexKey)>,
    _edge_marker: std::marker::PhantomData<fn(EdgeKey)>,
}

impl<VertexKey, EdgeKey> CompositePropertyMap<VertexKey, EdgeKey> {
    pub fn new() -> Self {
        Self {
            vertex_properties: HashMap::new(),
            edge_properties: HashMap::new(),
            _vertex_marker: std::marker::PhantomData,
            _edge_marker: std::marker::PhantomData,
        }
    }

    pub fn add_vertex_property<P>(&mut self, name: impl Into<String>, pmap: Arc<P>)
    where
        P: Send + Sync + 'static,
    {
        self.vertex_properties.insert(name.into(), Box::new(pmap));
    }

    pub fn add_edge_property<P>(&mut self, name: impl Into<String>, pmap: Arc<P>)
    where
        P: Send + Sync + 'static,
    {
        self.edge_properties.insert(name.into(), Box::new(pmap));
    }

    pub fn get_vertex_property<P>(&self, name: &str) -> Option<Arc<P>>
    where
        P: Send + Sync + 'static,
    {
        self.vertex_properties
            .get(name)
            .and_then(|boxed| boxed.downcast_ref::<Arc<P>>())
            .cloned()
    }

    pub fn get_edge_property<P>(&self, name: &str) -> Option<Arc<P>>
    where
        P: Send + Sync + 'static,
    {
        self.edge_properties
            .get(name)
            .and_then(|boxed| boxed.downcast_ref::<Arc<P>>())
            .cloned()
    }

    pub fn has_vertex_property(&self, name: &str) -> bool {
        self.vertex_properties.contains_key(name)
    }

    pub fn has_edge_property(&self, name: &str) -> bool {
        self.edge_properties.contains_key(name)
    }

    pub fn remove_vertex_property(&mut self, name: &str) {
        self.vertex_properties.remove(name);
    }

    pub fn remove_edge_property(&mut self, name: &str) {
        self.edge_properties.remove(name);
    }

    pub fn clear(&mut self) {
        self.vertex_properties.clear();
        self.edge_properties.clear();
    }
}

impl<VertexKey, EdgeKey> Default for CompositePropertyMap<VertexKey, EdgeKey> {
    fn default() -> Self {
        Self::new()
    }
}
