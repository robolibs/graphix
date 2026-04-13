use std::any::Any;

use datapod::trees::OrderedMap;

use crate::core::Key;

trait Value: Send + Sync {
    fn clone_box(&self) -> Box<dyn Value>;
    fn as_any(&self) -> &dyn Any;
}

#[derive(Clone)]
struct GenericValue<T: Clone + Send + Sync + 'static>(T);

impl<T: Clone + Send + Sync + 'static> Value for GenericValue<T> {
    fn clone_box(&self) -> Box<dyn Value> {
        Box::new(self.clone())
    }

    fn as_any(&self) -> &dyn Any {
        &self.0
    }
}

#[derive(Default)]
pub struct Values {
    values: OrderedMap<Key, Box<dyn Value>>,
}

pub struct ValuesIter<'a> {
    inner: std::collections::btree_map::Iter<'a, Key, Box<dyn Value>>,
}

impl std::fmt::Debug for Values {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Values")
            .field("keys", &self.keys())
            .finish()
    }
}

impl Clone for Values {
    fn clone(&self) -> Self {
        let mut values = OrderedMap::new();
        for (key, value) in &self.values {
            values.insert(*key, value.clone_box());
        }
        Self { values }
    }
}

impl Values {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn insert<T>(&mut self, key: Key, value: T) -> Result<(), String>
    where
        T: Clone + Send + Sync + 'static,
    {
        if self.exists(key) {
            return Err("Key already exists in Values".to_string());
        }
        self.values.insert(key, Box::new(GenericValue(value)));
        Ok(())
    }

    pub fn at<T>(&self, key: Key) -> Result<&T, String>
    where
        T: Clone + Send + Sync + 'static,
    {
        let value = self
            .values
            .get(&key)
            .ok_or_else(|| "Key not found in Values".to_string())?;
        value
            .as_any()
            .downcast_ref::<T>()
            .ok_or_else(|| "Type mismatch: requested type does not match stored type".to_string())
    }

    pub fn exists(&self, key: Key) -> bool {
        self.values.contains_key(&key)
    }

    pub fn size(&self) -> usize {
        self.values.len()
    }

    pub fn is_empty(&self) -> bool {
        self.values.is_empty()
    }

    pub fn empty(&self) -> bool {
        self.is_empty()
    }

    pub fn erase(&mut self, key: Key) {
        self.values.remove(&key);
    }

    pub fn clear(&mut self) {
        self.values.clear();
    }

    pub fn keys(&self) -> Vec<Key> {
        self.values.keys().copied().collect()
    }

    pub fn iter(&self) -> impl Iterator<Item = (&Key, &dyn Any)> {
        ValuesIter {
            inner: self.values.iter(),
        }
    }
}

impl<'a> Iterator for ValuesIter<'a> {
    type Item = (&'a Key, &'a dyn Any);

    fn next(&mut self) -> Option<Self::Item> {
        self.inner.next().map(|(key, value)| (key, value.as_any()))
    }
}

impl<'a> IntoIterator for &'a Values {
    type Item = (&'a Key, &'a dyn Any);
    type IntoIter = ValuesIter<'a>;

    fn into_iter(self) -> Self::IntoIter {
        ValuesIter {
            inner: self.values.iter(),
        }
    }
}
