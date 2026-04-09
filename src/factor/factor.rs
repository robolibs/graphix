use crate::core::Key;
use crate::smallvec::SmallVec;

pub const DEFAULT_FACTOR_SIZE: usize = 6;

pub trait FactorLike {
    fn keys(&self) -> &[Key];

    fn size(&self) -> usize {
        self.keys().len()
    }

    fn involves(&self, key: Key) -> bool {
        self.keys().contains(&key)
    }
}

#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Factor {
    keys: SmallVec<Key, DEFAULT_FACTOR_SIZE>,
    keys_cache: Vec<Key>,
}

impl Factor {
    pub fn new(keys: impl Into<Vec<Key>>) -> Self {
        let keys_cache = keys.into();
        let mut keys_storage = SmallVec::new();
        for key in keys_cache.iter().copied() {
            keys_storage.push_back(key);
        }
        Self {
            keys: keys_storage,
            keys_cache,
        }
    }
}

impl From<(Key,)> for Factor {
    fn from(value: (Key,)) -> Self {
        Self::new(vec![value.0])
    }
}

impl From<(Key, Key)> for Factor {
    fn from(value: (Key, Key)) -> Self {
        Self::new(vec![value.0, value.1])
    }
}

impl From<(Key, Key, Key)> for Factor {
    fn from(value: (Key, Key, Key)) -> Self {
        Self::new(vec![value.0, value.1, value.2])
    }
}

impl From<(Key, Key, Key, Key)> for Factor {
    fn from(value: (Key, Key, Key, Key)) -> Self {
        Self::new(vec![value.0, value.1, value.2, value.3])
    }
}

impl From<(Key, Key, Key, Key, Key)> for Factor {
    fn from(value: (Key, Key, Key, Key, Key)) -> Self {
        Self::new(vec![value.0, value.1, value.2, value.3, value.4])
    }
}

impl From<(Key, Key, Key, Key, Key, Key)> for Factor {
    fn from(value: (Key, Key, Key, Key, Key, Key)) -> Self {
        Self::new(vec![value.0, value.1, value.2, value.3, value.4, value.5])
    }
}

impl From<(Key, Key, Key, Key, Key, Key, Key)> for Factor {
    fn from(value: (Key, Key, Key, Key, Key, Key, Key)) -> Self {
        Self::new(vec![
            value.0, value.1, value.2, value.3, value.4, value.5, value.6,
        ])
    }
}

impl From<(Key, Key, Key, Key, Key, Key, Key, Key)> for Factor {
    fn from(value: (Key, Key, Key, Key, Key, Key, Key, Key)) -> Self {
        Self::new(vec![
            value.0, value.1, value.2, value.3, value.4, value.5, value.6, value.7,
        ])
    }
}

impl<const N: usize> From<[Key; N]> for Factor {
    fn from(value: [Key; N]) -> Self {
        Self::new(value.into_iter().collect::<Vec<_>>())
    }
}

impl From<Vec<Key>> for Factor {
    fn from(value: Vec<Key>) -> Self {
        Self::new(value)
    }
}

impl FactorLike for Factor {
    fn keys(&self) -> &[Key] {
        &self.keys_cache
    }
}
