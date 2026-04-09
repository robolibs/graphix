use core::cmp::Ordering;
use core::hash::{Hash, Hasher};
use core::marker::PhantomData;

#[derive(Debug)]
pub struct Id<T> {
    value: u32,
    marker: PhantomData<fn() -> T>,
}

impl<T> Id<T> {
    pub const INVALID: u32 = u32::MAX;

    pub const fn new(value: u32) -> Self {
        Self {
            value,
            marker: PhantomData,
        }
    }

    pub const fn value(self) -> u32 {
        self.value
    }

    pub const fn is_valid(self) -> bool {
        self.value != Self::INVALID
    }
}

impl<T> Default for Id<T> {
    fn default() -> Self {
        Self::new(Self::INVALID)
    }
}

impl<T> Copy for Id<T> {}

impl<T> Clone for Id<T> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<T> PartialEq for Id<T> {
    fn eq(&self, other: &Self) -> bool {
        self.value == other.value
    }
}

impl<T> Eq for Id<T> {}

impl<T> PartialOrd for Id<T> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl<T> Ord for Id<T> {
    fn cmp(&self, other: &Self) -> Ordering {
        self.value.cmp(&other.value)
    }
}

impl<T> Hash for Id<T> {
    fn hash<H: Hasher>(&self, state: &mut H) {
        self.value.hash(state);
    }
}
