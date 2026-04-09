use std::array;
use std::ops::{Index, IndexMut};

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SmallVec<T, const N: usize> {
    size: usize,
    using_heap: bool,
    inline: [Option<T>; N],
    heap: Vec<T>,
}

impl<T, const N: usize> SmallVec<T, N> {
    pub fn new() -> Self {
        Self {
            size: 0,
            using_heap: false,
            inline: array::from_fn(|_| None),
            heap: Vec::new(),
        }
    }

    pub fn size(&self) -> usize {
        self.size
    }

    pub fn empty(&self) -> bool {
        self.size == 0
    }

    pub fn clear(&mut self) {
        self.size = 0;
        self.using_heap = false;
        self.heap.clear();
        for slot in &mut self.inline {
            *slot = None;
        }
    }

    pub fn push_back(&mut self, value: T)
    where
        T: Clone,
    {
        if self.using_heap {
            self.heap.push(value);
            self.size = self.heap.len();
            return;
        }

        if self.size < N {
            self.inline[self.size] = Some(value);
            self.size += 1;
            return;
        }

        self.heap.reserve(self.size + 1);
        for slot in self.inline.iter_mut().take(self.size) {
            self.heap
                .push(slot.take().expect("inline slot should be populated"));
        }
        self.heap.push(value);
        self.using_heap = true;
        self.size = self.heap.len();
    }

    pub fn iter(&self) -> SmallVecIter<'_, T, N> {
        SmallVecIter {
            smallvec: self,
            index: 0,
        }
    }

    pub fn begin(&self) -> SmallVecIter<'_, T, N> {
        self.iter()
    }

    pub fn end(&self) -> SmallVecIter<'_, T, N> {
        SmallVecIter {
            smallvec: self,
            index: self.size,
        }
    }
}

impl<T, const N: usize> Default for SmallVec<T, N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<T, const N: usize> Index<usize> for SmallVec<T, N> {
    type Output = T;

    fn index(&self, index: usize) -> &Self::Output {
        if self.using_heap {
            &self.heap[index]
        } else {
            self.inline[index]
                .as_ref()
                .expect("index out of bounds for inline storage")
        }
    }
}

impl<T, const N: usize> IndexMut<usize> for SmallVec<T, N> {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        if self.using_heap {
            &mut self.heap[index]
        } else {
            self.inline[index]
                .as_mut()
                .expect("index out of bounds for inline storage")
        }
    }
}

impl<'a, T, const N: usize> IntoIterator for &'a SmallVec<T, N> {
    type Item = &'a T;
    type IntoIter = SmallVecIter<'a, T, N>;

    fn into_iter(self) -> Self::IntoIter {
        self.iter()
    }
}

pub struct SmallVecIter<'a, T, const N: usize> {
    smallvec: &'a SmallVec<T, N>,
    index: usize,
}

impl<'a, T, const N: usize> Iterator for SmallVecIter<'a, T, N> {
    type Item = &'a T;

    fn next(&mut self) -> Option<Self::Item> {
        if self.index >= self.smallvec.size {
            return None;
        }
        let item = &self.smallvec[self.index];
        self.index += 1;
        Some(item)
    }
}
