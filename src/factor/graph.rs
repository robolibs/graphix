use std::collections::BTreeSet;
use std::ops::{Index, IndexMut};
use std::rc::Rc;

use crate::core::Key;

use super::FactorLike;

#[derive(Debug, Default)]
pub struct Graph<F: FactorLike + ?Sized = super::Factor> {
    factors: Vec<Rc<F>>,
}

impl<F: FactorLike + ?Sized> Clone for Graph<F> {
    fn clone(&self) -> Self {
        Self {
            factors: self.factors.clone(),
        }
    }
}

impl<F: FactorLike + ?Sized> Graph<F> {
    pub fn new() -> Self {
        Self {
            factors: Vec::new(),
        }
    }

    pub fn add(&mut self, factor: Rc<F>) {
        self.factors.push(factor);
    }

    pub fn add_optional(&mut self, factor: Option<Rc<F>>) {
        if let Some(factor) = factor {
            self.add(factor);
        }
    }

    pub fn len(&self) -> usize {
        self.factors.len()
    }

    pub fn size(&self) -> usize {
        self.len()
    }

    pub fn is_empty(&self) -> bool {
        self.factors.is_empty()
    }

    pub fn empty(&self) -> bool {
        self.is_empty()
    }

    pub fn at(&self, index: usize) -> Option<&Rc<F>> {
        self.factors.get(index)
    }

    pub fn at_or_err(&self, index: usize) -> Result<&Rc<F>, String> {
        self.at(index)
            .ok_or_else(|| "factor index out of bounds".to_string())
    }

    pub fn iter(&self) -> impl Iterator<Item = &Rc<F>> {
        self.factors.iter()
    }

    pub fn clear(&mut self) {
        self.factors.clear();
    }

    pub fn keys(&self) -> BTreeSet<Key> {
        let mut keys = BTreeSet::new();
        for factor in &self.factors {
            keys.extend(factor.keys().iter().copied());
        }
        keys
    }
}

impl<F: FactorLike + ?Sized> Index<usize> for Graph<F> {
    type Output = Rc<F>;

    fn index(&self, index: usize) -> &Self::Output {
        &self.factors[index]
    }
}

impl<F: FactorLike + ?Sized> IndexMut<usize> for Graph<F> {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.factors[index]
    }
}

impl<'a, F: FactorLike + ?Sized> IntoIterator for &'a Graph<F> {
    type Item = &'a Rc<F>;
    type IntoIter = std::slice::Iter<'a, Rc<F>>;

    fn into_iter(self) -> Self::IntoIter {
        self.factors.iter()
    }
}
