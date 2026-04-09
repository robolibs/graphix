use std::collections::BTreeMap;
use std::ops::{Index, IndexMut};

use crate::core::Key;

use super::{Factor, FactorLike};

#[derive(Debug, Clone, PartialEq)]
pub struct Vector {
    data: Vec<f64>,
}

impl Vector {
    pub fn new(size: usize) -> Self {
        Self {
            data: vec![0.0; size],
        }
    }

    pub fn from_vec(data: Vec<f64>) -> Self {
        Self { data }
    }

    pub fn size(&self) -> usize {
        self.data.len()
    }

    pub fn iter(&self) -> impl Iterator<Item = &f64> {
        self.data.iter()
    }

    pub fn scale(&mut self, factor: f64) {
        for value in &mut self.data {
            *value *= factor;
        }
    }
}

impl Index<usize> for Vector {
    type Output = f64;

    fn index(&self, index: usize) -> &Self::Output {
        &self.data[index]
    }
}

impl IndexMut<usize> for Vector {
    fn index_mut(&mut self, index: usize) -> &mut Self::Output {
        &mut self.data[index]
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct Matrix {
    rows: usize,
    cols: usize,
    data: Vec<f64>,
}

impl Matrix {
    pub fn new(rows: usize, cols: usize) -> Self {
        Self {
            rows,
            cols,
            data: vec![0.0; rows * cols],
        }
    }

    pub fn rows(&self) -> usize {
        self.rows
    }

    pub fn cols(&self) -> usize {
        self.cols
    }

    pub fn scale(&mut self, factor: f64) {
        for value in &mut self.data {
            *value *= factor;
        }
    }
}

impl Index<(usize, usize)> for Matrix {
    type Output = f64;

    fn index(&self, index: (usize, usize)) -> &Self::Output {
        &self.data[index.0 * self.cols + index.1]
    }
}

impl IndexMut<(usize, usize)> for Matrix {
    fn index_mut(&mut self, index: (usize, usize)) -> &mut Self::Output {
        &mut self.data[index.0 * self.cols + index.1]
    }
}

#[derive(Debug, Clone)]
pub struct GaussianFactor {
    factor: Factor,
    jacobians: Vec<Matrix>,
    b: Vector,
    key_index: BTreeMap<Key, usize>,
}

impl GaussianFactor {
    pub fn new(keys: Vec<Key>, jacobians: Vec<Matrix>, b: Vector) -> Result<Self, String> {
        if keys.len() != jacobians.len() {
            return Err("Number of keys must match number of Jacobians".to_string());
        }
        if jacobians.iter().any(|jacobian| jacobian.rows() != b.size()) {
            return Err("All Jacobians must have same number of rows as error vector".to_string());
        }

        let key_index = keys
            .iter()
            .enumerate()
            .map(|(index, key)| (*key, index))
            .collect();

        Ok(Self {
            factor: Factor::new(keys),
            jacobians,
            b,
            key_index,
        })
    }

    pub fn jacobian(&self, key: Key) -> Option<&Matrix> {
        self.key_index
            .get(&key)
            .map(|index| &self.jacobians[*index])
    }

    pub fn size(&self) -> usize {
        self.factor.size()
    }

    pub fn jacobians(&self) -> &[Matrix] {
        &self.jacobians
    }

    pub fn b(&self) -> &Vector {
        &self.b
    }

    pub fn dim(&self) -> usize {
        self.b.size()
    }

    pub fn scale(&mut self, factor: f64) {
        self.b.scale(factor);
        for jacobian in &mut self.jacobians {
            jacobian.scale(factor);
        }
    }

    pub fn error(&self, deltas: &BTreeMap<Key, Vector>) -> Result<f64, String> {
        let mut residual = self.b.clone();
        for key in self.factor.keys() {
            let Some(jacobian) = self.jacobian(*key) else {
                continue;
            };
            let Some(delta) = deltas.get(key) else {
                continue;
            };
            if delta.size() != jacobian.cols() {
                return Err("Delta dimension mismatch for key".to_string());
            }
            for row in 0..jacobian.rows() {
                for col in 0..jacobian.cols() {
                    residual[row] += jacobian[(row, col)] * delta[col];
                }
            }
        }

        let sum_squared = residual.iter().map(|value| value * value).sum::<f64>();
        Ok(0.5 * sum_squared)
    }
}

impl FactorLike for GaussianFactor {
    fn keys(&self) -> &[Key] {
        self.factor.keys()
    }
}
