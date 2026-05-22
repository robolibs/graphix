use std::collections::BTreeMap;

use crate::core::Key;

use super::{Factor, FactorLike};

pub type Vector = nalgebra::DVector<f64>;
pub type Matrix = nalgebra::DMatrix<f64>;

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
        if jacobians.iter().any(|jacobian| jacobian.nrows() != b.len()) {
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
        self.b.len()
    }

    pub fn scale(&mut self, factor: f64) {
        for value in self.b.iter_mut() {
            *value *= factor;
        }
        for jacobian in &mut self.jacobians {
            for value in jacobian.iter_mut() {
                *value *= factor;
            }
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
            if delta.len() != jacobian.ncols() {
                return Err("Delta dimension mismatch for key".to_string());
            }
            for row in 0..jacobian.nrows() {
                for col in 0..jacobian.ncols() {
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
