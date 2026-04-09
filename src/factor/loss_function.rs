use std::sync::Arc;

pub trait LossFunction: Send + Sync {
    fn evaluate(&self, squared_error: f64) -> f64;
    fn weight(&self, squared_error: f64) -> f64;
    fn clone_arc(&self) -> Arc<dyn LossFunction>;

    fn clone_box(&self) -> Arc<dyn LossFunction> {
        self.clone_arc()
    }
}

#[derive(Debug, Clone, Default)]
pub struct NullLoss;

impl LossFunction for NullLoss {
    fn evaluate(&self, squared_error: f64) -> f64 {
        squared_error
    }

    fn weight(&self, _squared_error: f64) -> f64 {
        1.0
    }

    fn clone_arc(&self) -> Arc<dyn LossFunction> {
        Arc::new(self.clone())
    }
}

#[derive(Debug, Clone)]
pub struct HuberLoss {
    k: f64,
}

impl HuberLoss {
    pub fn new(k: f64) -> Self {
        Self { k }
    }

    pub fn threshold(&self) -> f64 {
        self.k
    }
}

impl Default for HuberLoss {
    fn default() -> Self {
        Self::new(1.345)
    }
}

impl LossFunction for HuberLoss {
    fn evaluate(&self, squared_error: f64) -> f64 {
        let error = squared_error.sqrt();
        if error <= self.k {
            0.5 * squared_error
        } else {
            self.k * error - 0.5 * self.k * self.k
        }
    }

    fn weight(&self, squared_error: f64) -> f64 {
        let error = squared_error.sqrt();
        if error <= self.k || error == 0.0 {
            1.0
        } else {
            self.k / error
        }
    }

    fn clone_arc(&self) -> Arc<dyn LossFunction> {
        Arc::new(self.clone())
    }
}

#[derive(Debug, Clone)]
pub struct CauchyLoss {
    k: f64,
}

impl CauchyLoss {
    pub fn new(k: f64) -> Self {
        Self { k }
    }

    pub fn scale(&self) -> f64 {
        self.k
    }
}

impl Default for CauchyLoss {
    fn default() -> Self {
        Self::new(2.3849)
    }
}

impl LossFunction for CauchyLoss {
    fn evaluate(&self, squared_error: f64) -> f64 {
        let k2 = self.k * self.k;
        0.5 * k2 * (1.0 + squared_error / k2).ln()
    }

    fn weight(&self, squared_error: f64) -> f64 {
        let k2 = self.k * self.k;
        k2 / (k2 + squared_error)
    }

    fn clone_arc(&self) -> Arc<dyn LossFunction> {
        Arc::new(self.clone())
    }
}

#[derive(Debug, Clone)]
pub struct TukeyLoss {
    k: f64,
}

impl TukeyLoss {
    pub fn new(k: f64) -> Self {
        Self { k }
    }

    pub fn threshold(&self) -> f64 {
        self.k
    }
}

impl Default for TukeyLoss {
    fn default() -> Self {
        Self::new(4.6851)
    }
}

impl LossFunction for TukeyLoss {
    fn evaluate(&self, squared_error: f64) -> f64 {
        let k2 = self.k * self.k;
        if squared_error <= k2 {
            let term = 1.0 - squared_error / k2;
            (k2 / 6.0) * (1.0 - term * term * term)
        } else {
            k2 / 6.0
        }
    }

    fn weight(&self, squared_error: f64) -> f64 {
        let k2 = self.k * self.k;
        if squared_error <= k2 {
            let term = 1.0 - squared_error / k2;
            term * term
        } else {
            0.0
        }
    }

    fn clone_arc(&self) -> Arc<dyn LossFunction> {
        Arc::new(self.clone())
    }
}

pub fn no_loss() -> Arc<dyn LossFunction> {
    Arc::new(NullLoss)
}

pub fn huber_loss(k: f64) -> Arc<dyn LossFunction> {
    Arc::new(HuberLoss::new(k))
}

pub fn cauchy_loss(k: f64) -> Arc<dyn LossFunction> {
    Arc::new(CauchyLoss::new(k))
}

pub fn tukey_loss(k: f64) -> Arc<dyn LossFunction> {
    Arc::new(TukeyLoss::new(k))
}
