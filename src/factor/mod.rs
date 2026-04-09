//! Curated public API for factor graphs and optimization.
//!
//! Keep the root focused on graph structure, factors, losses, and optimizers.
//! Low-level math aliases and helper functions live under `graphix::factor::types`.

mod factor;
mod graph;
mod linear;
mod loss_function;
mod nonlinear;
mod pose_graph;
pub mod types;
mod values;

pub use factor::{Factor, FactorLike};
pub use graph::Graph;
pub use linear::{GaussianFactor, Matrix, Vector};
pub use loss_function::{
    CauchyLoss, HuberLoss, LossFunction, NullLoss, TukeyLoss, cauchy_loss, huber_loss, no_loss,
    tukey_loss,
};
pub use nonlinear::{
    BetweenFactor, FactorGraphAdapter, GaussNewtonOptimizer, GaussNewtonParameters,
    GaussNewtonResult, GradientDescentOptimizer, GradientDescentResult,
    LevenbergMarquardtOptimizer, LevenbergMarquardtParameters, LevenbergMarquardtResult,
    NonlinearFactor, OptimizationResult, OptinumGaussNewton, OptinumGradientDescent,
    OptinumLevenbergMarquardt, Parameters, PriorFactor, SE2BetweenFactor, SE2PriorFactor,
    VariableInfo,
};
pub use pose_graph::PoseGraph2d;
pub use types::{SE2d, Vec3d, se2_sigmas};
pub use values::Values;
