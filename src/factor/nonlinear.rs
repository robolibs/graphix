use std::sync::Arc;

use glam::DVec2;

use crate::core::Key;

use super::{
    FactorLike, GaussianFactor, Graph, LossFunction, Matrix, SE2d, Values, Vec3d, Vector,
    se2_sigmas,
};

pub trait NonlinearFactor: FactorLike {
    fn error(&self, values: &Values) -> f64;

    fn dim(&self, _key: Key) -> usize {
        1
    }

    fn error_vector(&self, values: &Values) -> Vector {
        Vector::from_vec(vec![(2.0 * self.error(values)).max(0.0).sqrt()])
    }

    fn linearize(&self, values: &Values) -> Result<GaussianFactor, String> {
        default_linearize(self, values)
    }
}

fn default_linearize<F>(factor: &F, values: &Values) -> Result<GaussianFactor, String>
where
    F: NonlinearFactor + ?Sized,
{
    let epsilon = 1e-7;
    let b = factor.error_vector(values);
    let error_dim = b.size();
    let mut keys = Vec::with_capacity(factor.keys().len());
    let mut jacobians = Vec::with_capacity(factor.keys().len());

    for key in factor.keys() {
        keys.push(*key);
        let var_dim = factor.dim(*key);
        let mut jacobian = Matrix::new(error_dim, var_dim);

        for col in 0..var_dim {
            let perturbed = perturb_value(values, *key, col, epsilon)?;
            let e_perturbed = factor.error_vector(&perturbed);
            for row in 0..error_dim {
                jacobian[(row, col)] = (e_perturbed[row] - b[row]) / epsilon;
            }
        }

        jacobians.push(jacobian);
    }

    GaussianFactor::new(keys, jacobians, b)
}

fn apply_robust_loss(
    mut gaussian: GaussianFactor,
    loss: Option<&Arc<dyn LossFunction>>,
) -> Result<GaussianFactor, String> {
    let Some(loss) = loss else {
        return Ok(gaussian);
    };

    let squared_error = gaussian.b().iter().map(|value| value * value).sum::<f64>();
    let weight = loss.weight(squared_error);
    if !weight.is_finite() || weight < 0.0 {
        return Err("robust loss produced invalid weight".to_string());
    }
    gaussian.scale(weight.sqrt());
    Ok(gaussian)
}

#[derive(Clone)]
pub struct PriorFactor {
    key: Key,
    prior: f64,
    sigma: f64,
    loss: Option<Arc<dyn LossFunction>>,
}

impl PriorFactor {
    pub fn new(key: Key, prior: f64, sigma: f64) -> Result<Self, String> {
        if sigma <= 0.0 {
            return Err("sigma must be positive".to_string());
        }
        Ok(Self {
            key,
            prior,
            sigma,
            loss: None,
        })
    }

    pub fn prior(&self) -> f64 {
        self.prior
    }

    pub fn sigma(&self) -> f64 {
        self.sigma
    }

    pub fn set_loss_function(&mut self, loss: Arc<dyn LossFunction>) {
        self.loss = Some(loss);
    }

    pub fn loss_function(&self) -> Option<Arc<dyn LossFunction>> {
        self.loss.as_ref().map(|loss| loss.clone_box())
    }

    pub fn has_loss_function(&self) -> bool {
        self.loss.is_some()
    }
}

impl FactorLike for PriorFactor {
    fn keys(&self) -> &[Key] {
        std::slice::from_ref(&self.key)
    }
}

impl NonlinearFactor for PriorFactor {
    fn error(&self, values: &Values) -> f64 {
        let x = *values.at::<f64>(self.key).expect("missing key");
        let residual = (x - self.prior) / self.sigma;
        let squared = residual * residual;
        match &self.loss {
            Some(loss) => loss.evaluate(squared),
            None => 0.5 * squared,
        }
    }

    fn error_vector(&self, values: &Values) -> Vector {
        let x = *values.at::<f64>(self.key).expect("missing key");
        Vector::from_vec(vec![(x - self.prior) / self.sigma])
    }

    fn linearize(&self, values: &Values) -> Result<GaussianFactor, String> {
        apply_robust_loss(default_linearize(self, values)?, self.loss.as_ref())
    }
}

#[derive(Clone)]
pub struct BetweenFactor {
    keys: [Key; 2],
    measured: f64,
    sigma: f64,
    loss: Option<Arc<dyn LossFunction>>,
}

impl BetweenFactor {
    pub fn new(key1: Key, key2: Key, measured: f64, sigma: f64) -> Result<Self, String> {
        if sigma <= 0.0 {
            return Err("sigma must be positive".to_string());
        }
        Ok(Self {
            keys: [key1, key2],
            measured,
            sigma,
            loss: None,
        })
    }

    pub fn measured(&self) -> f64 {
        self.measured
    }

    pub fn sigma(&self) -> f64 {
        self.sigma
    }

    pub fn set_loss_function(&mut self, loss: Arc<dyn LossFunction>) {
        self.loss = Some(loss);
    }

    pub fn loss_function(&self) -> Option<Arc<dyn LossFunction>> {
        self.loss.as_ref().map(|loss| loss.clone_box())
    }

    pub fn has_loss_function(&self) -> bool {
        self.loss.is_some()
    }
}

impl FactorLike for BetweenFactor {
    fn keys(&self) -> &[Key] {
        &self.keys
    }
}

impl NonlinearFactor for BetweenFactor {
    fn error(&self, values: &Values) -> f64 {
        let x1 = *values.at::<f64>(self.keys[0]).expect("missing key1");
        let x2 = *values.at::<f64>(self.keys[1]).expect("missing key2");
        let residual = ((x2 - x1) - self.measured) / self.sigma;
        let squared = residual * residual;
        match &self.loss {
            Some(loss) => loss.evaluate(squared),
            None => 0.5 * squared,
        }
    }

    fn error_vector(&self, values: &Values) -> Vector {
        let x1 = *values.at::<f64>(self.keys[0]).expect("missing key1");
        let x2 = *values.at::<f64>(self.keys[1]).expect("missing key2");
        Vector::from_vec(vec![((x2 - x1) - self.measured) / self.sigma])
    }

    fn linearize(&self, values: &Values) -> Result<GaussianFactor, String> {
        apply_robust_loss(default_linearize(self, values)?, self.loss.as_ref())
    }
}

#[derive(Clone)]
pub struct SE2PriorFactor {
    key: Key,
    prior: SE2d,
    sigmas: Vec3d,
    loss: Option<Arc<dyn LossFunction>>,
}

impl SE2PriorFactor {
    pub fn new(key: Key, prior: SE2d, sigmas: Vec3d) -> Result<Self, String> {
        if sigmas[0] <= 0.0 || sigmas[1] <= 0.0 || sigmas[2] <= 0.0 {
            return Err("all sigmas must be positive".to_string());
        }
        Ok(Self {
            key,
            prior,
            sigmas,
            loss: None,
        })
    }

    pub fn from_translation_sigmas(
        key: Key,
        prior: SE2d,
        translation_sigma: DVec2,
        rotation_sigma: f64,
    ) -> Result<Self, String> {
        Self::new(key, prior, se2_sigmas(translation_sigma, rotation_sigma))
    }

    pub fn prior(&self) -> SE2d {
        self.prior
    }

    pub fn sigmas(&self) -> Vec3d {
        self.sigmas
    }

    pub fn set_loss_function(&mut self, loss: Arc<dyn LossFunction>) {
        self.loss = Some(loss);
    }

    pub fn with_loss_function(mut self, loss: Arc<dyn LossFunction>) -> Self {
        self.set_loss_function(loss);
        self
    }

    pub fn loss_function(&self) -> Option<Arc<dyn LossFunction>> {
        self.loss.as_ref().map(|loss| loss.clone_box())
    }

    pub fn has_loss_function(&self) -> bool {
        self.loss.is_some()
    }
}

impl FactorLike for SE2PriorFactor {
    fn keys(&self) -> &[Key] {
        std::slice::from_ref(&self.key)
    }
}

impl NonlinearFactor for SE2PriorFactor {
    fn error(&self, values: &Values) -> f64 {
        let pose = *values.at::<SE2d>(self.key).expect("missing SE2 prior key");
        let delta = (self.prior.inverse() * pose).log();
        let squared = (0..3)
            .map(|i| {
                let w = delta[i] / self.sigmas[i];
                w * w
            })
            .sum::<f64>();
        match &self.loss {
            Some(loss) => loss.evaluate(squared),
            None => 0.5 * squared,
        }
    }

    fn dim(&self, _key: Key) -> usize {
        3
    }

    fn error_vector(&self, values: &Values) -> Vector {
        let pose = *values.at::<SE2d>(self.key).expect("missing SE2 prior key");
        let delta = (self.prior.inverse() * pose).log();
        Vector::from_vec((0..3).map(|i| delta[i] / self.sigmas[i]).collect())
    }

    fn linearize(&self, values: &Values) -> Result<GaussianFactor, String> {
        apply_robust_loss(default_linearize(self, values)?, self.loss.as_ref())
    }
}

#[derive(Clone)]
pub struct SE2BetweenFactor {
    keys: [Key; 2],
    measured: SE2d,
    sigmas: Vec3d,
    loss: Option<Arc<dyn LossFunction>>,
}

impl SE2BetweenFactor {
    pub fn new(key1: Key, key2: Key, measured: SE2d, sigmas: Vec3d) -> Result<Self, String> {
        if sigmas[0] <= 0.0 || sigmas[1] <= 0.0 || sigmas[2] <= 0.0 {
            return Err("all sigmas must be positive".to_string());
        }
        Ok(Self {
            keys: [key1, key2],
            measured,
            sigmas,
            loss: None,
        })
    }

    pub fn from_translation_sigmas(
        key1: Key,
        key2: Key,
        measured: SE2d,
        translation_sigma: DVec2,
        rotation_sigma: f64,
    ) -> Result<Self, String> {
        Self::new(
            key1,
            key2,
            measured,
            se2_sigmas(translation_sigma, rotation_sigma),
        )
    }

    pub fn measured(&self) -> SE2d {
        self.measured
    }

    pub fn sigmas(&self) -> Vec3d {
        self.sigmas
    }

    pub fn set_loss_function(&mut self, loss: Arc<dyn LossFunction>) {
        self.loss = Some(loss);
    }

    pub fn with_loss_function(mut self, loss: Arc<dyn LossFunction>) -> Self {
        self.set_loss_function(loss);
        self
    }

    pub fn loss_function(&self) -> Option<Arc<dyn LossFunction>> {
        self.loss.as_ref().map(|loss| loss.clone_box())
    }

    pub fn has_loss_function(&self) -> bool {
        self.loss.is_some()
    }
}

impl FactorLike for SE2BetweenFactor {
    fn keys(&self) -> &[Key] {
        &self.keys
    }
}

impl NonlinearFactor for SE2BetweenFactor {
    fn error(&self, values: &Values) -> f64 {
        let pose_i = *values.at::<SE2d>(self.keys[0]).expect("missing SE2 key_i");
        let pose_j = *values.at::<SE2d>(self.keys[1]).expect("missing SE2 key_j");
        let predicted = pose_i.inverse() * pose_j;
        let delta = (self.measured.inverse() * predicted).log();
        let squared = (0..3)
            .map(|i| {
                let w = delta[i] / self.sigmas[i];
                w * w
            })
            .sum::<f64>();
        match &self.loss {
            Some(loss) => loss.evaluate(squared),
            None => 0.5 * squared,
        }
    }

    fn dim(&self, _key: Key) -> usize {
        3
    }

    fn error_vector(&self, values: &Values) -> Vector {
        let pose_i = *values.at::<SE2d>(self.keys[0]).expect("missing SE2 key_i");
        let pose_j = *values.at::<SE2d>(self.keys[1]).expect("missing SE2 key_j");
        let predicted = pose_i.inverse() * pose_j;
        let delta = (self.measured.inverse() * predicted).log();
        Vector::from_vec((0..3).map(|i| delta[i] / self.sigmas[i]).collect())
    }

    fn linearize(&self, values: &Values) -> Result<GaussianFactor, String> {
        apply_robust_loss(default_linearize(self, values)?, self.loss.as_ref())
    }
}

#[derive(Debug, Clone)]
pub struct Parameters {
    pub max_iterations: usize,
    pub step_size: f64,
    pub tolerance: f64,
    pub h: f64,
    pub verbose: bool,
}

impl Default for Parameters {
    fn default() -> Self {
        Self {
            max_iterations: 100,
            step_size: 0.01,
            tolerance: 1e-6,
            h: 1e-5,
            verbose: false,
        }
    }
}

#[derive(Debug, Clone)]
pub struct OptimizationResult {
    pub values: Values,
    pub final_error: f64,
    pub iterations: usize,
    pub converged: bool,
    pub gradient_norm: f64,
}

pub type GradientDescentResult = OptimizationResult;
pub type GaussNewtonResult = OptimizationResult;
pub type LevenbergMarquardtResult = OptimizationResult;

#[derive(Debug, Clone)]
pub struct GaussNewtonParameters {
    pub max_iterations: usize,
    pub tolerance: f64,
    pub min_step_norm: f64,
    pub verbose: bool,
}

impl Default for GaussNewtonParameters {
    fn default() -> Self {
        Self {
            max_iterations: 100,
            tolerance: 1e-6,
            min_step_norm: 1e-9,
            verbose: false,
        }
    }
}

#[derive(Debug, Clone)]
pub struct LevenbergMarquardtParameters {
    pub max_iterations: usize,
    pub tolerance: f64,
    pub min_step_norm: f64,
    pub initial_lambda: f64,
    pub lambda_factor: f64,
    pub min_lambda: f64,
    pub max_lambda: f64,
    pub verbose: bool,
}

impl Default for LevenbergMarquardtParameters {
    fn default() -> Self {
        Self {
            max_iterations: 100,
            tolerance: 1e-6,
            min_step_norm: 1e-9,
            initial_lambda: 1e-3,
            lambda_factor: 10.0,
            min_lambda: 1e-7,
            max_lambda: 1e7,
            verbose: false,
        }
    }
}

#[derive(Default)]
pub struct GradientDescentOptimizer {
    params: Parameters,
}

impl GradientDescentOptimizer {
    pub fn new() -> Self {
        Self {
            params: Parameters::default(),
        }
    }

    pub fn with_parameters(params: Parameters) -> Self {
        Self { params }
    }

    pub fn parameters(&self) -> &Parameters {
        &self.params
    }

    pub fn set_parameters(&mut self, params: Parameters) {
        self.params = params;
    }

    pub fn optimize<F>(&self, graph: &Graph<F>, initial: &Values) -> OptimizationResult
    where
        F: NonlinearFactor + ?Sized,
    {
        let adapter = match FactorGraphAdapter::new(graph, initial) {
            Ok(adapter) => adapter,
            Err(_) => {
                return OptimizationResult {
                    values: initial.clone(),
                    final_error: total_error(graph, initial),
                    iterations: 0,
                    converged: false,
                    gradient_norm: 0.0,
                };
            }
        };
        let mut params = match adapter.values_to_params(initial) {
            Ok(params) => params,
            Err(_) => {
                return OptimizationResult {
                    values: initial.clone(),
                    final_error: total_error(graph, initial),
                    iterations: 0,
                    converged: false,
                    gradient_norm: 0.0,
                };
            }
        };
        let mut values = initial.clone();
        let mut final_error = adapter.compute_error(&values);
        let mut gradient_norm = 0.0;
        let mut converged = false;

        for iteration in 0..=self.params.max_iterations {
            let gradients: Vec<f64> = (0..params.size())
                .map(|index| {
                    finite_difference_gradient_params(&adapter, &params, index, self.params.h)
                })
                .collect();
            gradient_norm = gradients.iter().map(|g| g * g).sum::<f64>().sqrt();

            if gradient_norm < self.params.tolerance {
                converged = true;
                return OptimizationResult {
                    values,
                    final_error,
                    iterations: iteration,
                    converged,
                    gradient_norm,
                };
            }

            let mut step = self.params.step_size;
            let mut updated = false;
            let mut best_candidate: Option<(Vector, Values, f64)> = None;
            while step > 1e-12 {
                let mut candidate_params = params.clone();
                for (index, gradient) in gradients.iter().enumerate() {
                    candidate_params[index] -= step * *gradient;
                }

                let Ok(candidate) = adapter.params_to_values(&candidate_params) else {
                    break;
                };
                let candidate_error = adapter.compute_error(&candidate);
                if best_candidate
                    .as_ref()
                    .map(|(_, _, error)| candidate_error < *error)
                    .unwrap_or(true)
                {
                    best_candidate =
                        Some((candidate_params.clone(), candidate.clone(), candidate_error));
                }
                if candidate_error < final_error {
                    let improvement = final_error - candidate_error;
                    params = candidate_params;
                    values = candidate;
                    final_error = candidate_error;
                    updated = true;
                    if improvement < self.params.tolerance {
                        converged = true;
                        return OptimizationResult {
                            values,
                            final_error,
                            iterations: iteration + 1,
                            converged,
                            gradient_norm,
                        };
                    }
                    break;
                }
                step *= 0.5;
            }

            if !updated {
                if let Some((_, candidate_values, candidate_error)) = best_candidate {
                    if candidate_error < final_error {
                        values = candidate_values;
                        final_error = candidate_error;
                    }
                }

                return OptimizationResult {
                    values,
                    final_error,
                    iterations: iteration,
                    converged: final_error <= self.params.tolerance
                        || gradient_norm <= self.params.tolerance,
                    gradient_norm,
                };
            }
        }

        OptimizationResult {
            values,
            final_error,
            iterations: self.params.max_iterations,
            converged,
            gradient_norm,
        }
    }
}

#[derive(Default)]
pub struct GaussNewtonOptimizer {
    params: GaussNewtonParameters,
}

impl GaussNewtonOptimizer {
    pub fn new() -> Self {
        Self {
            params: GaussNewtonParameters::default(),
        }
    }

    pub fn with_parameters(params: GaussNewtonParameters) -> Self {
        Self { params }
    }

    pub fn parameters(&self) -> &GaussNewtonParameters {
        &self.params
    }

    pub fn set_parameters(&mut self, params: GaussNewtonParameters) {
        self.params = params;
    }

    pub fn optimize<F>(&self, graph: &Graph<F>, initial: &Values) -> OptimizationResult
    where
        F: NonlinearFactor + ?Sized,
    {
        optimize_with_linearization(graph, initial, &self.params, None)
    }
}

#[derive(Default)]
pub struct LevenbergMarquardtOptimizer {
    params: LevenbergMarquardtParameters,
}

impl LevenbergMarquardtOptimizer {
    pub fn new() -> Self {
        Self {
            params: LevenbergMarquardtParameters::default(),
        }
    }

    pub fn with_parameters(params: LevenbergMarquardtParameters) -> Self {
        Self { params }
    }

    pub fn parameters(&self) -> &LevenbergMarquardtParameters {
        &self.params
    }

    pub fn set_parameters(&mut self, params: LevenbergMarquardtParameters) {
        self.params = params;
    }

    pub fn optimize<F>(&self, graph: &Graph<F>, initial: &Values) -> OptimizationResult
    where
        F: NonlinearFactor + ?Sized,
    {
        let mut values = initial.clone();
        let mut lambda = self
            .params
            .initial_lambda
            .clamp(self.params.min_lambda, self.params.max_lambda);
        let mut final_error = total_error(graph, &values);
        let mut converged = false;
        let mut gradient_norm = 0.0;

        for iteration in 0..=self.params.max_iterations {
            let system = match build_normal_equations(graph, &values) {
                Ok(system) => system,
                Err(_) => {
                    return OptimizationResult {
                        values,
                        final_error,
                        iterations: iteration,
                        converged: false,
                        gradient_norm,
                    };
                }
            };
            gradient_norm = euclidean_norm(&system.gradient);

            if gradient_norm < self.params.tolerance {
                converged = true;
                return OptimizationResult {
                    values,
                    final_error,
                    iterations: iteration,
                    converged,
                    gradient_norm,
                };
            }

            let mut accepted = false;
            let mut current_lambda = lambda;
            while current_lambda <= self.params.max_lambda {
                let mut damped = system.hessian.clone();
                for i in 0..damped.len() {
                    damped[i][i] += current_lambda;
                }

                let rhs: Vec<f64> = system.gradient.iter().map(|value| -*value).collect();
                let Some(step) = solve_linear_system(damped, rhs) else {
                    current_lambda *= self.params.lambda_factor;
                    continue;
                };
                let step_norm = euclidean_norm(&step);
                if step_norm < self.params.min_step_norm {
                    converged = true;
                    return OptimizationResult {
                        values,
                        final_error,
                        iterations: iteration,
                        converged,
                        gradient_norm,
                    };
                }

                let candidate = match apply_step(&values, &system.ordering, &step) {
                    Ok(candidate) => candidate,
                    Err(_) => break,
                };
                let candidate_error = total_error(graph, &candidate);
                if candidate_error + self.params.tolerance < final_error {
                    values = candidate;
                    final_error = candidate_error;
                    lambda = (current_lambda / self.params.lambda_factor)
                        .clamp(self.params.min_lambda, self.params.max_lambda);
                    accepted = true;
                    break;
                }

                current_lambda *= self.params.lambda_factor;
            }

            if !accepted {
                return OptimizationResult {
                    values,
                    final_error,
                    iterations: iteration,
                    converged: false,
                    gradient_norm,
                };
            }
        }

        OptimizationResult {
            values,
            final_error,
            iterations: self.params.max_iterations,
            converged,
            gradient_norm,
        }
    }
}

#[derive(Debug, Clone)]
pub struct OptinumGradientDescent {
    pub max_iterations: usize,
    pub step_size: f64,
    pub tolerance: f64,
    pub h: f64,
    pub use_adam: bool,
    pub verbose: bool,
}

impl Default for OptinumGradientDescent {
    fn default() -> Self {
        let params = Parameters::default();
        Self {
            max_iterations: params.max_iterations,
            step_size: params.step_size,
            tolerance: params.tolerance,
            h: params.h,
            use_adam: false,
            verbose: params.verbose,
        }
    }
}

impl OptinumGradientDescent {
    pub fn optimize<F>(&self, graph: &Graph<F>, initial: &Values) -> OptimizationResult
    where
        F: NonlinearFactor + ?Sized,
    {
        let optimizer = GradientDescentOptimizer::with_parameters(Parameters {
            max_iterations: self.max_iterations,
            step_size: self.step_size,
            tolerance: self.tolerance,
            h: self.h,
            verbose: self.verbose,
        });
        optimizer.optimize(graph, initial)
    }
}

#[derive(Debug, Clone)]
pub struct OptinumGaussNewton {
    pub max_iterations: usize,
    pub tolerance: f64,
    pub min_step_norm: f64,
    pub verbose: bool,
}

impl Default for OptinumGaussNewton {
    fn default() -> Self {
        let params = GaussNewtonParameters::default();
        Self {
            max_iterations: params.max_iterations,
            tolerance: params.tolerance,
            min_step_norm: params.min_step_norm,
            verbose: params.verbose,
        }
    }
}

impl OptinumGaussNewton {
    pub fn optimize<F>(&self, graph: &Graph<F>, initial: &Values) -> OptimizationResult
    where
        F: NonlinearFactor + ?Sized,
    {
        let optimizer = GaussNewtonOptimizer::with_parameters(GaussNewtonParameters {
            max_iterations: self.max_iterations,
            tolerance: self.tolerance,
            min_step_norm: self.min_step_norm,
            verbose: self.verbose,
        });
        optimizer.optimize(graph, initial)
    }
}

#[derive(Debug, Clone)]
pub struct OptinumLevenbergMarquardt {
    pub max_iterations: usize,
    pub tolerance: f64,
    pub min_step_norm: f64,
    pub initial_lambda: f64,
    pub lambda_factor: f64,
    pub min_lambda: f64,
    pub max_lambda: f64,
    pub verbose: bool,
}

impl Default for OptinumLevenbergMarquardt {
    fn default() -> Self {
        let params = LevenbergMarquardtParameters::default();
        Self {
            max_iterations: params.max_iterations,
            tolerance: params.tolerance,
            min_step_norm: params.min_step_norm,
            initial_lambda: params.initial_lambda,
            lambda_factor: params.lambda_factor,
            min_lambda: params.min_lambda,
            max_lambda: params.max_lambda,
            verbose: params.verbose,
        }
    }
}

impl OptinumLevenbergMarquardt {
    pub fn optimize<F>(&self, graph: &Graph<F>, initial: &Values) -> OptimizationResult
    where
        F: NonlinearFactor + ?Sized,
    {
        let optimizer =
            LevenbergMarquardtOptimizer::with_parameters(LevenbergMarquardtParameters {
                max_iterations: self.max_iterations,
                tolerance: self.tolerance,
                min_step_norm: self.min_step_norm,
                initial_lambda: self.initial_lambda,
                lambda_factor: self.lambda_factor,
                min_lambda: self.min_lambda,
                max_lambda: self.max_lambda,
                verbose: self.verbose,
            });
        optimizer.optimize(graph, initial)
    }
}

fn total_error<F>(graph: &Graph<F>, values: &Values) -> f64
where
    F: NonlinearFactor + ?Sized,
{
    graph.iter().map(|factor| factor.error(values)).sum()
}

fn finite_difference_gradient_params<F>(
    adapter: &FactorGraphAdapter<'_, F>,
    params: &Vector,
    index: usize,
    h: f64,
) -> f64
where
    F: NonlinearFactor + ?Sized,
{
    let mut plus = params.clone();
    plus[index] += h;
    let mut minus = params.clone();
    minus[index] -= h;

    let plus_error = adapter
        .compute_error_from_params(&plus)
        .unwrap_or(f64::INFINITY);
    let minus_error = adapter
        .compute_error_from_params(&minus)
        .unwrap_or(f64::INFINITY);

    (plus_error - minus_error) / (2.0 * h)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct VariableInfo {
    pub key: Key,
    pub dim: usize,
    pub offset: usize,
}

struct LinearSystem {
    ordering: Vec<VariableInfo>,
    hessian: Vec<Vec<f64>>,
    gradient: Vec<f64>,
}

pub struct FactorGraphAdapter<'a, F: NonlinearFactor + ?Sized> {
    graph: &'a Graph<F>,
    template_values: Values,
    ordering: Vec<VariableInfo>,
    total_param_dim: usize,
    total_residual_dim: usize,
}

impl<'a, F> FactorGraphAdapter<'a, F>
where
    F: NonlinearFactor + ?Sized,
{
    pub fn new(graph: &'a Graph<F>, values: &Values) -> Result<Self, String> {
        let ordering = build_variable_ordering(graph);
        let total_param_dim = ordering
            .last()
            .map(|info| info.offset + info.dim)
            .unwrap_or(0);
        let total_residual_dim = graph.iter().try_fold(0usize, |acc, factor| {
            factor
                .linearize(values)
                .map(|gaussian| acc + gaussian.b().size())
        })?;

        Ok(Self {
            graph,
            template_values: values.clone(),
            ordering,
            total_param_dim,
            total_residual_dim,
        })
    }

    pub fn param_dim(&self) -> usize {
        self.total_param_dim
    }

    pub fn residual_dim(&self) -> usize {
        self.total_residual_dim
    }

    pub fn ordering(&self) -> &[VariableInfo] {
        &self.ordering
    }

    pub fn values_to_params(&self, values: &Values) -> Result<Vector, String> {
        let mut params = Vector::new(self.total_param_dim);
        for info in &self.ordering {
            match info.dim {
                1 => {
                    params[info.offset] = *values
                        .at::<f64>(info.key)
                        .map_err(|_| "missing scalar variable".to_string())?;
                }
                3 => {
                    if let Ok(pose) = values.at::<SE2d>(info.key) {
                        params[info.offset] = pose.x();
                        params[info.offset + 1] = pose.y();
                        params[info.offset + 2] = pose.angle();
                    } else if let Ok(vec) = values.at::<Vec3d>(info.key) {
                        params[info.offset] = vec[0];
                        params[info.offset + 1] = vec[1];
                        params[info.offset + 2] = vec[2];
                    } else {
                        return Err("unsupported 3D variable type".to_string());
                    }
                }
                _ => return Err("unsupported variable dimension".to_string()),
            }
        }
        Ok(params)
    }

    pub fn params_to_values(&self, params: &Vector) -> Result<Values, String> {
        if params.size() != self.total_param_dim {
            return Err("parameter vector dimension mismatch".to_string());
        }

        let mut values = Values::new();
        for info in &self.ordering {
            match info.dim {
                1 => values.insert(info.key, params[info.offset])?,
                3 => {
                    if self.template_values.at::<SE2d>(info.key).is_ok() {
                        values.insert(
                            info.key,
                            SE2d::new(
                                params[info.offset + 2],
                                params[info.offset],
                                params[info.offset + 1],
                            ),
                        )?;
                    } else if self.template_values.at::<Vec3d>(info.key).is_ok() {
                        values.insert(
                            info.key,
                            Vec3d::from_array([
                                params[info.offset],
                                params[info.offset + 1],
                                params[info.offset + 2],
                            ]),
                        )?;
                    } else {
                        return Err("unknown template type for 3D variable".to_string());
                    }
                }
                _ => return Err("unsupported variable dimension".to_string()),
            }
        }
        Ok(values)
    }

    pub fn residuals(&self, params: &Vector) -> Result<Vector, String> {
        let values = self.params_to_values(params)?;
        let mut residuals = Vector::new(self.total_residual_dim);
        let mut offset = 0;

        for factor in self.graph.iter() {
            let gaussian = factor.linearize(&values)?;
            for i in 0..gaussian.b().size() {
                residuals[offset + i] = gaussian.b()[i];
            }
            offset += gaussian.b().size();
        }

        Ok(residuals)
    }

    pub fn jacobian(&self, params: &Vector) -> Result<Matrix, String> {
        let values = self.params_to_values(params)?;
        let mut jacobian = Matrix::new(self.total_residual_dim, self.total_param_dim);
        let mut residual_offset = 0;

        for factor in self.graph.iter() {
            let gaussian = factor.linearize(&values)?;
            for (ki, key) in gaussian.keys().iter().enumerate() {
                let info = self
                    .ordering
                    .iter()
                    .find(|info| info.key == *key)
                    .ok_or_else(|| "key not found in ordering".to_string())?;
                let block = &gaussian.jacobians()[ki];
                for row in 0..block.rows() {
                    for col in 0..block.cols() {
                        jacobian[(residual_offset + row, info.offset + col)] = block[(row, col)];
                    }
                }
            }
            residual_offset += gaussian.b().size();
        }

        Ok(jacobian)
    }

    pub fn compute_error(&self, values: &Values) -> f64 {
        total_error(self.graph, values)
    }

    pub fn compute_error_from_params(&self, params: &Vector) -> Result<f64, String> {
        self.params_to_values(params)
            .map(|values| self.compute_error(&values))
    }

    pub fn graph(&self) -> &'a Graph<F> {
        self.graph
    }
}

fn optimize_with_linearization<F>(
    graph: &Graph<F>,
    initial: &Values,
    params: &GaussNewtonParameters,
    damping: Option<f64>,
) -> OptimizationResult
where
    F: NonlinearFactor + ?Sized,
{
    let mut values = initial.clone();
    let mut final_error = total_error(graph, &values);
    let mut converged = false;
    let mut gradient_norm = 0.0;

    for iteration in 0..=params.max_iterations {
        let mut system = match build_normal_equations(graph, &values) {
            Ok(system) => system,
            Err(_) => {
                return OptimizationResult {
                    values,
                    final_error,
                    iterations: iteration,
                    converged: false,
                    gradient_norm,
                };
            }
        };
        gradient_norm = euclidean_norm(&system.gradient);

        if gradient_norm < params.tolerance {
            converged = true;
            return OptimizationResult {
                values,
                final_error,
                iterations: iteration,
                converged,
                gradient_norm,
            };
        }

        if let Some(lambda) = damping {
            for i in 0..system.hessian.len() {
                system.hessian[i][i] += lambda;
            }
        }

        let rhs: Vec<f64> = system.gradient.iter().map(|value| -*value).collect();
        let Some(step) = solve_linear_system(system.hessian, rhs) else {
            return OptimizationResult {
                values,
                final_error,
                iterations: iteration,
                converged: false,
                gradient_norm,
            };
        };
        let step_norm = euclidean_norm(&step);
        if step_norm < params.min_step_norm {
            converged = true;
            return OptimizationResult {
                values,
                final_error,
                iterations: iteration,
                converged,
                gradient_norm,
            };
        }

        let candidate = match apply_step(&values, &system.ordering, &step) {
            Ok(candidate) => candidate,
            Err(_) => {
                return OptimizationResult {
                    values,
                    final_error,
                    iterations: iteration,
                    converged: false,
                    gradient_norm,
                };
            }
        };
        let candidate_error = total_error(graph, &candidate);

        if (final_error - candidate_error).abs() < params.tolerance {
            converged = true;
            return OptimizationResult {
                values: candidate,
                final_error: candidate_error,
                iterations: iteration + 1,
                converged,
                gradient_norm,
            };
        }

        if candidate_error > final_error {
            return OptimizationResult {
                values,
                final_error,
                iterations: iteration,
                converged: false,
                gradient_norm,
            };
        }

        values = candidate;
        final_error = candidate_error;
    }

    OptimizationResult {
        values,
        final_error,
        iterations: params.max_iterations,
        converged,
        gradient_norm,
    }
}

fn build_normal_equations<F>(graph: &Graph<F>, values: &Values) -> Result<LinearSystem, String>
where
    F: NonlinearFactor + ?Sized,
{
    let ordering = build_variable_ordering(graph);
    let total_dim = ordering
        .last()
        .map(|info| info.offset + info.dim)
        .unwrap_or(0);
    let mut hessian = vec![vec![0.0; total_dim]; total_dim];
    let mut gradient = vec![0.0; total_dim];

    for factor in graph.iter() {
        let gaussian = factor.linearize(values)?;
        let residual = gaussian.b();

        for (a_index, key_a) in gaussian.keys().iter().enumerate() {
            let info_a = ordering
                .iter()
                .find(|info| info.key == *key_a)
                .ok_or_else(|| "missing variable ordering".to_string())?;
            let jacobian_a = &gaussian.jacobians()[a_index];

            for col_a in 0..info_a.dim {
                let global_a = info_a.offset + col_a;
                for row in 0..residual.size() {
                    gradient[global_a] += jacobian_a[(row, col_a)] * residual[row];
                }
            }

            for (b_index, key_b) in gaussian.keys().iter().enumerate() {
                let info_b = ordering
                    .iter()
                    .find(|info| info.key == *key_b)
                    .ok_or_else(|| "missing variable ordering".to_string())?;
                let jacobian_b = &gaussian.jacobians()[b_index];

                for col_a in 0..info_a.dim {
                    let global_a = info_a.offset + col_a;
                    for col_b in 0..info_b.dim {
                        let global_b = info_b.offset + col_b;
                        let mut sum = 0.0;
                        for row in 0..residual.size() {
                            sum += jacobian_a[(row, col_a)] * jacobian_b[(row, col_b)];
                        }
                        hessian[global_a][global_b] += sum;
                    }
                }
            }
        }
    }

    Ok(LinearSystem {
        ordering,
        hessian,
        gradient,
    })
}

fn build_variable_ordering<F>(graph: &Graph<F>) -> Vec<VariableInfo>
where
    F: NonlinearFactor + ?Sized,
{
    let mut dims = std::collections::BTreeMap::new();

    for factor in graph.iter() {
        for key in factor.keys() {
            dims.entry(*key).or_insert_with(|| factor.dim(*key));
        }
    }

    let mut ordering = Vec::with_capacity(dims.len());
    let mut offset = 0;

    for (key, dim) in dims {
        ordering.push(VariableInfo { key, dim, offset });
        offset += dim;
    }

    ordering
}

fn apply_step(values: &Values, ordering: &[VariableInfo], step: &[f64]) -> Result<Values, String> {
    let mut updated = values.clone();
    for info in ordering {
        match info.dim {
            1 => {
                let current = *updated
                    .at::<f64>(info.key)
                    .map_err(|_| "missing scalar variable".to_string())?;
                updated.erase(info.key);
                updated.insert(info.key, current + step[info.offset])?;
            }
            3 => {
                if let Ok(current) = updated.at::<SE2d>(info.key).copied() {
                    let delta = Vec3d::from_array([
                        step[info.offset],
                        step[info.offset + 1],
                        step[info.offset + 2],
                    ]);
                    updated.erase(info.key);
                    updated.insert(info.key, current.retract(delta))?;
                } else if let Ok(current) = updated.at::<Vec3d>(info.key).copied() {
                    let next = Vec3d::from_array([
                        current[0] + step[info.offset],
                        current[1] + step[info.offset + 1],
                        current[2] + step[info.offset + 2],
                    ]);
                    updated.erase(info.key);
                    updated.insert(info.key, next)?;
                } else {
                    return Err("unsupported 3D variable type".to_string());
                }
            }
            _ => return Err("unsupported variable dimension".to_string()),
        }
    }
    Ok(updated)
}

fn perturb_value(values: &Values, key: Key, dim: usize, epsilon: f64) -> Result<Values, String> {
    let mut perturbed = values.clone();
    if let Ok(current) = values.at::<f64>(key) {
        if dim != 0 {
            return Err("scalar variable only has one dimension".to_string());
        }
        perturbed.erase(key);
        perturbed.insert(key, *current + epsilon)?;
        return Ok(perturbed);
    }

    if let Ok(current) = values.at::<SE2d>(key) {
        let mut delta = [0.0; 3];
        if dim >= 3 {
            return Err("SE2 variable dimension out of bounds".to_string());
        }
        delta[dim] = epsilon;
        perturbed.erase(key);
        perturbed.insert(key, current.retract(Vec3d::from_array(delta)))?;
        return Ok(perturbed);
    }

    if let Ok(current) = values.at::<Vec3d>(key) {
        if dim >= 3 {
            return Err("Vec3d variable dimension out of bounds".to_string());
        }
        let mut next = [current[0], current[1], current[2]];
        next[dim] += epsilon;
        perturbed.erase(key);
        perturbed.insert(key, Vec3d::from_array(next))?;
        return Ok(perturbed);
    }

    Err("unsupported variable type in linearize".to_string())
}

fn solve_linear_system(mut a: Vec<Vec<f64>>, mut b: Vec<f64>) -> Option<Vec<f64>> {
    let n = b.len();
    if a.len() != n || a.iter().any(|row| row.len() != n) {
        return None;
    }

    for pivot in 0..n {
        let mut best = pivot;
        for row in (pivot + 1)..n {
            if a[row][pivot].abs() > a[best][pivot].abs() {
                best = row;
            }
        }
        if a[best][pivot].abs() < 1e-12 {
            return None;
        }
        if best != pivot {
            a.swap(best, pivot);
            b.swap(best, pivot);
        }

        let diag = a[pivot][pivot];
        for row in (pivot + 1)..n {
            let factor = a[row][pivot] / diag;
            if factor == 0.0 {
                continue;
            }
            for col in pivot..n {
                a[row][col] -= factor * a[pivot][col];
            }
            b[row] -= factor * b[pivot];
        }
    }

    let mut x = vec![0.0; n];
    for row in (0..n).rev() {
        let mut sum = b[row];
        for (col, value) in x.iter().enumerate().skip(row + 1) {
            sum -= a[row][col] * value;
        }
        if a[row][row].abs() < 1e-12 {
            return None;
        }
        x[row] = sum / a[row][row];
    }

    Some(x)
}

fn euclidean_norm(values: &[f64]) -> f64 {
    values.iter().map(|value| value * value).sum::<f64>().sqrt()
}
