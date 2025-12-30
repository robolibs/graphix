#pragma once

/**
 * @file optimizer_adapter.hpp
 * @brief Adapter to use optinum optimizers with graphix factor graphs
 *
 * This adapter bridges graphix's factor graph representation with optinum's
 * optimizer interface. It provides:
 * - Conversion between graphix Values and flat parameter vectors
 * - A residual function wrapper compatible with optinum optimizers
 * - Support for SE2d, Vec3d, and scalar variable types
 */

#include "graphix/factor/graph.hpp"
#include "graphix/factor/nonlinear/nonlinear_factor.hpp"
#include "graphix/factor/types.hpp"
#include "graphix/factor/values.hpp"

#include <cmath>
#include <datapod/matrix.hpp>
#include <iostream>
#include <limits>
#include <map>
#include <stdexcept>
#include <vector>

namespace graphix::factor {

    /**
     * @brief Variable info for tracking variable ordering and dimensions
     */
    struct VariableInfo {
        Key key;            ///< Variable key
        std::size_t dim;    ///< Dimension of the variable (1, 3, etc.)
        std::size_t offset; ///< Offset in the flattened parameter vector

        VariableInfo(Key k, std::size_t d, std::size_t o) : key(k), dim(d), offset(o) {}
    };

    /**
     * @brief Adapter that wraps a factor graph for use with optinum optimizers
     *
     * Converts between graphix's Values representation and flat parameter vectors
     * that optinum optimizers expect. Also provides a residual function that
     * computes all factor residuals from a parameter vector.
     *
     * Usage:
     * @code
     * Graph<NonlinearFactor> graph;
     * Values initial_values;
     * // ... populate graph and values ...
     *
     * FactorGraphAdapter adapter(graph, initial_values);
     *
     * // Use with optinum::opti::GaussNewton
     * optinum::opti::GaussNewton<double> gn;
     * auto result = gn.optimize(adapter, adapter.values_to_params(initial_values));
     *
     * // Convert result back to Values
     * Values optimized = adapter.params_to_values(result.x);
     * @endcode
     */
    class FactorGraphAdapter {
      public:
        /**
         * @brief Construct adapter from a factor graph and initial values
         *
         * Analyzes the graph to determine variable ordering and dimensions.
         * The initial values are used as a template for type information.
         *
         * @param graph Factor graph containing constraints
         * @param values Initial variable values (used for type inference)
         */
        inline FactorGraphAdapter(const Graph<NonlinearFactor> &graph, const Values &values)
            : graph_(graph), template_values_(values), total_param_dim_(0), total_residual_dim_(0) {
            // Build variable ordering from graph
            std::map<Key, std::size_t> key_to_dim;

            // Collect all unique variables and their dimensions
            for (std::size_t fi = 0; fi < graph_.size(); ++fi) {
                const auto &factor = graph_[fi];
                for (Key key : factor->keys()) {
                    if (key_to_dim.find(key) == key_to_dim.end()) {
                        std::size_t dim = factor->dim(key);
                        key_to_dim[key] = dim;
                    }
                }
            }

            // Build ordered variable list with offsets
            std::size_t offset = 0;
            for (const auto &[key, dim] : key_to_dim) {
                ordering_.emplace_back(key, dim, offset);
                key_to_index_[key] = ordering_.size() - 1;
                offset += dim;
            }
            total_param_dim_ = offset;

            // Compute total residual dimension
            for (std::size_t fi = 0; fi < graph_.size(); ++fi) {
                // Linearize to get residual dimension (uses template values)
                auto gaussian = graph_[fi]->linearize(values);
                total_residual_dim_ += gaussian->b().size();
            }
        }

        /**
         * @brief Get total dimension of the parameter vector
         */
        std::size_t param_dim() const { return total_param_dim_; }

        /**
         * @brief Get total dimension of the residual vector
         */
        std::size_t residual_dim() const { return total_residual_dim_; }

        /**
         * @brief Get the variable ordering
         */
        const std::vector<VariableInfo> &ordering() const { return ordering_; }

        /**
         * @brief Convert Values to a flat parameter vector
         *
         * @param values Variable values to flatten
         * @return Flat parameter vector
         */
        inline DynVec values_to_params(const Values &values) const {
            DynVec params(total_param_dim_);

            for (const auto &var_info : ordering_) {
                Key key = var_info.key;
                std::size_t dim = var_info.dim;
                std::size_t offset = var_info.offset;

                if (dim == 1) {
                    // Scalar variable
                    params[offset] = values.at<double>(key);
                } else if (dim == 3) {
                    // Try SE2d first (store as x, y, theta)
                    bool handled = false;
                    try {
                        SE2d pose = values.at<SE2d>(key);
                        params[offset] = pose.x();
                        params[offset + 1] = pose.y();
                        params[offset + 2] = pose.angle();
                        handled = true;
                    } catch (...) {
                        // Not an SE2d
                    }

                    if (!handled) {
                        // Try Vec3d
                        try {
                            Vec3d vec = values.at<Vec3d>(key);
                            params[offset] = vec[0];
                            params[offset + 1] = vec[1];
                            params[offset + 2] = vec[2];
                            handled = true;
                        } catch (...) {
                            throw std::runtime_error("Unknown 3D variable type for key");
                        }
                    }
                } else {
                    throw std::runtime_error("Unsupported variable dimension: " + std::to_string(dim));
                }
            }

            return params;
        }

        /**
         * @brief Convert flat parameter vector back to Values
         *
         * Uses the template values to determine the type of each variable.
         *
         * @param params Flat parameter vector
         * @return Reconstructed Values object
         */
        inline Values params_to_values(const DynVec &params) const {
            Values values;

            for (const auto &var_info : ordering_) {
                Key key = var_info.key;
                std::size_t dim = var_info.dim;
                std::size_t offset = var_info.offset;

                if (dim == 1) {
                    // Scalar variable
                    values.insert(key, params[offset]);
                } else if (dim == 3) {
                    // Check template to determine type
                    bool handled = false;
                    try {
                        // Try SE2d first
                        template_values_.at<SE2d>(key);
                        // It's an SE2d - reconstruct from (x, y, theta)
                        SE2d pose(params[offset], params[offset + 1], params[offset + 2]);
                        values.insert(key, pose);
                        handled = true;
                    } catch (...) {
                        // Not an SE2d
                    }

                    if (!handled) {
                        try {
                            // Try Vec3d
                            template_values_.at<Vec3d>(key);
                            Vec3d vec;
                            vec[0] = params[offset];
                            vec[1] = params[offset + 1];
                            vec[2] = params[offset + 2];
                            values.insert(key, vec);
                            handled = true;
                        } catch (...) {
                            throw std::runtime_error("Unknown 3D variable type in template values");
                        }
                    }
                } else {
                    throw std::runtime_error("Unsupported variable dimension: " + std::to_string(dim));
                }
            }

            return values;
        }

        /**
         * @brief Residual function operator for optinum optimizers
         *
         * Computes residuals from all factors given the parameter vector.
         * This is the main interface expected by optinum's GaussNewton.
         *
         * @param params Current parameter vector
         * @return Residual vector from all factors
         */
        inline DynVec operator()(const DynVec &params) const {
            // Convert params to Values
            Values values = params_to_values(params);

            // Compute residuals from all factors
            DynVec residuals(total_residual_dim_);
            std::size_t residual_offset = 0;

            for (std::size_t fi = 0; fi < graph_.size(); ++fi) {
                // Get error vector from factor
                auto gaussian = graph_[fi]->linearize(values);
                const auto &b = gaussian->b();

                for (std::size_t i = 0; i < b.size(); ++i) {
                    residuals[residual_offset + i] = b[i];
                }
                residual_offset += b.size();
            }

            return residuals;
        }

        /**
         * @brief Compute the analytical Jacobian (optional, for better performance)
         *
         * If this method is present, optinum will use it instead of numerical
         * differentiation. Uses graphix's built-in linearization.
         *
         * @param params Current parameter vector
         * @return Jacobian matrix (residual_dim x param_dim)
         */
        inline DynMat jacobian(const DynVec &params) const {
            Values values = params_to_values(params);

            DynMat J(total_residual_dim_, total_param_dim_);
            // Initialize to zero
            for (std::size_t i = 0; i < total_residual_dim_; ++i) {
                for (std::size_t j = 0; j < total_param_dim_; ++j) {
                    J(i, j) = 0.0;
                }
            }

            std::size_t residual_offset = 0;

            for (std::size_t fi = 0; fi < graph_.size(); ++fi) {
                auto gaussian = graph_[fi]->linearize(values);
                const auto &factor_keys = gaussian->keys();
                const auto &jacobians = gaussian->jacobians();
                const auto &b = gaussian->b();
                std::size_t factor_residual_dim = b.size();

                // Fill in Jacobian blocks for each variable in this factor
                for (std::size_t ki = 0; ki < factor_keys.size(); ++ki) {
                    Key key = factor_keys[ki];
                    auto it = key_to_index_.find(key);
                    if (it == key_to_index_.end()) {
                        throw std::runtime_error("Key not found in ordering");
                    }

                    const VariableInfo &var_info = ordering_[it->second];
                    std::size_t param_offset = var_info.offset;

                    const auto &Jk = jacobians[ki];

                    // Copy Jacobian block into full Jacobian matrix
                    for (std::size_t i = 0; i < Jk.rows(); ++i) {
                        for (std::size_t j = 0; j < Jk.cols(); ++j) {
                            J(residual_offset + i, param_offset + j) = Jk(i, j);
                        }
                    }
                }

                residual_offset += factor_residual_dim;
            }

            return J;
        }

        /**
         * @brief Compute total error of the graph given values
         *
         * @param values Variable values
         * @return Sum of all factor errors
         */
        inline double compute_error(const Values &values) const {
            double total_error = 0.0;
            for (std::size_t i = 0; i < graph_.size(); ++i) {
                total_error += graph_[i]->error(values);
            }
            return total_error;
        }

        /**
         * @brief Compute total error given parameter vector
         *
         * @param params Parameter vector
         * @return Sum of all factor errors
         */
        inline double compute_error(const DynVec &params) const { return compute_error(params_to_values(params)); }

        /**
         * @brief Get reference to the underlying graph
         */
        const Graph<NonlinearFactor> &graph() const { return graph_; }

      private:
        const Graph<NonlinearFactor> &graph_;
        Values template_values_;
        std::vector<VariableInfo> ordering_;
        std::map<Key, std::size_t> key_to_index_;
        std::size_t total_param_dim_;
        std::size_t total_residual_dim_;
    };

    /**
     * @brief Result of optimization using optinum
     *
     * Mirrors the graphix GaussNewtonOptimizer::Result interface.
     */
    struct OptimizerResult {
        Values values;      ///< Optimized values
        double final_error; ///< Final total error
        int iterations;     ///< Number of iterations performed
        bool converged;     ///< Whether convergence was achieved

        OptimizerResult(const Values &v, double err, int iter, bool conv)
            : values(v), final_error(err), iterations(iter), converged(conv) {}
    };

    /**
     * @brief Wrapper to use optinum::opti::GaussNewton with graphix factor graphs
     *
     * Provides the same interface as graphix::factor::GaussNewtonOptimizer
     * but uses optinum's implementation internally.
     *
     * Usage:
     * @code
     * OptinumGaussNewton optimizer;
     * optimizer.max_iterations = 50;
     * optimizer.tolerance = 1e-6;
     *
     * auto result = optimizer.optimize(graph, initial_values);
     * Values optimized = result.values;
     * @endcode
     */
    class OptinumGaussNewton {
      public:
        // Configuration parameters (matching optinum::opti::GaussNewton)
        std::size_t max_iterations = 100;
        double tolerance = 1e-6;
        double min_step_norm = 1e-9;
        double min_gradient_norm = 1e-6;
        bool verbose = false;
        bool use_line_search = false;

        OptinumGaussNewton() = default;

        /**
         * @brief Optimize a factor graph starting from initial values
         *
         * @param graph Factor graph to optimize
         * @param initial Initial variable values
         * @return Optimization result with final values and diagnostics
         */
        inline OptimizerResult optimize(const Graph<NonlinearFactor> &graph, const Values &initial) const {
            // Create adapter
            FactorGraphAdapter adapter(graph, initial);

            // Convert initial values to parameter vector
            DynVec x = adapter.values_to_params(initial);

            // Manual Gauss-Newton implementation using adapter
            // (Could also instantiate optinum::opti::GaussNewton<double> here)
            double current_error = adapter.compute_error(initial);

            if (verbose) {
                std::cout << "OptinumGaussNewton Optimization" << std::endl;
                std::cout << "Variables: " << adapter.param_dim() << ", Residuals: " << adapter.residual_dim()
                          << std::endl;
                std::cout << "Initial error: " << current_error << std::endl;
            }

            Values current_values = initial;
            bool converged = false;
            std::size_t iteration = 0;

            for (; iteration < max_iterations; ++iteration) {
                // Get Jacobian and residual
                DynMat J = adapter.jacobian(x);
                DynVec r = adapter(x);

                // Build normal equations: J^T * J * dx = -J^T * r
                DynMat JtJ = matmul(transpose(J), J);
                DynVec Jtr = matmul(transpose(J), r);

                // Negate for -J^T * r
                for (std::size_t i = 0; i < Jtr.size(); ++i) {
                    Jtr[i] = -Jtr[i];
                }

                // Solve linear system
                DynVec dx = solve(JtJ, Jtr);

                // Compute step norm
                double step_norm = norm(dx);

                if (verbose) {
                    std::cout << "Iter " << iteration << ": error = " << current_error << ", step_norm = " << step_norm
                              << std::endl;
                }

                if (step_norm < min_step_norm) {
                    if (verbose) {
                        std::cout << "Converged: step_norm < " << min_step_norm << std::endl;
                    }
                    converged = true;
                    break;
                }

                // Apply update
                for (std::size_t i = 0; i < x.size(); ++i) {
                    x[i] += dx[i];
                }

                // Convert back to values
                current_values = adapter.params_to_values(x);
                double new_error = adapter.compute_error(current_values);

                // Check for error decrease convergence
                double error_decrease = current_error - new_error;
                if (std::abs(error_decrease) < tolerance) {
                    if (verbose) {
                        std::cout << "Converged: error_decrease < " << tolerance << std::endl;
                    }
                    current_error = new_error;
                    converged = true;
                    break;
                }

                current_error = new_error;
            }

            if (!converged && verbose) {
                std::cout << "Max iterations reached" << std::endl;
            }

            return OptimizerResult(current_values, current_error, static_cast<int>(iteration), converged);
        }
    };

    /**
     * @brief Wrapper to use optinum::opti::LevenbergMarquardt with graphix factor graphs
     *
     * Provides the same interface as graphix::factor::LevenbergMarquardtOptimizer
     * but uses optinum-style Levenberg-Marquardt implementation internally.
     *
     * Levenberg-Marquardt improves on Gauss-Newton by adding a damping term:
     *   (J^T * J + λ*I) * dx = -J^T * r
     *
     * The damping parameter λ is adaptively adjusted:
     * - If error decreases: λ is reduced (approach Gauss-Newton)
     * - If error increases: λ is increased (approach gradient descent)
     *
     * Usage:
     * @code
     * OptinumLevenbergMarquardt optimizer;
     * optimizer.max_iterations = 50;
     * optimizer.tolerance = 1e-6;
     * optimizer.initial_lambda = 1e-3;
     *
     * auto result = optimizer.optimize(graph, initial_values);
     * Values optimized = result.values;
     * @endcode
     */
    class OptinumLevenbergMarquardt {
      public:
        // Configuration parameters (matching optinum::opti::LevenbergMarquardt)
        std::size_t max_iterations = 100;
        double tolerance = 1e-6;
        double min_step_norm = 1e-9;
        double min_gradient_norm = 1e-6;
        double initial_lambda = 1e-3;
        double lambda_factor = 10.0;
        double min_lambda = 1e-7;
        double max_lambda = 1e7;
        bool verbose = false;

        OptinumLevenbergMarquardt() = default;

        /**
         * @brief Optimize a factor graph starting from initial values
         *
         * @param graph Factor graph to optimize
         * @param initial Initial variable values
         * @return Optimization result with final values and diagnostics
         */
        inline OptimizerResult optimize(const Graph<NonlinearFactor> &graph, const Values &initial) const {
            // Create adapter
            FactorGraphAdapter adapter(graph, initial);

            // Convert initial values to parameter vector
            DynVec x = adapter.values_to_params(initial);
            const std::size_t n = x.size();

            // Initial error and damping
            double current_error = adapter.compute_error(initial);
            double lambda = initial_lambda;

            if (verbose) {
                std::cout << "OptinumLevenbergMarquardt Optimization" << std::endl;
                std::cout << "Variables: " << adapter.param_dim() << ", Residuals: " << adapter.residual_dim()
                          << std::endl;
                std::cout << "Initial error: " << current_error << std::endl;
                std::cout << "Initial lambda: " << lambda << std::endl;
            }

            Values current_values = initial;
            bool converged = false;
            std::size_t iteration = 0;

            for (; iteration < max_iterations; ++iteration) {
                // Get Jacobian and residual
                DynMat J = adapter.jacobian(x);
                DynVec r = adapter(x);

                // Build damped normal equations: (J^T * J + λ*I) * dx = -J^T * r
                DynMat JtJ = matmul(transpose(J), J);
                DynVec Jtr = matmul(transpose(J), r);

                // Add damping to diagonal
                add_diagonal(JtJ, lambda);

                // Negate for -J^T * r
                for (std::size_t i = 0; i < Jtr.size(); ++i) {
                    Jtr[i] = -Jtr[i];
                }

                // Try to solve linear system
                auto solve_result = try_solve(JtJ, Jtr);
                if (!solve_result.is_ok()) {
                    // Solve failed - increase damping and retry
                    if (verbose) {
                        std::cout << "Iter " << iteration << ": Solve failed, increasing lambda from " << lambda
                                  << " to " << lambda * lambda_factor << std::endl;
                    }
                    lambda = std::min(lambda * lambda_factor, max_lambda);

                    if (lambda >= max_lambda) {
                        if (verbose) {
                            std::cout << "Lambda reached maximum, terminating" << std::endl;
                        }
                        break;
                    }
                    continue; // Retry with larger lambda
                }

                DynVec dx = solve_result.value();

                // Compute step norm
                double step_norm = norm(dx);

                if (verbose) {
                    std::cout << "Iter " << iteration << ": error = " << current_error << ", step_norm = " << step_norm
                              << ", lambda = " << lambda << std::endl;
                }

                if (step_norm < min_step_norm) {
                    if (verbose) {
                        std::cout << "Converged: step_norm < " << min_step_norm << std::endl;
                    }
                    converged = true;
                    break;
                }

                // Try the step: x_new = x + dx
                DynVec x_new(n);
                for (std::size_t i = 0; i < n; ++i) {
                    x_new[i] = x[i] + dx[i];
                }

                // Evaluate new error
                Values new_values = adapter.params_to_values(x_new);
                double new_error = adapter.compute_error(new_values);

                // Adaptive lambda adjustment based on error improvement
                if (new_error < current_error) {
                    // Good step - accept and decrease lambda (approach Gauss-Newton)
                    double error_decrease = current_error - new_error;

                    if (verbose) {
                        std::cout << "  Step accepted, error decreased by " << error_decrease << std::endl;
                        std::cout << "  Decreasing lambda from " << lambda << " to "
                                  << std::max(lambda / lambda_factor, min_lambda) << std::endl;
                    }

                    x = x_new;
                    current_values = new_values;
                    current_error = new_error;
                    lambda = std::max(lambda / lambda_factor, min_lambda);

                    // Check convergence on error decrease
                    if (std::abs(error_decrease) < tolerance) {
                        if (verbose) {
                            std::cout << "Converged: error_decrease < " << tolerance << std::endl;
                        }
                        converged = true;
                        break;
                    }
                } else {
                    // Bad step - reject and increase lambda (approach gradient descent)
                    if (verbose) {
                        std::cout << "  Step rejected, error increased from " << current_error << " to " << new_error
                                  << std::endl;
                        std::cout << "  Increasing lambda from " << lambda << " to "
                                  << std::min(lambda * lambda_factor, max_lambda) << std::endl;
                    }

                    lambda = std::min(lambda * lambda_factor, max_lambda);

                    if (lambda >= max_lambda) {
                        if (verbose) {
                            std::cout << "Lambda reached maximum, terminating" << std::endl;
                        }
                        break;
                    }
                    // Don't update x or current_error - retry with larger lambda
                }
            }

            if (!converged && verbose) {
                std::cout << "Max iterations reached" << std::endl;
            }

            return OptimizerResult(current_values, current_error, static_cast<int>(iteration), converged);
        }
    };

    /**
     * @brief Objective function adapter for gradient-based optimizers
     *
     * Wraps a factor graph to provide the evaluate/gradient interface expected
     * by optinum's GradientDescent optimizer. Computes:
     * - evaluate(x): total weighted squared error
     * - gradient(x, g): numerical gradient using finite differences
     * - evaluate_with_gradient(x, g): combined evaluation
     */
    class FactorGraphObjective {
      public:
        /**
         * @brief Construct objective from a factor graph and initial values
         *
         * @param graph Factor graph containing constraints
         * @param values Initial values (used for type inference)
         * @param h Step size for finite difference gradient (default: 1e-5)
         */
        inline FactorGraphObjective(const Graph<NonlinearFactor> &graph, const Values &values, double h = 1e-5)
            : adapter_(graph, values), h_(h) {}

        /**
         * @brief Evaluate the objective function (total error)
         *
         * @param params Parameter vector
         * @return Total weighted squared error
         */
        inline double evaluate(const DynVec &params) const { return adapter_.compute_error(params); }

        /**
         * @brief Compute gradient using finite differences
         *
         * @param params Parameter vector
         * @param gradient Output gradient vector
         */
        inline void gradient(const DynVec &params, DynVec &gradient) const {
            const std::size_t n = params.size();

            // Ensure gradient is properly sized
            if (gradient.size() != n) {
                gradient = DynVec(n);
            }

            // Central finite difference for each parameter
            DynVec params_plus = params;
            DynVec params_minus = params;

            for (std::size_t i = 0; i < n; ++i) {
                params_plus[i] = params[i] + h_;
                params_minus[i] = params[i] - h_;

                double error_plus = adapter_.compute_error(params_plus);
                double error_minus = adapter_.compute_error(params_minus);

                gradient[i] = (error_plus - error_minus) / (2.0 * h_);

                // Restore original values
                params_plus[i] = params[i];
                params_minus[i] = params[i];
            }
        }

        /**
         * @brief Combined evaluation for efficiency (required by optinum GradientDescent)
         *
         * @param params Parameter vector
         * @param grad Output gradient vector
         * @return Objective value
         */
        inline double evaluate_with_gradient(const DynVec &params, DynVec &grad) const {
            gradient(params, grad);
            return evaluate(params);
        }

        /**
         * @brief Get the underlying adapter
         */
        const FactorGraphAdapter &adapter() const { return adapter_; }

      private:
        FactorGraphAdapter adapter_;
        double h_; ///< Step size for finite differences
    };

    /**
     * @brief Wrapper to use optinum-style gradient descent with graphix factor graphs
     *
     * Provides a simple gradient descent optimizer that uses the FactorGraphObjective
     * adapter. Supports vanilla gradient descent and Adam-style adaptive learning rates.
     *
     * Usage:
     * @code
     * OptinumGradientDescent optimizer;
     * optimizer.step_size = 0.01;
     * optimizer.max_iterations = 1000;
     * optimizer.use_adam = true;  // Enable Adam optimizer
     *
     * auto result = optimizer.optimize(graph, initial_values);
     * Values optimized = result.values;
     * @endcode
     */
    class OptinumGradientDescent {
      public:
        // Configuration parameters (matching optinum::opti::GradientDescent)
        double step_size = 0.01;
        std::size_t max_iterations = 10000;
        double tolerance = 1e-6;
        double h = 1e-5; ///< Step size for finite difference gradient
        bool verbose = false;

        // Adam parameters
        bool use_adam = false;
        double adam_beta1 = 0.9;
        double adam_beta2 = 0.999;
        double adam_epsilon = 1e-8;

        OptinumGradientDescent() = default;

        /**
         * @brief Gradient descent result with additional gradient norm info
         */
        struct Result : public OptimizerResult {
            double gradient_norm;

            Result(const Values &v, double err, int iter, bool conv, double gnorm)
                : OptimizerResult(v, err, iter, conv), gradient_norm(gnorm) {}
        };

        /**
         * @brief Optimize a factor graph starting from initial values
         *
         * @param graph Factor graph to optimize
         * @param initial Initial variable values
         * @return Optimization result with final values and diagnostics
         */
        inline Result optimize(const Graph<NonlinearFactor> &graph, const Values &initial) const {
            // Create objective function adapter
            FactorGraphObjective objective(graph, initial, h);
            const auto &adapter = objective.adapter();

            // Convert initial values to parameter vector
            DynVec x = adapter.values_to_params(initial);
            const std::size_t n = x.size();

            // Allocate gradient
            DynVec gradient(n);

            // Adam state (first and second moments)
            DynVec m(n, 0.0); // First moment
            DynVec v(n, 0.0); // Second moment
            std::size_t t = 0;

            double current_error = objective.evaluate(x);
            double last_error = std::numeric_limits<double>::max();
            double gnorm = 0.0;

            if (verbose) {
                std::cout << "OptinumGradientDescent Optimization" << std::endl;
                std::cout << "Variables: " << adapter.param_dim() << std::endl;
                std::cout << "Initial error: " << current_error << std::endl;
                if (use_adam) {
                    std::cout << "Using Adam optimizer" << std::endl;
                }
            }

            Values current_values = initial;
            bool converged = false;
            std::size_t iteration = 0;

            for (; iteration < max_iterations; ++iteration) {
                // Compute gradient
                objective.gradient(x, gradient);

                // Compute gradient norm
                gnorm = norm(gradient);

                if (verbose && (iteration % 100 == 0 || iteration < 10)) {
                    std::cout << "Iter " << iteration << ": error = " << current_error << ", ||g|| = " << gnorm
                              << std::endl;
                }

                // Check convergence on gradient norm
                if (gnorm < tolerance) {
                    if (verbose) {
                        std::cout << "Converged: gradient_norm < " << tolerance << std::endl;
                    }
                    converged = true;
                    break;
                }

                // Apply update
                if (use_adam) {
                    // Adam update
                    ++t;
                    double bias_correction1 = 1.0 - std::pow(adam_beta1, static_cast<double>(t));
                    double bias_correction2 = 1.0 - std::pow(adam_beta2, static_cast<double>(t));
                    double step_correction = step_size * std::sqrt(bias_correction2) / bias_correction1;

                    for (std::size_t i = 0; i < n; ++i) {
                        // Update moments
                        m[i] = adam_beta1 * m[i] + (1.0 - adam_beta1) * gradient[i];
                        v[i] = adam_beta2 * v[i] + (1.0 - adam_beta2) * gradient[i] * gradient[i];

                        // Apply update
                        x[i] -= step_correction * m[i] / (std::sqrt(v[i]) + adam_epsilon);
                    }
                } else {
                    // Vanilla gradient descent with backtracking line search
                    double step = step_size;
                    DynVec x_new(n);

                    for (std::size_t i = 0; i < n; ++i) {
                        x_new[i] = x[i] - step * gradient[i];
                    }

                    double new_error = objective.evaluate(x_new);
                    int backtrack_count = 0;
                    const int max_backtrack = 20;

                    while (new_error > current_error && backtrack_count < max_backtrack) {
                        step *= 0.5;
                        for (std::size_t i = 0; i < n; ++i) {
                            x_new[i] = x[i] - step * gradient[i];
                        }
                        new_error = objective.evaluate(x_new);
                        ++backtrack_count;
                    }

                    if (verbose && backtrack_count > 0 && (iteration % 100 == 0 || iteration < 10)) {
                        std::cout << "  Backtracked " << backtrack_count << " times, step = " << step << std::endl;
                    }

                    // If we couldn't improve, we're stuck
                    if (new_error >= current_error && backtrack_count >= max_backtrack) {
                        if (verbose) {
                            std::cout << "Cannot improve further, stopping" << std::endl;
                        }
                        break;
                    }

                    x = x_new;
                }

                // Evaluate new error
                double new_error = objective.evaluate(x);

                // Check convergence on error decrease
                if (std::abs(last_error - new_error) < tolerance) {
                    if (verbose) {
                        std::cout << "Converged: error_change < " << tolerance << std::endl;
                    }
                    current_error = new_error;
                    converged = true;
                    break;
                }

                last_error = current_error;
                current_error = new_error;
            }

            if (!converged && verbose) {
                std::cout << "Max iterations reached" << std::endl;
            }

            current_values = adapter.params_to_values(x);
            return Result(current_values, current_error, static_cast<int>(iteration), converged, gnorm);
        }
    };

} // namespace graphix::factor
