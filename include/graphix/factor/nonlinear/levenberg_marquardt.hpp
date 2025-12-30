#pragma once

#include "graphix/factor/graph.hpp"
#include "graphix/factor/linear/gaussian_factor.hpp"
#include "graphix/factor/nonlinear/nonlinear_factor.hpp"
#include "graphix/factor/types.hpp"
#include "graphix/factor/values.hpp"
#include <cmath>
#include <iostream>
#include <map>

namespace graphix::factor {

    /**
     * @brief Levenberg-Marquardt optimizer for nonlinear least squares
     *
     * Similar to Gauss-Newton but adds damping for better robustness:
     *   (J^T * J + λ*I) * dx = -J^T * b
     *
     * The damping parameter λ is adjusted based on error improvement:
     * - If error decreases: λ is reduced (approach Gauss-Newton)
     * - If error increases: λ is increased (approach gradient descent)
     *
     * This makes LM more robust than pure Gauss-Newton, especially for
     * poor initializations or ill-conditioned problems.
     */
    class LevenbergMarquardtOptimizer {
      public:
        /**
         * @brief Configuration parameters for Levenberg-Marquardt
         */
        struct Parameters {
            int max_iterations = 100;     ///< Maximum number of iterations
            double tolerance = 1e-6;      ///< Convergence tolerance on error decrease
            bool verbose = false;         ///< Print iteration info
            double min_step_norm = 1e-9;  ///< Minimum step norm for convergence
            double initial_lambda = 1e-3; ///< Initial damping parameter
            double lambda_factor = 10.0;  ///< Factor for increasing/decreasing lambda
            double min_lambda = 1e-7;     ///< Minimum lambda value
            double max_lambda = 1e7;      ///< Maximum lambda value

            Parameters() = default;
        };

        /**
         * @brief Result of optimization
         */
        struct Result {
            Values values;      ///< Optimized values
            double final_error; ///< Final total error
            int iterations;     ///< Number of iterations performed
            bool converged;     ///< Whether convergence was achieved

            Result(const Values &v, double err, int iter, bool conv)
                : values(v), final_error(err), iterations(iter), converged(conv) {}
        };

        /**
         * @brief Construct optimizer with default parameters
         */
        LevenbergMarquardtOptimizer() = default;

        /**
         * @brief Construct optimizer with custom parameters
         */
        inline explicit LevenbergMarquardtOptimizer(const Parameters &params) : params_(params) {}

        /**
         * @brief Optimize a factor graph starting from initial values
         *
         * @param graph Factor graph to optimize
         * @param initial Initial variable values
         * @return Optimization result with final values and diagnostics
         */
        inline Result optimize(const Graph<NonlinearFactor> &graph, const Values &initial) const {
            Values current = initial;
            double current_error = compute_error(graph, current);
            double lambda = params_.initial_lambda;

            if (params_.verbose) {
                std::cout << "Levenberg-Marquardt Optimization" << std::endl;
                std::cout << "Initial error: " << current_error << std::endl;
                std::cout << "Initial lambda: " << lambda << std::endl;
            }

            for (int iter = 0; iter < params_.max_iterations; ++iter) {
                // Build linear system (J^T*J + λ*I) * dx = -J^T*b
                DynMat JtJ_lambda(1, 1); // Temporary initialization, will be resized in build_linear_system
                DynVec Jtb;
                Vector<Key> variable_ordering;

                build_linear_system(graph, current, lambda, JtJ_lambda, Jtb, variable_ordering);

                // Solve for dx using try_solve (returns Result with error handling)
                auto solve_result = try_solve(JtJ_lambda, Jtb);
                if (!solve_result.is_ok()) {
                    // Solve failed - increase damping and retry
                    if (params_.verbose) {
                        std::cout << "Solve failed at iteration " << iter << std::endl;
                        std::cout << "Increasing lambda from " << lambda << " to " << lambda * params_.lambda_factor
                                  << std::endl;
                    }
                    lambda = std::min(lambda * params_.lambda_factor, params_.max_lambda);

                    // If we've hit max lambda, give up
                    if (lambda >= params_.max_lambda) {
                        if (params_.verbose) {
                            std::cout << "Lambda reached maximum, terminating" << std::endl;
                        }
                        return Result(current, current_error, iter, false);
                    }
                    continue; // Retry with larger lambda
                }

                DynVec dx = solve_result.value();

                // Check step norm for convergence
                double step_norm = norm(dx);

                if (params_.verbose) {
                    std::cout << "Iter " << iter << ": error = " << current_error << ", step_norm = " << step_norm
                              << ", lambda = " << lambda << std::endl;
                }

                if (step_norm < params_.min_step_norm) {
                    if (params_.verbose) {
                        std::cout << "Converged: step_norm < " << params_.min_step_norm << std::endl;
                    }
                    return Result(current, current_error, iter + 1, true);
                }

                // Update values
                Values new_values = current;
                for (size_t i = 0; i < variable_ordering.size(); ++i) {
                    Key key = variable_ordering[i];

                    // Get variable dimension and offset
                    size_t dim = 0;
                    size_t offset = 0;
                    for (size_t j = 0; j < i; ++j) {
                        Key prev_key = variable_ordering[j];
                        // Get dimension from any factor containing this key
                        for (size_t fi = 0; fi < graph.size(); ++fi) {
                            if (graph[fi]->involves(prev_key)) {
                                offset += graph[fi]->dim(prev_key);
                                break;
                            }
                        }
                    }

                    // Get dimension for current key
                    for (size_t fi = 0; fi < graph.size(); ++fi) {
                        if (graph[fi]->involves(key)) {
                            dim = graph[fi]->dim(key);
                            break;
                        }
                    }

                    // Apply update based on dimension
                    if (dim == 1) {
                        // Scalar variable
                        double x = current.at<double>(key);
                        new_values.erase(key);
                        new_values.insert(key, x + dx[offset]);
                    } else if (dim == 3) {
                        // Vec3d variable
                        Vec3d x = current.at<Vec3d>(key);
                        x[0] += dx[offset];
                        x[1] += dx[offset + 1];
                        x[2] += dx[offset + 2];
                        new_values.erase(key);
                        new_values.insert(key, x);
                    } else {
                        throw std::runtime_error("Unsupported variable dimension in Levenberg-Marquardt");
                    }
                }

                // Compute new error
                double new_error = compute_error(graph, new_values);

                // Adaptive lambda adjustment based on error improvement
                if (new_error < current_error) {
                    // Good step - accept and decrease lambda (approach Gauss-Newton)
                    double error_decrease = current_error - new_error;

                    if (params_.verbose) {
                        std::cout << "  Step accepted, error decreased by " << error_decrease << std::endl;
                        std::cout << "  Decreasing lambda from " << lambda << " to "
                                  << std::max(lambda / params_.lambda_factor, params_.min_lambda) << std::endl;
                    }

                    current = new_values;
                    current_error = new_error;
                    lambda = std::max(lambda / params_.lambda_factor, params_.min_lambda);

                    // Check for convergence based on error decrease
                    if (std::abs(error_decrease) < params_.tolerance) {
                        if (params_.verbose) {
                            std::cout << "Converged: error_decrease < " << params_.tolerance << std::endl;
                        }
                        return Result(current, current_error, iter + 1, true);
                    }
                } else {
                    // Bad step - reject and increase lambda (approach gradient descent)
                    if (params_.verbose) {
                        std::cout << "  Step rejected, error increased from " << current_error << " to " << new_error
                                  << std::endl;
                        std::cout << "  Increasing lambda from " << lambda << " to "
                                  << std::min(lambda * params_.lambda_factor, params_.max_lambda) << std::endl;
                    }

                    lambda = std::min(lambda * params_.lambda_factor, params_.max_lambda);

                    // If we've hit max lambda, give up
                    if (lambda >= params_.max_lambda) {
                        if (params_.verbose) {
                            std::cout << "Lambda reached maximum, terminating" << std::endl;
                        }
                        return Result(current, current_error, iter, false);
                    }
                    // Don't update current/current_error - retry with larger lambda
                }
            }

            if (params_.verbose) {
                std::cout << "Max iterations reached" << std::endl;
            }

            return Result(current, current_error, params_.max_iterations, false);
        }

        /**
         * @brief Get current parameters
         */
        const Parameters &parameters() const { return params_; }

        /**
         * @brief Set parameters
         */
        void set_parameters(const Parameters &params) { params_ = params; }

      private:
        Parameters params_;

        /**
         * @brief Compute total error of graph given values
         */
        inline double compute_error(const Graph<NonlinearFactor> &graph, const Values &values) const {
            double total_error = 0.0;
            for (size_t i = 0; i < graph.size(); ++i) {
                total_error += graph[i]->error(values);
            }
            return total_error;
        }

        /**
         * @brief Build the augmented normal equations (J^T * J + λ*I) and J^T * b
         *
         * @param graph Factor graph
         * @param values Current variable values
         * @param lambda Damping parameter
         * @param JtJ_lambda Output: J^T * J + λ*I matrix
         * @param Jtb Output: -J^T * b vector
         * @param variable_ordering Output: mapping from variable index to Key
         */
        inline void build_linear_system(const Graph<NonlinearFactor> &graph, const Values &values, double lambda,
                                        DynMat &JtJ_lambda, DynVec &Jtb, Vector<Key> &variable_ordering) const {
            // Step 1: Collect all unique variables and assign indices
            std::map<Key, size_t> key_to_index;
            std::map<Key, size_t> key_to_dim;
            size_t total_dim = 0;

            for (size_t i = 0; i < graph.size(); ++i) {
                for (Key key : graph[i]->keys()) {
                    if (key_to_index.find(key) == key_to_index.end()) {
                        size_t dim = graph[i]->dim(key);
                        key_to_index[key] = total_dim;
                        key_to_dim[key] = dim;
                        total_dim += dim;
                        variable_ordering.push_back(key);
                    }
                }
            }

            // Step 2: Initialize J^T*J and J^T*b
            JtJ_lambda = DynMat(total_dim, total_dim);
            Jtb = DynVec(total_dim, 0.0);

            // Step 3: For each factor, linearize and accumulate J^T*J and J^T*b
            for (size_t fi = 0; fi < graph.size(); ++fi) {
                auto factor = graph[fi];
                // Linearize factor
                auto gaussian = factor->linearize(values);

                // Get Jacobian blocks and error vector
                const auto &factor_keys = gaussian->keys();
                const auto &jacobians = gaussian->jacobians();
                const auto &b = gaussian->b();

                // For each pair of variables in this factor
                for (size_t i = 0; i < factor_keys.size(); ++i) {
                    Key key_i = factor_keys[i];
                    size_t idx_i = key_to_index[key_i];
                    const auto &Ji = jacobians[i];

                    // Add J_i^T * b to Jtb
                    for (size_t row = 0; row < Ji.rows(); ++row) {
                        for (size_t col = 0; col < Ji.cols(); ++col) {
                            Jtb[idx_i + col] -= Ji(row, col) * b[row];
                        }
                    }

                    // Add J_i^T * J_j to JtJ for all j
                    for (size_t j = 0; j < factor_keys.size(); ++j) {
                        Key key_j = factor_keys[j];
                        size_t idx_j = key_to_index[key_j];
                        const auto &Jj = jacobians[j];

                        // Compute J_i^T * J_j
                        for (size_t row_i = 0; row_i < Ji.cols(); ++row_i) {
                            for (size_t row_j = 0; row_j < Jj.cols(); ++row_j) {
                                double sum = 0.0;
                                for (size_t k = 0; k < Ji.rows(); ++k) {
                                    sum += Ji(k, row_i) * Jj(k, row_j);
                                }
                                JtJ_lambda(idx_i + row_i, idx_j + row_j) += sum;
                            }
                        }
                    }
                }
            }

            // Step 4: Add damping to diagonal (λ*I)
            // This is the key difference from Gauss-Newton!
            add_diagonal(JtJ_lambda, lambda);
        }
    };

} // namespace graphix::factor
