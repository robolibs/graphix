#pragma once

#include "graphix/factor/graph.hpp"
#include "graphix/factor/linear/gaussian_factor.hpp"
#include "graphix/factor/nonlinear/nonlinear_factor.hpp"
#include "graphix/factor/values.hpp"
#include <cmath>
#include <datapod/matrix.hpp>
#include <iostream>
#include <map>
#include <optinum/lina/solve/solve_dynamic.hpp>

namespace graphix::factor {

    /**
     * @brief Gauss-Newton optimizer for nonlinear least squares
     *
     * Iteratively linearizes the factor graph and solves the linear system:
     *   J^T * J * dx = -J^T * b
     *
     * where J is the Jacobian matrix and b is the error vector.
     *
     * Gauss-Newton typically converges much faster than gradient descent
     * (5-10 iterations vs 100+), but requires solving a linear system each iteration.
     */
    class GaussNewtonOptimizer {
      public:
        /**
         * @brief Configuration parameters for Gauss-Newton
         */
        struct Parameters {
            int max_iterations = 100;    ///< Maximum number of iterations
            double tolerance = 1e-6;     ///< Convergence tolerance on error decrease
            bool verbose = false;        ///< Print iteration info
            double min_step_norm = 1e-9; ///< Minimum step norm for convergence

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
        GaussNewtonOptimizer() = default;

        /**
         * @brief Construct optimizer with custom parameters
         */
        inline explicit GaussNewtonOptimizer(const Parameters &params) : params_(params) {}

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

            if (params_.verbose) {
                std::cout << "Gauss-Newton Optimization" << std::endl;
                std::cout << "Initial error: " << current_error << std::endl;
            }

            for (int iter = 0; iter < params_.max_iterations; ++iter) {
                // Build linear system J^T*J * dx = -J^T*b
                datapod::mat::MatrixXd JtJ(1, 1); // Temporary initialization, will be resized
                datapod::mat::VectorXd Jtb;
                std::vector<Key> variable_ordering;

                build_linear_system(graph, current, JtJ, Jtb, variable_ordering);

                // Solve for dx using optinum solver
                optinum::simd::Matrix<double, optinum::simd::Dynamic, optinum::simd::Dynamic> JtJ_view(JtJ);
                optinum::simd::Vector<double, optinum::simd::Dynamic> Jtb_view(Jtb);
                auto dx = optinum::lina::solve_dynamic<double>(JtJ_view, Jtb_view);

                // Check step norm for convergence
                double step_norm = 0.0;
                for (size_t i = 0; i < dx.size(); ++i) {
                    step_norm += dx[i] * dx[i];
                }
                step_norm = std::sqrt(step_norm);

                if (params_.verbose) {
                    std::cout << "Iter " << iter << ": error = " << current_error << ", step_norm = " << step_norm
                              << std::endl;
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
                        // vector3d variable
                        datapod::mat::vector3d x = current.at<datapod::mat::vector3d>(key);
                        x[0] += dx[offset];
                        x[1] += dx[offset + 1];
                        x[2] += dx[offset + 2];
                        new_values.erase(key);
                        new_values.insert(key, x);
                    } else {
                        throw std::runtime_error("Unsupported variable dimension in Gauss-Newton");
                    }
                }

                // Compute new error
                double new_error = compute_error(graph, new_values);

                // Check for error decrease
                double error_decrease = current_error - new_error;
                if (std::abs(error_decrease) < params_.tolerance) {
                    if (params_.verbose) {
                        std::cout << "Converged: error_decrease < " << params_.tolerance << std::endl;
                    }
                    current = new_values;
                    current_error = new_error;
                    return Result(current, current_error, iter + 1, true);
                }

                // Accept step
                current = new_values;
                current_error = new_error;
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
         * @brief Build the normal equations J^T * J and J^T * b
         *
         * @param graph Factor graph
         * @param values Current variable values
         * @param JtJ Output: J^T * J matrix
         * @param Jtb Output: -J^T * b vector
         * @param variable_ordering Output: mapping from variable index to Key
         */
        inline void build_linear_system(const Graph<NonlinearFactor> &graph, const Values &values,
                                        datapod::mat::MatrixXd &JtJ, datapod::mat::VectorXd &Jtb,
                                        std::vector<Key> &variable_ordering) const {
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
            JtJ = datapod::mat::MatrixXd(total_dim, total_dim);
            Jtb.resize(total_dim, 0.0);

            // Step 3: For each factor, linearize and accumulate
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
                                JtJ(idx_i + row_i, idx_j + row_j) += sum;
                            }
                        }
                    }
                }
            }
        }
    };

} // namespace graphix::factor
