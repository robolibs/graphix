#pragma once

/**
 * @file gauss_newton.hpp
 * @brief Gauss-Newton optimizer - thin wrapper around OptinumGaussNewton
 *
 * This file provides backward-compatible API for the GaussNewtonOptimizer
 * while delegating to the optinum-based implementation.
 */

#include "graphix/factor/nonlinear/optimizer_adapter.hpp"

namespace graphix::factor {

    /**
     * @brief Gauss-Newton optimizer for nonlinear least squares
     *
     * Iteratively linearizes the factor graph and solves the linear system:
     *   J^T * J * dx = -J^T * b
     *
     * This is a thin wrapper around OptinumGaussNewton that provides
     * backward-compatible API with the original graphix implementation.
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
            // Create optinum-based optimizer with our parameters
            OptinumGaussNewton optinum_gn;
            optinum_gn.max_iterations = static_cast<std::size_t>(params_.max_iterations);
            optinum_gn.tolerance = params_.tolerance;
            optinum_gn.min_step_norm = params_.min_step_norm;
            optinum_gn.verbose = params_.verbose;

            // Delegate to optinum implementation
            auto optinum_result = optinum_gn.optimize(graph, initial);

            // Convert result to our format
            return Result(optinum_result.values, optinum_result.final_error, optinum_result.iterations,
                          optinum_result.converged);
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
    };

} // namespace graphix::factor
