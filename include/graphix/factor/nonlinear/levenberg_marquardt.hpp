#pragma once

/**
 * @file levenberg_marquardt.hpp
 * @brief Levenberg-Marquardt optimizer - thin wrapper around OptinumLevenbergMarquardt
 *
 * This file provides backward-compatible API for the LevenbergMarquardtOptimizer
 * while delegating to the optinum-based implementation.
 */

#include "graphix/factor/nonlinear/optimizer_adapter.hpp"

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
     * This is a thin wrapper around OptinumLevenbergMarquardt that provides
     * backward-compatible API with the original graphix implementation.
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
            // Create optinum-based optimizer with our parameters
            OptinumLevenbergMarquardt optinum_lm;
            optinum_lm.max_iterations = static_cast<std::size_t>(params_.max_iterations);
            optinum_lm.tolerance = params_.tolerance;
            optinum_lm.min_step_norm = params_.min_step_norm;
            optinum_lm.initial_lambda = params_.initial_lambda;
            optinum_lm.lambda_factor = params_.lambda_factor;
            optinum_lm.min_lambda = params_.min_lambda;
            optinum_lm.max_lambda = params_.max_lambda;
            optinum_lm.verbose = params_.verbose;

            // Delegate to optinum implementation
            auto optinum_result = optinum_lm.optimize(graph, initial);

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
