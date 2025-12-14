#pragma once

#include "graphix/factor/graph.hpp"
#include "graphix/factor/nonlinear/nonlinear_factor.hpp"
#include "graphix/factor/values.hpp"

namespace graphix::factor {

    /**
     * @brief Simple gradient descent optimizer using finite differences
     *
     * Optimizes a factor graph by minimizing total error using gradient descent.
     * Gradients are computed using central finite differences.
     */
    class GradientDescentOptimizer {
      public:
        /**
         * @brief Configuration parameters for gradient descent
         */
        struct Parameters {
            int max_iterations = 100;     ///< Maximum number of iterations
            double step_size = 0.01;      ///< Initial step size for gradient descent
            double tolerance = 1e-6;      ///< Convergence tolerance on gradient norm
            double h = 1e-5;              ///< Step size for finite differences
            bool verbose = false;         ///< Print iteration info
            double momentum = 0.9;        ///< Momentum coefficient (0 = no momentum, 0.9 typical)
            bool use_adaptive_lr = false; ///< Use adaptive learning rate (Adam-like)
            double adam_beta1 = 0.9;      ///< First moment decay
            double adam_beta2 = 0.999;    ///< Second moment decay
            double adam_epsilon = 1e-8;   ///< Numerical stability

            Parameters() = default;
        };

        /**
         * @brief Result of optimization
         */
        struct Result {
            Values values;        ///< Optimized values
            double final_error;   ///< Final total error
            int iterations;       ///< Number of iterations performed
            bool converged;       ///< Whether convergence was achieved
            double gradient_norm; ///< Final gradient norm

            Result(const Values &v, double err, int iter, bool conv, double gnorm)
                : values(v), final_error(err), iterations(iter), converged(conv), gradient_norm(gnorm) {}
        };

        /**
         * @brief Construct optimizer with default parameters
         */
        GradientDescentOptimizer() = default;

        /**
         * @brief Construct optimizer with custom parameters
         */
        explicit GradientDescentOptimizer(const Parameters &params);

        /**
         * @brief Optimize a factor graph starting from initial values
         *
         * @param graph Factor graph to optimize
         * @param initial Initial variable values
         * @return Optimization result with final values and diagnostics
         */
        Result optimize(const Graph<NonlinearFactor> &graph, const Values &initial) const;

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
        double compute_error(const Graph<NonlinearFactor> &graph, const Values &values) const;
    };

} // namespace graphix::factor
