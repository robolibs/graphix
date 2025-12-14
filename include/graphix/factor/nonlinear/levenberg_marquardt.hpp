#pragma once

#include "graphix/factor/graph.hpp"
#include "graphix/factor/nonlinear/nonlinear_factor.hpp"
#include "graphix/factor/values.hpp"

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
        explicit LevenbergMarquardtOptimizer(const Parameters &params);

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
        void build_linear_system(const Graph<NonlinearFactor> &graph, const Values &values, double lambda,
                                 Matrix &JtJ_lambda, std::vector<double> &Jtb,
                                 std::vector<Key> &variable_ordering) const;
    };

} // namespace graphix::factor
