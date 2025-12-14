#pragma once

#include "graphix/factor/graph.hpp"
#include "graphix/factor/nonlinear/nonlinear_factor.hpp"
#include "graphix/factor/values.hpp"

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
        explicit GaussNewtonOptimizer(const Parameters &params);

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
         * @brief Build the normal equations J^T * J and J^T * b
         *
         * @param graph Factor graph
         * @param values Current variable values
         * @param JtJ Output: J^T * J matrix
         * @param Jtb Output: -J^T * b vector
         * @param variable_ordering Output: mapping from variable index to Key
         */
        void build_linear_system(const Graph<NonlinearFactor> &graph, const Values &values, Matrix &JtJ,
                                 std::vector<double> &Jtb, std::vector<Key> &variable_ordering) const;
    };

} // namespace graphix::factor
