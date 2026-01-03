#pragma once

#include "graphix/factor/factor.hpp"
#include "graphix/factor/linear/gaussian_factor.hpp"
#include "graphix/factor/loss_function.hpp"
#include "graphix/factor/types.hpp"
#include "graphix/factor/values.hpp"

#include <datapod/matrix.hpp>
#include <datapod/sequential.hpp>

#include <cmath>
#include <memory>

namespace graphix::factor {

    namespace dp = ::datapod;

    /**
     * @brief NonlinearFactor: Factor with error computation
     *
     * Extends Factor (structure) with math (error function)
     */
    class NonlinearFactor : public Factor {
      public:
        using Factor::Factor; // Inherit constructors

        virtual ~NonlinearFactor() = default;

        /**
         * @brief Compute error given variable values
         *
         * Returns scalar error (0.5 * weighted squared residual)
         * If a robust loss function is set, applies it to the squared error
         * Used for optimization objective: minimize sum of errors
         */
        virtual double error(const Values &values) const = 0;

        /**
         * @brief Set a robust loss function for this factor
         *
         * @param loss Shared pointer to loss function (or nullptr for standard squared error)
         */
        void set_loss_function(std::shared_ptr<LossFunction> loss) { loss_function_ = loss; }

        /**
         * @brief Get the current loss function (may be nullptr)
         */
        std::shared_ptr<LossFunction> loss_function() const { return loss_function_; }

        /**
         * @brief Check if this factor has a robust loss function
         */
        bool has_loss_function() const { return loss_function_ != nullptr; }

        /**
         * @brief Get dimension of the variable with given key
         *
         * Subclasses should override this to specify their variable dimensions.
         * Default returns 1 (scalar).
         *
         * @param key Variable key
         * @return Dimension of the variable
         */
        virtual std::size_t dim(Key key) const { return 1; }

        /**
         * @brief Linearize this factor around the given values
         *
         * Computes a first-order Taylor expansion:
         *   error(x + dx) ≈ error(x) + J * dx
         *
         * Where J is the Jacobian matrix computed using finite differences.
         *
         * @param values Current variable values (linearization point)
         * @return Gaussian factor representing the linearized constraint
         */
        inline std::shared_ptr<GaussianFactor> linearize(const Values &values) const {
            // Compute Jacobians using finite differences
            const double epsilon = 1e-7; // Perturbation size

            // Get error at linearization point
            dp::mat::VectorXd b = error_vector(values);
            std::size_t error_dim = b.size();

            // Build Jacobian for each variable
            dp::Vector<dp::mat::MatrixXd> jacobians;
            dp::Vector<Key> factor_keys;

            for (Key key : m_keys) {
                factor_keys.push_back(key);

                std::size_t var_dim = dim(key);
                dp::mat::MatrixXd J(error_dim, var_dim);

                // For each dimension of the variable
                for (std::size_t j = 0; j < var_dim; ++j) {
                    // Create perturbed values
                    Values perturbed = values;

                    // Perturb this dimension
                    // We need to handle different types (double, SE2d, vector3d, etc.)
                    if (var_dim == 1) {
                        // Scalar variable
                        double x = values.at<double>(key);
                        perturbed.erase(key);
                        perturbed.insert(key, x + epsilon);
                    } else if (var_dim == 3) {
                        // Try SE2d first (manifold perturbation via exp map)
                        bool handled = false;
                        try {
                            SE2d pose = values.at<SE2d>(key);
                            // Create tangent perturbation with epsilon at position j
                            SE2d::Tangent delta;
                            delta[0] = (j == 0) ? epsilon : 0.0;
                            delta[1] = (j == 1) ? epsilon : 0.0;
                            delta[2] = (j == 2) ? epsilon : 0.0;
                            // Left perturbation: exp(delta) * pose
                            SE2d perturbed_pose = SE2d::exp(delta) * pose;
                            perturbed.erase(key);
                            perturbed.insert(key, perturbed_pose);
                            handled = true;
                        } catch (...) {
                            // Not an SE2d, fall through to vector3d
                        }

                        if (!handled) {
                            // Fall back to Vector3d (for backward compatibility)
                            dp::mat::Vector3d x = values.at<dp::mat::Vector3d>(key);
                            x[j] += epsilon;
                            perturbed.erase(key);
                            perturbed.insert(key, x);
                        }
                    } else {
                        throw std::runtime_error("Unsupported variable dimension in linearize()");
                    }

                    // Compute error with perturbed variable
                    dp::mat::VectorXd e_perturbed = error_vector(perturbed);

                    // Numerical derivative: (f(x+h) - f(x)) / h
                    for (std::size_t i = 0; i < error_dim; ++i) {
                        J(i, j) = (e_perturbed[i] - b[i]) / epsilon;
                    }
                }

                jacobians.push_back(J);
            }

            return std::make_shared<GaussianFactor>(factor_keys, jacobians, b);
        }

      protected:
        /**
         * @brief Compute error vector (for multi-dimensional errors)
         *
         * Default implementation returns a 1D vector with the scalar error.
         * Override this for factors with multi-dimensional residuals.
         *
         * @param values Current variable values
         * @return Error vector
         */
        virtual dp::mat::VectorXd error_vector(const Values &values) const {
            dp::mat::VectorXd result(1);
            result[0] = std::sqrt(2.0 * error(values));
            return result;
        }

        // Optional robust loss function
        std::shared_ptr<LossFunction> loss_function_;
    };

} // namespace graphix::factor
