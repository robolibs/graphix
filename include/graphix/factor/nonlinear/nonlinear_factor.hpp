#pragma once

#include "graphix/factor/factor.hpp"
#include "graphix/factor/linear/gaussian_factor.hpp"
#include "graphix/factor/loss_function.hpp"
#include "graphix/factor/values.hpp"
#include <memory>

namespace graphix {
    namespace factor {

        // NonlinearFactor: Factor with error computation
        // Extends Factor (structure) with math (error function)
        class NonlinearFactor : public Factor {
          public:
            using Factor::Factor; // Inherit constructors

            virtual ~NonlinearFactor() = default;

            // Compute error given variable values
            // Returns scalar error (0.5 * weighted squared residual)
            // If a robust loss function is set, applies it to the squared error
            // Used for optimization objective: minimize sum of errors
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
            virtual size_t dim(Key key) const { return 1; }

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
            std::shared_ptr<GaussianFactor> linearize(const Values &values) const;

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
            virtual std::vector<double> error_vector(const Values &values) const {
                return {std::sqrt(2.0 * error(values))};
            }

            // Optional robust loss function
            std::shared_ptr<LossFunction> loss_function_;
        };

    } // namespace factor
} // namespace graphix
