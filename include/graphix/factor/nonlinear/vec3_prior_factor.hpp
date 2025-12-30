#pragma once

#include "graphix/factor/nonlinear/nonlinear_factor.hpp"
#include <datapod/matrix.hpp>
#include <stdexcept>

namespace graphix::factor {

    /**
     * @brief Prior factor for Vec3d variables
     *
     * Constrains a Vec3d variable to be near a prior value.
     * Error = 0.5 * sum((x[i] - prior[i]) / sigma[i])^2
     *
     * Example uses:
     * - GPS measurement: Vec3d(x, y, theta) with known position
     * - Initial pose constraint
     * - Anchor point in optimization
     */
    class Vec3PriorFactor : public NonlinearFactor {
      public:
        using Vec3d = datapod::mat::vector3d;

        /**
         * @brief Construct prior factor
         *
         * @param key Variable key
         * @param prior Prior value (mean)
         * @param sigmas Standard deviations for each dimension
         */
        inline Vec3PriorFactor(Key key, const Vec3d &prior, const Vec3d &sigmas)
            : NonlinearFactor({key}), prior_(prior), sigmas_(sigmas) {

            // Validate sigmas
            if (sigmas_[0] <= 0.0 || sigmas_[1] <= 0.0 || sigmas_[2] <= 0.0) {
                throw std::invalid_argument("All sigmas must be positive");
            }
        }

        /**
         * @brief Compute error for this factor
         *
         * Error = 0.5 * sum((x[i] - prior[i]) / sigma[i])^2
         *
         * @param values Current variable values
         * @return Squared Mahalanobis distance (weighted squared error)
         */
        inline double error(const Values &values) const override {
            // Get the variable value
            Vec3d x = values.at<Vec3d>(keys()[0]);

            // Compute difference
            Vec3d diff = Vec3d{x[0] - prior_[0], x[1] - prior_[1], x[2] - prior_[2]};

            // Compute weighted squared error: sum((diff[i] / sigma[i])^2)
            double squared_error = 0.0;
            for (int i = 0; i < 3; i++) {
                double weighted = diff[i] / sigmas_[i];
                squared_error += weighted * weighted;
            }

            // Apply robust loss function if present
            if (loss_function_) {
                return loss_function_->evaluate(squared_error);
            }

            return 0.5 * squared_error;
        }

        /**
         * @brief Get dimension of variable (always 3 for Vec3d)
         */
        size_t dim(Key) const override { return 3; }

        /**
         * @brief Compute error vector for linearization
         *
         * Returns the weighted residual: (x - prior) ./ sigma
         * where ./ is element-wise division
         */
        inline datapod::mat::VectorXd error_vector(const Values &values) const override {
            // Get the variable value
            Vec3d x = values.at<Vec3d>(keys()[0]);

            // Compute weighted residual: (x - prior) / sigma
            Vec3d diff = Vec3d{x[0] - prior_[0], x[1] - prior_[1], x[2] - prior_[2]};

            return datapod::mat::VectorXd{diff[0] / sigmas_[0], diff[1] / sigmas_[1], diff[2] / sigmas_[2]};
        }

        /**
         * @brief Get prior value
         */
        const Vec3d &prior() const { return prior_; }

        /**
         * @brief Get sigmas
         */
        const Vec3d &sigmas() const { return sigmas_; }

      private:
        Vec3d prior_;  ///< Prior value (mean)
        Vec3d sigmas_; ///< Standard deviations (diagonal covariance)
    };

} // namespace graphix::factor
