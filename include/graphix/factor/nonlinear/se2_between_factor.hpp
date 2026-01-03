#pragma once

/**
 * @file se2_between_factor.hpp
 * @brief Between factor for SE2 poses using optinum Lie groups
 *
 * This factor constrains the relative transformation between two SE2 poses
 * using proper manifold operations (exp/log maps).
 */

#include "graphix/factor/nonlinear/nonlinear_factor.hpp"
#include "graphix/factor/types.hpp"

#include <cmath>
#include <stdexcept>

namespace graphix::factor {

    /**
     * @brief Between factor for SE2 poses
     *
     * Constrains the relative transformation between two SE2 poses.
     * Given poses T_i and T_j, the measured relative transformation is:
     *
     *   T_ij = T_i^{-1} * T_j
     *
     * The error is computed in the tangent space:
     *
     *   error = log(measured^{-1} * predicted)
     *         = log(measured^{-1} * T_i^{-1} * T_j)
     *
     * This gives a proper manifold error that respects the SE2 geometry.
     *
     * Example uses:
     * - Odometry constraints between consecutive poses
     * - Loop closure constraints
     * - Relative pose measurements from sensors
     */
    class SE2BetweenFactor : public NonlinearFactor {
      public:
        using Tangent = SE2d::Tangent; // dp::mat::Vector<double, 3>

        /**
         * @brief Construct SE2 between factor
         *
         * @param key_i First pose key (from)
         * @param key_j Second pose key (to)
         * @param measured Measured relative transformation T_i^{-1} * T_j
         * @param sigmas Standard deviations [sigma_x, sigma_y, sigma_theta]
         */
        SE2BetweenFactor(Key key_i, Key key_j, const SE2d &measured, const Vec3d &sigmas)
            : NonlinearFactor({key_i, key_j}), measured_(measured), sigmas_(sigmas) {

            if (sigmas_[0] <= 0.0 || sigmas_[1] <= 0.0 || sigmas_[2] <= 0.0) {
                throw std::invalid_argument("All sigmas must be positive");
            }
        }

        /**
         * @brief Compute error for this factor
         *
         * @param values Current variable values
         * @return Weighted squared error (scalar)
         */
        double error(const Values &values) const override {
            SE2d pose_i = values.at<SE2d>(keys()[0]);
            SE2d pose_j = values.at<SE2d>(keys()[1]);

            // Predicted relative transformation
            SE2d predicted = pose_i.inverse() * pose_j;

            // Error in tangent space: log(measured^{-1} * predicted)
            Tangent delta = (measured_.inverse() * predicted).log();

            // Weighted squared error
            double squared_error = 0.0;
            for (int i = 0; i < 3; ++i) {
                double weighted = delta[i] / sigmas_[i];
                squared_error += weighted * weighted;
            }

            // Apply robust loss function if present
            if (loss_function_) {
                return loss_function_->evaluate(squared_error);
            }

            return 0.5 * squared_error;
        }

        /**
         * @brief Get dimension of variable (always 3 for SE2 tangent space)
         */
        std::size_t dim(Key) const override { return 3; }

        /**
         * @brief Compute error vector for linearization
         *
         * Returns the weighted residual in tangent space.
         */
        dp::mat::VectorXd error_vector(const Values &values) const override {
            SE2d pose_i = values.at<SE2d>(keys()[0]);
            SE2d pose_j = values.at<SE2d>(keys()[1]);

            // Predicted relative transformation
            SE2d predicted = pose_i.inverse() * pose_j;

            // Error in tangent space
            Tangent delta = (measured_.inverse() * predicted).log();

            // Return weighted residual
            dp::mat::VectorXd result(3);
            result[0] = delta[0] / sigmas_[0];
            result[1] = delta[1] / sigmas_[1];
            result[2] = delta[2] / sigmas_[2];
            return result;
        }

        /**
         * @brief Get measured relative transformation
         */
        const SE2d &measured() const { return measured_; }

        /**
         * @brief Get sigmas
         */
        const Vec3d &sigmas() const { return sigmas_; }

      private:
        SE2d measured_; ///< Measured relative transformation
        Vec3d sigmas_;  ///< Standard deviations [sigma_x, sigma_y, sigma_theta]
    };

} // namespace graphix::factor
