#include "graphix/factor/nonlinear/vec3_prior_factor.hpp"

namespace graphix::factor {

    Vec3PriorFactor::Vec3PriorFactor(Key key, const Vec3d &prior, const Vec3d &sigmas)
        : NonlinearFactor({key}), prior_(prior), sigmas_(sigmas) {

        // Validate sigmas
        if (sigmas.x() <= 0.0 || sigmas.y() <= 0.0 || sigmas.z() <= 0.0) {
            throw std::invalid_argument("All sigmas must be positive");
        }
    }

    double Vec3PriorFactor::error(const Values &values) const {
        // Get the variable value
        Vec3d x = values.at<Vec3d>(keys()[0]);

        // Compute difference
        Vec3d diff = x - prior_;

        // Compute weighted squared error: 0.5 * sum((diff[i] / sigma[i])^2)
        double error = 0.0;
        for (int i = 0; i < 3; i++) {
            double weighted = diff[i] / sigmas_[i];
            error += weighted * weighted;
        }

        return 0.5 * error;
    }

    std::vector<double> Vec3PriorFactor::error_vector(const Values &values) const {
        // Get the variable value
        Vec3d x = values.at<Vec3d>(keys()[0]);

        // Compute weighted residual: (x - prior) / sigma
        Vec3d diff = x - prior_;

        return {diff.x() / sigmas_.x(), diff.y() / sigmas_.y(), diff.z() / sigmas_.z()};
    }

} // namespace graphix::factor
