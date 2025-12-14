#include "graphix/factor/nonlinear/nonlinear_factor.hpp"
#include "graphix/factor/types/vec3d.hpp"
#include <cmath>

namespace graphix::factor {

    std::shared_ptr<GaussianFactor> NonlinearFactor::linearize(const Values &values) const {
        // Compute Jacobians using finite differences

        const double epsilon = 1e-7; // Perturbation size

        // Get error at linearization point
        std::vector<double> b = error_vector(values);
        size_t error_dim = b.size();

        // Build Jacobian for each variable
        std::vector<Matrix> jacobians;
        std::vector<Key> factor_keys;

        for (Key key : m_keys) {
            factor_keys.push_back(key);

            size_t var_dim = dim(key);
            Matrix J(error_dim, var_dim);

            // For each dimension of the variable
            for (size_t j = 0; j < var_dim; ++j) {
                // Create perturbed values
                Values perturbed = values;

                // Perturb this dimension
                // We need to handle different types (double, Vec3d, etc.)
                if (var_dim == 1) {
                    // Scalar variable
                    double x = values.at<double>(key);
                    perturbed.erase(key);
                    perturbed.insert(key, x + epsilon);
                } else if (var_dim == 3) {
                    // Vec3d variable
                    Vec3d x = values.at<Vec3d>(key);
                    x[j] += epsilon;
                    perturbed.erase(key);
                    perturbed.insert(key, x);
                } else {
                    throw std::runtime_error("Unsupported variable dimension in linearize()");
                }

                // Compute error with perturbed variable
                std::vector<double> e_perturbed = error_vector(perturbed);

                // Numerical derivative: (f(x+h) - f(x)) / h
                for (size_t i = 0; i < error_dim; ++i) {
                    J(i, j) = (e_perturbed[i] - b[i]) / epsilon;
                }
            }

            jacobians.push_back(J);
        }

        return std::make_shared<GaussianFactor>(factor_keys, jacobians, b);
    }

} // namespace graphix::factor
