#include "graphix/factor/linear/gaussian_factor.hpp"
#include <stdexcept>

namespace graphix::factor {

    GaussianFactor::GaussianFactor(const std::vector<Key> &keys, const std::vector<Matrix> &jacobians,
                                   const std::vector<double> &b)
        : Factor(), jacobians_(jacobians), b_(b) {
        // Copy keys to parent
        for (Key key : keys) {
            m_keys.push_back(key);
        }

        if (keys.size() != jacobians.size()) {
            throw std::invalid_argument("Number of keys must match number of Jacobians");
        }

        // Build key index map
        for (size_t i = 0; i < keys.size(); ++i) {
            key_index_[keys[i]] = i;
        }

        // Validate dimensions
        if (!jacobians.empty() && !b.empty()) {
            size_t error_dim = b.size();
            for (size_t i = 0; i < jacobians.size(); ++i) {
                if (jacobians[i].rows() != error_dim) {
                    throw std::invalid_argument("All Jacobians must have same number of rows as error vector");
                }
            }
        }
    }

    const Matrix &GaussianFactor::jacobian(Key key) const {
        auto it = key_index_.find(key);
        if (it == key_index_.end()) {
            throw std::runtime_error("Key not found in Gaussian factor");
        }
        return jacobians_[it->second];
    }

    double GaussianFactor::error(const std::map<Key, std::vector<double>> &deltas) const {
        // Compute: 0.5 * ||A * dx + b||²

        // Start with b
        std::vector<double> residual = b_;

        // Add A * dx for each variable
        for (size_t i = 0; i < m_keys.size(); ++i) {
            Key key = m_keys[i];

            // Get delta for this key (or zero if not in map)
            std::vector<double> dx;
            auto it = deltas.find(key);
            if (it != deltas.end()) {
                dx = it->second;
            } else {
                // Delta not provided, assume zero
                dx.resize(jacobians_[i].cols(), 0.0);
            }

            // Validate dimensions
            if (dx.size() != jacobians_[i].cols()) {
                throw std::invalid_argument("Delta dimension mismatch for key");
            }

            // Add J_i * dx_i to residual
            std::vector<double> J_dx = jacobians_[i] * dx;
            for (size_t j = 0; j < residual.size(); ++j) {
                residual[j] += J_dx[j];
            }
        }

        // Compute 0.5 * ||residual||²
        double sum_squared = 0.0;
        for (double r : residual) {
            sum_squared += r * r;
        }

        return 0.5 * sum_squared;
    }

} // namespace graphix::factor
