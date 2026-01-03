#pragma once

#include "graphix/factor/nonlinear/nonlinear_factor.hpp"
#include <cmath>

namespace graphix {
    namespace factor {

        // PriorFactor: Unary constraint on a single variable
        // Constrains variable to a prior value with uncertainty (sigma)
        // Error = 0.5 * ((x - prior) / sigma)^2
        class PriorFactor : public NonlinearFactor {
          public:
            PriorFactor(Key key, double prior, double sigma) : NonlinearFactor({key}), m_prior(prior), m_sigma(sigma) {
                if (sigma <= 0.0) {
                    throw std::invalid_argument("Sigma must be positive");
                }
            }

            double error(const Values &values) const override {
                double x = values.at<double>(keys()[0]);
                double residual = (x - m_prior) / m_sigma;
                return 0.5 * residual * residual;
            }

            double prior() const { return m_prior; }
            double sigma() const { return m_sigma; }

          private:
            double m_prior;
            double m_sigma;
        };

    } // namespace factor
} // namespace graphix
