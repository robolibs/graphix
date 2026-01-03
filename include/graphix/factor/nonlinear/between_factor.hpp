#pragma once

#include "graphix/factor/nonlinear/nonlinear_factor.hpp"
#include <cmath>

namespace graphix {
    namespace factor {

        // BetweenFactor: Binary constraint between two variables
        // Constrains relative difference between variables with uncertainty (sigma)
        // Error = 0.5 * (((x2 - x1) - measured) / sigma)^2
        class BetweenFactor : public NonlinearFactor {
          public:
            BetweenFactor(Key key1, Key key2, double measured, double sigma)
                : NonlinearFactor({key1, key2}), m_measured(measured), m_sigma(sigma) {
                if (sigma <= 0.0) {
                    throw std::invalid_argument("Sigma must be positive");
                }
            }

            double error(const Values &values) const override {
                double x1 = values.at<double>(keys()[0]);
                double x2 = values.at<double>(keys()[1]);
                double residual = ((x2 - x1) - m_measured) / m_sigma;
                return 0.5 * residual * residual;
            }

            double measured() const { return m_measured; }
            double sigma() const { return m_sigma; }

          private:
            double m_measured;
            double m_sigma;
        };

    } // namespace factor
} // namespace graphix
