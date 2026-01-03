#pragma once

#include <cmath>
#include <memory>

namespace graphix::factor {

    /**
     * @brief Base class for robust loss functions
     *
     * Robust loss functions modify the squared error to reduce the influence of outliers.
     * Given a squared error e² from a factor, the robust loss ρ(e²) is used instead.
     *
     * Standard least squares:  cost = Σ e²
     * Robust least squares:    cost = Σ ρ(e²)
     *
     * The weight function w(e²) = ρ'(e²) is used during optimization.
     */
    class LossFunction {
      public:
        virtual ~LossFunction() = default;

        /**
         * @brief Evaluate the loss function
         *
         * @param squared_error The squared error e²
         * @return The robust loss ρ(e²)
         */
        virtual double evaluate(double squared_error) const = 0;

        /**
         * @brief Evaluate the weight function (derivative of loss)
         *
         * Used during optimization: weighted_error = weight * error
         * For standard least squares, weight = 1.0
         *
         * @param squared_error The squared error e²
         * @return The weight w(e²) = ρ'(e²) / e²
         */
        virtual double weight(double squared_error) const = 0;

        /**
         * @brief Create a copy of this loss function
         */
        virtual std::shared_ptr<LossFunction> clone() const = 0;
    };

    /**
     * @brief No robustification - standard squared error
     *
     * ρ(e²) = e²
     * w(e²) = 1
     */
    class NullLoss : public LossFunction {
      public:
        double evaluate(double squared_error) const override { return squared_error; }

        double weight(double squared_error) const override {
            (void)squared_error;
            return 1.0;
        }

        std::shared_ptr<LossFunction> clone() const override { return std::make_shared<NullLoss>(); }
    };

    /**
     * @brief Huber loss function
     *
     * Quadratic for small errors, linear for large errors:
     *   ρ(e²) = e²/2           if e ≤ k
     *   ρ(e²) = k|e| - k²/2    if e > k
     *
     * Good general-purpose robust loss. Parameter k is the threshold
     * where the function transitions from quadratic to linear.
     *
     * Typical k values: 1.345 (for ~95% efficiency with Gaussian noise)
     */
    class HuberLoss : public LossFunction {
      public:
        /**
         * @brief Construct Huber loss with given threshold
         * @param k Threshold parameter (default 1.345)
         */
        explicit HuberLoss(double k = 1.345) : k_(k), k_squared_(k * k) {}

        double evaluate(double squared_error) const override {
            double error = std::sqrt(squared_error);
            if (error <= k_) {
                return 0.5 * squared_error;
            } else {
                return k_ * error - 0.5 * k_squared_;
            }
        }

        double weight(double squared_error) const override {
            double error = std::sqrt(squared_error);
            if (error <= k_) {
                return 1.0;
            } else {
                // w = ρ'(e²) / e² = (k/e) / (2e) = k / (2e²) for e > k
                // But we want w such that w * e² = ρ(e²)
                // Actually: w * e = k for e > k (linear regime)
                // So: w = k / e
                return k_ / error;
            }
        }

        std::shared_ptr<LossFunction> clone() const override { return std::make_shared<HuberLoss>(k_); }

        double threshold() const { return k_; }

      private:
        double k_;
        double k_squared_;
    };

    /**
     * @brief Cauchy loss function (Lorentzian)
     *
     * ρ(e²) = (k²/2) * log(1 + e²/k²)
     *
     * Very aggressive outlier rejection. Approaches logarithmic growth
     * for large errors, effectively bounding the influence of outliers.
     *
     * Good for data with many outliers.
     * Typical k values: 2.3849 (for similar performance to Huber)
     */
    class CauchyLoss : public LossFunction {
      public:
        /**
         * @brief Construct Cauchy loss with given scale parameter
         * @param k Scale parameter (default 2.3849)
         */
        explicit CauchyLoss(double k = 2.3849) : k_(k), k_squared_(k * k) {}

        double evaluate(double squared_error) const override {
            return 0.5 * k_squared_ * std::log1p(squared_error / k_squared_);
        }

        double weight(double squared_error) const override {
            // ρ'(e²) = (k²/2) * (1 / (k² + e²))
            // w = ρ'(e²) / e² = k² / (2 * e² * (k² + e²))
            // But for weighting: w * e² should give reduced influence
            // w = k² / (k² + e²)
            return k_squared_ / (k_squared_ + squared_error);
        }

        std::shared_ptr<LossFunction> clone() const override { return std::make_shared<CauchyLoss>(k_); }

        double scale() const { return k_; }

      private:
        double k_;
        double k_squared_;
    };

    /**
     * @brief Tukey biweight loss function
     *
     * ρ(e²) = (k²/6) * (1 - (1 - e²/k²)³)   if e ≤ k
     * ρ(e²) = k²/6                           if e > k
     *
     * Completely rejects outliers beyond threshold k. Quadratic near zero,
     * then smoothly transitions to complete rejection at e = k.
     *
     * Most aggressive outlier rejection - use when you're certain about
     * the noise distribution and want to ignore all large errors.
     *
     * Typical k values: 4.6851 (for ~95% efficiency)
     */
    class TukeyLoss : public LossFunction {
      public:
        /**
         * @brief Construct Tukey loss with given threshold
         * @param k Threshold parameter (default 4.6851)
         */
        explicit TukeyLoss(double k = 4.6851) : k_(k), k_squared_(k * k) {}

        double evaluate(double squared_error) const override {
            if (squared_error <= k_squared_) {
                double ratio = squared_error / k_squared_;
                double term = 1.0 - ratio;
                return (k_squared_ / 6.0) * (1.0 - term * term * term);
            } else {
                return k_squared_ / 6.0;
            }
        }

        double weight(double squared_error) const override {
            if (squared_error <= k_squared_) {
                double ratio = squared_error / k_squared_;
                double term = 1.0 - ratio;
                // w = (1 - e²/k²)²
                return term * term;
            } else {
                return 0.0; // Complete rejection
            }
        }

        std::shared_ptr<LossFunction> clone() const override { return std::make_shared<TukeyLoss>(k_); }

        double threshold() const { return k_; }

      private:
        double k_;
        double k_squared_;
    };

    /**
     * @brief Helper to create loss functions
     */
    inline std::shared_ptr<LossFunction> no_loss() { return std::make_shared<NullLoss>(); }

    inline std::shared_ptr<LossFunction> huber_loss(double k = 1.345) { return std::make_shared<HuberLoss>(k); }

    inline std::shared_ptr<LossFunction> cauchy_loss(double k = 2.3849) { return std::make_shared<CauchyLoss>(k); }

    inline std::shared_ptr<LossFunction> tukey_loss(double k = 4.6851) { return std::make_shared<TukeyLoss>(k); }

} // namespace graphix::factor
