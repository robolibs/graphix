#include "graphix/factor/nonlinear/gradient_descent.hpp"
#include "graphix/factor/types/vec3d.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <variant>

namespace graphix::factor {

    // Type to store gradient for either double or Vec3d
    using GradientValue = std::variant<double, Vec3d>;

    GradientDescentOptimizer::GradientDescentOptimizer(const Parameters &params) : params_(params) {}

    double GradientDescentOptimizer::compute_error(const Graph<NonlinearFactor> &graph, const Values &values) const {
        double total_error = 0.0;
        for (const auto &factor : graph) {
            total_error += factor->error(values);
        }
        return total_error;
    }

    // Helper class to compute and cache gradients
    class GradientCache {
      public:
        std::unordered_map<Key, GradientValue> gradients;
        double norm = 0.0;

        void add_scalar_gradient(Key key, double grad) {
            gradients[key] = grad;
            norm += grad * grad;
        }

        void add_vec3_gradient(Key key, const Vec3d &grad) {
            gradients[key] = grad;
            norm += grad.norm_squared();
        }

        void finalize() { norm = std::sqrt(norm); }

        const GradientValue &get(Key key) const { return gradients.at(key); }

        bool has(Key key) const { return gradients.find(key) != gradients.end(); }
    };

    // Adam optimizer state
    class AdamState {
      public:
        std::unordered_map<Key, GradientValue> m; // First moment (mean)
        std::unordered_map<Key, GradientValue> v; // Second moment (variance)
        int t = 0;                                // Time step

        void next_step() { ++t; }

        void update(Key key, const GradientValue &grad, double beta1, double beta2) {
            if (std::holds_alternative<double>(grad)) {
                double g = std::get<double>(grad);

                // Initialize if needed
                if (m.find(key) == m.end()) {
                    m[key] = 0.0;
                    v[key] = 0.0;
                }

                double m_val = std::get<double>(m[key]);
                double v_val = std::get<double>(v[key]);

                // Update biased moments
                m_val = beta1 * m_val + (1.0 - beta1) * g;
                v_val = beta2 * v_val + (1.0 - beta2) * g * g;

                m[key] = m_val;
                v[key] = v_val;

            } else if (std::holds_alternative<Vec3d>(grad)) {
                Vec3d g = std::get<Vec3d>(grad);

                // Initialize if needed
                if (m.find(key) == m.end()) {
                    m[key] = Vec3d(0, 0, 0);
                    v[key] = Vec3d(0, 0, 0);
                }

                Vec3d m_val = std::get<Vec3d>(m[key]);
                Vec3d v_val = std::get<Vec3d>(v[key]);

                // Update biased moments (element-wise)
                for (int i = 0; i < 3; i++) {
                    m_val[i] = beta1 * m_val[i] + (1.0 - beta1) * g[i];
                    v_val[i] = beta2 * v_val[i] + (1.0 - beta2) * g[i] * g[i];
                }

                m[key] = m_val;
                v[key] = v_val;
            }
        }

        GradientValue get_corrected_update(Key key, double beta1, double beta2, double epsilon) const {
            if (t <= 0) {
                throw std::logic_error("AdamState::t must be >= 1 before computing corrected update");
            }
            double bias_correction1 = 1.0 - std::pow(beta1, t);
            double bias_correction2 = 1.0 - std::pow(beta2, t);

            if (std::holds_alternative<double>(m.at(key))) {
                double m_val = std::get<double>(m.at(key));
                double v_val = std::get<double>(v.at(key));

                double m_hat = m_val / bias_correction1;
                double v_hat = v_val / bias_correction2;

                return m_hat / (std::sqrt(v_hat) + epsilon);

            } else {
                Vec3d m_val = std::get<Vec3d>(m.at(key));
                Vec3d v_val = std::get<Vec3d>(v.at(key));

                Vec3d update;
                for (int i = 0; i < 3; i++) {
                    double m_hat = m_val[i] / bias_correction1;
                    double v_hat = v_val[i] / bias_correction2;
                    update[i] = m_hat / (std::sqrt(v_hat) + epsilon);
                }

                return update;
            }
        }
    };

    // Compute all gradients and return cached structure
    static GradientCache compute_all_gradients(const Graph<NonlinearFactor> &graph, const Values &values, double h) {
        GradientCache cache;

        auto keys = values.keys();
        for (Key key : keys) {
            // Try double first
            try {
                double x = values.at<double>(key);

                // Central difference for scalar
                Values values_plus = values;
                Values values_minus = values;

                values_plus.erase(key);
                values_plus.insert(key, x + h);
                values_minus.erase(key);
                values_minus.insert(key, x - h);

                double error_plus = 0.0;
                for (const auto &factor : graph) {
                    error_plus += factor->error(values_plus);
                }

                double error_minus = 0.0;
                for (const auto &factor : graph) {
                    error_minus += factor->error(values_minus);
                }

                double grad = (error_plus - error_minus) / (2.0 * h);
                cache.add_scalar_gradient(key, grad);

            } catch (const std::runtime_error &) {
                // Try Vec3d
                try {
                    Vec3d vec = values.at<Vec3d>(key);
                    Vec3d grad(0.0, 0.0, 0.0);

                    // Compute gradient for each dimension
                    for (int dim = 0; dim < 3; ++dim) {
                        Vec3d vec_plus = vec;
                        Vec3d vec_minus = vec;

                        vec_plus[dim] += h;
                        vec_minus[dim] -= h;

                        Values values_plus = values;
                        Values values_minus = values;

                        values_plus.erase(key);
                        values_plus.insert(key, vec_plus);
                        values_minus.erase(key);
                        values_minus.insert(key, vec_minus);

                        double error_plus = 0.0;
                        for (const auto &factor : graph) {
                            error_plus += factor->error(values_plus);
                        }

                        double error_minus = 0.0;
                        for (const auto &factor : graph) {
                            error_minus += factor->error(values_minus);
                        }

                        grad[dim] = (error_plus - error_minus) / (2.0 * h);
                    }

                    cache.add_vec3_gradient(key, grad);

                } catch (const std::runtime_error &) {
                    throw std::runtime_error("Unsupported type in gradient computation for key " + std::to_string(key));
                }
            }
        }

        cache.finalize();
        return cache;
    }

    // Apply Adam update with given step size
    static Values apply_adam_update(const Values &current, const AdamState &adam, const GradientCache &grad_cache,
                                    double step_size, double beta1, double beta2, double epsilon) {
        Values updated = current;

        for (const auto &[key, grad_val] : grad_cache.gradients) {
            GradientValue update = adam.get_corrected_update(key, beta1, beta2, epsilon);

            if (std::holds_alternative<double>(update)) {
                double x = current.at<double>(key);
                double u = std::get<double>(update);
                updated.erase(key);
                updated.insert(key, x - step_size * u);

            } else if (std::holds_alternative<Vec3d>(update)) {
                Vec3d vec = current.at<Vec3d>(key);
                Vec3d u = std::get<Vec3d>(update);
                updated.erase(key);
                updated.insert(key, vec - u * step_size);
            }
        }

        return updated;
    }

    // Apply standard gradient update
    static Values apply_gradient_update(const Values &current, const GradientCache &grad_cache, double step_size) {
        Values updated = current;

        for (const auto &[key, grad_val] : grad_cache.gradients) {
            if (std::holds_alternative<double>(grad_val)) {
                double x = current.at<double>(key);
                double grad = std::get<double>(grad_val);
                updated.erase(key);
                updated.insert(key, x - step_size * grad);

            } else if (std::holds_alternative<Vec3d>(grad_val)) {
                Vec3d vec = current.at<Vec3d>(key);
                Vec3d grad = std::get<Vec3d>(grad_val);
                updated.erase(key);
                updated.insert(key, vec - grad * step_size);
            }
        }

        return updated;
    }

    GradientDescentOptimizer::Result GradientDescentOptimizer::optimize(const Graph<NonlinearFactor> &graph,
                                                                        const Values &initial) const {

        Values current_values = initial;
        double current_error = compute_error(graph, current_values);

        if (params_.verbose) {
            std::cout << "Initial error: " << current_error << std::endl;
            if (params_.use_adaptive_lr) {
                std::cout << "Using Adam optimizer (adaptive learning rate)" << std::endl;
            }
        }

        AdamState adam;
        int iteration = 0;
        bool converged = false;
        double gnorm = 0.0;

        for (iteration = 0; iteration < params_.max_iterations; ++iteration) {
            // Compute gradients once
            auto grad_cache = compute_all_gradients(graph, current_values, params_.h);
            gnorm = grad_cache.norm;

            if (params_.verbose && (iteration % 100 == 0 || iteration < 10)) {
                std::cout << "Iteration " << iteration << ": error = " << current_error << ", gradient norm = " << gnorm
                          << std::endl;
            }

            // Check convergence
            if (gnorm < params_.tolerance) {
                converged = true;
                if (params_.verbose) {
                    std::cout << "Converged! Gradient norm " << gnorm << " < tolerance " << params_.tolerance
                              << std::endl;
                }
                break;
            }

            // Update Adam state if using adaptive learning rate
            Values new_values;
            if (params_.use_adaptive_lr) {
                // Advance the Adam time step once per iteration (not per parameter).
                adam.next_step();

                // Update Adam moments
                for (const auto &[key, grad_val] : grad_cache.gradients) {
                    adam.update(key, grad_val, params_.adam_beta1, params_.adam_beta2);
                }

                // Apply Adam update
                new_values = apply_adam_update(current_values, adam, grad_cache, params_.step_size, params_.adam_beta1,
                                               params_.adam_beta2, params_.adam_epsilon);
            } else {
                // Standard gradient descent
                new_values = apply_gradient_update(current_values, grad_cache, params_.step_size);
            }

            double new_error = compute_error(graph, new_values);

            // Backtracking line search if error increased
            double step = params_.step_size;
            int backtrack_count = 0;
            const int max_backtrack = 20;
            const double backtrack_factor = 0.5;

            while (new_error > current_error && backtrack_count < max_backtrack) {
                step *= backtrack_factor;

                if (params_.use_adaptive_lr) {
                    new_values = apply_adam_update(current_values, adam, grad_cache, step, params_.adam_beta1,
                                                   params_.adam_beta2, params_.adam_epsilon);
                } else {
                    new_values = apply_gradient_update(current_values, grad_cache, step);
                }

                new_error = compute_error(graph, new_values);
                backtrack_count++;
            }

            if (params_.verbose && backtrack_count > 0 && (iteration % 100 == 0 || iteration < 10)) {
                std::cout << "  Backtracked " << backtrack_count << " times, step size = " << step << std::endl;
            }

            // If we couldn't improve even with tiny steps, we're stuck
            if (new_error >= current_error && backtrack_count >= max_backtrack) {
                if (params_.verbose) {
                    std::cout << "Cannot improve further at iteration " << iteration << ", stopping" << std::endl;
                }
                break;
            }

            // Accept the step
            current_values = new_values;
            current_error = new_error;
        }

        if (params_.verbose) {
            std::cout << "Final error: " << current_error << " after " << iteration << " iterations" << std::endl;
            if (!converged) {
                std::cout << "Did not converge within max iterations" << std::endl;
            }
        }

        return Result(current_values, current_error, iteration, converged, gnorm);
    }

} // namespace graphix::factor
