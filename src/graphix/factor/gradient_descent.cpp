#include "graphix/factor/nonlinear/gradient_descent.hpp"
#include <cmath>
#include <iostream>
#include <limits>

namespace graphix::factor {

    GradientDescentOptimizer::GradientDescentOptimizer(const Parameters &params) : params_(params) {}

    double GradientDescentOptimizer::compute_error(const Graph<NonlinearFactor> &graph, const Values &values) const {

        double total_error = 0.0;
        for (const auto &factor : graph) {
            total_error += factor->error(values);
        }
        return total_error;
    }

    std::unordered_map<Key, double> GradientDescentOptimizer::compute_gradient(const Graph<NonlinearFactor> &graph,
                                                                               const Values &values) const {

        std::unordered_map<Key, double> gradient;

        // Get all keys from values
        auto keys = values.keys();

        // Compute gradient for each variable using central differences
        for (Key key : keys) {
            // Get current value
            double x = values.at<double>(key);

            // Create perturbed values
            Values values_plus = values;
            Values values_minus = values;

            values_plus.erase(key);
            values_plus.insert(key, x + params_.h);
            values_minus.erase(key);
            values_minus.insert(key, x - params_.h);

            // Compute errors
            double error_plus = compute_error(graph, values_plus);
            double error_minus = compute_error(graph, values_minus);

            // Central difference
            gradient[key] = (error_plus - error_minus) / (2.0 * params_.h);
        }

        return gradient;
    }

    double GradientDescentOptimizer::gradient_norm(const std::unordered_map<Key, double> &gradient) const {

        double norm_squared = 0.0;
        for (const auto &[key, grad] : gradient) {
            norm_squared += grad * grad;
        }
        return std::sqrt(norm_squared);
    }

    Values GradientDescentOptimizer::update_values(const Values &current,
                                                   const std::unordered_map<Key, double> &gradient,
                                                   double step_size) const {

        Values updated = current;

        for (const auto &[key, grad] : gradient) {
            double x = current.at<double>(key);
            updated.erase(key);
            updated.insert(key, x - step_size * grad);
        }

        return updated;
    }

    GradientDescentOptimizer::Result GradientDescentOptimizer::optimize(const Graph<NonlinearFactor> &graph,
                                                                        const Values &initial) const {

        Values current_values = initial;
        double current_error = compute_error(graph, current_values);

        if (params_.verbose) {
            std::cout << "Initial error: " << current_error << std::endl;
        }

        int iteration = 0;
        bool converged = false;
        double gnorm = 0.0;

        for (iteration = 0; iteration < params_.max_iterations; ++iteration) {
            // Compute gradient
            auto grad = compute_gradient(graph, current_values);
            gnorm = gradient_norm(grad);

            if (params_.verbose) {
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

            // Update values
            Values new_values = update_values(current_values, grad, params_.step_size);
            double new_error = compute_error(graph, new_values);

            // Simple line search: if error increases, reduce step size
            double step = params_.step_size;
            int backtrack_count = 0;
            const int max_backtrack = 10;

            while (new_error > current_error && backtrack_count < max_backtrack) {
                step *= 0.5;
                new_values = update_values(current_values, grad, step);
                new_error = compute_error(graph, new_values);
                backtrack_count++;
            }

            if (params_.verbose && backtrack_count > 0) {
                std::cout << "  Backtracked " << backtrack_count << " times, step size = " << step << std::endl;
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
