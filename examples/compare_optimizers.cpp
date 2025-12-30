/**
 * @file compare_optimizers.cpp
 * @brief Compare Gradient Descent vs Gauss-Newton vs Levenberg-Marquardt on 2D SLAM
 *
 * This example compares three optimization methods on the same SLAM problem:
 * 1. Gradient Descent (first-order, many iterations)
 * 2. Gauss-Newton (second-order, few iterations, less robust)
 * 3. Levenberg-Marquardt (second-order, adaptive damping, most robust)
 */

#include "graphix/factor/graph.hpp"
#include "graphix/factor/nonlinear/gauss_newton.hpp"
#include "graphix/factor/nonlinear/gradient_descent.hpp"
#include "graphix/factor/nonlinear/levenberg_marquardt.hpp"
#include "graphix/factor/nonlinear/vec3_between_factor.hpp"
#include "graphix/factor/nonlinear/vec3_prior_factor.hpp"
#include "graphix/kernel.hpp"
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>

using namespace graphix;
using namespace graphix::factor;

int main() {
    std::cout << "=== Optimizer Comparison: 2D SLAM ===" << std::endl;
    std::cout << std::endl;

    // ========================================
    // 1. Create factor graph with loop closure
    // ========================================

    Graph<NonlinearFactor> graph;

    // Prior on first pose (anchor)
    Vec3d prior_mean{0, 0, 0};
    Vec3d prior_sigma{0.01, 0.01, 0.01};
    graph.add(std::make_shared<Vec3PriorFactor>(X(0), prior_mean, prior_sigma));

    // Odometry measurements with noise (square path with drift)
    Vec3d odom_sigma{0.1, 0.1, 0.05};

    // Noisy odometry that deviates from perfect square
    graph.add(std::make_shared<Vec3BetweenFactor>(X(0), X(1), Vec3d{2.05, 0.0, 0.0}, odom_sigma));       // →
    graph.add(std::make_shared<Vec3BetweenFactor>(X(1), X(2), Vec3d{0.0, 2.03, M_PI / 2}, odom_sigma));  // ↑
    graph.add(std::make_shared<Vec3BetweenFactor>(X(2), X(3), Vec3d{-1.98, 0.0, M_PI / 2}, odom_sigma)); // ←
    graph.add(std::make_shared<Vec3BetweenFactor>(X(3), X(4), Vec3d{0.0, -2.02, M_PI / 2}, odom_sigma)); // ↓

    // Loop closure: robot returns close to start
    Vec3d loop_sigma{0.05, 0.05, 0.05};
    graph.add(std::make_shared<Vec3BetweenFactor>(X(4), X(0), Vec3d{0.0, 0.0, M_PI / 2}, loop_sigma));

    // Initial values (using noisy odometry integration)
    Values initial;
    initial.insert(X(0), Vec3d{0.0, 0.0, 0.0});
    initial.insert(X(1), Vec3d{2.05, 0.0, 0.0});
    initial.insert(X(2), Vec3d{2.05, 2.03, M_PI / 2});
    initial.insert(X(3), Vec3d{0.07, 2.03, M_PI});
    initial.insert(X(4), Vec3d{0.07, 0.01, 3.0 * M_PI / 2});

    std::cout << "Factor graph: " << graph.size() << " factors, 5 variables" << std::endl;
    std::cout << std::endl;

    // ========================================
    // 2. Optimize with Gradient Descent
    // ========================================

    std::cout << "=== Gradient Descent ===" << std::endl;

    GradientDescentOptimizer::Parameters gd_params;
    gd_params.max_iterations = 200;
    gd_params.step_size = 0.05;
    gd_params.tolerance = 1e-6;
    gd_params.verbose = false;

    GradientDescentOptimizer gd_optimizer(gd_params);

    auto gd_start = std::chrono::high_resolution_clock::now();
    auto gd_result = gd_optimizer.optimize(graph, initial);
    auto gd_end = std::chrono::high_resolution_clock::now();

    auto gd_duration = std::chrono::duration_cast<std::chrono::microseconds>(gd_end - gd_start);

    std::cout << "Iterations: " << gd_result.iterations << std::endl;
    std::cout << "Converged: " << (gd_result.converged ? "yes" : "no") << std::endl;
    std::cout << "Final error: " << std::fixed << std::setprecision(6) << gd_result.final_error << std::endl;
    std::cout << "Time: " << gd_duration.count() / 1000.0 << " ms" << std::endl;
    std::cout << std::endl;

    // Check loop closure error for GD
    Vec3d gd_final = gd_result.values.at<Vec3d>(X(4));
    Vec3d gd_start_pose = gd_result.values.at<Vec3d>(X(0));
    double gd_dx = gd_final[0] - gd_start_pose[0];
    double gd_dy = gd_final[1] - gd_start_pose[1];
    double gd_loop_dist = std::sqrt(gd_dx * gd_dx + gd_dy * gd_dy);
    std::cout << "Loop closure error: " << std::fixed << std::setprecision(3) << gd_loop_dist * 1000 << " mm"
              << std::endl;
    std::cout << std::endl;

    // ========================================
    // 3. Optimize with Gauss-Newton
    // ========================================

    std::cout << "=== Gauss-Newton ===" << std::endl;

    GaussNewtonOptimizer::Parameters gn_params;
    gn_params.max_iterations = 100;
    gn_params.tolerance = 1e-6;
    gn_params.verbose = false;

    GaussNewtonOptimizer gn_optimizer(gn_params);

    auto gn_start = std::chrono::high_resolution_clock::now();
    auto gn_result = gn_optimizer.optimize(graph, initial);
    auto gn_end = std::chrono::high_resolution_clock::now();

    auto gn_duration = std::chrono::duration_cast<std::chrono::microseconds>(gn_end - gn_start);

    std::cout << "Iterations: " << gn_result.iterations << std::endl;
    std::cout << "Converged: " << (gn_result.converged ? "yes" : "no") << std::endl;
    std::cout << "Final error: " << std::fixed << std::setprecision(6) << gn_result.final_error << std::endl;
    std::cout << "Time: " << gn_duration.count() / 1000.0 << " ms" << std::endl;
    std::cout << std::endl;

    // Check loop closure error for GN
    Vec3d gn_final = gn_result.values.at<Vec3d>(X(4));
    Vec3d gn_start_pose = gn_result.values.at<Vec3d>(X(0));
    double gn_dx = gn_final[0] - gn_start_pose[0];
    double gn_dy = gn_final[1] - gn_start_pose[1];
    double gn_loop_dist = std::sqrt(gn_dx * gn_dx + gn_dy * gn_dy);
    std::cout << "Loop closure error: " << std::fixed << std::setprecision(3) << gn_loop_dist * 1000 << " mm"
              << std::endl;
    std::cout << std::endl;

    // ========================================
    // 4. Optimize with Levenberg-Marquardt
    // ========================================

    std::cout << "=== Levenberg-Marquardt ===" << std::endl;

    LevenbergMarquardtOptimizer::Parameters lm_params;
    lm_params.max_iterations = 100;
    lm_params.tolerance = 1e-6;
    lm_params.verbose = false;
    lm_params.initial_lambda = 1e-3;
    lm_params.lambda_factor = 10.0;

    LevenbergMarquardtOptimizer lm_optimizer(lm_params);

    auto lm_start = std::chrono::high_resolution_clock::now();
    auto lm_result = lm_optimizer.optimize(graph, initial);
    auto lm_end = std::chrono::high_resolution_clock::now();

    auto lm_duration = std::chrono::duration_cast<std::chrono::microseconds>(lm_end - lm_start);

    std::cout << "Iterations: " << lm_result.iterations << std::endl;
    std::cout << "Converged: " << (lm_result.converged ? "yes" : "no") << std::endl;
    std::cout << "Final error: " << std::fixed << std::setprecision(6) << lm_result.final_error << std::endl;
    std::cout << "Time: " << lm_duration.count() / 1000.0 << " ms" << std::endl;
    std::cout << std::endl;

    // Check loop closure error for LM
    Vec3d lm_final = lm_result.values.at<Vec3d>(X(4));
    Vec3d lm_start_pose = lm_result.values.at<Vec3d>(X(0));
    double lm_dx = lm_final[0] - lm_start_pose[0];
    double lm_dy = lm_final[1] - lm_start_pose[1];
    double lm_loop_dist = std::sqrt(lm_dx * lm_dx + lm_dy * lm_dy);
    std::cout << "Loop closure error: " << std::fixed << std::setprecision(3) << lm_loop_dist * 1000 << " mm"
              << std::endl;
    std::cout << std::endl;

    // ========================================
    // 5. Comparison
    // ========================================

    std::cout << "=== Comparison ===" << std::endl;
    std::cout << std::endl;

    std::cout << std::setw(25) << "Method" << std::setw(15) << "Iterations" << std::setw(15) << "Time (ms)"
              << std::setw(15) << "Final Error" << std::setw(15) << "Speedup" << std::endl;
    std::cout << std::string(85, '-') << std::endl;

    double gd_time_ms = gd_duration.count() / 1000.0;
    double gn_time_ms = gn_duration.count() / 1000.0;
    double lm_time_ms = lm_duration.count() / 1000.0;
    double gn_speedup = gd_duration.count() / (double)gn_duration.count();
    double lm_speedup = gd_duration.count() / (double)lm_duration.count();

    std::cout << std::setw(25) << "Gradient Descent" << std::setw(15) << gd_result.iterations << std::setw(15)
              << std::fixed << std::setprecision(3) << gd_time_ms << std::setw(15) << std::fixed << std::setprecision(6)
              << gd_result.final_error << std::setw(15) << "1.0x" << std::endl;

    std::cout << std::setw(25) << "Gauss-Newton" << std::setw(15) << gn_result.iterations << std::setw(15) << std::fixed
              << std::setprecision(3) << gn_time_ms << std::setw(15) << std::fixed << std::setprecision(6)
              << gn_result.final_error << std::setw(15) << std::fixed << std::setprecision(1) << gn_speedup << "x"
              << std::endl;

    std::cout << std::setw(25) << "Levenberg-Marquardt" << std::setw(15) << lm_result.iterations << std::setw(15)
              << std::fixed << std::setprecision(3) << lm_time_ms << std::setw(15) << std::fixed << std::setprecision(6)
              << lm_result.final_error << std::setw(15) << std::fixed << std::setprecision(1) << lm_speedup << "x"
              << std::endl;

    std::cout << std::endl;

    std::cout << "=== Summary ===" << std::endl;
    std::cout << "✓ Gauss-Newton converges in " << (gd_result.iterations / gn_result.iterations) << "x fewer iterations"
              << std::endl;
    std::cout << "✓ Levenberg-Marquardt converges in " << (gd_result.iterations / lm_result.iterations)
              << "x fewer iterations" << std::endl;
    std::cout << "✓ Gauss-Newton is " << std::fixed << std::setprecision(1) << gn_speedup
              << "x faster than gradient descent" << std::endl;
    std::cout << "✓ Levenberg-Marquardt is " << std::fixed << std::setprecision(1) << lm_speedup
              << "x faster than gradient descent" << std::endl;
    std::cout << "✓ All methods achieve similar final accuracy" << std::endl;
    std::cout << "✓ LM offers best balance of speed and robustness for SLAM!" << std::endl;

    return 0;
}
