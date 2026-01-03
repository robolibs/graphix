/**
 * @file robust_slam_example.cpp
 * @brief Demonstrate robust loss functions handling outliers in SLAM
 *
 * This example shows how different loss functions handle bad loop closures
 * (outlier measurements) in a 2D SLAM problem using SE2 poses.
 */

#include "graphix/factor/graph.hpp"
#include "graphix/factor/loss_function.hpp"
#include "graphix/factor/nonlinear/levenberg_marquardt.hpp"
#include "graphix/factor/nonlinear/se2_between_factor.hpp"
#include "graphix/factor/nonlinear/se2_prior_factor.hpp"
#include "graphix/kernel.hpp"
#include <cmath>
#include <iomanip>
#include <iostream>

using namespace graphix;
using namespace graphix::factor;

int main() {
    std::cout << "=== Robust Loss Functions for SLAM ===" << std::endl;
    std::cout << std::endl;

    // ========================================
    // Build SLAM problem with outlier
    // ========================================

    // Robot drives in square: (0,0) -> (2,0) -> (2,2) -> (0,2) -> (0,0)
    // All odometry is accurate
    // BUT: one loop closure is completely wrong (outlier)

    Graph<NonlinearFactor> graph;

    // Prior on start position
    SE2d prior_mean(0, 0, 0);
    Vec3d prior_sigma{0.01, 0.01, 0.01};
    graph.add(std::make_shared<SE2PriorFactor>(X(0), prior_mean, prior_sigma));

    // Accurate odometry (square path)
    Vec3d odom_sigma{0.1, 0.1, 0.05};

    graph.add(std::make_shared<SE2BetweenFactor>(X(0), X(1), SE2d(2.0, 0.0, 0.0), odom_sigma));       // -> East
    graph.add(std::make_shared<SE2BetweenFactor>(X(1), X(2), SE2d(0.0, 2.0, M_PI / 2), odom_sigma));  // ^ North
    graph.add(std::make_shared<SE2BetweenFactor>(X(2), X(3), SE2d(-2.0, 0.0, M_PI / 2), odom_sigma)); // <- West
    graph.add(std::make_shared<SE2BetweenFactor>(X(3), X(4), SE2d(0.0, -2.0, M_PI / 2), odom_sigma)); // v South

    // Good loop closure
    Vec3d loop_sigma{0.05, 0.05, 0.05};
    graph.add(std::make_shared<SE2BetweenFactor>(X(4), X(0), SE2d(0.0, 0.0, M_PI / 2), loop_sigma));

    // BAD loop closure (outlier!) - claims X(2) is near X(0) but it's not!
    // This is completely wrong - X(2) should be at (2, 2) not (0.5, 0.5)
    auto bad_loop = std::make_shared<SE2BetweenFactor>(X(2), X(0), SE2d(-0.5, -0.5, -M_PI), loop_sigma);

    std::cout << "Problem: Square path (0,0) -> (2,0) -> (2,2) -> (0,2) -> (0,0)" << std::endl;
    std::cout << "  - 4 accurate odometry measurements" << std::endl;
    std::cout << "  - 1 good loop closure (X(4) -> X(0))" << std::endl;
    std::cout << "  - 1 BAD loop closure (outlier: X(2) -> X(0))" << std::endl;
    std::cout << std::endl;

    // Initial guess (from odometry)
    Values initial;
    initial.insert(X(0), SE2d(0.0, 0.0, 0.0));
    initial.insert(X(1), SE2d(2.0, 0.0, 0.0));
    initial.insert(X(2), SE2d(2.0, 2.0, M_PI / 2));
    initial.insert(X(3), SE2d(0.0, 2.0, M_PI));
    initial.insert(X(4), SE2d(0.0, 0.0, 3 * M_PI / 2));

    // ========================================
    // Test 1: No robust loss (standard LS)
    // ========================================

    std::cout << "=== Test 1: Standard Least Squares (No Robust Loss) ===" << std::endl;

    Graph<NonlinearFactor> graph_no_robust = graph;
    graph_no_robust.add(bad_loop); // Add outlier without robust loss

    LevenbergMarquardtOptimizer::Parameters params;
    params.verbose = false;
    params.max_iterations = 100;

    LevenbergMarquardtOptimizer optimizer(params);
    auto result_no_robust = optimizer.optimize(graph_no_robust, initial);

    SE2d pose2_no_robust = result_no_robust.values.at<SE2d>(X(2));
    std::cout << "X(2) optimized position: (" << std::fixed << std::setprecision(3) << pose2_no_robust.x() << ", "
              << pose2_no_robust.y() << ")" << std::endl;
    std::cout << "Expected position:        (2.000, 2.000)" << std::endl;
    std::cout << "Error (mm): "
              << std::sqrt(std::pow(pose2_no_robust.x() - 2.0, 2) + std::pow(pose2_no_robust.y() - 2.0, 2)) * 1000
              << std::endl;
    std::cout << "Final error: " << result_no_robust.final_error << std::endl;
    std::cout << "Problem: Outlier pulls solution away from truth!" << std::endl;
    std::cout << std::endl;

    // ========================================
    // Test 2: Huber loss
    // ========================================

    std::cout << "=== Test 2: Huber Loss (Balanced) ===" << std::endl;

    Graph<NonlinearFactor> graph_huber = graph;
    auto bad_loop_huber = std::make_shared<SE2BetweenFactor>(X(2), X(0), SE2d(-0.5, -0.5, -M_PI), loop_sigma);
    bad_loop_huber->set_loss_function(huber_loss(1.345));
    graph_huber.add(bad_loop_huber);

    auto result_huber = optimizer.optimize(graph_huber, initial);

    SE2d pose2_huber = result_huber.values.at<SE2d>(X(2));
    std::cout << "X(2) optimized position: (" << std::fixed << std::setprecision(3) << pose2_huber.x() << ", "
              << pose2_huber.y() << ")" << std::endl;
    std::cout << "Expected position:        (2.000, 2.000)" << std::endl;
    std::cout << "Error (mm): "
              << std::sqrt(std::pow(pose2_huber.x() - 2.0, 2) + std::pow(pose2_huber.y() - 2.0, 2)) * 1000 << std::endl;
    std::cout << "Final error: " << result_huber.final_error << std::endl;
    std::cout << "Improvement: Reduced influence of outlier" << std::endl;
    std::cout << std::endl;

    // ========================================
    // Test 3: Cauchy loss
    // ========================================

    std::cout << "=== Test 3: Cauchy Loss (Aggressive) ===" << std::endl;

    Graph<NonlinearFactor> graph_cauchy = graph;
    auto bad_loop_cauchy = std::make_shared<SE2BetweenFactor>(X(2), X(0), SE2d(-0.5, -0.5, -M_PI), loop_sigma);
    bad_loop_cauchy->set_loss_function(cauchy_loss(2.3849));
    graph_cauchy.add(bad_loop_cauchy);

    auto result_cauchy = optimizer.optimize(graph_cauchy, initial);

    SE2d pose2_cauchy = result_cauchy.values.at<SE2d>(X(2));
    std::cout << "X(2) optimized position: (" << std::fixed << std::setprecision(3) << pose2_cauchy.x() << ", "
              << pose2_cauchy.y() << ")" << std::endl;
    std::cout << "Expected position:        (2.000, 2.000)" << std::endl;
    std::cout << "Error (mm): "
              << std::sqrt(std::pow(pose2_cauchy.x() - 2.0, 2) + std::pow(pose2_cauchy.y() - 2.0, 2)) * 1000
              << std::endl;
    std::cout << "Final error: " << result_cauchy.final_error << std::endl;
    std::cout << "Improvement: Heavy downweighting of outlier" << std::endl;
    std::cout << std::endl;

    // ========================================
    // Test 4: Tukey loss
    // ========================================

    std::cout << "=== Test 4: Tukey Loss (Complete Rejection) ===" << std::endl;

    Graph<NonlinearFactor> graph_tukey = graph;
    auto bad_loop_tukey = std::make_shared<SE2BetweenFactor>(X(2), X(0), SE2d(-0.5, -0.5, -M_PI), loop_sigma);
    bad_loop_tukey->set_loss_function(tukey_loss(4.6851));
    graph_tukey.add(bad_loop_tukey);

    auto result_tukey = optimizer.optimize(graph_tukey, initial);

    SE2d pose2_tukey = result_tukey.values.at<SE2d>(X(2));
    std::cout << "X(2) optimized position: (" << std::fixed << std::setprecision(3) << pose2_tukey.x() << ", "
              << pose2_tukey.y() << ")" << std::endl;
    std::cout << "Expected position:        (2.000, 2.000)" << std::endl;
    std::cout << "Error (mm): "
              << std::sqrt(std::pow(pose2_tukey.x() - 2.0, 2) + std::pow(pose2_tukey.y() - 2.0, 2)) * 1000 << std::endl;
    std::cout << "Final error: " << result_tukey.final_error << std::endl;
    std::cout << "Improvement: Outlier completely ignored!" << std::endl;
    std::cout << std::endl;

    // ========================================
    // Comparison
    // ========================================

    std::cout << "=== Summary ===" << std::endl;
    std::cout << std::endl;

    auto compute_error = [](const SE2d &pose, double true_x, double true_y) {
        return std::sqrt(std::pow(pose.x() - true_x, 2) + std::pow(pose.y() - true_y, 2)) * 1000;
    };

    std::cout << std::setw(20) << "Method" << std::setw(20) << "X(2) Error (mm)" << std::setw(20) << "Total Cost"
              << std::endl;
    std::cout << std::string(60, '-') << std::endl;

    std::cout << std::setw(20) << "No Robust Loss" << std::setw(20) << std::fixed << std::setprecision(1)
              << compute_error(pose2_no_robust, 2.0, 2.0) << std::setw(20) << std::fixed << std::setprecision(3)
              << result_no_robust.final_error << std::endl;

    std::cout << std::setw(20) << "Huber" << std::setw(20) << std::fixed << std::setprecision(1)
              << compute_error(pose2_huber, 2.0, 2.0) << std::setw(20) << std::fixed << std::setprecision(3)
              << result_huber.final_error << std::endl;

    std::cout << std::setw(20) << "Cauchy" << std::setw(20) << std::fixed << std::setprecision(1)
              << compute_error(pose2_cauchy, 2.0, 2.0) << std::setw(20) << std::fixed << std::setprecision(3)
              << result_cauchy.final_error << std::endl;

    std::cout << std::setw(20) << "Tukey" << std::setw(20) << std::fixed << std::setprecision(1)
              << compute_error(pose2_tukey, 2.0, 2.0) << std::setw(20) << std::fixed << std::setprecision(3)
              << result_tukey.final_error << std::endl;

    std::cout << std::endl;
    std::cout << "Conclusion:" << std::endl;
    std::cout << "  - Standard LS: Severely affected by outliers" << std::endl;
    std::cout << "  - Huber: Good balance, reduces outlier influence" << std::endl;
    std::cout << "  - Cauchy: Aggressive rejection, very robust" << std::endl;
    std::cout << "  - Tukey: Complete rejection beyond threshold" << std::endl;
    std::cout << std::endl;
    std::cout << "For real SLAM: Use Huber for general robustness, Cauchy/Tukey for heavy outliers" << std::endl;

    return 0;
}
