/**
 * @file se2_slam_2d.cpp
 * @brief 2D SLAM example using SE2 poses with proper Lie group operations
 *
 * This example demonstrates a complete SLAM workflow using SE2 poses:
 * 1. Robot moves in a square: (0,0) -> (2,0) -> (2,2) -> (0,2) -> (0,0)
 * 2. Odometry measurements with noise accumulate drift
 * 3. Loop closure measurement when robot returns to start
 * 4. Levenberg-Marquardt optimizer corrects the trajectory
 *
 * Unlike Vec3-based SLAM, SE2 factors use proper manifold operations
 * (exp/log maps) for computing errors and Jacobians.
 */

#include "graphix/factor/graph.hpp"
#include "graphix/factor/nonlinear/levenberg_marquardt.hpp"
#include "graphix/factor/nonlinear/se2_between_factor.hpp"
#include "graphix/factor/nonlinear/se2_prior_factor.hpp"
#include "graphix/factor/types.hpp"
#include "graphix/kernel.hpp"
#include <cmath>
#include <iomanip>
#include <iostream>

using namespace graphix;
using namespace graphix::factor;

// Helper function to print a pose
void print_pose(const std::string &label, const SE2d &pose) {
    std::cout << std::setw(12) << label << ": "
              << "x=" << std::setw(7) << std::fixed << std::setprecision(3) << pose.x() << ", "
              << "y=" << std::setw(7) << std::fixed << std::setprecision(3) << pose.y() << ", "
              << "theta=" << std::setw(7) << std::fixed << std::setprecision(3) << pose.angle() * 180.0 / M_PI << " deg"
              << std::endl;
}

// Helper function to print error
void print_error(const std::string &label, double error) {
    std::cout << label << ": " << std::fixed << std::setprecision(6) << error << std::endl;
}

int main() {
    std::cout << "=== 2D SLAM Example: Square Trajectory with Loop Closure (SE2) ===\n" << std::endl;

    // ========================================
    // 1. Setup: Define ground truth trajectory
    // ========================================

    // Robot moves in a 2x2 meter square:
    // X0: (0, 0, 0)      - Start at origin, facing east
    // X1: (2, 0, 0)      - Move 2m forward (east)
    // X2: (2, 0, pi/2)   - Turn left 90 deg
    // X3: (2, 2, pi/2)   - Move 2m forward (north)
    // X4: (2, 2, pi)     - Turn left 90 deg
    // X5: (0, 2, pi)     - Move 2m forward (west)
    // X6: (0, 2, 3pi/2)  - Turn left 90 deg
    // X7: (0, 0, 3pi/2)  - Move 2m forward (south), back at start!
    // X8: (0, 0, 2pi)    - Turn left 90 deg, facing original direction

    std::cout << "Ground Truth Trajectory (square path):" << std::endl;
    SE2d gt_x0(0.0, 0.0, 0.0);
    SE2d gt_x1(2.0, 0.0, 0.0);
    SE2d gt_x2(2.0, 0.0, M_PI / 2);
    SE2d gt_x3(2.0, 2.0, M_PI / 2);
    SE2d gt_x4(2.0, 2.0, M_PI);
    SE2d gt_x5(0.0, 2.0, M_PI);
    SE2d gt_x6(0.0, 2.0, 3 * M_PI / 2);
    SE2d gt_x7(0.0, 0.0, 3 * M_PI / 2);
    SE2d gt_x8(0.0, 0.0, 2 * M_PI);

    print_pose("X0 (start)", gt_x0);
    print_pose("X1", gt_x1);
    print_pose("X2", gt_x2);
    print_pose("X3", gt_x3);
    print_pose("X4", gt_x4);
    print_pose("X5", gt_x5);
    print_pose("X6", gt_x6);
    print_pose("X7", gt_x7);
    print_pose("X8 (end)", gt_x8);
    std::cout << std::endl;

    // ========================================
    // 2. Create factor graph with odometry
    // ========================================

    Graph<NonlinearFactor> graph;

    // Prior on starting pose (we know where we started)
    Vec3d prior_sigma{0.01, 0.01, 0.01}; // Very confident about start
    graph.add(std::make_shared<SE2PriorFactor>(X(0), gt_x0, prior_sigma));

    // Odometry measurements (translation then rotation pattern)
    Vec3d odom_sigma{0.1, 0.1, 0.05}; // More confident in rotation than translation

    // Odometry has slight systematic bias (drift)
    double translation_bias = 0.05; // Overestimate distance by 5cm each time
    double rotation_bias = -0.02;   // Underestimate rotation by ~1.1 deg

    // X0 -> X1: Move forward 2m (with bias: measures 2.05m)
    SE2d odom_0_1(2.0 + translation_bias, 0.0, 0.0);
    graph.add(std::make_shared<SE2BetweenFactor>(X(0), X(1), odom_0_1, odom_sigma));

    // X1 -> X2: Turn left 90 deg (with bias: measures ~88.9 deg)
    SE2d odom_1_2(0.0, 0.0, M_PI / 2 + rotation_bias);
    graph.add(std::make_shared<SE2BetweenFactor>(X(1), X(2), odom_1_2, odom_sigma));

    // X2 -> X3: Move forward 2m (with bias: measures 2.05m)
    SE2d odom_2_3(2.0 + translation_bias, 0.0, 0.0);
    graph.add(std::make_shared<SE2BetweenFactor>(X(2), X(3), odom_2_3, odom_sigma));

    // X3 -> X4: Turn left 90 deg
    SE2d odom_3_4(0.0, 0.0, M_PI / 2 + rotation_bias);
    graph.add(std::make_shared<SE2BetweenFactor>(X(3), X(4), odom_3_4, odom_sigma));

    // X4 -> X5: Move forward 2m
    SE2d odom_4_5(2.0 + translation_bias, 0.0, 0.0);
    graph.add(std::make_shared<SE2BetweenFactor>(X(4), X(5), odom_4_5, odom_sigma));

    // X5 -> X6: Turn left 90 deg
    SE2d odom_5_6(0.0, 0.0, M_PI / 2 + rotation_bias);
    graph.add(std::make_shared<SE2BetweenFactor>(X(5), X(6), odom_5_6, odom_sigma));

    // X6 -> X7: Move forward 2m
    SE2d odom_6_7(2.0 + translation_bias, 0.0, 0.0);
    graph.add(std::make_shared<SE2BetweenFactor>(X(6), X(7), odom_6_7, odom_sigma));

    // X7 -> X8: Turn left 90 deg
    SE2d odom_7_8(0.0, 0.0, M_PI / 2 + rotation_bias);
    graph.add(std::make_shared<SE2BetweenFactor>(X(7), X(8), odom_7_8, odom_sigma));

    std::cout << "Factor graph constructed with " << graph.size() << " factors:" << std::endl;
    std::cout << "  - 1 prior factor (X0)" << std::endl;
    std::cout << "  - 8 odometry factors (X0->X1, X1->X2, ..., X7->X8)" << std::endl;
    std::cout << std::endl;

    // ========================================
    // 3. Create initial values from odometry integration
    // ========================================

    std::cout << "Using odometry integration as initial guess" << std::endl;

    Values initial;
    initial.insert(X(0), gt_x0);

    // Integrate odometry using SE2 composition
    SE2d current_pose = gt_x0;

    // X0 -> X1
    current_pose = current_pose * odom_0_1;
    initial.insert(X(1), current_pose);

    // X1 -> X2
    current_pose = current_pose * odom_1_2;
    initial.insert(X(2), current_pose);

    // X2 -> X3
    current_pose = current_pose * odom_2_3;
    initial.insert(X(3), current_pose);

    // X3 -> X4
    current_pose = current_pose * odom_3_4;
    initial.insert(X(4), current_pose);

    // X4 -> X5
    current_pose = current_pose * odom_4_5;
    initial.insert(X(5), current_pose);

    // X5 -> X6
    current_pose = current_pose * odom_5_6;
    initial.insert(X(6), current_pose);

    // X6 -> X7
    current_pose = current_pose * odom_6_7;
    initial.insert(X(7), current_pose);

    // X7 -> X8
    current_pose = current_pose * odom_7_8;
    initial.insert(X(8), current_pose);

    std::cout << "\nInitial Estimate:" << std::endl;
    for (size_t i = 0; i < 9; i++) {
        SE2d pose = initial.at<SE2d>(X(i));
        print_pose("X" + std::to_string(i), pose);
    }
    std::cout << std::endl;

    // Calculate initial error
    double initial_error = 0.0;
    for (const auto &factor : graph) {
        initial_error += factor->error(initial);
    }
    print_error("Initial total error", initial_error);
    std::cout << std::endl;

    // ========================================
    // 4. Optimize WITHOUT loop closure
    // ========================================

    std::cout << "--- Optimizing WITHOUT loop closure ---" << std::endl;

    // Configure Levenberg-Marquardt optimizer
    LevenbergMarquardtOptimizer::Parameters params;
    params.max_iterations = 100;
    params.tolerance = 1e-6;
    params.verbose = false;

    LevenbergMarquardtOptimizer optimizer(params);
    auto result_no_loop = optimizer.optimize(graph, initial);

    std::cout << "Optimization completed:" << std::endl;
    std::cout << "  Iterations: " << result_no_loop.iterations << std::endl;
    std::cout << "  Converged: " << (result_no_loop.converged ? "yes" : "no") << std::endl;
    std::cout << "  Final error: " << std::fixed << std::setprecision(6) << result_no_loop.final_error << std::endl;
    std::cout << std::endl;

    std::cout << "Optimized Trajectory (without loop closure):" << std::endl;
    for (size_t i = 0; i < 9; i++) {
        SE2d pose = result_no_loop.values.at<SE2d>(X(i));
        print_pose("X" + std::to_string(i), pose);
    }

    // Check loop closure error
    SE2d final_pose_no_loop = result_no_loop.values.at<SE2d>(X(8));
    SE2d start_pose = result_no_loop.values.at<SE2d>(X(0));
    double loop_dx = final_pose_no_loop.x() - start_pose.x();
    double loop_dy = final_pose_no_loop.y() - start_pose.y();
    std::cout << "\nLoop closure error (X8 - X0):" << std::endl;
    std::cout << "  dx: " << loop_dx << " m, dy: " << loop_dy << " m" << std::endl;
    std::cout << "  Distance: " << std::sqrt(loop_dx * loop_dx + loop_dy * loop_dy) << " meters" << std::endl;
    std::cout << std::endl;

    // ========================================
    // 5. Add loop closure and re-optimize
    // ========================================

    std::cout << "--- Adding loop closure constraint ---" << std::endl;

    // Loop closure: We observe that X8 is very close to X0 (same position, same orientation)
    // This is a powerful constraint that will distribute the accumulated error
    SE2d loop_closure_measurement(0.0, 0.0, 2 * M_PI); // X8 should equal X0 + full rotation
    Vec3d loop_closure_sigma{0.15, 0.15, 0.1};         // Slightly less confident than prior

    auto loop_closure_factor =
        std::make_shared<SE2BetweenFactor>(X(0), X(8), loop_closure_measurement, loop_closure_sigma);
    graph.add(loop_closure_factor);

    std::cout << "Added loop closure factor: X0 -> X8" << std::endl;
    std::cout << "  Measurement: robot should return to start after full square" << std::endl;
    std::cout << "  Total factors: " << graph.size() << std::endl;
    std::cout << std::endl;

    // Re-optimize with loop closure
    std::cout << "--- Optimizing WITH loop closure ---" << std::endl;
    auto result_with_loop = optimizer.optimize(graph, initial);

    std::cout << "Optimization completed:" << std::endl;
    std::cout << "  Iterations: " << result_with_loop.iterations << std::endl;
    std::cout << "  Converged: " << (result_with_loop.converged ? "yes" : "no") << std::endl;
    std::cout << "  Final error: " << std::fixed << std::setprecision(6) << result_with_loop.final_error << std::endl;
    std::cout << std::endl;

    std::cout << "Optimized Trajectory (WITH loop closure):" << std::endl;
    for (size_t i = 0; i < 9; i++) {
        SE2d pose = result_with_loop.values.at<SE2d>(X(i));
        print_pose("X" + std::to_string(i), pose);
    }

    // Check loop closure error
    SE2d final_pose_with_loop = result_with_loop.values.at<SE2d>(X(8));
    SE2d start_pose_final = result_with_loop.values.at<SE2d>(X(0));
    double loop_dx2 = final_pose_with_loop.x() - start_pose_final.x();
    double loop_dy2 = final_pose_with_loop.y() - start_pose_final.y();
    std::cout << "\nLoop closure error (X8 - X0):" << std::endl;
    std::cout << "  dx: " << loop_dx2 << " m, dy: " << loop_dy2 << " m" << std::endl;
    std::cout << "  Distance: " << std::sqrt(loop_dx2 * loop_dx2 + loop_dy2 * loop_dy2) << " meters" << std::endl;
    std::cout << std::endl;

    // ========================================
    // 6. Compare to ground truth
    // ========================================

    std::cout << "=== Comparison to Ground Truth ===" << std::endl;
    std::cout << std::endl;

    std::vector<SE2d> ground_truth = {gt_x0, gt_x1, gt_x2, gt_x3, gt_x4, gt_x5, gt_x6, gt_x7, gt_x8};

    double total_position_error = 0.0;
    double total_angle_error = 0.0;

    std::cout << "Per-pose errors (optimized vs ground truth):" << std::endl;
    for (size_t i = 0; i < 9; i++) {
        SE2d optimized = result_with_loop.values.at<SE2d>(X(i));
        SE2d gt = ground_truth[i];
        double err_x = optimized.x() - gt.x();
        double err_y = optimized.y() - gt.y();
        double err_theta = optimized.angle() - gt.angle();

        double pos_error = std::sqrt(err_x * err_x + err_y * err_y);
        double angle_error = std::abs(err_theta) * 180.0 / M_PI;

        total_position_error += pos_error;
        total_angle_error += angle_error;

        std::cout << "  X" << i << ": pos=" << std::fixed << std::setprecision(4) << pos_error
                  << "m, angle=" << angle_error << " deg" << std::endl;
    }

    std::cout << "\nAverage errors:" << std::endl;
    std::cout << "  Position: " << std::fixed << std::setprecision(4) << (total_position_error / 9.0) << " meters"
              << std::endl;
    std::cout << "  Angle: " << (total_angle_error / 9.0) << " degrees" << std::endl;
    std::cout << std::endl;

    std::cout << "=== Summary ===" << std::endl;
    std::cout << "Demonstrated 2D SLAM with SE2 poses using proper Lie group operations" << std::endl;
    std::cout << "Odometry drift accumulated without loop closure" << std::endl;
    std::cout << "Loop closure constraint distributed error across trajectory" << std::endl;
    std::cout << "Levenberg-Marquardt successfully optimized the factor graph" << std::endl;
    std::cout << "Final trajectory close to ground truth square path" << std::endl;

    return 0;
}
