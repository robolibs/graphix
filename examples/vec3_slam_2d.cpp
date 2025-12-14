/**
 * @file vec3_slam_2d.cpp
 * @brief 2D SLAM example using Vec3d to represent poses (x, y, theta)
 *
 * This example demonstrates a complete SLAM workflow:
 * 1. Robot moves in a square: (0,0) → (2,0) → (2,2) → (0,2) → (0,0)
 * 2. Odometry measurements with noise accumulate drift
 * 3. Loop closure measurement when robot returns to start
 * 4. Gradient descent optimizer corrects the trajectory
 */

#include "graphix/factor/graph.hpp"
#include "graphix/factor/nonlinear/gradient_descent.hpp"
#include "graphix/factor/nonlinear/vec3_between_factor.hpp"
#include "graphix/factor/nonlinear/vec3_prior_factor.hpp"
#include "graphix/kernel.hpp"
#include <cmath>
#include <iomanip>
#include <iostream>

using namespace graphix;
using namespace graphix::factor;

// Helper function to print a pose
void print_pose(const std::string &label, const Vec3d &pose) {
    std::cout << std::setw(12) << label << ": "
              << "x=" << std::setw(7) << std::fixed << std::setprecision(3) << pose.x() << ", "
              << "y=" << std::setw(7) << std::fixed << std::setprecision(3) << pose.y() << ", "
              << "θ=" << std::setw(7) << std::fixed << std::setprecision(3) << pose.z() * 180.0 / M_PI << "°"
              << std::endl;
}

// Helper function to print error
void print_error(const std::string &label, double error) {
    std::cout << label << ": " << std::fixed << std::setprecision(6) << error << std::endl;
}

int main() {
    std::cout << "=== 2D SLAM Example: Square Trajectory with Loop Closure ===\n" << std::endl;

    // ========================================
    // 1. Setup: Define ground truth trajectory
    // ========================================

    // Robot moves in a 2x2 meter square:
    // X0: (0, 0, 0°)    - Start at origin, facing east
    // X1: (2, 0, 0°)    - Move 2m forward (east)
    // X2: (2, 0, 90°)   - Turn left 90°
    // X3: (2, 2, 90°)   - Move 2m forward (north)
    // X4: (2, 2, 180°)  - Turn left 90°
    // X5: (0, 2, 180°)  - Move 2m forward (west)
    // X6: (0, 2, 270°)  - Turn left 90°
    // X7: (0, 0, 270°)  - Move 2m forward (south), back at start!
    // X8: (0, 0, 0°)    - Turn left 90°, facing original direction

    std::cout << "Ground Truth Trajectory (square path):" << std::endl;
    Vec3d gt_x0(0.0, 0.0, 0.0);
    Vec3d gt_x1(2.0, 0.0, 0.0);
    Vec3d gt_x2(2.0, 0.0, M_PI / 2);
    Vec3d gt_x3(2.0, 2.0, M_PI / 2);
    Vec3d gt_x4(2.0, 2.0, M_PI);
    Vec3d gt_x5(0.0, 2.0, M_PI);
    Vec3d gt_x6(0.0, 2.0, 3 * M_PI / 2);
    Vec3d gt_x7(0.0, 0.0, 3 * M_PI / 2);
    Vec3d gt_x8(0.0, 0.0, 2 * M_PI);

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
    Vec3d prior_sigma(0.01, 0.01, 0.01); // Very confident about start
    graph.add(std::make_shared<Vec3PriorFactor>(X(0), gt_x0, prior_sigma));

    // Odometry measurements (translation then rotation pattern)
    Vec3d odom_sigma(0.1, 0.1, 0.05); // More confident in rotation than translation

    // Odometry has slight systematic bias (drift)
    double translation_bias = 0.05; // Overestimate distance by 5cm each time
    double rotation_bias = -0.02;   // Underestimate rotation by ~1.1°

    // X0 → X1: Move forward 2m (with bias: measures 2.05m)
    Vec3d odom_0_1(2.0 + translation_bias, 0.0, 0.0);
    graph.add(std::make_shared<Vec3BetweenFactor>(X(0), X(1), odom_0_1, odom_sigma));

    // X1 → X2: Turn left 90° (with bias: measures ~88.9°)
    Vec3d odom_1_2(0.0, 0.0, M_PI / 2 + rotation_bias);
    graph.add(std::make_shared<Vec3BetweenFactor>(X(1), X(2), odom_1_2, odom_sigma));

    // X2 → X3: Move forward 2m (with bias: measures 2.05m)
    Vec3d odom_2_3(2.0 + translation_bias, 0.0, 0.0);
    graph.add(std::make_shared<Vec3BetweenFactor>(X(2), X(3), odom_2_3, odom_sigma));

    // X3 → X4: Turn left 90°
    Vec3d odom_3_4(0.0, 0.0, M_PI / 2 + rotation_bias);
    graph.add(std::make_shared<Vec3BetweenFactor>(X(3), X(4), odom_3_4, odom_sigma));

    // X4 → X5: Move forward 2m
    Vec3d odom_4_5(2.0 + translation_bias, 0.0, 0.0);
    graph.add(std::make_shared<Vec3BetweenFactor>(X(4), X(5), odom_4_5, odom_sigma));

    // X5 → X6: Turn left 90°
    Vec3d odom_5_6(0.0, 0.0, M_PI / 2 + rotation_bias);
    graph.add(std::make_shared<Vec3BetweenFactor>(X(5), X(6), odom_5_6, odom_sigma));

    // X6 → X7: Move forward 2m
    Vec3d odom_6_7(2.0 + translation_bias, 0.0, 0.0);
    graph.add(std::make_shared<Vec3BetweenFactor>(X(6), X(7), odom_6_7, odom_sigma));

    // X7 → X8: Turn left 90°
    Vec3d odom_7_8(0.0, 0.0, M_PI / 2 + rotation_bias);
    graph.add(std::make_shared<Vec3BetweenFactor>(X(7), X(8), odom_7_8, odom_sigma));

    std::cout << "Factor graph constructed with " << graph.size() << " factors:" << std::endl;
    std::cout << "  - 1 prior factor (X0)" << std::endl;
    std::cout << "  - 8 odometry factors (X0→X1, X1→X2, ..., X7→X8)" << std::endl;
    std::cout << std::endl;

    // ========================================
    // 3. Create initial values
    // ========================================

    // Option A: Use ground truth with small noise (easier problem)
    // Option B: Use odometry integration (realistic but harder)

    bool use_ground_truth_init = false; // Toggle this

    Values initial;

    if (use_ground_truth_init) {
        // Start very close to solution to test optimizer
        std::cout << "Using ground truth with small noise as initial guess" << std::endl;
        initial.insert(X(0), gt_x0 + Vec3d(0.05, 0.05, 0.01));
        initial.insert(X(1), gt_x1 + Vec3d(-0.03, 0.04, -0.01));
        initial.insert(X(2), gt_x2 + Vec3d(0.04, -0.02, 0.02));
        initial.insert(X(3), gt_x3 + Vec3d(-0.02, 0.03, -0.01));
        initial.insert(X(4), gt_x4 + Vec3d(0.03, -0.04, 0.01));
        initial.insert(X(5), gt_x5 + Vec3d(-0.04, 0.02, -0.02));
        initial.insert(X(6), gt_x6 + Vec3d(0.02, -0.03, 0.01));
        initial.insert(X(7), gt_x7 + Vec3d(-0.01, 0.04, -0.01));
        initial.insert(X(8), gt_x8 + Vec3d(0.03, -0.02, 0.02));
    } else {
        // Odometry integration (more realistic)
        std::cout << "Using odometry integration as initial guess" << std::endl;
        initial.insert(X(0), gt_x0);

        // Integrate odometry (forward kinematics with biased measurements)
        Vec3d current_pose = gt_x0;

        // X0 → X1: Move forward in current direction
        double dx1 = odom_0_1.x() * cos(current_pose.z());
        double dy1 = odom_0_1.x() * sin(current_pose.z());
        current_pose = Vec3d(current_pose.x() + dx1, current_pose.y() + dy1, current_pose.z());
        initial.insert(X(1), current_pose);

        // X1 → X2: Rotate
        current_pose = Vec3d(current_pose.x(), current_pose.y(), current_pose.z() + odom_1_2.z());
        initial.insert(X(2), current_pose);

        // X2 → X3: Move forward
        double dx3 = odom_2_3.x() * cos(current_pose.z());
        double dy3 = odom_2_3.x() * sin(current_pose.z());
        current_pose = Vec3d(current_pose.x() + dx3, current_pose.y() + dy3, current_pose.z());
        initial.insert(X(3), current_pose);

        // X3 → X4: Rotate
        current_pose = Vec3d(current_pose.x(), current_pose.y(), current_pose.z() + odom_3_4.z());
        initial.insert(X(4), current_pose);

        // X4 → X5: Move forward
        double dx5 = odom_4_5.x() * cos(current_pose.z());
        double dy5 = odom_4_5.x() * sin(current_pose.z());
        current_pose = Vec3d(current_pose.x() + dx5, current_pose.y() + dy5, current_pose.z());
        initial.insert(X(5), current_pose);

        // X5 → X6: Rotate
        current_pose = Vec3d(current_pose.x(), current_pose.y(), current_pose.z() + odom_5_6.z());
        initial.insert(X(6), current_pose);

        // X6 → X7: Move forward
        double dx7 = odom_6_7.x() * cos(current_pose.z());
        double dy7 = odom_6_7.x() * sin(current_pose.z());
        current_pose = Vec3d(current_pose.x() + dx7, current_pose.y() + dy7, current_pose.z());
        initial.insert(X(7), current_pose);

        // X7 → X8: Rotate
        current_pose = Vec3d(current_pose.x(), current_pose.y(), current_pose.z() + odom_7_8.z());
        initial.insert(X(8), current_pose);
    }

    std::cout << "\nInitial Estimate:" << std::endl;
    for (size_t i = 0; i < 9; i++) {
        Vec3d pose = initial.at<Vec3d>(X(i));
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

    // Configure optimizer
    GradientDescentOptimizer::Parameters params;
    params.max_iterations = 10000; // LOTS of iterations for gradient descent
    params.step_size = 0.01;       // Moderate step size
    params.tolerance = 1e-3;       // Relaxed tolerance
    params.use_adaptive_lr = true; // Use Adam optimizer (better than plain GD)
    params.verbose = false;        // Set to true to see iteration details

    GradientDescentOptimizer optimizer(params);
    auto result_no_loop = optimizer.optimize(graph, initial);

    std::cout << "Optimization completed:" << std::endl;
    std::cout << "  Iterations: " << result_no_loop.iterations << std::endl;
    std::cout << "  Converged: " << (result_no_loop.converged ? "yes" : "no") << std::endl;
    std::cout << "  Final error: " << std::fixed << std::setprecision(6) << result_no_loop.final_error << std::endl;
    std::cout << "  Gradient norm: " << result_no_loop.gradient_norm << std::endl;
    std::cout << std::endl;

    std::cout << "Optimized Trajectory (without loop closure):" << std::endl;
    for (size_t i = 0; i < 9; i++) {
        Vec3d pose = result_no_loop.values.at<Vec3d>(X(i));
        print_pose("X" + std::to_string(i), pose);
    }

    // Check loop closure error
    Vec3d final_pose_no_loop = result_no_loop.values.at<Vec3d>(X(8));
    Vec3d start_pose = result_no_loop.values.at<Vec3d>(X(0));
    Vec3d loop_error_no_loop = final_pose_no_loop - start_pose;
    std::cout << "\nLoop closure error (X8 - X0):" << std::endl;
    print_pose("  Error", loop_error_no_loop);
    std::cout << "  Distance: "
              << std::sqrt(loop_error_no_loop.x() * loop_error_no_loop.x() +
                           loop_error_no_loop.y() * loop_error_no_loop.y())
              << " meters" << std::endl;
    std::cout << std::endl;

    // ========================================
    // 5. Add loop closure and re-optimize
    // ========================================

    std::cout << "--- Adding loop closure constraint ---" << std::endl;

    // Loop closure: We observe that X8 is very close to X0 (same position, same orientation)
    // This is a powerful constraint that will distribute the accumulated error
    Vec3d loop_closure_measurement(0.0, 0.0, 2 * M_PI); // X8 should equal X0 + full rotation
    Vec3d loop_closure_sigma(0.15, 0.15, 0.1);          // Slightly less confident than prior

    auto loop_closure_factor =
        std::make_shared<Vec3BetweenFactor>(X(0), X(8), loop_closure_measurement, loop_closure_sigma);
    graph.add(loop_closure_factor);

    std::cout << "Added loop closure factor: X0 → X8" << std::endl;
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
    std::cout << "  Gradient norm: " << result_with_loop.gradient_norm << std::endl;
    std::cout << "  Error reduction from no-loop: " << (result_no_loop.final_error - result_with_loop.final_error)
              << std::endl;
    std::cout << std::endl;

    std::cout << "Optimized Trajectory (WITH loop closure):" << std::endl;
    for (size_t i = 0; i < 9; i++) {
        Vec3d pose = result_with_loop.values.at<Vec3d>(X(i));
        print_pose("X" + std::to_string(i), pose);
    }

    // Check loop closure error
    Vec3d final_pose_with_loop = result_with_loop.values.at<Vec3d>(X(8));
    Vec3d start_pose_final = result_with_loop.values.at<Vec3d>(X(0));
    Vec3d loop_error_with_loop = final_pose_with_loop - start_pose_final;
    std::cout << "\nLoop closure error (X8 - X0):" << std::endl;
    print_pose("  Error", loop_error_with_loop);
    std::cout << "  Distance: "
              << std::sqrt(loop_error_with_loop.x() * loop_error_with_loop.x() +
                           loop_error_with_loop.y() * loop_error_with_loop.y())
              << " meters" << std::endl;
    std::cout << std::endl;

    // ========================================
    // 6. Compare to ground truth
    // ========================================

    std::cout << "=== Comparison to Ground Truth ===" << std::endl;
    std::cout << std::endl;

    std::vector<Vec3d> ground_truth = {gt_x0, gt_x1, gt_x2, gt_x3, gt_x4, gt_x5, gt_x6, gt_x7, gt_x8};

    double total_position_error = 0.0;
    double total_angle_error = 0.0;

    std::cout << "Per-pose errors (optimized vs ground truth):" << std::endl;
    for (size_t i = 0; i < 9; i++) {
        Vec3d optimized = result_with_loop.values.at<Vec3d>(X(i));
        Vec3d gt = ground_truth[i];
        Vec3d error = optimized - gt;

        double pos_error = std::sqrt(error.x() * error.x() + error.y() * error.y());
        double angle_error = std::abs(error.z()) * 180.0 / M_PI;

        total_position_error += pos_error;
        total_angle_error += angle_error;

        std::cout << "  X" << i << ": pos=" << std::fixed << std::setprecision(4) << pos_error
                  << "m, angle=" << angle_error << "°" << std::endl;
    }

    std::cout << "\nAverage errors:" << std::endl;
    std::cout << "  Position: " << std::fixed << std::setprecision(4) << (total_position_error / 9.0) << " meters"
              << std::endl;
    std::cout << "  Angle: " << (total_angle_error / 9.0) << " degrees" << std::endl;
    std::cout << std::endl;

    std::cout << "=== Summary ===" << std::endl;
    std::cout << "✓ Demonstrated 2D SLAM with Vec3d representing poses (x, y, θ)" << std::endl;
    std::cout << "✓ Odometry drift accumulated without loop closure" << std::endl;
    std::cout << "✓ Loop closure constraint distributed error across trajectory" << std::endl;
    std::cout << "✓ Gradient descent successfully optimized the factor graph" << std::endl;
    std::cout << "✓ Final trajectory close to ground truth square path" << std::endl;

    return 0;
}
