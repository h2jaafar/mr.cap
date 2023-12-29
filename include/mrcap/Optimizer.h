#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Point2.h>
#include <gtsam/base/Vector.h>

#include <gtsam/base/Matrix.h>
#include <gtsam/nonlinear/NonlinearOptimizerParams.h>
#include <gtsam/nonlinear/NonlinearOptimizer.h>
#include <gtsam/nonlinear/NonlinearConjugateGradientOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
// between factor
#include <gtsam/slam/BetweenFactor.h>

// inference
#include <gtsam/inference/Symbol.h>
#include <gtsam/inference/LabeledSymbol.h>
#include <gtsam/inference/Ordering.h>

// headers
#include "customFactor.h"
#include "Trajectory.h"
#include "DataTypes/RobotData.h"
#include "FactorGraph.h"
#include "ROSCommunications.h"
#include "Geometry.h"
// #include "DataTypes/Atan2LUT.h"
// std classes
#include <fstream>
#include <vector>
#include <tuple>
#include <random>

// plotting
#include "../implot/implot.h"
#include "../implot/implot_internal.h"
// ROS2
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>

#include <thread>
#include <mutex>
// Logger
extern ImGuiLogger logger;
// extern Atan2LUT lut;
std::mutex vicon_mutex;
int robot_failed;

void robotController(
    const RobotData &robot,
    std::shared_ptr<GazeboSubscriber> subscriber,
    // std::shared_ptr<ViconSubscriber> subscriber,

    std::shared_ptr<VelocityPublisher> publisher,
    double proportional_gain,
    double pid_error_threshold,
    double wheel_speed_limit) {
    // Get the start time
    auto start_time = std::chrono::steady_clock::now();
    int timeout_seconds = 160;
    rclcpp::spin_some(subscriber);
    gtsam::Pose2 X_robot_during_rotation = gtsam::Pose2(robot_poses[robot.robot_id - 1].x, robot_poses[robot.robot_id - 1].y, robot_poses[robot.robot_id - 1].yaw);
    double angle_diff = Utils::ensure_orientation_range2(robot.target_orientation - X_robot_during_rotation.theta());

    while (abs(angle_diff) > pid_error_threshold) {
        // Check if the timeout has been reached
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();
        if (elapsed_time >= timeout_seconds) {
            std::cerr << "Timeout reached for Robot " << robot.robot_id << std::endl;
            robot_failed = 3;
            break;  // Exit the loop if timeout is reached
        }

        double V = 0;
        double w = proportional_gain * angle_diff;
        w = std::clamp(w, -wheel_speed_limit, wheel_speed_limit);
        publisher->publish(V, w);
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        rclcpp::spin_some(subscriber);
        X_robot_during_rotation = gtsam::Pose2(robot_poses[robot.robot_id - 1].x, robot_poses[robot.robot_id - 1].y, robot_poses[robot.robot_id - 1].yaw);
        angle_diff = Utils::ensure_orientation_range2(robot.target_orientation - X_robot_during_rotation.theta());
    }

    publisher->publish(0, 0);
    std::cout << "Robot " << robot.robot_id << " finished rotating" << std::endl;
}


// ==================================== PointMotion =====================================
std::pair<std::vector<RobotData>, CentroidData> PointMotion(Optimization_parameter &optimization_parameter, Geometry_information geometry_information, std::vector<gtsam::Vector3> covariance_info,
                                                            Utils::Disturbance disturbance_info, std::vector<double> solverer_params, SDF_s sdf_s) {

    gtsam::Values init_values;

    // create arrays to store reference poses
    Trajectory::ref_trajv X_ref(optimization_parameter, geometry_information); // change Trajectory.h

    // optimization parameter
    int Ts_translation = optimization_parameter.time_for_translation;
    int Ts_robot_rotation = optimization_parameter.time_for_robot_rotation;
    int Ts_centroid_rotation = optimization_parameter.time_for_centroid_rotation;
    int max_states = optimization_parameter.nr_of_steps;
    int nr_of_robots = optimization_parameter.nr_of_robots;
    double wheel_speed_limit = optimization_parameter.wheel_speed_limit;
    double proportional_gain = optimization_parameter.proportional_gain;
    double integral_gain = optimization_parameter.integral_gain;
    double derivative_gain = optimization_parameter.derivative_gain;

    auto ROS_Enabled = optimization_parameter.run_ros_nodes;
    int reference_trajectory_type = optimization_parameter.reference_trajectory_type;
    int push_box = optimization_parameter.push_box;
    int separation_of_action = optimization_parameter.separation_of_action;
    int use_sdf = optimization_parameter.obstacle_avoidance;
    int adjust_centroid_orientation = optimization_parameter.adjust_centroid_orientation;
    int use_custom_trajectory = optimization_parameter.use_custom_trajectory;

    auto print_fg_factors = optimization_parameter.print_fg_factors;
    auto print_fg_initial_values = optimization_parameter.print_fg_initial_values;
    auto print_fg_iterated_results = optimization_parameter.print_fg_iterated_results;
    auto print_ref_traj = optimization_parameter.print_ref_traj;
    auto print_modelled_traj = optimization_parameter.print_modelled_traj;
    auto use_gazebo = optimization_parameter.gazebo;
    gtsam::Pose2 ref_traj_end_pose = optimization_parameter.ref_traj_end_pose;
    gtsam::Pose2 ref_traj_center_of_rotation = optimization_parameter.ref_traj_center_of_rotation;
    double ref_traj_angular_frequency = optimization_parameter.ref_traj_angular_frequency;
    double ref_traj_amplitude = optimization_parameter.ref_traj_amplitude;
    double pid_error_threshold = optimization_parameter.pid_error_threshold;
    gtsam::Pose2 custom_reference_trajectory[21];
    for (int i = 0; i <= 20; i++) {
        custom_reference_trajectory[i] = optimization_parameter.custom_reference_trajectory[i];
    }
    robot_failed = 10;

    // geometry
    double r = geometry_information.robot_wheel_radius;
    double L = geometry_information.robot_width;

    std::system("clear");
    std::cout << "\n================= Start =================" << std::endl;
    if(use_custom_trajectory) {
        std::cout << "Using custom reference trajectory" << std::endl;
    }
    if(use_sdf) {
        std::cout << "Using SDF" << std::endl;
    }
    if(ROS_Enabled) {
        std::cout << "ROS Enabled" << std::endl;
    }


    std::cout << "=========================================" << std::endl;

    std::vector<RobotData> robots(nr_of_robots);
    CentroidData centroid;

    /* -------------------------------------------------------------------------- */
    /*                             ROS Initialization                             */
    /* -------------------------------------------------------------------------- */
    int k;
    rclcpp::init(0, nullptr);
    if (ROS_Enabled == true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(4000));
    }
    std::vector<std::shared_ptr<VelocityPublisher>> publishers;
    std::vector<std::shared_ptr<GazeboSubscriber>> subscribers;
    auto ref_traj_publisher_node = std::make_shared<RefTrajectoryPublisher>(centroid, use_gazebo);
    // std::vector<std::shared_ptr<ViconSubscriber>> subscribers;

    auto rc_subscriber_node = std::make_shared<GazeboSubscriber>("C");
    // auto rc_subscriber_node = std::make_shared<ViconSubscriber>("C");
    int i = 0;
    for (auto &&robot : robots) {
        robot.robot_id = i + 1;
        int robot_id_sub = robot.robot_id;
        // std::cout << "robot " << robot_id_sub << std::endl;
        auto subscriber_node = std::make_shared<GazeboSubscriber>("0" + std::to_string(robot_id_sub));
        // auto subscriber_node = std::make_shared<ViconSubscriber>("0" + std::to_string(robot_id_sub));
        subscribers.push_back(subscriber_node);
        i = i + 1;
    }

    for (auto &&robot : robots) {
        // publishers
        int robot_id_sub = robot.robot_id;
        auto publisher_node = std::make_shared<VelocityPublisher>("0" + std::to_string(robot_id_sub));
        // publisher_executor.add_node(publisher_node);
        publishers.push_back(publisher_node);
    }

    if (ROS_Enabled) {
        // std::cout << "Stopping robots" << std::endl;
        for (auto &&publisher : publishers) {
            publisher->publish(0, 0);
        }
        for (auto &&subscriber : subscribers) {
            rclcpp::spin_some(subscriber);
            rclcpp::spin_some(subscriber);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        rclcpp::spin_some(rc_subscriber_node);
        for (auto &&subscriber : subscribers) {
            rclcpp::spin_some(subscriber);
            rclcpp::spin_some(subscriber);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        rclcpp::spin_some(rc_subscriber_node);
        rclcpp::spin_some(rc_subscriber_node);

        std::cout << "Subscribing to initial robot positions. . . " << std::endl;
        for (int i = 0; i < robot_poses.size(); i++) {
            while (robot_poses[i].x == 0 && robot_poses[i].y == 0 && robot_poses[i].yaw == 0) {
                rclcpp::spin_some(subscribers[i]);
                rclcpp::spin_some(subscribers[i]);
                rclcpp::spin_some(rc_subscriber_node);
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            robots[i].X_k_real.push_back(gtsam::Pose2(robot_poses[i].x, robot_poses[i].y, robot_poses[i].yaw));
            robots[i].X_k_modelled.push_back(gtsam::Pose2(robot_poses[i].x, robot_poses[i].y, robot_poses[i].yaw));
            // std::cout << "robot " << i + 1 << " position: " << robot_poses[i].x << ", " << robot_poses[i].y << ", " << robot_poses[i].yaw << std::endl;
        }
        // while(centroid_pose.x == 0 && centroid_pose.y == 0 && centroid_pose.yaw == 0) {
            rclcpp::spin_some(rc_subscriber_node);
            rclcpp::spin_some(rc_subscriber_node);
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        // }
        centroid.X_k_real.push_back(gtsam::Pose2(centroid_pose.x, centroid_pose.y, centroid_pose.yaw));
        centroid.X_k_modelled.push_back(gtsam::Pose2(centroid_pose.x, centroid_pose.y, centroid_pose.yaw));
    }
    gtsam::Pose2 ref_traj_start_pose;
    if (ROS_Enabled) {
        ref_traj_start_pose = gtsam::Pose2(centroid_pose.x, centroid_pose.y, centroid_pose.yaw);
        Utils::calcualte_R_and_theta_vectors(centroid, robots, geometry_information);
    } else {
            ref_traj_start_pose = optimization_parameter.ref_traj_start_pose;
    }

    X_ref.set_initial(ref_traj_start_pose);

    switch (reference_trajectory_type) {
    case 0: {
        X_ref.set_final(ref_traj_end_pose);
        X_ref.set_overall_dist();
    } break;

    case 1: {
        X_ref.set_final(ref_traj_end_pose);
        X_ref.set_overall_angle();
        X_ref.set_amplitude(ref_traj_amplitude);
        X_ref.set_angular_freq(ref_traj_angular_frequency);
    } break;

    case 2: {
        X_ref.set_center_of_rotation(ref_traj_center_of_rotation);
    } break;
    }

    X_ref.fill();

    gtsam::Vector2 covariance_ternary_vec_2D(covariance_info[2][0], covariance_info[2][1]);
    gtsam::Vector1 covariance_obs_vec_1D(covariance_info[3][0]);
    gtsam::Vector2 covariance_obs_vec_2D(covariance_info[3][0], covariance_info[3][1]);
    gtsam::Vector2 covariance_speed_vec_2D(covariance_info[1][0], covariance_info[1][1]);
    gtsam::Vector1 covariance_speed_vec_1D(covariance_info[1][0]);

    NoiseModels noise_models;
    noise_models = {
        noiseModel::Diagonal::Sigmas(covariance_info[0]),
        // noiseModel::Diagonal::Sigmas(covariance_speed_vec_1D),
        noiseModel::Diagonal::Sigmas(covariance_info[1]),
        noiseModel::Diagonal::Sigmas(covariance_ternary_vec_2D),
        noiseModel::Diagonal::Sigmas(covariance_obs_vec_1D),
        noiseModel::Diagonal::Sigmas(covariance_info[4]),
    };

    std::vector<gtsam::Pose2> custom_reference_trajectory_vec = optimization_parameter.custom_reference_trajectory_vector;
    for (int k = 0; k <= max_states; k++) {
        gtsam::Pose2 pose_ref;
        if (use_custom_trajectory) {
            // pose_ref = custom_reference_trajectory[k];
            pose_ref = custom_reference_trajectory_vec[k];
        } else {
            pose_ref = X_ref.get_ref_pose(k);
        }
        centroid.X_k_ref.push_back(pose_ref);
    }

    if (print_ref_traj) {
        std::cout << "\nCentroid Reference Trajectory" << std::endl;
        Utils::printPoseVector(centroid.X_k_ref);
    }

    for (int k = 0; k < max_states; k++) {
        gtsam::Pose2 ref_control_input = MotionModelArc::centroid_solveForControl(centroid.X_k_ref[k], centroid.X_k_ref[k + 1], optimization_parameter, geometry_information);
        centroid.U_k_ref.push_back(ref_control_input);
    }

    // initialize robot positions and speeds
    for (auto &&robot : robots) {
        double R_i = geometry_information.distance_to_robot[robot.robot_id - 1];
        double th_i = geometry_information.angle_to_robot[robot.robot_id - 1];
        if (!ROS_Enabled) {
            robot.X_k_modelled.push_back(Geometry::solveForRobotPose(centroid.X_k_ref[0], R_i, th_i));
            robot.X_k_real.push_back(Geometry::solveForRobotPose(centroid.X_k_ref[0], R_i, th_i));
            // std::cout << "robot " << i + 1 << " position" << robot.X_k_real[0].x() << ", " << robot.X_k_real[0].y() << ", " << robot.X_k_real[0].theta() << std::endl;
        }
    }
    if (!ROS_Enabled) {
        centroid.X_k_modelled.push_back(centroid.X_k_ref[0]);
        centroid.X_k_real.push_back(centroid.X_k_ref[0]);
    }
 
    centroid.prev_X_k_optimized = centroid.X_k_ref;
    centroid.prev_U_k_optimized = centroid.U_k_ref;

    Benchmark benchmark;

    std::vector<std::string> solverTypes = {
                    "MULTIFRONTAL_CHOLESKY",
                    "MULTIFRONTAL_QR",
                    "SEQUENTIAL_CHOLESKY",
                    "SEQUENTIAL_QR",
                    "CHOLMOD"};
    gtsam::LevenbergMarquardtParams parameters;
    // gtsam::GaussNewtonParams parameters;
    // gtsam::DoglegParams parameters;
    // gtsam::NonlinearOptimizerParams parameters;
    parameters.setMaxIterations(static_cast<int>(solverer_params[0]));
    parameters.setRelativeErrorTol(solverer_params[1]);
    parameters.setAbsoluteErrorTol(solverer_params[2]);
    parameters.setErrorTol(solverer_params[3]);

    int index = static_cast<int>(solverer_params[4]);
    parameters.setLinearSolverType(index >= 0 && index < solverTypes.size() ? solverTypes[index] : "CHOLMOD");
    parameters.setOrderingType("METIS");
    // set LM params
    parameters.setlambdaUpperBound(optimization_parameter.lambdaUpperBound);
    parameters.setlambdaLowerBound(optimization_parameter.lambdaLowerBound);
    parameters.setlambdaInitial(optimization_parameter.lambdaInitial);
    parameters.setlambdaFactor(optimization_parameter.lambdaFactor);
    parameters.setUseFixedLambdaFactor(optimization_parameter.useFixedLambdaFactor);
    parameters.setDiagonalDamping(optimization_parameter.diagonalDamping);

    // print parameters
    std::cout << "=========================================" << std::endl;
    std::cout << "Solver Parameters" << std::endl;
    std::cout << "=========================================" << std::endl;

    benchmark.start();
    for (k = 0; k < max_states; k++) {

        std::cout << "\n---------------------------------" << std::endl;
        std::cout << "iteration " << k << std::endl;

        gtsam::Pose2 disturbance_pose;
        if ((k == disturbance_info.pose_number) && (!ROS_Enabled)) {
            if (disturbance_info.axis == 0 || disturbance_info.axis == 1) {
                double disturbance = (disturbance_info.axis == 0) ? disturbance_info.disturbance_value : 0.5;
                double disturbance_y = (disturbance_info.axis == 1) ? disturbance_info.disturbance_value : -0.5;
                disturbance_pose = gtsam::Pose2(disturbance, disturbance_y, 0);
            }
        }

        NonlinearFactorGraph graph;
        // Unpack directly into graph and init_values
        std::tie(graph, init_values) = MultiRobotFG(k, optimization_parameter, robots, centroid, noise_models, geometry_information, sdf_s);

        gtsam::LevenbergMarquardtOptimizer optimizer(graph, init_values, parameters);
        // gtsam::GaussNewtonOptimizer optimizer(graph, init_values, parameters);
        // gtsam::DoglegOptimizer optimizer(graph, init_values, parameters);
        optimizer.optimize();
        gtsam::Values result = optimizer.values();

        // Conversion and assignment
        centroid.X_k_fg = Utils::valuesToPose_mod(result, k, optimization_parameter, 0);
        centroid.all_fg_poses.push_back(centroid.X_k_fg);
        centroid.U_k_fg = Utils::valuesToVelocity_mod(result, k, optimization_parameter, 0);

        // publish updated reference trajectory
        if (ROS_Enabled) {
            ref_traj_publisher_node->publish();
        }

        result.clear();

        std::vector<gtsam::Pose2> X_modelled_CentroidRotation_RobotArc_vector;
        std::vector<gtsam::Pose2> X_modelled_CentroidTranslation_RobotTranslation_vector;
        
        double target_orientation_robot;

        for (auto &&robot : robots) {
            gtsam::Pose2 X_modelled_CentroidRotation_RobotRotation, X_modelled_CentroidRotation_RobotArc;
            gtsam::Pose2 X_modelled_CentroidTranslation_RobotRotation, X_modelled_CentroidTranslation_RobotTranslation;

            if (adjust_centroid_orientation) {
                /* -------------------------------------------------------------------------- */
                /*                      Centroid Rotation: Rotate Robots                      */
                /* -------------------------------------------------------------------------- */

                std::vector<std::pair<gtsam::Vector2, double>> rotation_and_arc_speeds;


                    rotation_and_arc_speeds = MotionModelArc::robot_solveForControl_CentroidRotation(centroid.U_k_fg[k], centroid.X_k_real[k], robot.X_k_real[k], robot.robot_id, optimization_parameter, geometry_information);

                gtsam::Vector2 robot_rotation_control_effort = rotation_and_arc_speeds[0].first;
                robot.target_orientation = rotation_and_arc_speeds[0].second;
                gtsam::Vector2 robot_arc_control_effort = rotation_and_arc_speeds[1].first;
                robot.U_k_fg_CentroidRotation_RobotRotation.push_back(robot_rotation_control_effort);
                robot.U_k_fg_CentroidRotation_RobotArc.push_back(robot_arc_control_effort);

                gtsam::Pose2 robot_rotation_control_effort_pose2(robot_rotation_control_effort.x(), robot_rotation_control_effort.y(), 0); // represent as pose2
                gtsam::Pose2 robot_arc_control_effort_pose2(robot_arc_control_effort.x(), robot_arc_control_effort.y(), 0);
                robot.velocity_pose_CentroidRotation_RobotRotation.push_back(robot_rotation_control_effort_pose2);
                robot.velocity_pose_CentroidRotation_RobotArc.push_back(robot_arc_control_effort_pose2);

                // Position of robots after they rotate in place
                if (!ROS_Enabled) {
                    X_modelled_CentroidRotation_RobotRotation = MotionModelArc::robot_solveForNextPose(robot.X_k_real[k], robot.velocity_pose_CentroidRotation_RobotRotation[k], optimization_parameter, geometry_information);
                    X_modelled_CentroidRotation_RobotArc = MotionModelArc::robot_solveForNextPose(X_modelled_CentroidRotation_RobotRotation, robot.velocity_pose_CentroidRotation_RobotArc[k], optimization_parameter, geometry_information);
                    X_modelled_CentroidRotation_RobotArc_vector.push_back(X_modelled_CentroidRotation_RobotArc);
                } else {
                    publishers[robot.robot_id - 1]->publish(0, 0);
                }
                

            } else {
                X_modelled_CentroidRotation_RobotArc = robot.X_k_real[k];
                X_modelled_CentroidRotation_RobotArc_vector.push_back(X_modelled_CentroidRotation_RobotArc);
            }
        }

        if (ROS_Enabled) {
            if (use_gazebo)
                std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // give time for the robots to have stopped from before
            
            std::vector<std::thread> threads;
            for (const auto &robot : robots)
            {
                if (robot.robot_id == robot_failed)
                {
                    std::cout << "skipped" << std::endl;
                }
                else
                {
                    threads.push_back(std::thread(
                        robotController,
                        robot,
                        subscribers[robot.robot_id - 1],
                        publishers[robot.robot_id - 1],
                        proportional_gain,
                        pid_error_threshold,
                        wheel_speed_limit));
                }
            }
            for (auto&& thread : threads) {
                thread.join();
            }
            for(auto&& robot: robots) {
                publishers[robot.robot_id - 1]->publish(0, 0);
            }
        }
        
        /* -------------------------------------------------------------------------- */
        /*                        Centroid Rotation: Arc Motion                       */
        /* -------------------------------------------------------------------------- */
        if (!ROS_Enabled) {
            for (auto &&robot : robots) {
                gtsam::Pose2 X_after_rotation = MotionModelArc::robot_solveForNextPose(robot.X_k_real[k], robot.velocity_pose_CentroidRotation_RobotRotation[k], optimization_parameter, geometry_information);
                // std::cout << "X after centroid rotation: robot rotation : Robot (" << robot.robot_id << ") " << X_after_rotation.x() << ", " << X_after_rotation.y() << ", " << X_after_rotation.theta() << std::endl;
                double Vl = robot.U_k_fg_CentroidRotation_RobotArc[k].x();
                double Vr = robot.U_k_fg_CentroidRotation_RobotArc[k].y();
                double V = (Vl + Vr) * r / 2;
                double w = (-Vl + Vr) * r / L;
                // std::cout << "Centroid Rotation: Robot (" << robot.robot_id << ") Arc (V, w) " << V << ", " << w << std::endl;
            }
            for (auto &&robot : robots) {
                gtsam::Pose2 X_modelled_CentroidRotation_RobotArc = X_modelled_CentroidRotation_RobotArc_vector[robot.robot_id - 1];
                // std::cout << "X after centroid rotation: Robot (" << robot.robot_id << ") " << X_modelled_CentroidRotation_RobotArc.x() << ", " << X_modelled_CentroidRotation_RobotArc.y() << ", " << X_modelled_CentroidRotation_RobotArc.theta() << std::endl;
            }
        }
        if (ROS_Enabled) {

            if(use_gazebo)
                std::this_thread::sleep_for(std::chrono::milliseconds(500)); // give time for the robots to have stopped from before
                

            for (auto &&robot : robots) {
                publishers[robot.robot_id - 1]->publish(0, 0);
            }

            rclcpp::spin_some(rc_subscriber_node);
            rclcpp::spin_some(rc_subscriber_node);
            gtsam::Pose2 centroid_pose2 = gtsam::Pose2(centroid_pose.x, centroid_pose.y, centroid_pose.yaw);
            bool should_rotate = false;
            std::vector<std::pair<double, double>> controls;
            for (auto &&robot : robots) {
                // Update robot positions after they rotated
                rclcpp::spin_some(subscribers[robot.robot_id - 1]);
                rclcpp::spin_some(subscribers[robot.robot_id - 1]);
                gtsam::Pose2 X_after_rotation = gtsam::Pose2(robot_poses[robot.robot_id - 1].x, robot_poses[robot.robot_id - 1].y, robot_poses[robot.robot_id - 1].yaw);
                // std::cout << "X after centroid rotation: robot rotation : Robot (" << robot.robot_id << ") " << X_after_rotation.x() << ", " << X_after_rotation.y() << ", " << X_after_rotation.theta() << std::endl;
                std::vector<std::pair<gtsam::Vector2, double>> rotation_and_arc_speeds;

                    rotation_and_arc_speeds = MotionModelArc::robot_solveForControl_CentroidRotation(centroid.U_k_fg[k], centroid_pose2, X_after_rotation, robot.robot_id, optimization_parameter, geometry_information);
                                
                gtsam::Vector2 robot_arc_control_effort = rotation_and_arc_speeds[1].first;
                robot.U_k_fg_CentroidRotation_RobotArc[k] = robot_arc_control_effort; // update the arc control effort

                double Vl = robot.U_k_fg_CentroidRotation_RobotArc[k].x();
                double Vr = robot.U_k_fg_CentroidRotation_RobotArc[k].y();
                double V = (Vl + Vr) * r / 2;
                double w = (-Vl + Vr) * r / L;
                controls.push_back(std::pair(V, w));
                if(w != 0) {
                    should_rotate = true;
                }
            }
            for(auto&& robot: robots) {
                double V = controls[robot.robot_id - 1].first;
                double w = controls[robot.robot_id - 1].second;
                publishers[robot.robot_id - 1]->publish(V, w);
                std::cout << "Centroid Rotation: Robot (" << robot.robot_id << ") Arc (V, w) " << V << ", " << w << std::endl;
            }
            if(should_rotate){
            std::this_thread::sleep_for(std::chrono::milliseconds(Ts_centroid_rotation * 1000));
            }
            for(auto&& robot: robots) {
                publishers[robot.robot_id - 1]->publish(0, 0);
            }
            for (auto &&robot : robots) {
                rclcpp::spin_some(subscribers[robot.robot_id - 1]);
                rclcpp::spin_some(subscribers[robot.robot_id - 1]);
                gtsam::Pose2 X_modelled_CentroidRotation_RobotArc = gtsam::Pose2(robot_poses[robot.robot_id - 1].x, robot_poses[robot.robot_id - 1].y, robot_poses[robot.robot_id - 1].yaw);
                X_modelled_CentroidRotation_RobotArc_vector.push_back(X_modelled_CentroidRotation_RobotArc);
                // std::cout << "X after centroid rotation: Robot (" << robot.robot_id << ") " << X_modelled_CentroidRotation_RobotArc.x() << ", " << X_modelled_CentroidRotation_RobotArc.y() << ", " << X_modelled_CentroidRotation_RobotArc.theta() << std::endl;
            }
            rclcpp::spin_some(rc_subscriber_node);
        }

        /* -------------------------------------------------------------------------- */
        /*                            Centroid Translation                            */
        /* -------------------------------------------------------------------------- */
        for (auto &&robot : robots) {
            // std::cout << "X_k_real before centroid translation " << robot.X_k_real[k].x() << ", " << robot.X_k_real[k].y() << ", " << robot.X_k_real[k].theta() << std::endl;
            std::vector<std::pair<gtsam::Vector2, double>> combined_speed_CentroidTranslation;

                combined_speed_CentroidTranslation = MotionModelArc::robot_solveForControl_CentroidTanslation(centroid.U_k_fg[k], centroid.X_k_real[k], X_modelled_CentroidRotation_RobotArc_vector[robot.robot_id - 1], robot.robot_id, optimization_parameter, geometry_information);

            gtsam::Vector2 robot_rotation_control_effort = combined_speed_CentroidTranslation[0].first;
            robot.target_orientation = combined_speed_CentroidTranslation[0].second;
            gtsam::Vector2 robot_translation_control_effort = combined_speed_CentroidTranslation[1].first;
            robot.U_k_fg_CentroidTranslation_RobotRotation.push_back(robot_rotation_control_effort);
            robot.U_k_fg_CentroidTranslation_RobotTranslation.push_back(robot_translation_control_effort);

            // convert to pose
            gtsam::Pose2 robot_rotation_control_effort_pose2(robot_rotation_control_effort.x(), robot_rotation_control_effort.y(), 0);
            gtsam::Pose2 robot_translation_control_effort_pose2(robot_translation_control_effort.x(), robot_translation_control_effort.y(), 0);
            robot.velocity_pose_CentroidTranslation_RobotRotation.push_back(robot_rotation_control_effort_pose2);
            robot.velocity_pose_CentroidTranslation_RobotTranslation.push_back(robot_translation_control_effort_pose2);

            gtsam::Pose2 X_modelled_CentroidTranslation_RobotRotation = MotionModelArc::robot_solveForNextPose(X_modelled_CentroidRotation_RobotArc_vector[robot.robot_id - 1], robot.velocity_pose_CentroidTranslation_RobotRotation[k], optimization_parameter, geometry_information);
            gtsam::Pose2 X_modelled_CentroidTranslation_RobotTranslation = MotionModelArc::robot_solveForNextPose(X_modelled_CentroidTranslation_RobotRotation, robot.velocity_pose_CentroidTranslation_RobotTranslation[k], optimization_parameter, geometry_information);
            X_modelled_CentroidTranslation_RobotTranslation = disturbance_pose.compose(X_modelled_CentroidTranslation_RobotTranslation);
            robot.X_k_modelled.push_back(X_modelled_CentroidTranslation_RobotTranslation);
            if (!ROS_Enabled) {
                X_modelled_CentroidTranslation_RobotTranslation_vector.push_back(X_modelled_CentroidTranslation_RobotTranslation);
            }
        }

        if (ROS_Enabled) {
            if(use_gazebo)
                std::this_thread::sleep_for(std::chrono::milliseconds(500)); // give time for the robots to have stopped from before
            std::vector<std::thread> threads;
            for (const auto &robot : robots)
            {
                if (robot.robot_id == robot_failed)
                {
                    std::cout << "skipped" << std::endl;
                }
                else
                {
                    threads.push_back(std::thread(
                        robotController,
                        robot,
                        subscribers[robot.robot_id - 1],
                        publishers[robot.robot_id - 1],
                        proportional_gain,
                        pid_error_threshold,
                        wheel_speed_limit));
                }
            }
            for (auto &thread : threads) {
                thread.join();
            }
            for(auto&& robot: robots) {
                publishers[robot.robot_id - 1]->publish(0, 0);
            }
        }
        
        if (ROS_Enabled) {
            if(use_gazebo)
                std::this_thread::sleep_for(std::chrono::milliseconds(500)); // give time for the robots to have stopped from before
            std::vector<std::pair<double, double>> controls;
            for (auto &&robot : robots) {
                publishers[robot.robot_id - 1]->publish(0, 0);
                // subscribe to pose after rotation
                rclcpp::spin_some(subscribers[robot.robot_id - 1]);
                rclcpp::spin_some(subscribers[robot.robot_id - 1]);
                gtsam::Pose2 X_after_rotation = gtsam::Pose2(robot_poses[robot.robot_id - 1].x, robot_poses[robot.robot_id - 1].y, robot_poses[robot.robot_id - 1].yaw);
                // Perform translation
                double Vl = robot.U_k_fg_CentroidTranslation_RobotTranslation[k].x();
                double Vr = robot.U_k_fg_CentroidTranslation_RobotTranslation[k].y();
                double V = (Vl + Vr) * r / 2;
                double w = (-Vl + Vr) * r / L;
                controls.push_back(std::pair(V, w));
            }
            for(auto&& robot: robots) {
                double V = controls[robot.robot_id - 1].first;
                double w = controls[robot.robot_id - 1].second;
                publishers[robot.robot_id - 1]->publish(V, w);
                // std::cout << "Centroid Translation: Robot (" << robot.robot_id << ") Translation (V, w) " << V << ", " << w << std::endl;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(Ts_translation * 1000));
            for(auto&& robot: robots) {
            publishers[robot.robot_id - 1]->publish(0, 0);
            }

            rclcpp::spin_some(rc_subscriber_node);
            rclcpp::spin_some(rc_subscriber_node);
            for (auto &&robot : robots) {
                // stop
                publishers[robot.robot_id - 1]->publish(0, 0);
                // subscribe to pose after translation
                rclcpp::spin_some(subscribers[robot.robot_id - 1]);
                rclcpp::spin_some(subscribers[robot.robot_id - 1]);
                X_modelled_CentroidTranslation_RobotTranslation_vector.push_back(gtsam::Pose2(robot_poses[robot.robot_id - 1].x, robot_poses[robot.robot_id - 1].y, robot_poses[robot.robot_id - 1].yaw));
            }
        } else {
            for (auto &&robot : robots) {
                double Vl = robot.U_k_fg_CentroidTranslation_RobotTranslation[k].x();
                double Vr = robot.U_k_fg_CentroidTranslation_RobotTranslation[k].y();
                double V = (Vl + Vr) * r / 2;
                double w = (-Vl + Vr) * r / L;
                // std::cout << "Centroid Translation: Robot (" << robot.robot_id << ") Translation (V, w) " << V << ", " << w << std::endl;
            }
        }

        gtsam::Pose2 X_k_modelled_centroid = Geometry::solveForCentroidPose(robots[0].X_k_modelled[k + 1], robots[1].X_k_modelled[k + 1], geometry_information.distance_to_robot[0], geometry_information.angle_to_robot[0]);
        centroid.X_k_modelled.push_back(X_k_modelled_centroid);

        if (!ROS_Enabled) {
            for (auto &&robot : robots) {
                robot.X_k_real.push_back(robot.X_k_modelled[k + 1]);
            }
            centroid.X_k_real.push_back(X_k_modelled_centroid);
        } else {
            for (auto &&subscriber : subscribers) {
                rclcpp::spin_some(subscriber);
                rclcpp::spin_some(subscriber);
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
            rclcpp::spin_some(rc_subscriber_node);
            rclcpp::spin_some(rc_subscriber_node);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            for (auto &&robot : robots) {
                robot.X_k_real.push_back(gtsam::Pose2(robot_poses[robot.robot_id - 1].x, robot_poses[robot.robot_id - 1].y, robot_poses[robot.robot_id - 1].yaw));
            }
            centroid.X_k_real.push_back(gtsam::Pose2(centroid_pose.x, centroid_pose.y, centroid_pose.yaw));
        }
        

        centroid.prev_X_k_optimized = centroid.X_k_fg;
        centroid.all_fg_velocities.push_back(centroid.U_k_fg);
        
    } // end of main loop

    // ====================================================================================================

    benchmark.end();

   
    if (print_modelled_traj) {
        std::cout << "\nCentroid Modelled Trajectory" << std::endl;
        Utils::printPoseVector(centroid.X_k_modelled);

        for (auto &&robot : robots) {
            std::cout << "\nRobot " << robot.robot_id << " Modelled Trajectory" << std::endl;
            Utils::printPoseVector(robot.X_k_modelled);
        }
    }
    rclcpp::shutdown();
    benchmark.get_error(centroid);
    centroid.time = benchmark.print_results();
    centroid.collision = false;
    std::pair<bool,std::pair<int, double>> collision = benchmark.check_if_collides(centroid.X_k_real, sdf_s.obstacles, sdf_s.system_radius);
    std::ostringstream oss;
    if (collision.first) {
        oss << "Collision at State: " << std::to_string(collision.second.first) << " (" << centroid.X_k_real[collision.second.first].x() << ", " << centroid.X_k_real[collision.second.first].y() << ") distance to obstacle: " << sqrt(collision.second.second) << "";
        logger.AddLog(LOG_INFO, "%s", oss.str().c_str());
        centroid.collision = true;
    }
    // optimization_parameter.collision_flag = collision.first;

    init_values.clear();
    return std::pair(robots, centroid);
}