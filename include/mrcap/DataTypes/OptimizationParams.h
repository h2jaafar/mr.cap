#pragma once
#include <gtsam/geometry/Pose2.h>

struct Optimization_parameter {

    // parameters
    int time_for_translation;
    int time_for_robot_rotation;
    int time_for_centroid_rotation;
    int nr_of_steps;
    int nr_of_robots;
    int max_nr_of_robots;
    bool collision_flag;
    double wheel_speed_limit;
    double nominal_reference_speed;
    double proportional_gain;
    double integral_gain;
    double derivative_gain;
    double robot_rotation_threshold; // the threshold to determine if robot should do arc rotation or not
    double centroid_rotation_threshold; // the threshold to determine if centroid should do arc rotation or not
    double pid_error_threshold;

    // operating conditions
    int run_ros_nodes;
    int numerical_jacobian;
    int push_box;
    int separation_of_action;
    int obstacle_avoidance;
    int adjust_centroid_orientation;
    int use_custom_trajectory;
    int test;
    int use_mpc;

    // print options
    int print_fg_factors;
    int print_fg_initial_values;
    int print_fg_iterated_results;
    int print_ref_traj;
    int print_modelled_traj;
    int print_velocities; 
    bool gazebo;
    bool save_traj_;
    // reference trajectory parameters
    int reference_trajectory_type;
    gtsam::Pose2 ref_traj_start_pose;
    gtsam::Pose2 ref_traj_end_pose;
    gtsam::Pose2 ref_traj_center_of_rotation;
    double ref_traj_angular_frequency;
    double ref_traj_amplitude;

    double error_scale_ternary;
    gtsam::Pose2 custom_reference_trajectory[21];
    std::vector<gtsam::Pose2> custom_reference_trajectory_vector;

    bool diagonalDamping;
    double lambdaFactor;
    double lambdaInitial;
    double lambdaLowerBound;
    double lambdaUpperBound;
    bool useFixedLambdaFactor;
    std::string verbosityLM;
};