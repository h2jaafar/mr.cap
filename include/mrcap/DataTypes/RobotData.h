#pragma once
#include <gtsam/geometry/Pose2.h>

struct RobotData {
    int robot_id;
    double target_orientation;
    std::vector<gtsam::Pose2> X_k_real;
    std::vector<gtsam::Pose2> X_k_modelled;
    std::vector<gtsam::Vector2> U_k_fg_CentroidTranslation_RobotRotation;
    std::vector<gtsam::Vector2> U_k_fg_CentroidTranslation_RobotTranslation;
    std::vector<gtsam::Vector2> U_k_fg_CentroidRotation_RobotRotation;
    std::vector<gtsam::Vector2> U_k_fg_CentroidRotation_RobotArc;
    std::vector<gtsam::Pose2> velocity_pose_CentroidTranslation_RobotRotation;
    std::vector<gtsam::Pose2> velocity_pose_CentroidTranslation_RobotTranslation;
    std::vector<gtsam::Pose2> velocity_pose_CentroidRotation_RobotRotation;
    std::vector<gtsam::Pose2> velocity_pose_CentroidRotation_RobotArc;
    std::vector<std::vector<gtsam::Pose2>> all_velocities_pose_vec_rotation; // not used
    std::vector<std::vector<gtsam::Pose2>> all_velocities_pose_vec_translation; // not used
};
    
struct CentroidData {
    std::vector<gtsam::Pose2> X_k_ref;
    std::vector<gtsam::Pose2> X_k_real;
    std::vector<gtsam::Pose2> X_k_modelled;
    std::vector<gtsam::Pose2> U_k_ref;
    std::vector<gtsam::Pose2> prev_X_k_optimized;
    std::vector<gtsam::Pose2> prev_U_k_optimized;
    std::vector<gtsam::Pose2> X_k_fg;
    std::vector<gtsam::Pose2> U_k_fg;
    std::vector<std::vector<gtsam::Pose2>> all_fg_poses;
    std::vector<std::vector<gtsam::Pose2>> all_fg_velocities;
    std::vector<std::vector<gtsam::Pose2>> X_k_ref_vector;
    bool collision;
    double dist_to_goal;
    double time;
    double deviation;
    double path_length;

};
