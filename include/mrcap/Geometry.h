#pragma once
#include "Trajectory.h"

namespace Geometry
{
    int sign_robot_geometry (int robot_id, int nr_of_robots)
    {
        int sign = 0;

        switch (nr_of_robots) {
            case 2:
            {
                if (robot_id == 1) {
                    sign = 0;
                }
                else if (robot_id == 2) {
                    sign = 1;
                }
                else {}
            }
        }
        return sign;
    }

    /**
     * @brief 
     * 
     * @param robot_1_pose pose f robot 1
     * @param robot_2_pose pose of robot 2
     * @param robot_1_radius distance between robot 1 and centroid
     * @param robot_1_angle angle of robot 1 wrt the centroid
     * @return gtsam::Pose2 
     */
    gtsam::Pose2 solveForCentroidPose(gtsam::Pose2 robot_1_pose, gtsam::Pose2 robot_2_pose, double robot_1_radius, double robot_1_angle) {
        double system_orientation = Utils::angle_between_points(robot_2_pose, robot_1_pose);
        system_orientation = Utils::ensure_orientation_range(system_orientation);
        robot_1_pose = gtsam::Pose2(robot_1_pose.x(), robot_1_pose.y(), system_orientation);
        gtsam::Pose2 inverse_relation = gtsam::Pose2(robot_1_radius * cos(robot_1_angle), robot_1_radius * sin(robot_1_angle), 0).inverse();
        return gtsam::Pose2(robot_1_pose.compose(inverse_relation));
    }

    /**
     * @brief 
     * 
     * @param centroid_pose centroid pose
     * @param robot_radius distance between robot and centroid
     * @param robot_angle angle of robot wrt the centroid
     * @return gtsam::Pose2 
     */
    gtsam::Pose2 solveForRobotPose(gtsam::Pose2 centroid_pose, double robot_radius, double robot_angle){
        return gtsam::Pose2(centroid_pose.compose(gtsam::Pose2(robot_radius * cos(robot_angle), robot_radius * sin(robot_angle), 0)));
    }

}