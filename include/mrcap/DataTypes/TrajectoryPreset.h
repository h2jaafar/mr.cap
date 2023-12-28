#pragma once
#include <gtsam/geometry/Pose2.h>

class TrajectoryPreset {
public:
    std::string name;
    gtsam::Pose2 pose0;
    gtsam::Pose2 pose1;
    gtsam::Pose2 pose2;
    gtsam::Pose2 pose3;
    gtsam::Pose2 pose4;
    gtsam::Pose2 pose5;
    gtsam::Pose2 pose6;
    gtsam::Pose2 pose7;
    gtsam::Pose2 pose8;
    gtsam::Pose2 pose9;
    gtsam::Pose2 pose10;
    gtsam::Pose2 pose11;
    gtsam::Pose2 pose12;
    gtsam::Pose2 pose13;
    gtsam::Pose2 pose14;
    gtsam::Pose2 pose15;
    gtsam::Pose2 pose16;
    gtsam::Pose2 pose17;
    gtsam::Pose2 pose18;
    gtsam::Pose2 pose19;
    gtsam::Pose2 pose20;

    TrajectoryPreset(const std::string &n, gtsam::Pose2 pose_0, gtsam::Pose2 pose_1, gtsam::Pose2 pose_2, gtsam::Pose2 pose_3, gtsam::Pose2 pose_4, gtsam::Pose2 pose_5, 
    gtsam::Pose2 pose_6, gtsam::Pose2 pose_7, gtsam::Pose2 pose_8, gtsam::Pose2 pose_9, gtsam::Pose2 pose_10, gtsam::Pose2 pose_11, gtsam::Pose2 pose_12, gtsam::Pose2 pose_13, 
    gtsam::Pose2 pose_14, gtsam::Pose2 pose_15, gtsam::Pose2 pose_16, gtsam::Pose2 pose_17, gtsam::Pose2 pose_18, gtsam::Pose2 pose_19, gtsam::Pose2 pose_20)
        : name(n), pose0(pose_0), pose1(pose_1), pose2(pose_2), pose3(pose_3), pose4(pose_4), pose5(pose_5), pose6(pose_6), pose7(pose_7), pose8(pose_8), pose9(pose_9), pose10(pose_10), 
        pose11(pose_11), pose12(pose_12), pose13(pose_13), pose14(pose_14), pose15(pose_15), pose16(pose_16), pose17(pose_17), pose18(pose_18), pose19(pose_19), pose20(pose_20) {}
};
