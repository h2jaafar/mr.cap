#pragma once

#include <gtsam/geometry/Pose2.h>
// #include "../matplotlibcpp/matplotlibcpp.h"

// namespace plt = matplotlibcpp;
struct obstacle {
    double x;
    double y;
    double w; // weight
    obstacle() : x(0), y(0), w(0) {}
    obstacle(double x_val, double y_val, double w_val) : x(x_val), y(y_val), w(w_val) {}
};

struct SDF_s {
    std::vector<obstacle> obstacles;
    double system_radius; // radius of the robot
    double inv_system_radius; // 1/ robot radius to save computation
    double system_radius_squared; // radius of the robot squared
    double safety_radius; // safety radius
    double sys_radius_safety_radius; // system radius - safety radius
    double sys_radius_safety_radius_squared; // system radius - safety radius squared
    double inv_sys_radius_safety_radius;
};



