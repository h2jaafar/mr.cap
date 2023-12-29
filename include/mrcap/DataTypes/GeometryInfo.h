#pragma once
#include <vector>

struct Geometry_information {
    double robot_wheel_radius;
    double robot_width;
    std::vector<double> distance_to_robot;  // distance from centroid to robot
    std::vector<double> angle_to_robot;     // angle from x = 0 to robot
    double max_velocity;
};