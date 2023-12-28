#pragma once
#include "Trajectory.h"

using namespace std;
using namespace gtsam;
// using namespace boost;

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/nonlinear/NonlinearEquality.h>
#include <gtsam/slam/BoundingConstraint.h>
#include "MotionModelArc.h"
#include "Geometry.h"
#include "DataTypes/Atan2LUT.h"
extern Atan2LUT lut;

class TernaryFactorStateEstimation : public gtsam::NoiseModelFactor3<gtsam::Pose2, gtsam::Pose2, gtsam::Pose2> {
private:
    Optimization_parameter optimization_parameter;
    Geometry_information geometry_information;

    // Precompute the scaled Jacobian matrices for H1, H2, and H3
    static inline void computeScaledJacobians(double s, double dt, gtsam::Matrix& H1, gtsam::Matrix& H2, gtsam::Matrix& H3) {
        H1 = s * (gtsam::Matrix(2, 3) << 1, 0, 0, 0, 1, 0).finished();
        H2 = s * (gtsam::Matrix(2, 3) << dt, 0, 0, 0, dt, 0).finished();
        H3 = s * (gtsam::Matrix(2, 3) << -1, 0, 0, 0, -1, 0).finished();
    }

public:
    TernaryFactorStateEstimation(Optimization_parameter optimization_param, gtsam::Key key1, gtsam::Key key2, gtsam::Key key3, Geometry_information geometry_info, const gtsam::SharedNoiseModel& model)
        : gtsam::NoiseModelFactor3<gtsam::Pose2, gtsam::Pose2, gtsam::Pose2>(model, key1, key2, key3) {
        optimization_parameter = optimization_param;
        geometry_information = geometry_info;
    }

    gtsam::Vector evaluateError(const gtsam::Pose2& X_now, const gtsam::Pose2& U_now, const gtsam::Pose2& X_next, boost::optional<gtsam::Matrix&> H1 = boost::none, boost::optional<gtsam::Matrix&> H2 = boost::none, boost::optional<gtsam::Matrix&> H3 = boost::none) const {
        double dt = optimization_parameter.time_for_translation;

        gtsam::Matrix H1_, H2_, H3_;
        computeScaledJacobians(optimization_parameter.error_scale_ternary, dt, H1_, H2_, H3_);

        if (H1)
            *H1 = H1_;
        if (H2)
            *H2 = H2_;
        if (H3)
            *H3 = H3_;

        gtsam::Pose2 X_next_calculated = MotionModelArc::centroid_solveForNextPose(X_now, U_now, optimization_parameter, geometry_information);
        gtsam::Pose2 error = X_next.between(X_next_calculated);
        return (gtsam::Vector(2) << optimization_parameter.error_scale_ternary * error.x(),
                                    optimization_parameter.error_scale_ternary * error.y()).finished();
    }
};



// obstacle avoidance factor
// error will be X_next's value on the signed distance field, trying to minimize it to 0. It is a unary factor
class UnaryFactorObstacleAvoidance : public gtsam::NoiseModelFactor1<gtsam::Pose2> {
private:
    obstacle obs_;
    double r_;
    double ir_;

public:
    // Constructor
    UnaryFactorObstacleAvoidance(gtsam::Key X_next_key, const obstacle& obs, const double& r, const double& ir, const gtsam::SharedNoiseModel &model)
        : gtsam::NoiseModelFactor1<gtsam::Pose2>(model, X_next_key), obs_(obs) , r_(r), ir_(ir){
    }

    gtsam::Vector evaluateError(const gtsam::Pose2 &X_next, boost::optional<gtsam::Matrix &> H1 = boost::none) const {
        if (H1) {
            double x_diff = X_next.x() - obs_.x;
            double y_diff = X_next.y() - obs_.y;
            double distance_squared = x_diff * x_diff + y_diff * y_diff;
            double denom = r_ * std::sqrt(distance_squared);

            // Combine the partial derivatives into a Jacobian matrix
            // The creation of the matrix and the assignment is done in one step
            if (distance_squared < r_ * r_) {
                *H1 = (gtsam::Matrix(1, 3) << -(x_diff) / denom, // d1
                                            -(y_diff) / denom, // d2
                                            0).finished();    // d3 is always zero
            }
            else {
                *H1 = (gtsam::Matrix(1, 3) << 0, 0, 0).finished();
            }
        }

        // Compute squared distance to avoid sqrt
        double distance_squared = (X_next.x() - obs_.x) * (X_next.x() - obs_.x) +
                                  (X_next.y() - obs_.y) * (X_next.y() - obs_.y);
        // Use squared robot radius for comparison
        double error = 0;
        if (distance_squared < r_ * r_) {
            error = 1 - std::sqrt(distance_squared) * ir_; // Use pre-computed inverse of robot radius
        }
        return (gtsam::Vector(1) << error).finished();
    }
};

