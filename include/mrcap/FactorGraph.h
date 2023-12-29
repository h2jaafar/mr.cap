#pragma once

// Logger
extern ImGuiLogger g_Logger;

struct NoiseModels {
    gtsam::noiseModel::Diagonal::shared_ptr covariance_X;
    gtsam::noiseModel::Diagonal::shared_ptr covariance_U;
    gtsam::noiseModel::Diagonal::shared_ptr covariance_ternary;
    gtsam::noiseModel::Diagonal::shared_ptr covariance_obs;
    gtsam::noiseModel::Diagonal::shared_ptr covariance_priors;
};

std::pair<gtsam::NonlinearFactorGraph, gtsam::Values> MultiRobotFG(
    int k, const Optimization_parameter& optimization_parameter, // pass by const reference
    const std::vector<RobotData>& robots_data, const CentroidData& centroid, // pass by const reference
    const NoiseModels& covariance_information, const Geometry_information& geometry_information, const SDF_s& sdf_) { // pass by const reference

    gtsam::Values init_values;
    gtsam::NonlinearFactorGraph graph;

    int use_sdf = optimization_parameter.obstacle_avoidance;
    auto& covariance_X = covariance_information.covariance_X;
    auto& covariance_U = covariance_information.covariance_U;
    auto& covariance_ternary = covariance_information.covariance_ternary;
    auto& covariance_obs = covariance_information.covariance_obs;
    auto& covariance_anchors = covariance_information.covariance_priors;

    int max_states = optimization_parameter.nr_of_steps;
    std::vector<obstacle> modifiable_obstacles = sdf_.obstacles;
    
    for (int j = k; j < max_states; ++j) {
        for(auto&& obstacle : modifiable_obstacles) {
            // check if the current trajectory will collide with the obstacle
            // make the distance calculation faster by using x*x not std::pow
            double distance_to_obstacle = (centroid.X_k_ref[j].x() - obstacle.x) * (centroid.X_k_ref[j].x() - obstacle.x) + (centroid.X_k_ref[j].y() - obstacle.y) * (centroid.X_k_ref[j].y() - obstacle.y);
            // if so, set the weight to 1.0
            obstacle.w = (distance_to_obstacle < (sdf_.sys_radius_safety_radius_squared)) ? 1.0 : 0.0;
        }

        gtsam::Symbol key_pos_c('C', j);
        gtsam::Symbol key_control_c('U', j);
        gtsam::Symbol key_pos_next_c('C', j + 1);

        if (j == k) {
            graph.add(NonlinearEquality<Pose2>(key_pos_c, centroid.X_k_real[j]));
            init_values.insert(key_pos_c, centroid.X_k_real[j]);
        } else {
            if (use_sdf) {
                for (auto &&obstacle : modifiable_obstacles) {
                    if (obstacle.w > 0.0) {
                        graph.add(boost::make_shared<UnaryFactorObstacleAvoidance>(key_pos_next_c, obstacle, sdf_.sys_radius_safety_radius, sdf_.inv_sys_radius_safety_radius, covariance_obs));
                    }
                }
                graph.add(PriorFactor<Pose2>(key_pos_c, centroid.prev_X_k_optimized[j], covariance_X));
            } else {
                graph.add(PriorFactor<Pose2>(key_pos_c, centroid.X_k_ref[j], covariance_X));
            }
            init_values.insert(key_pos_c, centroid.prev_X_k_optimized[j]);
        }

        graph.add(PriorFactor<Pose2>(key_control_c, centroid.prev_U_k_optimized[j], covariance_U));
        init_values.insert(key_control_c, centroid.prev_U_k_optimized[j]); 
        graph.add(boost::make_shared<TernaryFactorStateEstimation>(optimization_parameter, key_pos_c, key_control_c, key_pos_next_c, geometry_information, covariance_ternary));

        if (j == (max_states - 1)) {
            graph.add(PriorFactor<Pose2>(key_pos_next_c, centroid.X_k_ref[j + 1], covariance_anchors));
            init_values.insert(key_pos_next_c, centroid.prev_X_k_optimized[j + 1]);
        }
    }
    // graph.print("Factor Graph: \n");
    return std::make_pair(graph, init_values);
}
