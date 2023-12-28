#pragma once
#include <chrono>
#include <sstream>
#include <iomanip>
#include <fstream>
    
#include <numeric>  // for std::iota
#include <cmath>    // for std::pow
#include <algorithm>// for std::adjacent_difference
#include <sys/resource.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>


extern ImGuiLogger logger;

class Benchmark {
public:
    void start() {
        start_time = std::chrono::high_resolution_clock::now();
        start_memory = get_memory_usage();
    }

    void end() {
        end_time = std::chrono::high_resolution_clock::now();
        end_memory = get_memory_usage();
    }

    double calculate_path_length(const std::vector<gtsam::Pose2> &path) const {
        if (path.size() < 2) {
            return 0; // If there's only one pose or none, the path length is zero.
        }

        float total_length = 0;
        auto prev_pose = path.front();
        for (size_t i = 1; i < path.size(); ++i) {
            const auto &pose = path[i];
            float dx = pose.x() - prev_pose.x();
            float dy = pose.y() - prev_pose.y();
            total_length += std::sqrt(dx * dx + dy * dy);
            prev_pose = pose;
        }
        return total_length;
    }

    float calculate_path_smoothness(const std::vector<gtsam::Pose2>& path) const {
        if (path.size() < 3)
            return 0.0f; // Not enough points to calculate smoothness

        std::vector<float> diffs_x(path.size() - 1);
        std::vector<float> diffs_y(path.size() - 1);

        auto prev_pose = path.front();
        auto it_diff_x = diffs_x.begin();
        auto it_diff_y = diffs_y.begin();
        for (const auto& pose : path) {
            *it_diff_x = pose.x() - prev_pose.x();
            *it_diff_y = pose.y() - prev_pose.y();
            ++it_diff_x;
            ++it_diff_y;
            prev_pose = pose;
        }
        float smoothness = 0.0f;
        auto it_x = diffs_x.begin();
        auto it_y = diffs_y.begin();
        for (; it_x != diffs_x.end() && it_y != diffs_y.end(); ++it_x, ++it_y) {
            float ddx = *it_x - *(it_x - 1);
            float ddy = *it_y - *(it_y - 1);
            smoothness += ddx * ddx + ddy * ddy;
        }
        return smoothness;
    }
    void get_error(CentroidData& centroid) {
        auto executed_traj = centroid.X_k_real;
        auto ref_traj = centroid.X_k_ref;
        trial_error = 0;
        float error = 0;
        int robot_id = 0;
        for (size_t k = 0; k < ref_traj.size(); ++k) {
            error += std::sqrt(std::pow(ref_traj[k].x() - executed_traj[k].x(), 2) +
                               std::pow(ref_traj[k].y() - executed_traj[k].y(), 2));
        }
        trial_error = error / ref_traj.size();
        double path_length = calculate_path_length(executed_traj);
        // float path_smoothness = calculate_path_smoothness(executed_traj);
        double distance_from_goal = std::sqrt(std::pow(executed_traj.back().x() - ref_traj.back().x(), 2) +
                                             std::pow(executed_traj.back().y() - ref_traj.back().y(), 2));
        std::ostringstream oss;
        oss << "\t (Robot " << std::setw(2) << robot_id << ") Trial Error:\t"
            << std::setw(11) << std::fixed << std::setprecision(10) << trial_error << "\n";
        oss << "\t (Robot " << std::setw(2) << robot_id << ") Path Length:\t"
            << std::setw(10) << std::fixed << std::setprecision(4) << path_length << " m\n";
        oss << "\t (Robot " << std::setw(2) << robot_id << ") Distance from Goal:\t"
            << std::setw(5) << std::fixed << std::setprecision(3) << distance_from_goal << " m\n";
            

        logger.AddLog(LOG_INFO, "%s", oss.str().c_str());
        centroid.dist_to_goal = distance_from_goal;
        centroid.path_length = path_length;
        centroid.deviation = trial_error;
    }

    void compute_residuals(const gtsam::NonlinearFactorGraph& graph, const gtsam::Values& result, std::vector<gtsam::Vector3> covariance_information) {
        // Compute the residuals
        residuals_prior_U.clear();
        residuals_prior_X.clear();
        residuals_ternary.clear();
        for (const auto& factor : graph) {

            // factor->printKeys("keys");
            gtsam::KeyVector key_vec = factor->keys();

            bool isPriorU = std::any_of(key_vec.begin(), key_vec.end(), [](gtsam::Key key) {
                return gtsam::Symbol(key).chr() == 'U';
            });

            bool isPriorX = std::any_of(key_vec.begin(), key_vec.end(), [](gtsam::Key key) {
                return gtsam::Symbol(key).chr() == 'C';
            });

            gtsam::Vector3 covariance_X = covariance_information[0];
            gtsam::Vector3 covariance_U = covariance_information[1];
            gtsam::Vector3 covariance_ternary = covariance_information[2];

            // Store the errors based on the factor type
            double error = factor->error(result);
            double unwhitened;
            if (isPriorU && !isPriorX) 
            {
                residuals_prior_U.push_back(error);
                unwhitened = error * 2 * pow(covariance_U.x(), 2);
                unwhitened_prior_U.push_back(unwhitened);
            } 
            else if (isPriorX && !isPriorU) 
            {
                residuals_prior_X.push_back(error);
                unwhitened = error * 2 * pow(covariance_X.x(), 2);
                unwhitened_prior_X.push_back(unwhitened);
            } else {
                residuals_ternary.push_back(error);
                unwhitened = error * 2 * pow(covariance_ternary.x(), 2);
                unwhitened_ternary.push_back(unwhitened);
            }
        }
    }

    std::pair<bool, std::pair<int, double>> check_if_collides(const std::vector<gtsam::Pose2>& path, const std::vector<obstacle>& obstacles, const float& system_radius) {
        // check if the current trajectory will collide with the obstacle
        // make the distance calculation faster by using x*x not std::pow
        int collision_index = -1;
        double distance_to_obstacle = 0;
        for (size_t k = 0; k < path.size(); ++k) {
            for (auto&& obstacle : obstacles) {
                distance_to_obstacle = (path[k].x() - obstacle.x) * (path[k].x() - obstacle.x) + (path[k].y() - obstacle.y) * (path[k].y() - obstacle.y);
                // if so, set the weight to 1.0
                if (distance_to_obstacle < (system_radius * system_radius)) {
                    collision_index = k;
                    return std::make_pair(true, std::make_pair(collision_index, distance_to_obstacle));
                }
            }
        }
        return std::make_pair(false, std::make_pair(collision_index, distance_to_obstacle));
    }

    void print_residual_statistics(int k) {
        // Compute the total residual error
        if(k==0)
        {
            MAX_STATES = unwhitened_prior_X.size();
        }
        // print last MAX_STATES of unhwhitened residual vectors
        int printed_index = 0;

        for (int pose = unwhitened_prior_X.size() - MAX_STATES; pose < unwhitened_prior_X.size() && (printed_index <= MAX_STATES-k-1); pose++)
        {
            std::ostringstream oss;
            oss << "\t (Itr. " << std::setw(2) << k << ") X" << printed_index << ": " << "unwhitened (U, X, T):\t (";
            oss << std::setw(14) << std::left << std::sqrt(unwhitened_prior_U[pose])  // unhitened_U
                 << std::setw(14) << std::sqrt(unwhitened_prior_X[pose]) << std::setw(14) // !todo change u from -
                << std::sqrt(unwhitened_ternary[pose]) << " )";
            logger.AddLog(LOG_INFO, "%s", oss.str().c_str());
            printed_index = printed_index + 1;
            // print to std out
            // std::cout << "\t (Itr. " << std::setw(2) << k << ") X" << printed_index << ": " << "unwhitened (U, X, T):\t (";
            // std::cout << std::setw(14) << std::left <<  std::sqrt(unwhitened_prior_U[pose]) << std::setw(14) << std::sqrt(unwhitened_prior_X[pose]) << std::setw(14)
            //     << std::sqrt(unwhitened_ternary[pose]) << " )" << std::endl;

        }
    }


    double print_results() {
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();
        auto memory_diff = end_memory - start_memory;

        std::ostringstream oss;
        oss << std::left << std::setw(20) << "Execution Time:" << duration << " microseconds\n";
        oss << std::left << std::setw(20) << "Memory Usage:" << memory_diff << " KB\n";
        logger.AddLog(LOG_INFO, "%s", oss.str().c_str());
        logger.AddLog(LOG_ERROR, "Metrics");
        return duration;
    }

private:
    std::chrono::high_resolution_clock::time_point start_time;
    std::chrono::high_resolution_clock::time_point end_time;
    double trial_error;
    long long start_memory;
    long long end_memory;
    std::vector<double> residuals_prior_U;
    std::vector<double> residuals_prior_X;
    std::vector<double> residuals_ternary;
    std::vector<double> unwhitened_prior_U;
    std::vector<double> unwhitened_prior_X;
    std::vector<double> unwhitened_ternary;
    int MAX_STATES;

    long long get_memory_usage() {
        rusage usage;
        getrusage(RUSAGE_SELF, &usage);
        return usage.ru_maxrss * 1024L;
    }
    
};