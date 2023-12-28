#pragma once
// #include <gtsam/geometry/Pose2.h>
#include "MotionModelArc.h"

namespace Trajectory
{
    /* -------------------------------------------------------------------------- */
    /*                   Reference Trajectory (std::vector form)                  */
    /* -------------------------------------------------------------------------- */
    class ref_trajv
    {
    private:
        int push_box;                               // flag of whether the system is trying to push a box
        double box_db;                              // a double cast from `push_box`, to avoid calculation errors used in set_inititial(), set_final(), and fill()
        int separation_of_action;                   // separating rotation and translation
        int trajectory_type;                        // checks the type of trajectory desired (0: linear, 1: sinusoidal, 2: circular)

        int nr_of_robots;                          // (multi-robot) the number of robots, if the centroid is being used to define the reference trajectory
        int max_nr_of_robots;                       // the maximum number of robots the system can have (currently 2)

        int last;                                   // the number of iterations
        double overall_angle;                       // (linear & sinusoidal) the angle formed by start and end positions
        std::vector<double> overall_dist;           // (all) distance between start and end positions in the x-direction, y-direction, and in length 
                                                    // | x_displace, y_displace, overall_dist | (3)
        double amplitude;                           // (sinusoidal) amplitude of sine curve
        double angular_freq;                        // (sinusoidal) angular frequency of sine surve
        double angle_increment;                     // (circular) angle of rotation required for each step, assuming CCW
        double radius;                              // (circular) radius formed by trajectory and center of rotation
        std::vector<double> angular_displacement;   // (circular) the angles in the polar coordinate required for the trajectory
                                                    // | angle0, angle1, ...... angle199, angle200 | (201)
        gtsam::Pose2 center;                        // (circular) center of a circular reference trajectory

        gtsam::Pose2 start_pose;                    // starting pose of the target (centroid or robot1)
        gtsam::Pose2 end_pose;                      // ending pose of the target (centroid or robot1)
        std::vector<gtsam::Pose2> reference_trajectory; // a vector storing the reference trajectory of the target

        Optimization_parameter optimization_parameter; // optimization parameters (ie. number of states, number of robots, etc)
        Geometry_information geometry_information;   // geometry information (ie. box width, distance between robot1 and centroid, etc.)

    public:
        /**
         * @brief Construct a new ref trajv object, initilizes the parameters
         * 
         * @param optimization_param Optimization parameter
         * @param geometry_info Geometry information
         */
        ref_trajv(Optimization_parameter optimization_param, Geometry_information geometry_info)
        {
            optimization_parameter = optimization_param;
            geometry_information = geometry_info;
            
            last = optimization_parameter.nr_of_steps;
            nr_of_robots = optimization_parameter.nr_of_robots;
            max_nr_of_robots = optimization_parameter.max_nr_of_robots;
            trajectory_type = optimization_parameter.reference_trajectory_type;
            push_box = optimization_parameter.push_box;
            box_db = static_cast<double>(push_box);
            separation_of_action = optimization_parameter.separation_of_action;

            overall_angle = 0;
            amplitude = 1;
            angular_freq = 1;
            angle_increment = 0;
            radius = 0;
            
            gtsam::Pose2 empty_pose(0, 0, 0);
            center = empty_pose;
            start_pose = empty_pose;
            end_pose = empty_pose;            
        }

        /**
         * @brief Sets the initial pose of the robot
         * 
         * @param initial_pose Initial pose
         */
        void set_initial(gtsam::Pose2 initial_pose)
        {
            start_pose = initial_pose;
            if (trajectory_type == 2)
            {
                set_final(initial_pose);
            }
        }

        /**
         * @brief Sets the final pose
         * 
         * @param final_pose Final pose
         */
        void set_final(gtsam::Pose2 final_pose)
        {
            end_pose = final_pose;
        }

        /**
         * @brief Sets the angular frequency for sinusoidqal reference trajectories
         * 
         * @param omega Angular frequency
         */
        void set_angular_freq(double omega)
        {
            angular_freq = omega;
        }

        /**
         * @brief Sets the amplitude for sinusoidal reference trajectories
         * 
         * @param A Amplitude
         */
        void set_amplitude(double A)
        {
            amplitude = A;
        }

        void set_overall_angle()
        {
            overall_angle = Utils::angle_between_points(start_pose, end_pose);
        }

        /**
         * @brief Sets the center of rotation for circular reference trajectories
         * 
         * @param center_traj_type_2  The center of rotation {x_center, y_center, 0}
         */
        void set_center_of_rotation(gtsam::Pose2 center_traj_type_2)
        {
            center = center_traj_type_2;
        }

        /**
         * @brief Calculates the angle at which the robot travels around the center of rotation and along the circular reference trajectory for each translation
         * 
         */
        void set_angle_increment()
        {
            if (separation_of_action)
            {
                angle_increment = 2 * M_PI / floor(last / 2);
            }
            else
            {
                angle_increment = 2 * M_PI / last;
            }
        }

        /**
         * @brief Determines the angles at which the robot travels around the center of rotation and along the circular reference trajectory
         * 
         */
        void set_angular_displacement()
        {
            set_angle_increment();
            Eigen::Vector2f vector_CenterToStart = Utils::make_vector(center, start_pose);
            radius = vector_CenterToStart.norm();
            double angle = Utils::ensure_orientation_range(atan2(vector_CenterToStart(1), vector_CenterToStart(0)));
            for (int i = 0; i <= last; i++)
            {
                double counter = static_cast<double>(i);
                if (separation_of_action)
                {
                    angular_displacement.push_back(Utils::ensure_orientation_range(angle + floor(counter / 2) * angle_increment));
                }
                else
                {
                    angular_displacement.push_back(Utils::ensure_orientation_range(angle + counter * angle_increment));
                }
            }
        }

        /**
         * @brief Computes the difference in x, y, annd theta between 2 arbitrary states
         * 
         * @param state_start 1st state
         * @param state_end 2nd state
         * @return A gtsam::Pose2 that stores the differences {diff_x, diff_Y, diff_theta}
         */
        gtsam::Pose2 compute_diff(int state_start, int state_end)
        {
            double diff_x, diff_y, diff_theta;
            
            diff_x = reference_trajectory[state_end].x() - reference_trajectory[state_start].x(); 
            diff_y = reference_trajectory[state_end].y() - reference_trajectory[state_start].y(); 
            diff_theta = reference_trajectory[state_end].theta() - reference_trajectory[state_start].theta(); 
            
            gtsam::Pose2 diff(diff_x, diff_y, diff_theta);
            return (diff);
        }

        /**
         * @brief Computes the linear distance between 2 arbitrary states
         * 
         * @param state_start 1st state
         * @param state_end 2nd state
         * @return The linear distance calculated
         */
        double compute_dist(int state_start, int state_end)
        {
            Eigen::Vector2f vector_AToB = Utils::make_vector(reference_trajectory[state_start], reference_trajectory[state_end]);
            double dist = vector_AToB.norm();
            return (dist);
        }

        /**
         * @brief Gets the desired gtsam::Pose2 value stored in the reference trajectory vector
         * 
         * @param state_number The desired gtsam::Pose2 index
         * @returns The value stored in the designated state
        */
        gtsam::Pose2 get_ref_pose(int state_number)
        {
            return reference_trajectory[state_number];
        }

        /**
         * @brief Calculates the difference in the x and y component between X_start and X_final, and the linear distance between X_start and X_final
         * 
        */
        void set_overall_dist()
        {
            double x_overall_length = end_pose.x() - start_pose.x();
            double y_overall_length = end_pose.y() - start_pose.y();
            double overall_length = sqrt(pow(x_overall_length, 2) + pow(y_overall_length, 2));
            overall_dist.push_back(x_overall_length);
            overall_dist.push_back(y_overall_length);
            overall_dist.push_back(overall_length);
        }

        /**
         * @brief Fills out the trajectory of designated type (linear, sinusoidal, or circular)
         * 
        */
        void fill()
        {
            double x, y, theta;
            int i;
            double counter;
            switch (trajectory_type)
            {
                // linear
                case 0:
                {
                    for (i = 0; i <= last; i++)
                    {
                        counter = static_cast<double>(i);
                        x = start_pose.x() + counter * overall_dist[0] / last;
                        y = start_pose.y() + counter * overall_dist[1] / last;
                        theta = start_pose.theta();
                        gtsam::Pose2 pose(x, y, theta);
                        reference_trajectory.push_back(pose);
                    }
                }
                break;
                
                // sinusoidal
                case 1:
                {
                    gtsam::Pose2 start_pose_transformed = Utils::perform_2D_transformation(start_pose, -overall_angle);
                    gtsam::Pose2 end_pose_transformed = Utils::perform_2D_transformation(end_pose, -overall_angle);
                    double distance = Utils::distance_between_points(start_pose_transformed, end_pose_transformed);
                    double shift = start_pose_transformed.y();

                    // ! we can use compose to simplify this, could be faster
                    for (i = 0; i <= last; i++)
                    {
                        counter = static_cast<double>(i);
                        double x_transformed = start_pose_transformed.x() + counter * distance / last;
                        double y_transformed = amplitude * sin(angular_freq * (x_transformed - start_pose_transformed.x())) + shift;
                        gtsam::Pose2 pose_transformed(x_transformed, y_transformed, 0);
                        gtsam::Pose2 pose = Utils::perform_2D_transformation(pose_transformed, overall_angle);
                        reference_trajectory.push_back(pose);
                    }
                }
                break;
                
                // circular
                case 2:
                {
                    for (i = 0; i <= last; i++)
                    {
                        set_angular_displacement();
                        x = center.x() + radius * cos(angular_displacement[i]);
                        y = center.x() + radius * sin(angular_displacement[i]);
                        theta = start_pose.theta();
                        gtsam::Pose2 pose(x, y, theta);
                        reference_trajectory.push_back(pose);
                    }
                }
                break;
            }
                
            for (i = 0; i <= last; i++)
            {            
                reference_trajectory[i] = gtsam::Pose2(reference_trajectory[i].x(), reference_trajectory[i].y(), start_pose.theta());
            }
        }

        /**
         * @brief Prints out the reference trajectory
        */
        void print()
        {
            std::cout << "Target reference trajectory: \n" << std::endl;
            Utils::printPoseVector(reference_trajectory);
        }
    };
}