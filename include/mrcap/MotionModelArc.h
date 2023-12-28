#pragma once

namespace MotionModelArc
{
    /* -------------------------------------------------------------------------- */
    /*                    Modelled Trajectory Used For Plotting                   */
    /* -------------------------------------------------------------------------- */
    struct traj_info
    {
        // linear       (if action_type = 0)
        // pivoting     (if action_type = 1)
        // arc          (if action_type = 2)
        int action_type;
        
        // {linear_distance}    (if action_type = 0)
        // {0}                  (if action type = 1)
        // {R}                  (if action_type = 2)
        double plot_length;

        // {x_now, y_now}           (if action_type = 0)
        // {x_now, y_now}           (if action_type = 1)
        // {x_center, y_center}     (if action_type = 2)
        std::vector<double> plot_coordinate;

        // {theta_now, theta_next}                                              (if action_type = 0) (we don't really need theta_next, only keeping it for size consistency)
        // {0, 0}                                                               (if action_type = 1)
        // {orientation_vector_CenterToNow, orientation_vector_CenterToNext}    (if action_type = 2)
        std::vector<double> plot_angles;
    };

    traj_info traj_info_default;
    
    gtsam::Pose2 centroid_solveForNextPose(gtsam::Pose2 current_centroid_pose, gtsam::Pose2 current_centroid_speed, Optimization_parameter optimization_parameter, Geometry_information geometry_information, traj_info *traj_info = &traj_info_default) {
        // Assuming Ts, r, and L are constants or change infrequently, pre-compute any dependent constants
        // and pass them as parameters to this function to avoid recalculating them every call

        // Extract current pose and speed values only once
        const double x_now = current_centroid_pose.x();
        const double y_now = current_centroid_pose.y();
        double theta_now = Utils::ensure_orientation_range(current_centroid_pose.theta());

        const double x_dot = current_centroid_speed.x();
        const double y_dot = current_centroid_speed.y();
        const double theta_dot = current_centroid_speed.theta();

        // Initialize next pose values to current values
        double x_next = x_now;
        double y_next = y_now;
        double theta_next = theta_now;

        // Assuming that the check_action_given_centroid_control function's result is pre-determined,
        // this switch case could be optimized away if you already know the action outside this function
        int action = Utils::check_action_given_centroid_control(current_centroid_speed);
        if (action == 0) {
            // Handle error outside of the optimization loop, if possible
            throw std::invalid_argument("Centroid speed does not allow for pure rotation or translation");
        } else if (action == 1) // pure translation
        {
            x_next += x_dot * optimization_parameter.time_for_translation;
            y_next += y_dot * optimization_parameter.time_for_translation;
            // theta_next is already set to theta_now
        }
        // Include other cases if necessary

        return gtsam::Pose2(x_next, y_next, theta_next);
    }

    /**
    * @brief get an arc from two points, with the heading of X_now tangent to circle
    * @param X_now: current pose
    * @param X_next: next pose
    * @param v: velocity
    * @param L: wheelbase
    * @param r: radius of the wheels
    * @return gtsam::Pose2: Wheel velocities (left and right)
    */
    gtsam::Pose2 centroid_solveForControl(gtsam::Pose2 current_centroid_pose, gtsam::Pose2 next_centroid_pose, Optimization_parameter optimization_parameter, Geometry_information geometry_information, traj_info *traj_info = &traj_info_default)
    {
        int Ts = optimization_parameter.time_for_translation;
        
        double x_now = current_centroid_pose.x();
        double y_now = current_centroid_pose.y();
        double theta_now = current_centroid_pose.theta();
        theta_now = Utils::ensure_orientation_range(theta_now);
        
        double x_next = next_centroid_pose.x();
        double y_next = next_centroid_pose.y();
        double theta_next = next_centroid_pose.theta();

        double x_dot, y_dot, theta_dot;

        switch (Utils::check_action_given_centroid_poses(current_centroid_pose, next_centroid_pose))
        {
            case 1: // pure translation
            {
                x_dot = (x_next - x_now) / Ts;
                y_dot = (y_next - y_now) / Ts;
                theta_dot = 0;
            }
            break;

        }

        return gtsam::Pose2(x_dot, y_dot, theta_dot);
    }
    
    gtsam::Pose2 robot_solveForNextPose(gtsam::Pose2 current_robot_pose, gtsam::Pose2 current_wheel_speeds, Optimization_parameter optimization_parameter, Geometry_information geometry_information, traj_info *traj_info = &traj_info_default)
    {
        int Ts = optimization_parameter.time_for_translation;
        int Ts_robot_rotation = optimization_parameter.time_for_robot_rotation;
        int Ts_centroid_rotation = optimization_parameter.time_for_centroid_rotation;

        double r = geometry_information.robot_wheel_radius;
        double L = geometry_information.robot_width;

        double x_next, y_next, theta_next;
        gtsam::Pose2 next_robot_pose;

        // current pose
        double x_now = current_robot_pose.x();
        double y_now = current_robot_pose.y();
        double theta_now = current_robot_pose.theta();
        theta_now = Utils::ensure_orientation_range(theta_now);

        // wheel velocities
        double u_L = current_wheel_speeds.x();
        double u_R = current_wheel_speeds.y();
        
        switch (Utils::check_action_given_robot_control(current_wheel_speeds))
        {
            case 1: // pure translation of robot
            {
                // std::cout << "mode 1 (pure translation)" << std::endl;
                traj_info -> action_type = 1;
                
                // angular speed and displacement
                double theta_dot = (u_R - u_L) * r / L;             // positive if CCW, negative if CW
                double delta_theta = theta_dot * Ts;                // positive if CCW, negative if CW
                double linear_distance = 0.5 * (u_L + u_R) * r * Ts;
                x_next = x_now + linear_distance * cos(theta_now);
                y_next = y_now + linear_distance * sin(theta_now);
                theta_next = Utils::ensure_orientation_range(theta_now + delta_theta);

                traj_info -> plot_length = linear_distance;
                traj_info -> plot_coordinate = {x_now, y_now};
                traj_info -> plot_angles = {theta_now, theta_next};
            }
            break;

            case 2: // pure rotation of robot
            {
                // std::cout << "mode 2 (pure rotation)" << std::endl;
                traj_info -> action_type = 2;

                double theta_dot = (u_R - u_L) * r / L;                     // positive if CCW, negative if CW
                double delta_theta = theta_dot * Ts_robot_rotation;               // positive if CCW, negative if CW
                x_next = x_now;
                y_next = y_now;
                theta_next = Utils::ensure_orientation_range(theta_now + delta_theta);

                traj_info -> plot_length = 0;
                traj_info -> plot_coordinate = {x_now, y_now};
                traj_info -> plot_angles = {0, 0};
            }
            break;

            case 3: // arc of robot
            {
                traj_info -> action_type = 2;

                // radius and direction of rotation
                double R = 0.5 * L * ((u_L + u_R) / (u_R - u_L));   // postive if center of rotation is on the left wheel side, and negative if center of rotation is on the right wheel side

                // center of rotation (x_center, y_center)
                double x_center = x_now - R * sin(theta_now);
                double y_center = y_now + R * cos(theta_now);
                gtsam::Pose2 center(x_center, y_center, 0);   

                // orientation of vector pointing from center of rotation to X_now
                double orientation_vector_CenterToNow = Utils::angle_between_points(center, current_robot_pose);

                // angle of rotation
                double theta_dot = (u_R - u_L) * r / L;                                 // positive if CCW, negative if CW
                double delta_theta = theta_dot * Ts_centroid_rotation;                  // positive if CCW, negative if CW

                // next pose
                double orientation_vector_CenterToNext = orientation_vector_CenterToNow + delta_theta;
                orientation_vector_CenterToNext = Utils::ensure_orientation_range(orientation_vector_CenterToNext);

                x_next = x_center + abs(R) * cos(orientation_vector_CenterToNext);
                y_next = y_center + abs(R) * sin(orientation_vector_CenterToNext);
                theta_next = Utils::ensure_orientation_range(theta_now + delta_theta);

                traj_info -> plot_length = R;
                traj_info -> plot_coordinate = {x_center, y_center};
                traj_info -> plot_angles = {orientation_vector_CenterToNow, orientation_vector_CenterToNext};
            }
            break;
        }
        next_robot_pose = gtsam::Pose2(x_next, y_next, theta_next);
        return next_robot_pose;
    }

    std::vector<std::pair<gtsam::Vector2, double>> robot_solveForControl_CentroidRotation(gtsam::Pose2 centroid_speed, gtsam::Pose2 current_centroid_pose, gtsam::Pose2 current_robot_pose, int robot_id, Optimization_parameter optimization_parameter, Geometry_information geometry_information, traj_info *traj_info = &traj_info_default)
    {
        int Ts = optimization_parameter.time_for_translation;
        int Ts_robot_rotation = optimization_parameter.time_for_robot_rotation;
        int Ts_centroid_rotation = optimization_parameter.time_for_centroid_rotation;
        double centroid_rotation_threshold = optimization_parameter.centroid_rotation_threshold;
        double robot_rotation_threshold = optimization_parameter.robot_rotation_threshold;
        double r = geometry_information.robot_wheel_radius;
        double L = geometry_information.robot_width;

        double R_i = geometry_information.distance_to_robot[robot_id - 1];
        double theta_i = geometry_information.angle_to_robot[robot_id - 1];

        double u_L, u_R;
        std::pair<gtsam::Vector2, double> speed_and_angle;
        std::vector<std::pair<gtsam::Vector2, double>> combined_action_speed;

        // todo: not sure if we wanna use the LUT, feel free to change it if so
        double target_orientation_centroid = atan2(centroid_speed.y(), centroid_speed.x());
        double angle_to_rotate_centroid_ccw = Utils::ensure_orientation_range(target_orientation_centroid - current_centroid_pose.theta());  
        double angle_to_rotate_centroid_cw = angle_to_rotate_centroid_ccw - 2 * M_PI;
        double angle_to_rotate_centroid;
        
        double orientation_CentroidToRobot = Utils::angle_between_points(current_centroid_pose, current_robot_pose);

        std::pair<int, int> direction_check_centroid = Utils::check_direction(current_centroid_pose.theta(), target_orientation_centroid);
        int direction_centroid = direction_check_centroid.second;
        if (direction_centroid == 1) {
            angle_to_rotate_centroid = angle_to_rotate_centroid_ccw;
        } else {
            angle_to_rotate_centroid = angle_to_rotate_centroid_cw;
        }
        
        double target_orientation_robot = orientation_CentroidToRobot + direction_centroid * M_PI_2;
        double angle_to_rotate_robot_ccw = Utils::ensure_orientation_range(target_orientation_robot - current_robot_pose.theta());  
        double angle_to_rotate_robot_cw = angle_to_rotate_robot_ccw - 2 * M_PI;
        double angle_to_rotate_robot;

        std::pair<int, int> direction_check_robot = Utils::check_direction(current_robot_pose.theta(), target_orientation_robot);
        int direction_robot = direction_check_robot.second;
        if (abs(angle_to_rotate_centroid) < centroid_rotation_threshold) { // determines when centroid arc should be done
            angle_to_rotate_robot = 0;
            target_orientation_robot = current_robot_pose.theta(); // set target to current robot orientation
        } else {
            if (direction_robot == 1) {
                angle_to_rotate_robot = angle_to_rotate_robot_ccw;
            } else {
                angle_to_rotate_robot = angle_to_rotate_robot_cw;
            }
        }

        u_R = (angle_to_rotate_robot * L) / (2 * r * Ts_robot_rotation);
        u_L = - u_R;
        speed_and_angle = std::make_pair(gtsam::Vector2(u_L, u_R), target_orientation_robot);
        combined_action_speed.push_back(speed_and_angle);

        double robot_orientation_after_rtotation = Utils::ensure_orientation_range(current_robot_pose.theta() + angle_to_rotate_robot);
        
        double centroid_angular_speed = 0;
        if(abs(angle_to_rotate_centroid) > centroid_rotation_threshold){
        centroid_angular_speed = angle_to_rotate_centroid / Ts_centroid_rotation;
        };
        // bad naming, but I couldn't come up with a better one
        auto robot_orthogonal_direction = Utils::check_direction(orientation_CentroidToRobot, robot_orientation_after_rtotation);

        double span_right_wheel, span_left_wheel; 
        double direction_correction;

        if (robot_orthogonal_direction.first == direction_centroid) {
            direction_correction = 1;
        }
        else {
            direction_correction = -1;
        }

        if (robot_orthogonal_direction.first == 1) {
            span_right_wheel = R_i + L / 2;
            span_left_wheel = R_i - L / 2;
        }
        else {
            span_right_wheel = R_i - L / 2;
            span_left_wheel = R_i + L / 2;
        }
        
        u_R = direction_correction * abs(centroid_angular_speed) * span_right_wheel / r;
        u_L = direction_correction * abs(centroid_angular_speed) * span_left_wheel / r;
        speed_and_angle = std::make_pair(gtsam::Vector2(u_L, u_R), target_orientation_centroid);
        combined_action_speed.push_back(speed_and_angle);

        return combined_action_speed;
    }

    std::vector<std::pair<gtsam::Vector2, double>> robot_solveForControl_CentroidTanslation(gtsam::Pose2 centroid_speed, gtsam::Pose2 current_centroid_pose, gtsam::Pose2 current_robot_pose, int robot_id, Optimization_parameter optimization_parameter, Geometry_information geometry_information, traj_info *traj_info = &traj_info_default)
    {
        int Ts = optimization_parameter.time_for_translation;
        int Ts_robot_rotation = optimization_parameter.time_for_robot_rotation;
        
        double r = geometry_information.robot_wheel_radius;
        double L = geometry_information.robot_width;

        double R_i = geometry_information.distance_to_robot[robot_id - 1];
        double theta_i = geometry_information.angle_to_robot[robot_id - 1];

        double u_L, u_R;
        std::pair<gtsam::Vector2, double> speed_and_angle;
        std::vector<std::pair<gtsam::Vector2, double>> combined_action_speed;

        // todo: not sure if we wanna use the LUT, feel free to change it if so
        double target_orientation_robot = atan2(centroid_speed.y(), centroid_speed.x());
        double angle_to_rotate_robot_ccw = Utils::ensure_orientation_range(target_orientation_robot - current_robot_pose.theta());  
        double angle_to_rotate_robot_cw = angle_to_rotate_robot_ccw - 2 * M_PI;
        double angle_to_rotate_robot;

        std::pair<int, int> direction_check_robot = Utils::check_direction(current_robot_pose.theta(), target_orientation_robot);
        int direction_robot = direction_check_robot.second;
        if (direction_robot == 1) {
            angle_to_rotate_robot = angle_to_rotate_robot_ccw;
        } else {
            angle_to_rotate_robot = angle_to_rotate_robot_cw;
        }

        u_R = (angle_to_rotate_robot * L) / (2 * r * Ts_robot_rotation);
        u_L = - u_R;
        speed_and_angle = std::make_pair(gtsam::Vector2(u_L, u_R), target_orientation_robot);
        combined_action_speed.push_back(speed_and_angle);

        double centroid_linear_speed = std::sqrt(pow(centroid_speed.x(), 2) + pow(centroid_speed.y(), 2));
        u_R = centroid_linear_speed / r;
        u_L = centroid_linear_speed / r;
        speed_and_angle = std::make_pair(gtsam::Vector2(u_L, u_R), current_centroid_pose.theta());
        combined_action_speed.push_back(speed_and_angle);

        return combined_action_speed;
    }
}