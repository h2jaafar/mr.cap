#pragma once
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <string>
#include <fstream>
#include <filesystem>
#include <vector>
#include <utility> // std::pair

namespace fs = std::filesystem; // Alias for the filesystem namespace

extern ImGuiLogger logger;

const double M_PI_8 = M_PI_4 / 2.0;

namespace Utils
{
  // Structure for disturbances, takes in:
    // 1. pose number which disturbance takes place (ex. at x = 10)
    // 2. disturbance axis (ex. x-axis put 0)
    // 3. disturbance value (ex. 0.5)
  struct Disturbance
  {
    int pose_number;
    int axis;
    double disturbance_value;
  };

  /**
   * @brief Creates a vector using two gtsam::Pose2, such that some calculations can be more easily done
   * 
   * @param vector_tail The gtsam::Pose2 used to form the vector tail
   * @param vector_head The gtsam::Pose2 used to form the vector tail
   * @return A vector formed by calculating vector_head - vector_tail
   */
  Eigen::Vector2f make_vector(gtsam::Pose2 vector_tail, gtsam::Pose2 vector_head)
  {
    Eigen::Vector2f vector_made((vector_head.x() - vector_tail.x()),(vector_head.y() - vector_tail.y()));
    return vector_made;
  }

  // get current time
  std::string get_current_time()
  {
    time_t now = time(0);
    tm *ltm = localtime(&now);
    std::string current_time = std::to_string(1900 + ltm->tm_year) + "_" + std::to_string(1 + ltm->tm_mon) + "_" + std::to_string(ltm->tm_mday) + "_" + std::to_string(ltm->tm_hour) + "_" + std::to_string(ltm->tm_min) + "_" + std::to_string(ltm->tm_sec);
    return current_time;
  }

  /**
   * @brief Shifts any arbitrary angle to the range of [0, 2PI)
   * 
   * @param orientation Input angle
   * @return Shifted angle
   */
    double ensure_orientation_range(double orientation)
    {
      return fmod((fmod(orientation, 2 * M_PI) + 2 * M_PI), 2 * M_PI);
    }

  double FastArcTan(double z) {
    if (z > 1.0) {
      z = 1.0 / z;
      return M_PI_2 - z * (M_PI_4 - (z - 1) * (M_PI_4 - z * (3.0 * M_PI_8)));
    } else if (z < -1.0) {
      z = 1.0 / z;
      return -M_PI_2 - z * (M_PI_4 - (z - 1) * (M_PI_4 - z * (3.0 * M_PI_8)));
    }
    return z * (M_PI_4 - (z - 1) * (M_PI_4 - z * (3.0 * M_PI_8)));
  }

  double FastAtan2(double y, double x) {
    if (x > 0) {
      return FastArcTan(y / x);
    } else if (y >= 0 > x) {
      return FastArcTan(y / x) + M_PI;
    } else if (y < 0 && x < 0) {
      return FastArcTan(y / x) - M_PI;
    } else if (y > 0 && x == 0) {
      return M_PI_2;
    } else if (y < 0 && x == 0) {
      return -M_PI_2;
    } else {
      return 0; // x = y = 0
    }
  }

  /**
   * @brief Shifts any arbitrary angle to the range of [-PI, PI)
   * 
   * @param orientation Input angle
   * @return Shifted Angle
   */
  double ensure_orientation_range2 (double orientation)
  {
    orientation = ensure_orientation_range(orientation);
    if ((orientation >= M_PI) && (orientation < 2 * M_PI))
    {
      orientation = orientation - 2 * M_PI;
    }
    return orientation;
  }

  double distance_between_points(gtsam::Pose2 start_point, gtsam::Pose2 end_point)
  {
    Eigen::Vector2f vector_StartToEnd = make_vector(start_point, end_point);
    double distance = vector_StartToEnd.norm();

    return distance;
  }

  double angle_between_points(gtsam::Pose2 start_point, gtsam::Pose2 end_point)
  {
    Eigen::Vector2f vector_StartToEnd = make_vector(start_point, end_point);
    double angle = atan2(vector_StartToEnd(1), vector_StartToEnd(0));
    angle = ensure_orientation_range(angle);

    return angle;
  }

  /**
   * @brief Checks if the two input angles are orthognal to one another and if the difference (start_angle - end_angle) is less than PI
   * 
   * @param start_angle 
   * @param end_angle 
   * @return A vetor of check results, first digit indicating orthogonality and second digit indicating proximity
   * orthogonality: 
   *    1:  start_angle = end_angle + PI/2
   *    -1: start_angle = end_angle - PI/2
   *    0:  start_angle and end_angle are not orthogonal
   * proximity:
   *    1:  start_angle - end_angle <= PI
   *    0:  start_angle - end_angle > PI
   */
  std::pair<int, int> check_direction(double start_angle, double end_angle)
  {
    int direction_orthogonal = 0;
    int direction;
    double angle_diff = ensure_orientation_range(end_angle - start_angle);

    std::pair<int, int> angular_relation;

    if (abs(angle_diff - M_PI / 2) <= 0.005)
    {
      direction_orthogonal = 1;
    }
    else if (abs(angle_diff - 3 * M_PI / 2) <= 0.005)
    {
      direction_orthogonal = -1;
    }
    else{}

    if (angle_diff <= M_PI)
    {
      direction = 1;
    }
    else
    {
      direction = -1;
    }

    angular_relation.first = direction_orthogonal;
    angular_relation.second = direction;
    
    return angular_relation;
  }

  /**
   * @brief Checks if the input angle is vertical
   * 
   * @param orientation Input angle
   * @return Check result, 1 for vertical and 0 otherwise 
   */
  int check_vertical(double orientation)
  {
    int vertical = 0;
    if (ensure_orientation_range(orientation) == M_PI / 2) 
    {
      vertical = 1;
    }
    if (ensure_orientation_range(orientation) == M_PI * 3 / 2) 
    {
      vertical = 1;
    }

    return vertical;
  }

  /**
   * @brief Checks if the input angle is a horizontal
   * 
   * @param orientation Input Angle
   * @return Check result, 1 for horizontal and 0 otherwise
   */
  int check_horizontal(double orientation)
  {
    int horizontal = 0;
    if (ensure_orientation_range(orientation) == 0) 
    {
      horizontal = 1;
    }
    if (ensure_orientation_range(orientation) == M_PI) 
    {
      horizontal = 1;
    }

    return horizontal;
  }

  int check_action_given_centroid_control(gtsam::Pose2 centroid_speed)
  {
    int centroid_action = 0;
    double zero_threshold = 0.00001;

    // pure translation of centroid
    if ((abs(centroid_speed.x()) > zero_threshold || abs(centroid_speed.y()) > zero_threshold) && abs(centroid_speed.theta()) < zero_threshold)
    {
      centroid_action = 1;
    }
    // wait (for robot rotation)
    else if (abs(centroid_speed.x()) < zero_threshold && abs(centroid_speed.y()) < zero_threshold && abs(centroid_speed.theta()) < zero_threshold)
    {
      centroid_action = 2;
    }
    // pure rotation of centroid
    else if (abs(centroid_speed.x()) < zero_threshold && abs(centroid_speed.y()) < zero_threshold && abs(centroid_speed.theta()) > zero_threshold)
    {
      centroid_action = 3;
    }
    return centroid_action; 
  }

  int check_action_given_centroid_poses(gtsam::Pose2 current_centroid_pose, gtsam::Pose2 next_centroid_pose)
  {
    int centroid_action = 0;
    // double zero_threshold = 0.00001;
    // double distance_threshold = 0.02;
    double distance_threshold = 0.00001;
    double zero_threshold = 0.05;

    double delta_theta = ensure_orientation_range(next_centroid_pose.theta() - current_centroid_pose.theta());
    double displacement = distance_between_points(current_centroid_pose, next_centroid_pose);

    // pure translation of centroid
    if (displacement > distance_threshold)
    {
      centroid_action = 1;
    }
    // wait (for robot rotation)
    else if (delta_theta < zero_threshold)
    {
      centroid_action = 2;
    }
    // pure rotation of centroid
    else if (delta_theta > zero_threshold)
    {
      centroid_action = 3;
    }
    return centroid_action; 
  }

  int check_action_given_robot_control(gtsam::Pose2 wheel_speed)
  {
    int robot_action = 0;
    double zero_threshold = 0.00001;
    
    // pure translation of robot
    if (abs(wheel_speed.x() - wheel_speed.y()) < zero_threshold)
    {
      robot_action = 1;
    }
    // pure rotation of robot
    else if (abs(wheel_speed.x() + wheel_speed.y()) < zero_threshold)
    {
      robot_action = 2;
    }
    // arc of robot
    else
    {
      robot_action = 3;
    }
    return robot_action;
  }

    /**
   * @brief Converts from a gtsam::Value to a vector of gtsam::Pose's (ONLY POSITION)
   * @param values: Values container with the values inside it 
   * @return returned_trajectory of std::vector<gtsam:Pose2> 
   */ 
  std::vector<gtsam::Pose2> valuesToPose_mod(gtsam::Values& values, int k, Optimization_parameter optimization_parameter, int robot_id)
{
    int max_states = optimization_parameter.nr_of_steps;
    std::vector<gtsam::Pose2> returned_trajectory;
    gtsam::Symbol key_pos;

    // Create the initial pose
    gtsam::Pose2 pose = (k < 0) ? gtsam::Pose2(0, 0, 0) : gtsam::Pose2();

    for (int i = 0; i <= max_states; ++i)
    {
        if (i >= k)
        {
            // Update the key_pos based on robot_id only when necessary
            switch (robot_id)
            {
                case 0:
                    key_pos = gtsam::Symbol('C', i);
                    break;
                case 1:
                    key_pos = gtsam::Symbol('X', i);
                    break;
                case 2:
                    key_pos = gtsam::Symbol('Y', i);
                    break;
            }

            // Update the pose directly from values
            pose = values.at<gtsam::Pose2>(key_pos);
        }

        returned_trajectory.push_back(pose);
    }

    return returned_trajectory;
}

std::vector<gtsam::Pose2> valuesToVelocity_mod(gtsam::Values &values, int k, Optimization_parameter optimization_parameter, int robot_id) {
    int max_states = optimization_parameter.nr_of_steps;
    std::vector<gtsam::Pose2> returned_velocity;
    gtsam::Symbol key_control;

    // Create the initial velocity
    gtsam::Pose2 velocity = (k < 0) ? gtsam::Pose2(0, 0, 0) : gtsam::Pose2();

    for (int i = 0; i < max_states; ++i) {
        // Update the key_control based on robot_id only when necessary
        if (i >= k && robot_id == 0) {
            key_control = gtsam::Symbol('U', i);
            // Update the velocity directly from values
            velocity = values.at<gtsam::Pose2>(key_control);
        }

        returned_velocity.push_back(velocity);
    }

    return returned_velocity;
}


  /**
   * @brief Print a std vector of poses
   * @param vec: std vector of pose2
   */
  void printPoseVector(std::vector<gtsam::Pose2> vec)
  { 
    // std::cout << "Printing pose2 vector" << std::endl;
    for(int k = 0; k < vec.size() ; ++k)
    {
      vec[k].print();
    }
  }


  int odd_number(int number)
  {
    int number_is_odd = 0;
    
    if (number % 2 == 1)
    {
      number_is_odd = 1;
    }
    
    return number_is_odd;
  }

  Eigen::Matrix3d create_2D_transformation_matrix(double rotation_angle, double dx, double dy)
  {
    Eigen::Matrix3d transformation_matrix_2D;
    
    transformation_matrix_2D << cos(rotation_angle),  -sin(rotation_angle), dx,
                                sin(rotation_angle),  cos(rotation_angle),  dy,
                                0,                    0,                    1;
    
    return transformation_matrix_2D;
  }
  gtsam::Pose2 perform_2D_transformation(gtsam::Pose2 pose_to_transform, double rotation_angle, double dx = 0, double dy = 0)
  {
    Eigen::Matrix3d transformation_matrix_2D;
    Eigen::Vector3d vector_to_transform, vector_transformed; 
    
    transformation_matrix_2D = create_2D_transformation_matrix(rotation_angle, dx, dy);
    vector_to_transform << pose_to_transform.x(), pose_to_transform.y(), pose_to_transform.theta();
    vector_transformed = transformation_matrix_2D * vector_to_transform;

    gtsam::Pose2 pose_transformed(vector_transformed(0), vector_transformed(1), vector_transformed(2));
    return pose_transformed;
  }


  /**
   * @brief writes a vector into a .csv file. before calling this function, the following should be done first:
   *        
   *        std::vector<std::pair<std::string, std::vector<double>>> vals = {{"1st element", vec.x()}, {"2nd element", vec2.y()}, {"3rd element", vec3.theta()}};
   *        write_csv("three_cols.csv", vals);
   * 
   * @param filename: the name of the csv file
   * @param dataset: the vector containing the desired info
  */
  std::string write_csv(std::string filename, std::vector<RobotData> robots, CentroidData centroid,  Geometry_information geometry_info, char robot_nr, bool metrics_running)
  {  
     // create string of date
    time_t now = time(0);
    tm *ltm = localtime(&now);
    std::string saved_path;
    if(metrics_running) {
      std::string date = "metrics" + std::to_string(1900 + ltm->tm_year) + "_" + std::to_string(1 + ltm->tm_mon) + "_" + std::to_string(ltm->tm_mday);
      std::string folder_name = "../../results/" + date;
      if (!fs::exists(folder_name)) {
        fs::create_directory(folder_name);
      }
      saved_path = folder_name + "/" + filename;
      return saved_path;
    }
    auto rc_real_trajectory = centroid.X_k_real;
    auto rc_reference_trajectory = centroid.X_k_ref;
    auto r1_wheel_velocities_rotation = robots[0].U_k_fg_CentroidTranslation_RobotRotation;
    auto r1_wheel_velocities_translation = robots[0].U_k_fg_CentroidTranslation_RobotTranslation;
    auto r1_real_trajectory = robots[0].X_k_real;
    auto r2_wheel_velocities_rotation = robots[1].U_k_fg_CentroidTranslation_RobotRotation;
    auto r2_wheel_velocities_translation = robots[1].U_k_fg_CentroidTranslation_RobotTranslation;
    auto r2_real_trajectory = robots[1].X_k_real;

    int max_states = r1_real_trajectory.size();

    auto r = geometry_info.robot_wheel_radius;
    auto L = geometry_info.robot_width;
    // auto r_c = geometry_info.distance_CentroidToConnection_vertical;
    // auto r_1 = geometry_info[3];

          // std::vector<double> geometry_information = {r, L, r_c, r_1, r_2, l_1, l_2};

   
    std::string date = std::to_string(1900 + ltm->tm_year) + "_" + std::to_string(1 + ltm->tm_mon) + "_" + std::to_string(ltm->tm_mday) + "_" + std::to_string(ltm->tm_hour) + "_" + std::to_string(ltm->tm_min) + "_" + std::to_string(ltm->tm_sec);

    // Create an output filestream object
    std::string folder_name = "../../results/" + date;

    std::ofstream myFile;
      std::string str2 = "_robot_";
      std::string str3 = ".csv";
      std::string str_combined = folder_name + "/" + filename + "_" + str3;
      if (!fs::exists(folder_name)) {
        fs::create_directory(folder_name);
      }

      myFile = std::ofstream(str_combined);
      saved_path = folder_name + "/";


    // Send column names to the stream
    myFile << "Results ,";
    myFile << "Robot" << robot_nr << ",";
    myFile << "r," << r << ",";
    myFile << "L," << L << ",";
    myFile << "r_c,," ;
    myFile << ",";
    // date
    myFile << "Date," << date << ",";
    myFile << ","; myFile << ","; myFile << ","; myFile << ",";
    myFile << ","; myFile << ","; myFile << ","; myFile << ",";
    myFile << "\n";
    myFile << "Iteration,";
    myFile << "R1_rl_x,";
    myFile << "R1_rl_y,";
    myFile << "R1_rl_theta,";
    myFile << ",";    
    myFile << "R2_rl_x,";
    myFile << "R2_rl_y,";
    myFile << "R2_rl_theta,";
    myFile << ",";
    myFile << "RC_rl_x,";
    myFile << "RC_rl_y,";
    myFile << "RC_rl_th,";
    myFile << ",";
    myFile << "RC_ref_x,";
    myFile << "RC_ref_y,";
    myFile << "RC_ref_th,";
    myFile << ",";
    myFile << "VL,";
    myFile << "VR,";
    myFile << "0,";
    myFile << "\n";
    
    // Send data to the stream
    for(int i = 0; i < r1_real_trajectory.size(); ++i)
    {
      myFile << i << ","; 
      myFile << r1_real_trajectory[i].x()<< ",";
      myFile << r1_real_trajectory[i].y() << ",";
      myFile << r1_real_trajectory[i].theta() << ",";
      myFile << ",";
      myFile << r2_real_trajectory[i].x()<< ",";
      myFile << r2_real_trajectory[i].y() << ",";
      myFile << r2_real_trajectory[i].theta() << ",";
      myFile << ",";
      myFile << rc_real_trajectory[i].x()<< ",";
      myFile << rc_real_trajectory[i].y() << ",";
      myFile << rc_real_trajectory[i].theta() << ",";
      myFile << ",";
      myFile << rc_reference_trajectory[i].x()<< ",";
      myFile << rc_reference_trajectory[i].y() << ",";
      myFile << ",";
      myFile << r1_wheel_velocities_rotation[i].x()<< ",";
      myFile << r1_wheel_velocities_rotation[i].y() << ",";
      myFile << ",";
      myFile << r1_wheel_velocities_translation[i].x()<< ",";
      myFile << r1_wheel_velocities_translation[i].y() << ",";
      myFile << "\n";
    }
    
    // Close the file
    myFile.close();
    return saved_path;
  }

  void calcualte_R_and_theta_vectors(CentroidData centroid, std::vector<RobotData> robots, Geometry_information& geometry_info){
      
      for(auto&& robot: robots){
        gtsam::Pose2 distance_ = centroid.X_k_real[0].inverse().compose(robot.X_k_real[0]); // use inv(centroid) * robot, and then take the translation and rotation components
        double r_i = sqrt(pow(distance_.x(), 2) + pow(distance_.y(), 2));
        double theta_i = atan2(distance_.y(), distance_.x()) - centroid.X_k_real[0].theta();
        // clamp to 0 and 2pi
        theta_i = ensure_orientation_range(theta_i);
        geometry_info.distance_to_robot[robot.robot_id - 1] = r_i;
        geometry_info.angle_to_robot[robot.robot_id - 1] = theta_i;
        std::cout << "Updated robot " << robot.robot_id << " distance to centroid: " << r_i << " and angle to centroid: " << theta_i << std::endl;
      }
  }


  void ApplyDarkTheme()
  {
    ImGuiStyle& style = ImGui::GetStyle();

    // Set the values for the dark theme
    style.WindowMinSize = ImVec2(160, 20);
    style.FramePadding = ImVec2(4, 2);
    style.ItemSpacing = ImVec2(6, 2);
    style.ItemInnerSpacing = ImVec2(6, 4);
    style.Alpha = 0.95f;
    style.WindowRounding = 4.0f;
    style.FrameRounding = 2.0f;
    style.IndentSpacing = 6.0f;
    style.ItemInnerSpacing = ImVec2(2, 4);
    style.ColumnsMinSpacing = 50.0f;
    style.GrabMinSize = 14.0f;
    style.GrabRounding = 16.0f;
    style.ScrollbarSize = 12.0f;
    style.ScrollbarRounding = 16.0f;

    style.Colors[ImGuiCol_Text] = ImVec4(0.86f, 0.93f, 0.89f, 0.78f);
    style.Colors[ImGuiCol_TextDisabled] = ImVec4(0.86f, 0.93f, 0.89f, 0.28f);
    style.Colors[ImGuiCol_WindowBg] = ImVec4(0.13f, 0.14f, 0.17f, 1.00f);
    style.Colors[ImGuiCol_Border] = ImVec4(0.31f, 0.31f, 1.00f, 0.00f);
    style.Colors[ImGuiCol_BorderShadow] = ImVec4(0.00f, 0.00f, 0.00f, 0.00f);
    style.Colors[ImGuiCol_FrameBg] = ImVec4(0.20f, 0.22f, 0.27f, 1.00f);
    style.Colors[ImGuiCol_FrameBgHovered] = ImVec4(0.92f, 0.18f, 0.29f, 0.78f);
    style.Colors[ImGuiCol_FrameBgActive] = ImVec4(0.92f, 0.18f, 0.29f, 1.00f);
    style.Colors[ImGuiCol_TitleBg] = ImVec4(0.20f, 0.22f, 0.27f, 1.00f);
    style.Colors[ImGuiCol_TitleBgCollapsed] = ImVec4(0.20f, 0.22f, 0.27f, 0.75f);
    style.Colors[ImGuiCol_TitleBgActive] = ImVec4(0.92f, 0.18f, 0.29f, 1.00f);
    style.Colors[ImGuiCol_MenuBarBg] = ImVec4(0.20f, 0.22f, 0.27f, 0.47f);
    style.Colors[ImGuiCol_ScrollbarBg] = ImVec4(0.20f, 0.22f, 0.27f, 1.00f);
    style.Colors[ImGuiCol_ScrollbarGrab] = ImVec4(0.09f, 0.15f, 0.16f, 1.00f);
    style.Colors[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.92f, 0.18f, 0.29f, 0.78f);
    style.Colors[ImGuiCol_ScrollbarGrabActive] = ImVec4(0.92f, 0.18f, 0.29f, 1.00f);
    style.Colors[ImGuiCol_CheckMark] = ImVec4(0.71f, 0.22f, 0.27f, 1.00f);
    style.Colors[ImGuiCol_SliderGrab] = ImVec4(0.47f, 0.77f, 0.83f, 0.14f);
    style.Colors[ImGuiCol_SliderGrabActive] = ImVec4(0.92f, 0.18f, 0.29f, 1.00f);
    style.Colors[ImGuiCol_Button] = ImVec4(0.47f, 0.77f, 0.83f, 0.14f);
    style.Colors[ImGuiCol_ButtonHovered] = ImVec4(0.92f, 0.18f, 0.29f, 0.86f);
    style.Colors[ImGuiCol_ButtonActive] = ImVec4(0.92f, 0.18f, 0.29f, 1.00f);
    style.Colors[ImGuiCol_Header] = ImVec4(0.92f, 0.18f, 0.29f, 0.76f);
    style.Colors[ImGuiCol_HeaderHovered] = ImVec4(0.92f, 0.18f, 0.29f, 0.86f);
    style.Colors[ImGuiCol_HeaderActive] = ImVec4(0.92f, 0.18f, 0.29f, 1.00f);
    style.Colors[ImGuiCol_Separator] = ImVec4(0.14f, 0.16f, 0.19f, 1.00f);
    style.Colors[ImGuiCol_SeparatorHovered] = ImVec4(0.92f, 0.18f, 0.29f, 0.78f);
    style.Colors[ImGuiCol_SeparatorActive] = ImVec4(0.92f, 0.18f, 0.29f, 1.00f);
    style.Colors[ImGuiCol_ResizeGrip] = ImVec4(0.47f, 0.77f, 0.83f, 0.04f);
    style.Colors[ImGuiCol_ResizeGripHovered] = ImVec4(0.92f, 0.18f, 0.29f, 0.78f);
    style.Colors[ImGuiCol_ResizeGripActive] = ImVec4(0.92f, 0.18f, 0.29f, 1.00f);
    style.Colors[ImGuiCol_PlotLines] = ImVec4(0.86f, 0.93f, 0.89f, 0.63f);
    style.Colors[ImGuiCol_PlotLinesHovered] = ImVec4(0.92f, 0.18f, 0.29f, 1.00f);
    style.Colors[ImGuiCol_PlotHistogram] = ImVec4(0.86f, 0.93f, 0.89f, 0.63f);
    style.Colors[ImGuiCol_PlotHistogramHovered] = ImVec4(0.92f, 0.18f, 0.29f, 1.00f);
    style.Colors[ImGuiCol_TextSelectedBg] = ImVec4(0.92f, 0.18f, 0.29f, 0.43f);
    style.Colors[ImGuiCol_PopupBg] = ImVec4(0.20f, 0.22f, 0.27f, 0.9f);
    style.Colors[ImGuiCol_ModalWindowDimBg] = ImVec4(0.20f, 0.22f, 0.27f, 0.73f);
  }

}