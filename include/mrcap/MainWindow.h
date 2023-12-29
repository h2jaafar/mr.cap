#pragma once

#include "../imgui/imgui.h"
#include "../imgui/imgui_impl_glfw.h"
#include "../imgui/imgui_impl_opengl3.h"
#include "../implot/implot.h"
#include "../implot/implot_internal.h"

#include "DataTypes/PositionPreset.h"
#include "DataTypes/RobotData.h"
#include "DataTypes/CovarianceInfo.h"
#include "DataTypes/GeometryInfo.h"
#include "DataTypes/TrajectoryPreset.h"
#include "DataTypes/OptimizationParams.h"

#include "Logger.h"
#include "SDF.h"
#include "Utils.h"
#include "Metrics.h"
#include "Optimizer.h"
#include "Trajectory.h"
#include <vector>
#include <string>

#include <stdio.h>
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <GLES2/gl2.h>
#endif
#include <GLFW/glfw3.h> // Will drag system OpenGL headers
#include <thread>

// ROS2 
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>

#include "Plotter.h"

#if defined(_MSC_VER) && (_MSC_VER >= 1900) && !defined(IMGUI_DISABLE_WIN32_FUNCTIONS)
#pragma comment(lib, "legacy_stdio_definitions")
#endif

static void glfw_error_callback(int error, const char *description) {
    fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

class MainWindow {
public:
    std::vector<RobotData> robots;
    CentroidData centroid;

    float ref_traj_start_pos[3] = {0, 0, 0};
    float ref_traj_end_pos[3] = {5, 3, 0};
    std::vector<PositionPreset> positionPresets = {
    {"presetA", 0.0f, 0.0f, 0.0f, 7.0f, 0.0f, 0.0f},
    {"presetB", 0.0f, 0.0f, 1.54f, 5.0f, 2.0f, 1.54f},
    // Add more presets as required...
    };

    gtsam::Pose2 custom_reference_trajectory[21];
    int currentTrajectoryPreset = {0};
    std::vector<TrajectoryPreset> trajectoryPresets = {
        {"Rectanglar",
        gtsam::Pose2(0.0, 0.0, 0.0), gtsam::Pose2(0.4, 0.0, 0.0), gtsam::Pose2(0.8, 0.0, 0.0), gtsam::Pose2(0.8, 0.5, 0.0), gtsam::Pose2(0.8, 1.0, 0.0), gtsam::Pose2(1.2, 1.0, 0.0), gtsam::Pose2(1.6, 1.0, 0.0), 
        gtsam::Pose2(1.6, 0.5, 0.0), gtsam::Pose2(1.6, 0.0, 0.0), gtsam::Pose2(2.0, 0.0, 0.0), gtsam::Pose2(2.4, 0.0, 0.0), gtsam::Pose2(2.4, 0.5, 0.0), gtsam::Pose2(2.4, 1.0, 0.0), gtsam::Pose2(2.8, 1.0, 0.0), 
        gtsam::Pose2(3.2, 1.0, 0.0), gtsam::Pose2(3.2, 0.5, 0.0), gtsam::Pose2(3.2, 0.0, 0.0), gtsam::Pose2(3.6, 0.0, 0.0), gtsam::Pose2(4.0, 0.0, 0.0), gtsam::Pose2(4.0, 0.5, 0.0), gtsam::Pose2(4.0, 1.0, 0.0)},
    };

    int currentPreset = {0};
    float ref_traj_center[2] = {1, 1};
    float ref_traj_amplitude = {0.08f};
    float ref_traj_angular_freq = {M_PI};

    // Parameters
    int Ts = {8};
    double L = {0.287};
    double r = {0.0325};
    // ? Hardware
    // double r_4 = {0.475362};
    // double r_1 = {0.478292};
    // double r_2 = {0.47928};
    // double r_3 = {0.455582};
    // double theta_4 = {5.31982};
    // double theta_1 = {0.86401};
    // double theta_2 = {2.16399};
    // double theta_3 = {4.08139};
    // ? Gazebo
    double r_1 = {0.6};
    double r_2 = {0.6};
    double r_3 = {0.6};
    double r_4 = {0.6};
   
    double theta_1 = {M_PI_4};
    double theta_2 = {3 * M_PI_4};
    double theta_3 = {5 * M_PI_4};
    double theta_4 = {7 * M_PI_4};



    int nr_of_obstacles = {5};

    // obstacle avoidance
    int currentCovPreset = 0;

    Plotter plotter;
    std::vector<CovariancePreset> covariancePresets = {
    //                     Name             x0,x1,x2,t0,t1,t2,o0,o1,o2,p0,    p1,   p2,     u0,   u1   
        CovariancePreset("2-obs", 1,1,10,0.001,0.001,1,1,0,0,0.0001,0.0001,0.001, 0.1, 0.1),     
        CovariancePreset("Software-Trials", 1, 1, 2, 1, 1, 1, 1, 0, 0, 1e-3,  1e-3, 1.5e-1, 200 , 200),

        // Add more presets as required...
    };

    double covariance_X[3] = {covariancePresets[0].covX[0], covariancePresets[0].covX[1], covariancePresets[0].covX[2]};
    double covariance_ternary[3] = {covariancePresets[0].covTernary[0], covariancePresets[0].covTernary[1], covariancePresets[0].covTernary[2]};
    double covariance_obs[3] = {covariancePresets[0].covObs[0], covariancePresets[0].covObs[1], covariancePresets[0].covObs[2]};
    double covariance_priors[3] =  {covariancePresets[0].covPriors[0], covariancePresets[0].covPriors[1], covariancePresets[0].covPriors[2]};
    double covariance_U[2] = {covariancePresets[0].covU[0], covariancePresets[0].covU[1]};
    Optimization_parameter optimization_parameter;
    Geometry_information geometry_information;
    // int velocity_step;
    int disturbance_pose = 3;
    int disturbance_axis = 0;
    double disturbance_value = 0.0; // m

    std::string currentSolverType = "MULTIFRONTAL_QR";
    std::vector<double> solver_parameters = {2000, 0.1, 0.1, 0.1, 3};
    SDF_s sdf_s; 
    const char* label_format = "%.1f";
    MainWindow() {

        sdf_s.obstacles.reserve(nr_of_obstacles);
        sdf_s.obstacles.push_back(obstacle(2, -0.4, 0));
        sdf_s.obstacles.push_back(obstacle(5, 0.4, 0));
        sdf_s.obstacles.push_back(obstacle(1, 0.4, 0));
        sdf_s.obstacles.push_back(obstacle(3, -0.6, 0));
        sdf_s.obstacles.push_back(obstacle(4, 0.7, 0));
        sdf_s.system_radius = 0.5;
        sdf_s.inv_system_radius = 1.0 / sdf_s.system_radius;
        sdf_s.system_radius_squared = sdf_s.system_radius * sdf_s.system_radius;
        sdf_s.safety_radius = 0.25;
        sdf_s.sys_radius_safety_radius = sdf_s.system_radius + sdf_s.safety_radius;
        sdf_s.sys_radius_safety_radius_squared = sdf_s.sys_radius_safety_radius * sdf_s.sys_radius_safety_radius;
        sdf_s.inv_sys_radius_safety_radius = 1.0 / sdf_s.sys_radius_safety_radius;
        optimization_parameter.lambdaFactor = 10;
        optimization_parameter.lambdaInitial = 1e-5;
        optimization_parameter.lambdaUpperBound = 100000;
        optimization_parameter.lambdaLowerBound = 0.0;
        optimization_parameter.useFixedLambdaFactor = true;
        optimization_parameter.diagonalDamping = false;
        optimization_parameter.nr_of_robots = 4;
        optimization_parameter.centroid_rotation_threshold = 10 * M_PI/180;
        optimization_parameter.robot_rotation_threshold = 3 * M_PI/180;
        optimization_parameter.nr_of_steps = 10;
        optimization_parameter.wheel_speed_limit = 0.26;
        optimization_parameter.nominal_reference_speed = 0.2;
        optimization_parameter.proportional_gain = 1;
        optimization_parameter.integral_gain = 1;
        optimization_parameter.derivative_gain = 1;
        optimization_parameter.pid_error_threshold = 0.03;
        optimization_parameter.run_ros_nodes = 0;
        optimization_parameter.numerical_jacobian = 0;
        optimization_parameter.print_fg_factors = false;
        optimization_parameter.print_fg_initial_values = false;
        optimization_parameter.print_fg_iterated_results = false;
        optimization_parameter.print_ref_traj = false;
        optimization_parameter.print_modelled_traj = false;
        optimization_parameter.print_velocities = false;
        optimization_parameter.error_scale_ternary = 1.0;
        optimization_parameter.adjust_centroid_orientation = 1;
        optimization_parameter.separation_of_action = 1;
    }

private:
public:
    static void HelpMarker(const char *desc) {
        ImGui::TextDisabled("(?)");
        if (ImGui::IsItemHovered()) {
            ImGui::BeginTooltip();
            ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
            ImGui::TextUnformatted(desc);
            ImGui::PopTextWrapPos();
            ImGui::EndTooltip();
        }
    }

    void runGui() {
        // Setup window
        glfwSetErrorCallback(glfw_error_callback);
        if (!glfwInit())
            return;

            // Decide GL+GLSL versions
#if defined(IMGUI_IMPL_OPENGL_ES2)
        // GL ES 2.0 + GLSL 100
        const char *glsl_version = "#version 100";
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
        glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);
#else
        // GL 3.0 + GLSL 130
        const char *glsl_version = "#version 130";
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
        // glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
        // glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // 3.0+ onlyNonlinearFactorGraph graph;
#endif

        // Create window with graphics context
        GLFWwindow *window = glfwCreateWindow(1920, 1080, "mrcap", NULL, NULL);
        if (window == NULL)
            return;
        glfwMakeContextCurrent(window);
        glfwSwapInterval(1); // Enable vsync

        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImPlot::CreateContext();

        ImGuiIO &io = ImGui::GetIO();
        (void)io;
        io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard; // Enable Keyboard Controls

        ImGui::StyleColorsDark();
        Utils::ApplyDarkTheme();

        ImGui_ImplGlfw_InitForOpenGL(window, true);
        ImGui_ImplOpenGL3_Init(glsl_version);
        ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

// setup full screen rendering
#ifdef IMGUI_HAS_VIEWPORT
        ImGuiViewport *viewport = ImGui::GetMainViewport();
        ImGui::SetNextWindowPos(viewport->GetWorkPos());
        ImGui::SetNextWindowSize(viewport->GetWorkSize());
        ImGui::SetNextWindowViewport(viewport->ID);
#else
        ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f));
        ImGui::SetNextWindowSize(ImGui::GetIO().DisplaySize);
#endif
        ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
        // Main loop
        while (!glfwWindowShouldClose(window)) {
            glfwPollEvents();
            // Start the Dear ImGui frame
            ImGui_ImplOpenGL3_NewFrame();
            ImGui_ImplGlfw_NewFrame();
            ImGui::NewFrame();

            static bool show_mrcap_main_window = true;
            static bool show_demo_window = false;
            static bool show_sdf_window = false;
            // static bool show_animation_window = false;
            static bool factor_graph_ran = false;
            static bool show_debug_window = true;
            static bool show_optimizer_window = true;
            static bool show_logger = true;
            static bool push_box = true;
            static bool use_sdf = false;
  
            static bool show_metrics_window = false;
            static bool use_custom_trajectory = false;


            if (show_debug_window) {
                // Debugging Window
                // set window size
                ImGui::SetNextWindowSize(ImVec2(400, 400), ImGuiCond_FirstUseEver);
                // set window location
                ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_FirstUseEver);
                ImGui::Begin("Debugging Window", &show_debug_window);

                const char* currentCovPresetName = covariancePresets[currentCovPreset].name.c_str();
                if (ImGui::BeginCombo("Covariance Presets", currentCovPresetName)) {
                    for (int i = 0; i < covariancePresets.size(); ++i) {
                        bool isSelected = (currentCovPreset == i);
                        if (ImGui::Selectable(covariancePresets[i].name.c_str(), isSelected)) {
                            currentCovPreset = i;
                            std::copy(std::begin(covariancePresets[i].covX), std::end(covariancePresets[i].covX), std::begin(covariance_X));
                            std::copy(std::begin(covariancePresets[i].covTernary), std::end(covariancePresets[i].covTernary), std::begin(covariance_ternary));
                            std::copy(std::begin(covariancePresets[i].covU), std::end(covariancePresets[i].covU), std::begin(covariance_U));
                            std::copy(std::begin(covariancePresets[i].covPriors), std::end(covariancePresets[i].covPriors), std::begin(covariance_priors));
                            std::copy(std::begin(covariancePresets[i].covObs), std::end(covariancePresets[i].covObs), std::begin(covariance_obs));
                            currentCovPresetName = covariancePresets[i].name.c_str();
                        }
                        if (isSelected) {
                            ImGui::SetItemDefaultFocus();
                        }
                    }
                    ImGui::EndCombo();
                }

                ImGui::Text("Covariance Options");

                ImGui::InputDouble("X (x)", &covariance_X[0], 100, 10000, "%.6f");
                ImGui::InputDouble("X (y)", &covariance_X[1], 100, 10000, "%.6f");
                ImGui::InputDouble("X (theta)", &covariance_X[2], 100, 1000, "%.6f");
                
                ImGui::InputDouble("U (u_L)", &covariance_U[0], 100, 10000, "%.6f");
                ImGui::InputDouble("U (u_R)", &covariance_U[1], 100, 10000, "%.6f");
                
                ImGui::InputDouble("Ternary (x_diff)", &covariance_ternary[0], 10, 100, "%.4f");
                ImGui::InputDouble("Ternary (y_diff)", &covariance_ternary[1], 10, 100, "%.4f");
                
                ImGui::InputDouble("Obstacle D", &covariance_obs[0], 0.00001, 0.001, "%e");

                ImGui::InputDouble("Anchor (x)", &covariance_priors[0], 0.00001, 0.001, "%e");
                ImGui::InputDouble("Anchor (y)", &covariance_priors[1], 0.00001, 0.001, "%e");
                ImGui::InputDouble("Anchor (theta)", &covariance_priors[2], 0.00001, 0.001, "%e");
                
                // Logger
                ImGui::Checkbox("Show Logger", &show_logger);
                ImGui::Checkbox("Print Factors", &optimization_parameter.print_fg_factors);
                ImGui::Checkbox("Print Initial Values", &optimization_parameter.print_fg_initial_values);
                ImGui::Checkbox("Print Iterated Results", &optimization_parameter.print_fg_iterated_results);
                ImGui::Checkbox("Print Reference Trajectories", &optimization_parameter.print_ref_traj);
                ImGui::Checkbox("Print Modelled Trajectories", &optimization_parameter.print_modelled_traj);
                ImGui::Checkbox("Print Velocities", &optimization_parameter.print_velocities);
                if (show_logger) {
                    logger.Draw("Application Log");
                }

                ImGui::End();
            }

            // Optimizer Window
            if (show_optimizer_window) {
                if (ImGui::Begin("Optimizer Window", &show_optimizer_window)) {
                    ImGui::Text("Optimizer Options");
                    ImGui::InputDouble("Max Iterations", &solver_parameters[0]);
                    ImGui::InputDouble("Relative Error Tol", &solver_parameters[1]);
                    ImGui::InputDouble("Absolute Error Tol", &solver_parameters[2]);
                    ImGui::InputDouble("Error Tol", &solver_parameters[3]);
                    ImGui::Text("LM Options");
                    ImGui::InputDouble("lambdaFactor", &optimization_parameter.lambdaFactor);
                    ImGui::InputDouble("lambdaInit", &optimization_parameter.lambdaInitial);
                    ImGui::InputDouble("lambdaLowerBound", &optimization_parameter.lambdaLowerBound);
                    ImGui::InputDouble("lambdaUpperBound", &optimization_parameter.lambdaUpperBound);
                    ImGui::Checkbox("diagonalDamping", &optimization_parameter.diagonalDamping);
                    ImGui::Checkbox("useFixedLambdaFactor", &optimization_parameter.useFixedLambdaFactor);

                    // dropdown window in imgui
                    if (ImGui::BeginCombo("Linear Solver Type", currentSolverType.c_str())) // Initialize with current selection
                    {
                        const char *solverTypes[] = {"MULTIFRONTAL_CHOLESKY", "MULTIFRONTAL_QR", "SEQUENTIAL_CHOLESKY", "SEQUENTIAL_QR", "CHOLMOD"};
                        for (int n = 0; n < IM_ARRAYSIZE(solverTypes); n++) {
                            bool isSelected = (currentSolverType == solverTypes[n]);
                            if (ImGui::Selectable(solverTypes[n], isSelected)) // Correctly mark the selected option
                            {
                                currentSolverType = solverTypes[n];
                                if (solver_parameters.size() > 4)
                                    solver_parameters[4] = n; // Change the 5th parameter, assuming solver_parameters has been initialized with enough size
                                else
                                    solver_parameters.push_back(n); // Or push_back if it's the first time setting
                            }
                            if (isSelected) {
                                ImGui::SetItemDefaultFocus();
                            }
                        }
                        ImGui::EndCombo();
                    }
                    ImGui::End();
                }
            }
            ImGui::SetNextWindowSizeConstraints(ImVec2(650, 650), ImVec2(FLT_MAX, ImGui::GetWindowHeight()));
            static ImGuiWindowFlags flags = ImGuiWindowFlags_MenuBar;
            
            if (show_mrcap_main_window) {
                ImGui::Begin("mrcap", &show_mrcap_main_window, flags);

                ImGui::PushItemWidth(ImGui::GetFontSize() * -20);


                // Position Presets Integration
                const char *currentPresetName = (currentPreset == -1) ? "<choose one>" : positionPresets[currentPreset].name.c_str();
                if (ImGui::BeginCombo("Position Presets", currentPresetName)) {
                    for (int i = 0; i < positionPresets.size(); ++i) {
                        bool isSelected = (currentPreset == i); // Check if current item is selected
                        if (ImGui::Selectable(positionPresets[i].name.c_str(), isSelected)) {
                            currentPreset = i;
                            ref_traj_start_pos[0] = positionPresets[i].startX;
                            ref_traj_start_pos[1] = positionPresets[i].startY;
                            ref_traj_start_pos[2] = positionPresets[i].startTheta;
                            ref_traj_end_pos[0] = positionPresets[i].goalX;
                            ref_traj_end_pos[1] = positionPresets[i].goalY;
                            ref_traj_end_pos[2] = positionPresets[i].goalTheta;
                        }
                        if (isSelected) {
                            ImGui::SetItemDefaultFocus();
                        }
                    }
                    ImGui::EndCombo();
                }
                static int traj_type = 0;
                if (ImGui::BeginTabBar("Trajectory Options", ImGuiTabBarFlags_None)) {
                    if (ImGui::BeginTabItem("Straight Line")) {
                        traj_type = 0;
                        ImGui::InputFloat3("Start Pt (x, y, theta)", ref_traj_start_pos);
                        ImGui::InputFloat3("End Pt (x, y, theta)", ref_traj_end_pos);
                        ImGui::EndTabItem();
                    }
                    if (ImGui::BeginTabItem("Sine Wave"))
                    { 
                        traj_type = 1;
                        ImGui::InputFloat3("Start Pt (x, y, theta)", ref_traj_start_pos);
                        ImGui::InputFloat3("End Pt (x, y, theta)", ref_traj_end_pos);
                        ImGui::InputFloat("Input Amplitude (m)", &ref_traj_amplitude);
                        ImGui::InputFloat("Input Angular Freq (rads)", &ref_traj_amplitude);
                        ImGui::EndTabItem();
                    }
                    if (ImGui::BeginTabItem("Circle")) {
                        traj_type = 2;
                        ImGui::InputFloat3("Start Pt (x, y, theta)", ref_traj_start_pos);
                        ImGui::InputFloat2("Input Centre of Rotation", ref_traj_center);
                        ImGui::EndTabItem();
                    }
                    ImGui::EndTabBar();
                }

                ImGui::InputInt("Number of steps", &optimization_parameter.nr_of_steps);
                ImGui::SameLine();
                HelpMarker("Steps which discretize the space");

                ImGui::InputInt("Time given to each step", &Ts);

                ImGui::SameLine();
                HelpMarker("Time given for each iteration");
                // Disturbance input
                ImGui::InputInt("Disturbance at pose: ", &disturbance_pose);
                ImGui::SameLine();
                HelpMarker("where the disturbance occurs (pose number 10 for example)");
                ImGui::InputInt("axis", &disturbance_axis);
                ImGui::SameLine();
                HelpMarker("which axis the disturbance occurs on (x or y) put 0 or 1");
                ImGui::InputDouble("Disturbance value: ", &disturbance_value);
                ImGui::SameLine();
                HelpMarker("how much the disturbance is (0.2 for example)");

                ImGui::Checkbox("Enable ROS nodes as well", &optimization_parameter.run_ros_nodes);
                ImGui::SameLine();
                HelpMarker("Enable ROS nodes");
                ImGui::Checkbox("avoid obstacles", &use_sdf);
                ImGui::InputInt("Number of robots", &optimization_parameter.nr_of_robots);
                ImGui::InputDouble("Centroid Rotation Threshold (deg)", &optimization_parameter.centroid_rotation_threshold);

                // geometry information
                geometry_information.robot_wheel_radius = r;
                geometry_information.robot_width = L;
                geometry_information.max_velocity = 0.25;
                // optimization parameteres
                optimization_parameter.time_for_robot_rotation = Ts;
                optimization_parameter.time_for_translation = Ts;
                optimization_parameter.time_for_centroid_rotation = Ts;
                optimization_parameter.obstacle_avoidance = use_sdf;
                optimization_parameter.use_custom_trajectory = use_custom_trajectory;

                optimization_parameter.collision_flag = 0;

                for (int i = 0; i <= 20; i++) {
                    optimization_parameter.custom_reference_trajectory[i] = custom_reference_trajectory[i];
                }

                if (ImGui::Button("Run Factorgraph") || ImGui::IsKeyPressed(ImGuiKey_R)) {
                    Utils::Disturbance disturbance = {disturbance_pose, disturbance_axis, disturbance_value};

                    ref_traj_start_pos[2] = Utils::ensure_orientation_range(ref_traj_start_pos[2]);
                    ref_traj_end_pos[2] = Utils::ensure_orientation_range(ref_traj_end_pos[2]);
                    gtsam::Pose2 ref_traj_start_pose(ref_traj_start_pos[0], ref_traj_start_pos[1], ref_traj_start_pos[2]);
                    gtsam::Pose2 ref_traj_end_pose(ref_traj_end_pos[0], ref_traj_end_pos[1], ref_traj_end_pos[2]);
                    gtsam::Pose2 ref_traj_center_of_rot(ref_traj_center[0], ref_traj_center[1], 0);

                    optimization_parameter.reference_trajectory_type = traj_type;
                    optimization_parameter.ref_traj_start_pose = ref_traj_start_pose;
                    optimization_parameter.ref_traj_end_pose = ref_traj_end_pose;
                    optimization_parameter.ref_traj_angular_frequency = ref_traj_angular_freq;
                    optimization_parameter.ref_traj_amplitude = ref_traj_amplitude;
                    optimization_parameter.ref_traj_center_of_rotation = ref_traj_center_of_rot;

                    gtsam::Vector3 covariance_X_vector(covariance_X[0], covariance_X[1], covariance_X[2]);
                    gtsam::Vector3 covariance_U_vector(covariance_U[0], covariance_U[1], 0.01);
                    gtsam::Vector3 covariance_ternary_vector(covariance_ternary[0], covariance_ternary[1], covariance_ternary[2]);
                    gtsam::Vector3 covariance_obs_vector(covariance_obs[0], 0, 0);
                    gtsam::Vector3 covariance_priors_vector(covariance_priors[0], covariance_priors[1], covariance_priors[2]);

                    std::vector<gtsam::Vector3> covariance_information;
                    covariance_information.push_back(covariance_X_vector);
                    covariance_information.push_back(covariance_U_vector);
                    covariance_information.push_back(covariance_ternary_vector);
                    covariance_information.push_back(covariance_obs_vector);
                    covariance_information.push_back(covariance_priors_vector);

                    std::vector<double> radiuses;
                    std::vector<double> thetas;
                    for(int i = 0; i < optimization_parameter.nr_of_robots; i++){
                        radiuses.push_back(0.6);
                        double theta;
                        switch(i % 4) {
                            case 0: theta = M_PI_4; break;       // i % 4 == 0 corresponds to pi/4
                            case 1: theta = 3 * M_PI_4; break;   // i % 4 == 1 corresponds to 3pi/4
                            case 2: theta = 5 * M_PI_4; break;   // i % 4 == 2 corresponds to 5pi/4
                            case 3: theta = 7 * M_PI_4; break;   // i % 4 == 3 corresponds to 7pi/4
                        }
                        thetas.push_back(theta);
                    }
                    geometry_information.distance_to_robot = radiuses;
                    geometry_information.angle_to_robot = thetas;

                    auto returned_info = PointMotion(optimization_parameter, geometry_information, covariance_information, disturbance, solver_parameters, sdf_s);

                    robots = returned_info.first;
                    centroid = returned_info.second;

                    plotter.Initialize(robots, centroid, optimization_parameter, geometry_information);
                    factor_graph_ran = true;
                }

                ImGui::Checkbox("Show obstacle settings", &show_sdf_window);
                ImGui::SameLine();
                HelpMarker("Show the settings for the obstacle avoidance");

                ImGui::Checkbox("Show covariance settings", &show_debug_window);


                if (factor_graph_ran) {
                    Utils::Disturbance disturbance = {disturbance_pose, disturbance_axis, disturbance_value};
                    if (ImPlot::BeginPlot("Environment", ImVec2(500, 500))) {
                    plotter.run(sdf_s);
                    ImPlot::EndPlot();                 
                    }
                }
                if (show_sdf_window) {
                    ImGui::Begin("SDF", &show_sdf_window);
                    ImGui::InputInt("Number of Obstacles", &nr_of_obstacles);

                    if (nr_of_obstacles != sdf_s.obstacles.size()) {
                        sdf_s.obstacles.resize(nr_of_obstacles);
                    }

                    for (int i = 0; i < nr_of_obstacles; ++i) {
                        std::string labelX = "X_obs_" + std::to_string(i + 1);
                        std::string labelY = "Y_obs_" + std::to_string(i + 1);
                        ImGui::InputDouble(labelX.c_str(), &sdf_s.obstacles[i].x);
                        ImGui::InputDouble(labelY.c_str(), &sdf_s.obstacles[i].y);
                    }
                    ImGui::InputDouble("System Radius", &sdf_s.system_radius);
                    ImGui::InputDouble("Safety Radius", &sdf_s.safety_radius);
                    ImGui::End();
                }

                ImGui::PopItemWidth();
                ImGui::End();
            }

            ImGui::Render();
            int display_w, display_h;
            glfwGetFramebufferSize(window, &display_w, &display_h);
            glViewport(0, 0, display_w, display_h);
            glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
            glClear(GL_COLOR_BUFFER_BIT);
            ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

            glfwSwapBuffers(window);
        }

        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();
        ImPlot::DestroyContext();

        glfwDestroyWindow(window);
        glfwTerminate();
    }
};