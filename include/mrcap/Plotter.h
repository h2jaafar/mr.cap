#pragma once
#include "../imgui/imgui.h"
#include "../imgui/imgui_impl_glfw.h"
#include "../imgui/imgui_impl_opengl3.h"
#include "../implot/implot.h"
#include "../implot/implot_internal.h"
// #include "MotionModel.h"
#include <gtsam/geometry/Pose2.h>
#include "DataTypes/RobotData.h"
#include <vector>

class Plotter {
private:

    bool initialized = false;
    std::vector<RobotData> robots_;
    CentroidData centroid_;
    int nr_of_robots;

public:
    void Initialize(const std::vector<RobotData> &robots, const CentroidData &centroid, 
                    const Optimization_parameter &optimization_parameter,
                    const Geometry_information &geometry_information) {

        nr_of_robots = optimization_parameter.nr_of_robots;
        robots_ = robots;
        centroid_ = centroid;
    }

    void PlotSDF(SDF_s sdf) {
        ImPlot::PushStyleVar(ImPlotStyleVar_Marker, ImPlotMarker_Square);
        ImPlot::PushStyleVar(ImPlotStyleVar_MarkerSize, 10);
        ImPlot::PushStyleVar(ImPlotStyleVar_MarkerWeight, 2);
        for(auto&& obstacle: sdf.obstacles) {
            ImPlot::PlotScatter("Obstacle", &obstacle.x, &obstacle.y, 1);
        }
        ImPlot::PopStyleVar(3);
    }


    void PlotPose2(std::string plot_name, std::vector<gtsam::Pose2> poses) {
        std::vector<float> x_array;
        std::vector<float> y_array;
        for (const auto &pose : poses) {
            x_array.push_back(pose.x());
            y_array.push_back(pose.y());
        }
        if (x_array.size() > 0 && y_array.size() > 0) {
            ImPlot::PlotLine(plot_name.c_str(), x_array.data(), y_array.data(), x_array.size());
            ImPlot::PlotScatter(plot_name.c_str(), x_array.data(), y_array.data(), x_array.size());
        }
    }

    void run(SDF_s sdf) {

        PlotSDF(sdf);
        PlotPose2("Centroid Real", centroid_.X_k_real);
        PlotPose2("Centroid Ref", centroid_.X_k_ref);
    }
};
