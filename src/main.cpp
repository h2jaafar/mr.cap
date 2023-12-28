#include <stdio.h>
#include <iostream>
#include <mrcap/MainWindow.h>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"



#include <stdio.h>
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <GLES2/gl2.h>
#endif
#include <GLFW/glfw3.h> // Will drag system OpenGL headers
using std::placeholders::_1;

int main()
{  
  std::cout << "Start" << std::endl;
  MainWindow gui;
  gui.runGui();
}