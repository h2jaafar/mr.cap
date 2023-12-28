#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "gazebo_msgs/msg/link_states.hpp"
#include "gazebo_msgs/msg/model_states.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>

// ==================================== ROS2 subscriber and publisher =========================================

using std::placeholders::_1;

struct ROSPose {
  double x;
  double y;
  double yaw;
};

std::vector<ROSPose> robot_poses(4);
ROSPose centroid_pose;


class RefTrajectoryPublisher : public rclcpp::Node {
  public:
  RefTrajectoryPublisher(const CentroidData& centroid, const bool is_simulation)
    : Node("ref_trajectory_publisher"), centroid_(centroid)
  {
    if (is_simulation) {
        ref_trajectory_msg_.header.frame_id = "map"; // Change to your frame ID if needed
    } else {
        ref_trajectory_msg_.header.frame_id = "vicon/world"; // Change to your frame ID if needed
    }
  }

  void publish()
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = this->now();
    pose.header.frame_id = ref_trajectory_msg_.header.frame_id;
    for (int i = 0; i < centroid_.X_k_fg.size(); i++)
    {
      pose.pose.position.x = centroid_.X_k_fg[i].x();
      pose.pose.position.y = centroid_.X_k_fg[i].y();
      pose.pose.position.z = 0;
      tf2::Quaternion quaternion;
      quaternion.setRPY(0.0, 0.0, centroid_.X_k_fg[i].theta());
      pose.pose.orientation.x = quaternion.x();
      pose.pose.orientation.y = quaternion.y();
      pose.pose.orientation.z = quaternion.z();
      pose.pose.orientation.w = quaternion.w();
      ref_trajectory_msg_.poses.push_back(pose);
    }
    ref_trajectory_pub_->publish(ref_trajectory_msg_);
  }
  private:
  CentroidData centroid_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr ref_trajectory_pub_;
  nav_msgs::msg::Path ref_trajectory_msg_;
};

class GazeboSubscriber : public rclcpp::Node {
public:
  GazeboSubscriber(const std::string& robot_id)
    : Node("gazebo_subs_" + robot_id), robot_id_(robot_id)
  {
    subscription_ = this->create_subscription<gazebo_msgs::msg::ModelStates>(
      "gazebo/model_states", 5, std::bind(&GazeboSubscriber::topicCallback, this, std::placeholders::_1));
          real_trajectory_msg_.header.frame_id = "map"; // Change to your frame ID if needed
    // ref_trajectory_msg_.header.frame_id = "vicon/world"; // Change to your frame ID if needed
    real_trajectory_pub_ = this->create_publisher<nav_msgs::msg::Path>("real_traj", 10);
    
  }

private:
  void topicCallback(gazebo_msgs::msg::ModelStates::SharedPtr msg)
  {
    int gazebo_index;
    int robot_id;
    double r_1 = 1.41421356237;
    double theta_1 = M_PI/4;
    if(robot_id_ == "C")
    {
      // centroid in gazebo will just be the midpoint between all the robots, and yaw will be the yaw of the first robot
      tf2::Quaternion q1(
      msg->pose[1].orientation.x,
      msg->pose[1].orientation.y,
      msg->pose[1].orientation.z,
      msg->pose[1].orientation.w);
      tf2::Matrix3x3 m1(q1);
      double roll1, pitch1, yaw_r1;
      m1.getRPY(roll1, pitch1, yaw_r1);
      gtsam::Pose2 robot_1_pose_gtsam(msg->pose[1].position.x, msg->pose[1].position.y, yaw_r1);
      tf2::Quaternion q2(
      msg->pose[2].orientation.x,
      msg->pose[2].orientation.y,
      msg->pose[2].orientation.z,
      msg->pose[2].orientation.w);
      tf2::Matrix3x3 m2(q2);
      double roll2, pitch2, yaw_r2;
      m2.getRPY(roll2, pitch2, yaw_r2);
      gtsam::Pose2 robot_2_pose_gtsam(msg->pose[2].position.x, msg->pose[2].position.y, yaw_r2);
      gtsam::Pose2 centroid_pose_gtsam = Geometry::solveForCentroidPose(robot_1_pose_gtsam, robot_2_pose_gtsam, r_1, theta_1);
      centroid_pose.x = centroid_pose_gtsam.x();
      centroid_pose.y = centroid_pose_gtsam.y();
      centroid_pose.yaw = centroid_pose_gtsam.theta();

      // RCLCPP_INFO(this->get_logger(), "Subscribed: (x, y, yaw): (%f, %f, %f)", centroid_pose.x, centroid_pose.y, centroid_pose.yaw);
    }
    else{
    robot_id = std::stoi(robot_id_);
    robot_poses[robot_id-1].x = msg->pose[robot_id].position.x;
    robot_poses[robot_id-1].y = msg->pose[robot_id].position.y;
    double tz = msg->pose[robot_id].position.z;
    tf2::Quaternion q(
    msg->pose[robot_id].orientation.x,
    msg->pose[robot_id].orientation.y,
    msg->pose[robot_id].orientation.z,
    msg->pose[robot_id].orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, robot_poses[robot_id-1].yaw);
    // RCLCPP_INFO(this->get_logger(), "Subscribed: (x, y, yaw): (%f, %f, %f)", robot_poses[robot_id-1].x, robot_poses[robot_id-1].y, robot_poses[robot_id-1].yaw);
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = this->now();
    pose.header.frame_id = "vicon/world"; // Change to your frame ID if needed

    if(robot_id_ == "C")
    {
    pose.pose.position.x = centroid_pose.x;
    pose.pose.position.y = centroid_pose.y;
    pose.pose.position.z = 0;
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, centroid_pose.yaw);
    pose.pose.orientation.x = quaternion.x();
    pose.pose.orientation.y = quaternion.y();
    pose.pose.orientation.z = quaternion.z();
    pose.pose.orientation.w = quaternion.w();
    real_trajectory_msg_.poses.push_back(pose);
    real_trajectory_pub_->publish(real_trajectory_msg_);
    }
  }
  std::string robot_id_;
  rclcpp::Subscription<gazebo_msgs::msg::ModelStates>::SharedPtr subscription_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr real_trajectory_pub_;
  nav_msgs::msg::Path real_trajectory_msg_;
};

class ViconSubscriber : public rclcpp::Node {
public:
  ViconSubscriber(const std::string& robot_id)
    : Node("vicon_subscriber_" + robot_id),  robot_id_(robot_id)
  {
    std::string topic = "vicon/B" + robot_id + "/B" + robot_id;
    subscription_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(
      topic, 10, std::bind(&ViconSubscriber::topicCallback, this, std::placeholders::_1));
    real_trajectory_msg_.header.frame_id = "vicon/world"; // Change to your frame ID if needed
    real_trajectory_pub_ = this->create_publisher<nav_msgs::msg::Path>("real_traj", 10);
  }

private:
  void topicCallback(geometry_msgs::msg::TransformStamped::SharedPtr msg) {
    double tz;
    int robot_id;
    if(robot_id_ == "C")
    {
    centroid_pose.x = msg->transform.translation.x;
    centroid_pose.y = msg->transform.translation.y;
    tz = msg->transform.translation.z;
    }
    else{
    
    robot_id = std::stoi(robot_id_);
    robot_poses[robot_id-1].x = msg->transform.translation.x;
    robot_poses[robot_id-1].y = msg->transform.translation.y;
    tz = msg->transform.translation.z;
    }

    tf2::Quaternion q(
      msg->transform.rotation.x,
      msg->transform.rotation.y,
      msg->transform.rotation.z,
      msg->transform.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch;
    if (robot_id_ == "C")
    {
    m.getRPY(roll, pitch, centroid_pose.yaw);
    }
    else{
    m.getRPY(roll, pitch, robot_poses[robot_id-1].yaw);
    // RCLCPP_INFO(this->get_logger(), "Subscribed (x,y,yaw): (%f,%f,%f)", robot_poses[robot_id-1].x, robot_poses[robot_id-1].y, robot_poses[robot_id-1].yaw);
    }
    // Visualize the trajectory
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = this->now();
    pose.header.frame_id = "vicon/world"; // Change to your frame ID if needed
    if(robot_id_ == "C")
    {
    pose.pose.position.x = centroid_pose.x;
    pose.pose.position.y = centroid_pose.y;
    pose.pose.position.z = 0;
    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, centroid_pose.yaw);
    pose.pose.orientation.x = quaternion.x();
    pose.pose.orientation.y = quaternion.y();
    pose.pose.orientation.z = quaternion.z();
    pose.pose.orientation.w = quaternion.w();
    real_trajectory_msg_.poses.push_back(pose);
    real_trajectory_pub_->publish(real_trajectory_msg_);
    }
  }
  rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr subscription_;
  std::string robot_id_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr real_trajectory_pub_;
  nav_msgs::msg::Path real_trajectory_msg_;
};


using namespace std::chrono_literals;

class VelocityPublisher : public rclcpp::Node {
public:
  VelocityPublisher(const std::string& robot_id)
    : Node("publisher_" + robot_id), robot_id_(robot_id)
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("B" + robot_id + "/cmd_vel", 10);
  }

  void publish(double cmd_vel_x, double cmd_vel_w)
  {
    geometry_msgs::msg::Twist twist;
    twist.linear.x = cmd_vel_x;
    twist.angular.z = cmd_vel_w;
    if((cmd_vel_x >0) || (cmd_vel_w > 0)){
    // RCLCPP_INFO(this->get_logger(), "Publishing (robot_id=%s): (v, w) ('%f', '%f')", robot_id_.c_str(), cmd_vel_x, cmd_vel_w);
    }
    publisher_->publish(twist);
  }
  private:
  std::string robot_id_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};