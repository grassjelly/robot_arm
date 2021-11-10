
#ifndef ROBOT_ARM_SERVER_H
#define ROBOT_ARM_SERVER_H

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <cstdlib>
#include <memory>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/pose.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

class RobotArmServer : public rclcpp::Node
{
  public:
    RobotArmServer();
    void init();
    rclcpp::NodeOptions node_options_;
    bool moveToPose(geometry_msgs::msg::Pose target_pose);

  private:
    void testLoop();
    // rclcpp::executors::SingleThreadedExecutor executor_;

    // rclcpp::Node::SharedPtr move_group_node_;
    // moveit::planning_interface::MoveGroupInterface move_group_;

    // const moveit::core::JointModelGroup* joint_model_group_;
    // const std::string planning_group_ = "robot_arm";
    // rclcpp::TimerBase::SharedPtr loop_timer_;

};

#endif

