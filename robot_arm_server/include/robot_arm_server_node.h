
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


using namespace std::chrono_literals;
using std::placeholders::_1;

class RobotArmServer : public rclcpp::Node
{
  public:
    RobotArmServer();

  private:
};

#endif

