
#ifndef ROBOT_ARM_SERVER_H
#define ROBOT_ARM_SERVER_H

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;
using std::placeholders::_1;

class RobotArmServer : public rclcpp::Node
{
  public:
    RobotArmServer();

  private:
};

#endif

