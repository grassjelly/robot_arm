
#ifndef PICK_N_PLACE_SERVER_H
#define PICK_N_PLACE_SERVER_H

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;
using std::placeholders::_1;

class PickNPlaceServer : public rclcpp::Node
{
  public:
    PickNPlaceServer();

  private:
};

#endif

