#include <robot_arm_server_node.h>

RobotArmServer::RobotArmServer(): 
  Node("robot_arm_server")
{
  node_options_.automatically_declare_parameters_from_overrides(true);
  // move_group_node_ = shared_from_this();
  // move_group_node_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(&node_ptr)
  // move_group_node_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this());

  // joint_model_group_ = move_group_.getCurrentState()->getJointModelGroup(planning_group_);

  // RCLCPP_INFO(this->get_logger(), "Planning frame: %s", move_group_.getPlanningFrame().c_str());
  // RCLCPP_INFO(this->get_logger(), "End effector link: %s", move_group_.getEndEffectorLink().c_str());
  // RCLCPP_INFO(this->get_logger(), "Available Planning Groups:");
  // std::copy(
  //   move_group_.getJointModelGroupNames().begin(),
  //   move_group_.getJointModelGroupNames().end(),
  //   std::ostream_iterator<std::string>(std::cout, ", ")
  // );

  geometry_msgs::msg::Pose target_pose;
  target_pose.orientation.x = -1.155227619165089e-05;
  target_pose.orientation.y = 0.46281740069389343;
  target_pose.orientation.z = -1.7791414848034037e-06;
  target_pose.orientation.w = 0.8864536285400391;
  target_pose.position.x = 0.23239827156066895;
  target_pose.position.y = 0.034903883934020996;
  target_pose.position.z = 0.5444427728652954;

  bool ret = moveToPose(target_pose);
  // std::cout << ret << std::endl;

}

void RobotArmServer::testLoop()
{
    RCLCPP_INFO(this->get_logger(), "hi");

}

bool RobotArmServer::moveToPose(geometry_msgs::msg::Pose target_pose)
{
  auto move_group_node = rclcpp::Node::make_shared("move_group_interface_tutorial", node_options_);

  // rclcpp::executors::SingleThreadedExecutor executor;
  // executor.add_node(move_group_node);
  // std::thread([&executor]() { executor.spin(); }).detach();

  static const std::string PLANNING_GROUP = "robot_arm";
  
  moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
  const moveit::core::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // RCLCPP_INFO(LOGGER, "Planning frame: %s", move_group.getPlanningFrame().c_str());
  // RCLCPP_INFO(LOGGER, "End effector link: %s", move_group.getEndEffectorLink().c_str());
  // RCLCPP_INFO(LOGGER, "Available Planning Groups:");
  // std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
  //           std::ostream_iterator<std::string>(std::cout, ", "));


  // .. _move_group_interface-planning-to-pose-goal:
  //
  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the
  // end-effector.

  // geometry_msgs::msg::Pose target_pose;
  // target_pose.orientation.x = -1.155227619165089e-05;
  // target_pose.orientation.y = 0.46281740069389343;
  // target_pose.orientation.z = -1.7791414848034037e-06;
  // target_pose.orientation.w = 0.8864536285400391;
  // target_pose.position.x = 0.23239827156066895;
  // target_pose.position.y = 0.034903883934020996;
  // target_pose.position.z = 0.5444427728652954;

  move_group.setPoseTarget(target_pose);

  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  RCLCPP_INFO(this->get_logger(), "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  move_group.move();
}