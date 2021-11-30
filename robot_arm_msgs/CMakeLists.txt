cmake_minimum_required(VERSION 3.5)
project(robot_arm_msgs)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GripperPose.srv"
    DEPENDENCIES geometry_msgs
)
ament_package()