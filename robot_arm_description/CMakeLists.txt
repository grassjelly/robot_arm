cmake_minimum_required(VERSION 3.5)
project(robot_arm_description)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)

install(
  DIRECTORY launch urdf rviz config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
