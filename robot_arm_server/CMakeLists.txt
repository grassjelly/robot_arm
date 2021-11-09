cmake_minimum_required(VERSION 3.5)
project(robot_arm_server)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)


include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(robot_arm_server_node src/robot_arm_server_node.cpp)

add_executable(robot_arm_server src/robot_arm_server.cpp)
target_link_libraries(robot_arm_server robot_arm_server_node)
ament_target_dependencies(robot_arm_server rclcpp Eigen3)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS robot_arm_server
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()