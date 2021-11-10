#!/usr/bin/env python3

import threading

import rclpy
from robot_arm_commander.move_group_interface import MoveGroupInterface
from geometry_msgs.msg import Quaternion, PoseStamped, Pose
import time
from math import sin, cos, pi


#refer https://index.ros.org/doc/ros2/Tutorials/URDF/Using-URDF-with-Robot-State-Publisher/
def euler_to_quaternion(roll, pitch, yaw):
  qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
  qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
  qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
  qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
  return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main(args=None):
    rclpy.init(args=args)

    # Initialise MoveIt2
    moveit2 = MoveGroupInterface("robot_arm","base_mount")

    # Spin MoveIt2 node in the background
    executor = rclpy.executors.MultiThreadedExecutor(1)
    executor.add_node(moveit2)
    thread = threading.Thread(target=executor.spin)
    thread.start()

    # Set pose goal to reach
    while(1): 
        target_pose = Pose()
        target_pose.orientation.x = -1.155227619165089e-05
        target_pose.orientation.y = 0.46281740069389343
        target_pose.orientation.z = -1.7791414848034037e-06
        target_pose.orientation.w = 0.8864536285400391
        target_pose.position.x = 0.23239827156066895
        target_pose.position.y = 0.034903883934020996
        target_pose.position.z = 0.5444427728652954


        target_posestamped = PoseStamped()
        # target_posestamped.header.stamp = self.get_clock().now().to_msg()
        target_posestamped.header.frame_id = "base_mount"
        target_posestamped.pose = target_pose
        moveit2.moveToPose(target_posestamped, "gripper_link")
        time.sleep(5)
   
    rclpy.shutdown()


if __name__ == "__main__":
    main()
