# #!/usr/bin/env python3
import time
import threading
import rclpy
from robot_arm_commander.move_group_interface import MoveGroupInterface
from geometry_msgs.msg import PoseStamped, Pose


def main(args=None):
    rclpy.init(args=args)

    manipulator = MoveGroupInterface("robot_arm", "base_mount", "tool_link")

    executor = rclpy.executors.MultiThreadedExecutor(1)
    executor.add_node(manipulator)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    
    joint_names = ['pan_joint', 'shoulder_joint', 'elbow_joint', 'wrist0_joint', 'wrist1_joint', 'wrist2_joint']
    while(1): 
        pose_msg = PoseStamped()
        pose_msg.pose.orientation.x = -1.155227619165089e-05
        pose_msg.pose.orientation.y = 0.46281740069389343
        pose_msg.pose.orientation.z = -1.7791414848034037e-06
        pose_msg.pose.orientation.w = 0.8864536285400391
        pose_msg.pose.position.x = 0.23239827156066895
        pose_msg.pose.position.y = 0.034903883934020996
        pose_msg.pose.position.z = 0.5444427728652954
        manipulator.move_to_pose(pose_msg)

        joint_pos = [0.0] * 6
        manipulator.move_to_joint_position(joint_names, joint_pos)
   
    rclpy.shutdown()

if __name__ == "__main__":
    main()
