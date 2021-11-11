# # #!/usr/bin/env python3
# import time
# import threading
# import rclpy
# from robot_arm_commander.move_group_interface import MoveGroupInterface
# from robot_arm_commander.gripper import Gripper

# from geometry_msgs.msg import PoseStamped, Pose


# def main(args=None):
#     rclpy.init(args=args)

#     manipulator = MoveGroupInterface("robot_arm", "base_mount", "tool_link")
#     gripper = Gripper()
#     executor = rclpy.executors.MultiThreadedExecutor(2)
#     executor.add_node(manipulator)
#     executor.add_node(gripper)
#     thread = threading.Thread(target=executor.spin)
#     thread.start()
    
#     joint_names = ['pan_joint', 'shoulder_joint', 'elbow_joint', 'wrist0_joint', 'wrist1_joint', 'wrist2_joint']
#     while(1): 
#         gripper.set_pos(0.0225)

#         pose_msg = PoseStamped()
#         pose_msg.pose.orientation.x = -1.155227619165089e-05
#         pose_msg.pose.orientation.y = 0.46281740069389343
#         pose_msg.pose.orientation.z = -1.7791414848034037e-06
#         pose_msg.pose.orientation.w = 0.8864536285400391
#         pose_msg.pose.position.x = 0.23239827156066895
#         pose_msg.pose.position.y = 0.034903883934020996
#         pose_msg.pose.position.z = 0.5444427728652954
#         manipulator.move_to_pose(pose_msg)

#         gripper.open()

#         joint_pos = [0.0] * 6
#         manipulator.move_to_joint_position(joint_names, joint_pos)

#         gripper.close()
#         time.sleep(5)

#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()
# #!/usr/bin/env python3
import time
import threading
import rclpy
from robot_arm_commander.move_group_interface import MoveGroupInterface
from robot_arm_commander.gripper import Gripper

from geometry_msgs.msg import PoseStamped, Pose


def main(args=None):
    rclpy.init(args=args)

    manipulator = MoveGroupInterface("robot_arm", "base_mount", "tool_link")
    gripper = Gripper()
    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(manipulator)
    executor.add_node(gripper)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    
    joint_names = ['pan_joint', 'shoulder_joint', 'elbow_joint', 'wrist0_joint', 'wrist1_joint', 'wrist2_joint']
    while(1): 
        gripper.open()
        pose_msg = PoseStamped()
        pose_msg.pose.orientation.x = -9.220700007972482e-07
        pose_msg.pose.orientation.y = 1.0
        pose_msg.pose.orientation.z = 7.551162752861273e-07
        pose_msg.pose.orientation.w = -5.477435843204148e-05
        pose_msg.pose.position.x = 0.192021444439888
        pose_msg.pose.position.y = -0.12776489555835724
        pose_msg.pose.position.z = 0.11563699692487717
        manipulator.move_to_pose(pose_msg)

        pose_msg.pose.orientation.x = -1.020348690872197e-06
        pose_msg.pose.orientation.y = 1.0
        pose_msg.pose.orientation.z = 1.022529431793373e-06
        pose_msg.pose.orientation.w = -5.4606909543508664e-05
        pose_msg.pose.position.x = 0.19201216101646423
        pose_msg.pose.position.y = -0.1277647763490677
        pose_msg.pose.position.z = 0.031097371131181717
        manipulator.move_to_pose(pose_msg)
        gripper.close()

        pose_msg = PoseStamped()
        pose_msg.pose.orientation.x = -9.220700007972482e-07
        pose_msg.pose.orientation.y = 1.0
        pose_msg.pose.orientation.z = 7.551162752861273e-07
        pose_msg.pose.orientation.w = -5.477435843204148e-05
        pose_msg.pose.position.x = 0.192021444439888
        pose_msg.pose.position.y = -0.12776489555835724
        pose_msg.pose.position.z = 0.11563699692487717
        manipulator.move_to_pose(pose_msg)

        joint_pos = [0.0] * 6
        manipulator.move_to_joint_position(joint_names, joint_pos)

        time.sleep(5)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
