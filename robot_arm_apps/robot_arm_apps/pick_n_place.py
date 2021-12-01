# # #!/usr/bin/env python3
import time
import threading
import rclpy
from robot_arm_commander.move_group_interface import MoveGroupInterface
from robot_arm_commander.gripper import Gripper
from robot_arm_perception.perception_client import PerceptionClient
from geometry_msgs.msg import PoseStamped, Pose


def main(args=None):
    rclpy.init(args=args)

    manipulator = MoveGroupInterface("robot_arm", "base_mount", "tool_link")
    gripper = Gripper()
    perception_client = PerceptionClient()

    executor = rclpy.executors.MultiThreadedExecutor(3)
    executor.add_node(manipulator)
    executor.add_node(gripper)
    executor.add_node(perception_client)

    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    gripper.open()
    perception_client.send_request()
    joint_names = ['pan_joint', 'shoulder_joint', 'elbow_joint', 'wrist0_joint', 'wrist1_joint', 'wrist2_joint']

    while(1): 
        poses = perception_client.get_poses()
        if poses:
            for pose in poses:
                ps = PoseStamped()
                ps.pose = pose
                manipulator.move_to_pose(ps)
                time.sleep(2)
                joint_pos = [0.0] * 6
                manipulator.move_to_joint_position(joint_names, joint_pos)
            break
        else:
            pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()