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

    manipulator = MoveGroupInterface(
        "robot_arm", 
        "base_mount", 
        "tool_link"
    )
    gripper = Gripper()
    perception_client = PerceptionClient()

    executor = rclpy.executors.MultiThreadedExecutor(3)
    executor.add_node(manipulator)
    executor.add_node(gripper)
    executor.add_node(perception_client)

    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    perception_client.send_request()
    joint_names = ['pan_joint', 'shoulder_joint', 'elbow_joint', 'wrist0_joint', 'wrist1_joint', 'wrist2_joint']
    drop_pos = [-0.9249904150950417, 0.4049709280018093, 1.0799224746714913, 0.006135923151542565, 1.6352235198860934, 0.5767767762450011]
    home_pose = [-1.533980787885641, -0.8513593372765309, 2.389942067525829, 0.01227184630308513, 1.5984079809768381, -0.01687378866674205]
                
    gripper.open()
    manipulator.move_to_joint_position(joint_names, home_pose)
    while(1): 
        poses = perception_client.get_poses()
        if poses:
            for pose in poses:
                ps = PoseStamped()
                ps.pose = pose
                ps.pose.position.x -= 0.01
                ps.pose.position.y += 0.01
                ps.pose.position.z = 0.005

                ps.pose.position.z += 0.10
                manipulator.move_to_pose(ps)
                time.sleep(1)

                ps.pose.position.z -= 0.10
                manipulator.move_to_pose(ps)
                time.sleep(1)
                gripper.close()

                ps.pose.position.z += 0.10
                manipulator.move_to_pose(ps)
                time.sleep(1)

                manipulator.move_to_joint_position(joint_names, drop_pos)
                gripper.open()
            break
        else:
            pass

        rclpy.spin_once(perception_client)

    manipulator.move_to_joint_position(joint_names, home_pose)
    rclpy.shutdown()


if __name__ == "__main__":
    main()