#!/usr/bin/env python3
"""
Pick-and-place demo using the robot_arm_python library, wrapped in a ROS2 node.

MoveIt and Gripper both manage their own internal executors, so only this node
needs to be added to the MultiThreadedExecutor.
"""
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from robot_arm_python.moveit import MoveIt
from robot_arm_python.gripper import Gripper


def make_pose(frame_id, x, y, z, qx=0.0, qy=1.0, qz=0.0, qw=0.0) -> PoseStamped:
    ps = PoseStamped()
    ps.header.frame_id = frame_id
    ps.pose.position.x = x
    ps.pose.position.y = y
    ps.pose.position.z = z
    ps.pose.orientation.x = qx
    ps.pose.orientation.y = qy
    ps.pose.orientation.z = qz
    ps.pose.orientation.w = qw
    return ps


class RobotArmNode(Node):
    def __init__(self) -> None:
        super().__init__("robot_arm_node")

        # Both objects are self-contained — no executor management needed here.
        self.robot = MoveIt()
        self.gripper = Gripper()

        self._ran = False
        # Defer motion until the executor is spinning.
        self.create_timer(1.0, self._run)

    def _run(self) -> None:
        if self._ran:
            return
        self._ran = True

        log = self.get_logger()

        # ------------------------------------------------------------------
        # 1. Go home
        # ------------------------------------------------------------------
        log.info("Step 1: moving to 'home' configuration")
        self.robot.move_to(configuration_name="home")

        # ------------------------------------------------------------------
        # 2. Move to pre-grasp pose
        # ------------------------------------------------------------------
        log.info("Step 2: moving to pre-grasp pose")
        self.robot.move_to(
            pose=make_pose("base_mount", x=0.192, y=-0.128, z=0.116)
        )
        self.gripper.open()

        # ------------------------------------------------------------------
        # 3. Descend to grasp pose and close gripper
        # ------------------------------------------------------------------
        log.info("Step 3: descending to grasp pose")
        self.robot.move_to(
            pose=make_pose("base_mount", x=0.192, y=-0.128, z=0.031)
        )
        self.gripper.close()

        # ------------------------------------------------------------------
        # 4. Move to drop pose via joint-space goal
        # ------------------------------------------------------------------
        log.info("Step 4: moving to drop pose (joint-space)")
        self.robot.move_to(
            joint_positions={
                "pan_joint":      -0.925,
                "shoulder_joint":  0.405,
                "elbow_joint":     1.080,
                "wrist0_joint":    0.006,
                "wrist1_joint":    1.635,
                "wrist2_joint":    0.577,
            }
        )
        self.gripper.open()

        # ------------------------------------------------------------------
        # 5. Return home
        # ------------------------------------------------------------------
        log.info("Step 5: returning home")
        self.robot.move_to(configuration_name="home")

        log.info("Demo complete")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RobotArmNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
