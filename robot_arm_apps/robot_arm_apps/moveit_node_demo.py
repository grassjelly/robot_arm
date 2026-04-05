#!/usr/bin/env python3
"""
Pick-and-place demo using the robot_arm_python library, wrapped in a ROS2 node.

MoveIt and Gripper both manage their own internal executors, so only this node
needs to be added to the MultiThreadedExecutor.
"""
import gc

import rclpy
import rclpy.logging
import tf2_ros
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from robot_arm_python.moveit import MoveIt
from robot_arm_python.gripper import Gripper
from robot_arm_python.utils import make_pose, transform_pose


class RobotArmNode(Node):
    def __init__(self) -> None:
        super().__init__("robot_arm_node")

        # Both objects are self-contained — no executor management needed here.
        self.robot = MoveIt()
        self.gripper = Gripper()

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Defer motion until the executor is spinning.
        self._timer = self.create_timer(1.0, self._run)

    def _run(self) -> None:
        self._timer.destroy()

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
            pose=transform_pose(
                make_pose("base_mount", x=0.1, y=0.0, z=0.1),
                self._tf_buffer,
                source_frame="base_mount",
                target_frame="tool_link",
            )
        )
        self.gripper.open()

        # ------------------------------------------------------------------
        # 3. Descend to grasp pose and close gripper
        # ------------------------------------------------------------------
        log.info("Step 3: descending to grasp pose")
        self.robot.move_to(
            pose=transform_pose(
                make_pose("base_mount", x=0.1, y=0.0, z=0.0),
                self._tf_buffer,
                source_frame="base_mount",
                target_frame="tool_link",
            )
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
        self.destroy_node()
        rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RobotArmNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
