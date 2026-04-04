#!/usr/bin/env python3
"""
MoveItPy demo: plan and execute motions using pose goals and joint goals.

References:
  Planning scene monitor : https://moveit.picknik.ai/main/doc/concepts/planning_scene_monitor.html
  MoveItPy node params   : https://github.com/moveit/moveit2/issues/2906#issuecomment-2259711256
  Jupyter tutorial       : https://github.com/moveit/moveit2_tutorials/.../jupyter_notebook_prototyping_tutorial.rst
  RobotState API         : https://github.com/moveit/moveit2/blob/main/moveit_py/moveit/core/robot_state.pyi
  MoveItPy API           : https://github.com/moveit/moveit2/blob/main/moveit_py/moveit/planning.pyi
"""
import time
import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from control_msgs.action import GripperCommand
from pathlib import Path
from moveit.planning import MoveItPy
from moveit_configs_utils import MoveItConfigsBuilder
from moveit.utils import create_params_file_from_dict
from moveit.core.robot_state import RobotState


# ---------------------------------------------------------------------------
# MoveIt configuration
# moveit_cpp.yaml sets use_sim_time=true and the planning pipeline.
# Change use_sim_time to false in config/moveit_cpp.yaml for real hardware.
#
# All configs that use /**:ros__params format (sensors_3d, kinematics,
# joint_limits, trajectory_execution) are mirrored in moveit_cpp.yaml in flat
# format. sensors_3d() is still monkeypatched because to_moveit_configs()
# calls it unconditionally and the original file format causes a KeyError.
# ---------------------------------------------------------------------------
MoveItConfigsBuilder.sensors_3d = lambda self, *args, **kwargs: self

_moveit_config = (
    MoveItConfigsBuilder(robot_name="robot_arm", package_name="robot_arm_moveit_config")
    .robot_description_semantic(Path("config") / "robot_arm.srdf")
    .trajectory_execution(Path("config") / "ros_controllers.yaml")
    .moveit_cpp(Path("config") / "moveit_cpp.yaml")
    .to_moveit_configs()
    .to_dict()
)
# use_sim_time in moveit_cpp.yaml only applies inside planning_scene_monitor_options.
# This sets it on the MoveItPy node itself so its clock matches sim time.
# Remove this line when running on real hardware.
# https://github.com/moveit/moveit2/issues/2906#issuecomment-2259711256
_moveit_config["use_sim_time"] = True


# ---------------------------------------------------------------------------
# Gripper
# ---------------------------------------------------------------------------
class Gripper(Node):
    """Action client for a parallel-jaw gripper.

    Supports both blocking (wait=True) and fire-and-forget (wait=False) modes.
    In blocking mode the return value indicates whether the goal was reached.
    In non-blocking mode a Future is returned.
    """

    OPEN_POS: float = 0.026   # metres – fully open
    CLOSED_POS: float = 0.0   # metres – fully closed

    def __init__(self, action_name: str = "/gripper_controller/gripper_cmd") -> None:
        super().__init__("robot_arm_gripper")
        self._client = ActionClient(self, GripperCommand, action_name)
        self._client.wait_for_server()
        self._goal_handle = None

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------
    def open(self, wait: bool = True):
        return self._send(self.OPEN_POS, wait)

    def close(self, wait: bool = True):
        return self._send(self.CLOSED_POS, wait)

    def set_pos(self, pos: float, wait: bool = True):
        return self._send(pos, wait)

    def cancel(self) -> None:
        """Cancel the currently active gripper goal (if any)."""
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------
    def _send(self, pos: float, wait: bool):
        goal = GripperCommand.Goal()
        goal.command.position = float(pos)
        goal.command.max_effort = -1.0  # -1 = no effort limit

        future = self._client.send_goal_async(goal)

        if not wait:
            future.add_done_callback(self._on_goal_response)
            return future

        # Executor is already spinning in a background thread, so polling is safe.
        while not future.done():
            time.sleep(0.01)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning("Gripper goal rejected by server")
            return False

        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        while not result_future.done():
            time.sleep(0.01)

        reached = result_future.result().result.reached_goal
        if reached:
            self.get_logger().info("Gripper reached goal position")
        else:
            self.get_logger().warning("Gripper stalled before reaching goal")
        return reached

    def _on_goal_response(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning("Gripper goal rejected (async)")
            return
        self._goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future) -> None:
        result = future.result().result
        if result.reached_goal:
            self.get_logger().info("Gripper reached goal position (async)")
        else:
            self.get_logger().warning("Gripper stalled before reaching goal (async)")


# ---------------------------------------------------------------------------
# Motion helpers
# ---------------------------------------------------------------------------
def make_pose(
    frame_id: str,
    x: float, y: float, z: float,
    qx: float = 0.0, qy: float = 1.0, qz: float = 0.0, qw: float = 0.0,
) -> PoseStamped:
    """Build a PoseStamped from position + quaternion components."""
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


def plan_and_execute(robot: MoveItPy, planning_component, logger=None) -> bool:
    """Plan from the current state to the previously set goal and execute."""
    plan_result = planning_component.plan()
    if plan_result:
        robot.execute(plan_result.trajectory, controllers=[])
        return True
    msg = "Planning failed"
    if logger:
        logger.error(msg)
    else:
        print(msg)
    return False


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main(args=None) -> None:
    rclpy.init(args=args)

    # Gripper node – needs to be spun so callbacks are processed
    gripper = Gripper()
    executor = MultiThreadedExecutor()
    executor.add_node(gripper)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # MoveItPy creates its own internal ROS node; no need to add it to executor
    params_file = create_params_file_from_dict(_moveit_config, "/**")
    robot = MoveItPy(node_name="moveit_py", launch_params_filepaths=[params_file])
    arm = robot.get_planning_component("robot_arm")
    robot_model = robot.get_robot_model()

    gripper.get_logger().info("=== MoveItPy demo starting ===")

    # -----------------------------------------------------------------------
    # 1. Go to named state from SRDF
    # -----------------------------------------------------------------------
    gripper.get_logger().info("Step 1: moving to 'home' named state")
    arm.set_start_state_to_current_state()
    arm.set_goal_state(configuration_name="home")
    plan_and_execute(robot, arm)

    # -----------------------------------------------------------------------
    # 2. Move to a Cartesian pose goal
    # -----------------------------------------------------------------------
    gripper.get_logger().info("Step 2: moving to pose goal (pre-grasp)")
    pose_pre = make_pose(
        frame_id="base_mount",
        x=0.192, y=-0.128, z=0.116,   # ← adjust for your scene
        qx=0.0, qy=1.0, qz=0.0, qw=0.0,
    )
    arm.set_start_state_to_current_state()
    arm.set_goal_state(pose_stamped_msg=pose_pre, pose_link="tool_link")
    plan_and_execute(robot, arm)

    gripper.open()

    gripper.get_logger().info("Step 3: moving to pose goal (grasp)")
    pose_grasp = make_pose(
        frame_id="base_mount",
        x=0.192, y=-0.128, z=0.031,   # ← lower z to approach object
        qx=0.0, qy=1.0, qz=0.0, qw=0.0,
    )
    arm.set_start_state_to_current_state()
    arm.set_goal_state(pose_stamped_msg=pose_grasp, pose_link="tool_link")
    plan_and_execute(robot, arm)

    gripper.close()

    # -----------------------------------------------------------------------
    # 3. Move to a joint-space goal via RobotState
    # -----------------------------------------------------------------------
    gripper.get_logger().info("Step 4: moving to joint-space drop pose")
    robot_state = RobotState(robot_model)
    joints = robot_state.joint_positions
    joints["pan_joint"] = -0.925
    joints["shoulder_joint"] = 0.405
    joints["elbow_joint"] = 1.080
    joints["wrist0_joint"] = 0.006
    joints["wrist1_joint"] = 1.635
    joints["wrist2_joint"] = 0.577
    robot_state.joint_positions = joints

    arm.set_start_state_to_current_state()
    arm.set_goal_state(robot_state=robot_state)
    plan_and_execute(robot, arm)

    gripper.open()

    # -----------------------------------------------------------------------
    # 4. Return home
    # -----------------------------------------------------------------------
    gripper.get_logger().info("Step 5: returning home")
    arm.set_start_state_to_current_state()
    arm.set_goal_state(configuration_name="home")
    plan_and_execute(robot, arm)

    gripper.get_logger().info("=== Demo complete ===")

    executor.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
