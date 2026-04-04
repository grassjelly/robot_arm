import time
import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from control_msgs.action import GripperCommand


class Gripper(Node):
    """Action-client wrapper for a parallel-jaw gripper.

    Manages its own ROS node and executor in a background thread — transparent
    to the caller, just like MoveItPy. rclpy.init() must have been called
    before instantiating this class.

    Args:
        open_pos:    Joint position (metres) for the fully open state.
        close_pos:   Joint position (metres) for the fully closed state.
        action_name: Action server name for GripperCommand.
    """

    def __init__(
        self,
        open_pos: float = 0.026,
        close_pos: float = 0.0,
        action_name: str = "/gripper_controller/gripper_cmd",
    ) -> None:
        super().__init__("robot_arm_gripper")
        self._open_pos = open_pos
        self._close_pos = close_pos
        self._goal_handle = None

        self._client = ActionClient(self, GripperCommand, action_name)

        # Self-contained executor — caller does not need to spin this node.
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

        self._client.wait_for_server()

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    def open(self, wait: bool = True):
        """Move gripper to the open position."""
        return self._send(self._open_pos, wait)

    def close(self, wait: bool = True):
        """Move gripper to the closed position."""
        return self._send(self._close_pos, wait)

    def set_position(self, pos: float, wait: bool = True):
        """Move gripper to an arbitrary joint position (metres)."""
        return self._send(pos, wait)

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

        # The executor is already spinning in the background thread, so polling
        # on futures here is safe and won't deadlock.
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
