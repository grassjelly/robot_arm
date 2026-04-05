#!/usr/bin/env python3
# Copyright (c) 2021 Juan Miguel Jimeno
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http:#www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint

from pyfeetech.servo import Revolute, Prismatic, Controller


joint_params = {
    'index': 257,
    'id': 257,
    'invert': False,
    'min_pos': -3.1416,
    'max_pos': 3.1416,
    'angle_resolution': 0.0879,
    'step_per_sec': 50.,
    'speed': 1.0,
    'acceleration': 1.0,
    'torque': 40,
    'max_torque': 40,
    'offset': 0.0,
    'offset_gain': 0.0,
    'horn_radius': 0.017
}


class GripperServer:
    def __init__(self, node, prismatic_joints, gripper_state, max_pos):
        self._node = node
        self._lock = threading.Lock()
        self._executing = False
        self._goal_handle = None

        self._gripper_state = gripper_state
        self._max_pos = max_pos
        self._gripper_command = [0.0]

        ActionServer(
            node,
            GripperCommand,
            '/gripper_controller/gripper_cmd',
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            handle_accepted_callback=self._handle_accepted_callback,
            cancel_callback=self._cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )

    def _goal_callback(self, goal_request):
        if self._executing:
            self._node.get_logger().info('Gripper busy, rejecting goal')
            return GoalResponse.REJECT
        self._node.get_logger().info('Gripper goal accepted')
        return GoalResponse.ACCEPT

    def _handle_accepted_callback(self, goal_handle):
        with self._lock:
            self._goal_handle = goal_handle
        goal_handle.execute()

    def _cancel_callback(self, goal_handle):
        return CancelResponse.REJECT

    def _execute_callback(self, handle):
        try:
            self._executing = True
            with self._lock:
                if handle.request.command.position == -1.0:
                    target_pos = self._max_pos * 2
                else:
                    target_pos = handle.request.command.position
                self._gripper_command[0] = abs(target_pos / 2.0)
                self._node.get_logger().info(f'GRIPPER REQUEST RECEIVED: {self._gripper_command}')

            time.sleep(0.5)
            while self._gripper_state['moving']:
                pass
            time.sleep(1)

            handle.succeed()
            result = GripperCommand.Result()
            result.position = self._gripper_state['position']
            return result
        finally:
            self._executing = False

    @property
    def gripper_command(self):
        with self._lock:
            return self._gripper_command


class ManipulatorServer:
    def __init__(self, node, hw_lock, revolute_joints, rev_joint_names, initial_command):
        self._node = node
        self._hw_lock = hw_lock
        self._lock = threading.Lock()
        self._goal_handle = None

        self._revolute_joints = revolute_joints
        self._rev_joint_names = rev_joint_names
        self._current_joint_command = list(initial_command)

        ActionServer(
            node,
            FollowJointTrajectory,
            '/robot_arm_controller/follow_joint_trajectory',
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            handle_accepted_callback=self._handle_accepted_callback,
            cancel_callback=self._cancel_callback,
            callback_group=ReentrantCallbackGroup()
        )

    def _goal_callback(self, goal_request):
        self._node.get_logger().info('Received trajectory goal')
        return GoalResponse.ACCEPT

    def _handle_accepted_callback(self, goal_handle):
        with self._lock:
            if self._goal_handle is not None and self._goal_handle.is_active:
                self._node.get_logger().info('Preempting current goal')
                self._goal_handle.abort()
            self._goal_handle = goal_handle
        goal_handle.execute()

    def _cancel_callback(self, goal_handle):
        self._node.get_logger().info('Cancel request received')
        return CancelResponse.ACCEPT

    def _execute_callback(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        joint_names = list(trajectory.joint_names)
        points = trajectory.points

        self._node.get_logger().info(f'Executing trajectory with {len(points)} waypoints')

        for point in points:
            if not goal_handle.is_active:
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = 'Goal was preempted'
                return result

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = FollowJointTrajectory.Result()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = 'Goal was cancelled'
                return result

            # Update command — release _lock before acquiring _hw_lock to avoid deadlock
            with self._lock:
                new_command = list(self._current_joint_command)
            for i, name in enumerate(joint_names):
                if name in self._rev_joint_names:
                    new_command[self._rev_joint_names.index(name)] = point.positions[i]
            with self._lock:
                self._current_joint_command = new_command

            # Wait for joints to reach the waypoint — _lock not held here
            time.sleep(0.05)
            while True:
                with self._hw_lock:
                    _, _, rev_mov_state = self._revolute_joints.state
                if not any(rev_mov_state):
                    break
                if not goal_handle.is_active or goal_handle.is_cancel_requested:
                    break
                time.sleep(0.02)

            # Publish feedback — _lock not held here
            with self._hw_lock:
                rev_pos, rev_vel, _ = self._revolute_joints.state
            feedback = FollowJointTrajectory.Feedback()
            feedback.joint_names = self._rev_joint_names
            feedback.desired = JointTrajectoryPoint()
            feedback.desired.positions = new_command
            feedback.actual = JointTrajectoryPoint()
            feedback.actual.positions = list(rev_pos)
            feedback.actual.velocities = list(rev_vel)
            feedback.error = JointTrajectoryPoint()
            feedback.error.positions = [a - d for a, d in zip(rev_pos, new_command)]
            goal_handle.publish_feedback(feedback)

        if not goal_handle.is_active:
            result = FollowJointTrajectory.Result()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = 'Goal was preempted'
            return result

        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

    @property
    def joint_command(self):
        with self._lock:
            return list(self._current_joint_command)


class RobotArmNode(Node):
    def __init__(self):
        super().__init__(
            'robot_arm_node',
            allow_undeclared_parameters=True
        )
        self._hw_lock = threading.Lock()
        self._gripper_state = {'position': 0.0, 'moving': False}

        self.declare_parameters(
            namespace='',
            parameters=[
                ('serial_port', '/dev/ttyUSB0'),
                ('baud_rate', 1000000),
                ('initial_pos', [0., 0., 0., 0., 0., 0.]),
                ('revolute_joint_names', ['pan_joint', 'shoulder_joint', 'elbow_joint', 'wrist0_joint', 'wrist1_joint', 'wrist2_joint']),
                ('prismatic_joint_names', ['finger_joint']),
            ]
        )

        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        rev_joint_names = self.get_parameter('revolute_joint_names').value
        pris_joint_names = self.get_parameter('prismatic_joint_names').value
        initial_pos = self.get_parameter('initial_pos').value

        self._controller = Controller(serial_port, baud_rate)

        revolute_config = self._get_joints_config(rev_joint_names, 'revolute_joints')
        self._revolute_joints = Revolute(self._controller, revolute_config)
        if not self._revolute_joints.init():
            self.get_logger().fatal('Failed to initialize revolute joints.')
            self.destroy_node()

        prismatic_config = self._get_joints_config(pris_joint_names, 'prismatic_joints')
        self._prismatic_joints = Prismatic(self._controller, prismatic_config)
        if not self._prismatic_joints.init():
            self.get_logger().fatal('Failed to initialize prismatic joints.')
            self.destroy_node()

        gripper_max_pos = prismatic_config['finger_joint']['max_pos']

        self._joint_states_msg = JointState()
        self._joint_states_msg.name = rev_joint_names + pris_joint_names
        self._joint_states_msg.position = (len(rev_joint_names) + len(pris_joint_names)) * [0.0]

        self._joint_states_publisher = self.create_publisher(JointState, 'joint_states', 10)

        self._gripper_server = GripperServer(
            self,
            self._prismatic_joints,
            self._gripper_state,
            gripper_max_pos
        )
        self._manipulator_server = ManipulatorServer(
            self,
            self._hw_lock,
            self._revolute_joints,
            list(rev_joint_names),
            initial_pos
        )

        self._control_timer = self.create_timer(
            0.01,
            self._control_callback,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

    def _get_joints_config(self, joint_names, joint_type):
        joints_config = {}
        for joint_name in joint_names:
            joints_config[joint_name] = {}
            for param_name in joint_params.keys():
                self.declare_parameter(f'{joint_type}.{joint_name}.{param_name}', joint_params[param_name])
                joints_config[joint_name][param_name] = self.get_parameter(f'{joint_type}.{joint_name}.{param_name}').value
        return joints_config

    def _control_callback(self):
        # Read commands outside hw_lock to avoid holding two locks simultaneously
        joint_cmd = self._manipulator_server.joint_command
        gripper_cmd = self._gripper_server.gripper_command

        with self._hw_lock:
            rev_pos, rev_vel, _ = self._revolute_joints.state
            pris_pos, pris_vel, pris_mov_state = self._prismatic_joints.state

            self._gripper_state['position'] = pris_pos[0]
            self._gripper_state['moving'] = pris_mov_state[0]

            self._revolute_joints.go_to(joint_cmd)
            self._prismatic_joints.go_to(gripper_cmd)

            self._joint_states_msg.position = rev_pos + pris_pos
            self._joint_states_msg.velocity = rev_vel + pris_vel
            self._joint_states_msg.header.stamp = self.get_clock().now().to_msg()
            self._joint_states_publisher.publish(self._joint_states_msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        robot_arm = RobotArmNode()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(robot_arm)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            robot_arm.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
