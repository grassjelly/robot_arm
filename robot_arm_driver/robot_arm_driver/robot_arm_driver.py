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
import copy
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import Executor, MultiThreadedExecutor
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

from sensor_msgs.msg import JointState
from control_msgs.msg import JointTrajectoryControllerState
from control_msgs.action import GripperCommand
from std_msgs.msg import Bool

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

class GripperActionServer(Node):
    def __init__(self, base_arm):
        super().__init__('gripper_action_server')
        self._lock = threading.Lock()

        self._gripper_state = base_arm.gripper_state

        self._max_pos = base_arm.gripper_max_position

        self._gripper_command = [0.0]
        self._action_server = ActionServer(
            self,
            GripperCommand,
            '/gripper_controller/gripper_cmd',
            self._gripper_callback
        )

    def _gripper_callback(self, handle):
        with self._lock:
            if handle.request.command.position == -1.0:
                target_pos = self._max_pos * 2
            else:
                target_pos = handle.request.command.position
                
            self._gripper_command[0] = abs(target_pos / 2.0)
            self.get_logger().info(f'GRIPPER REQUEST RECEIVED: {self._gripper_command}')

        time.sleep(0.5)
        while self._gripper_state['moving']:
            pass
        time.sleep(1)
        handle.succeed()
        result = GripperCommand.Result()
        result.position = self._gripper_state['position']
        return result

    @property
    def gripper_command(self):
        with self._lock:
            return self._gripper_command


class JointsCommandSubscriber(Node):
    def __init__(self, base_arm):
        super().__init__('joint_command_subscriber')
        self._lock = threading.Lock()
        self._joints_command = base_arm.initial_position

        self._pos_goal_subscirber = self.create_subscription(
            JointTrajectoryControllerState,
            '/robot_arm_controller/state',
            self._pos_goal_callback,
            10
        )
        self._pos_goal_subscirber

    def _pos_goal_callback(self, msg):
        with self._lock:
            for i, pos in enumerate(msg.actual.positions):
                self._joints_command[i] = pos

    @property
    def joints_command(self):
        with self._lock:
            return self._joints_command

class RobotArmDriver(Node):
    def __init__(self):
        super().__init__(
            'robot_arm_driver', 
            allow_undeclared_parameters=True
        )
        self._lock = threading.Lock()
        self._joints_cmd_subscriber = None
        self._gripper_cmd_server = None
        self._gripper_state = {}
        self._gripper_state['position'] = 0.0
        self._gripper_state['moving'] = False

        self.declare_parameters(
            namespace='',
            parameters=[
                ('serial_port', '/dev/ttyUSB0'),
                ('baud_rate', 1000000),
                ('initial_pos', [0.,0.,0.,0.,0.,0.]),
                ('revolute_joint_names', ['pan_joint', 'shoulder_joint', 'elbow_joint', 'wrist0_joint', 'wrist1_joint', 'wrist2_joint']),
                ('prismatic_joint_names', ['finger_joint']),
        ])

        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        rev_joint_names = self.get_parameter('revolute_joint_names').value
        pris_joint_names = self.get_parameter('prismatic_joint_names').value
        self._initial_position = self.get_parameter('initial_pos').value
        self._controller = Controller(serial_port, baud_rate)

        self._revolute_config = self._get_joints_config(rev_joint_names, 'revolute_joints')
        self._revolute_joints = Revolute(
            self._controller, 
            self._revolute_config
        )
        revolute_joints_ok = self._revolute_joints.init()
        if not revolute_joints_ok:
            self.get_logger().fatal(f'Failed to initialize revolute joints.')
            self.destroy_node()

        self._prismatic_config = self._get_joints_config(pris_joint_names, 'prismatic_joints')
        self._prismatic_joints = Prismatic(
            self._controller, 
            self._prismatic_config
        )
        prismatic_joints_ok = self._prismatic_joints.init()
        if not prismatic_joints_ok:
            self.get_logger().fatal(f'Failed to initialize prismatic joints.')
            self.destroy_node()

        self._joint_states_msg = JointState()
        self._joint_states_msg.name = rev_joint_names + pris_joint_names
        self._joint_states_msg.position = (len(rev_joint_names) + len(pris_joint_names)) * [0.0]

        self._joint_states_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self._control_timer = self.create_timer(0.01, self._control_callback)

    def _get_joints_config(self, joint_names, joint_type):
        joints_config = {}
        for joint_name in joint_names:
            joints_config[joint_name] = {}
            for param_name in joint_params.keys():
                self.declare_parameter(f'{joint_type}.{joint_name}.{param_name}', joint_params[param_name])
                param = self.get_parameter(f'{joint_type}.{joint_name}.{param_name}').value
                joints_config[joint_name][param_name] = param
        return joints_config

    def _control_callback(self):
        if self._joints_cmd_subscriber is None or self._gripper_cmd_server is None:
            return

        with self._lock:
            rev_pos, rev_vel, rev_mov_state = self._revolute_joints.state
            pris_pos, pris_vel, pris_mov_state = self._prismatic_joints.state

            self._gripper_state['position'] = pris_pos[0]
            self._gripper_state['moving']  = pris_mov_state[0]

            self._revolute_joints.go_to(self._joints_cmd_subscriber.joints_command)
            self._prismatic_joints.go_to(self._gripper_cmd_server.gripper_command)

            self._joint_states_msg.position = rev_pos + pris_pos
            self._joint_states_msg.velocity = rev_vel + rev_vel

            self._joint_states_msg.header.stamp = self.get_clock().now().to_msg()
            self._joint_states_publisher.publish(self._joint_states_msg)

    @property
    def joints_command_subscriber(self):
        pass

    @property
    def gripper_action_server(self):
        pass

    @joints_command_subscriber.setter
    def joints_command_subscriber(self, joints_command_subscriber):
        self._joints_cmd_subscriber = joints_command_subscriber

    @gripper_action_server.setter
    def gripper_action_server(self, gripper_action_server):
        self._gripper_cmd_server = gripper_action_server

    @property
    def gripper_max_position(self):
        return self._prismatic_config['finger_joint']['max_pos']

    @property
    def gripper_state(self):
        return self._gripper_state

    @property
    def initial_position(self):
        return copy.deepcopy(self._initial_position)


def main(args=None):
    rclpy.init(args=args)
    try:
        robot_arm = RobotArmDriver()
        joints_command_subscriber = JointsCommandSubscriber(robot_arm)
        gripper_action_server = GripperActionServer(robot_arm)
        robot_arm.joints_command_subscriber = joints_command_subscriber
        robot_arm.gripper_action_server = gripper_action_server

        executor = MultiThreadedExecutor(num_threads=3)
        executor.add_node(robot_arm)
        executor.add_node(joints_command_subscriber)
        executor.add_node(gripper_action_server)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            gripper_action_server.destroy_node()
            joints_command_subscriber.destroy_node()
            gripper_action_server.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()