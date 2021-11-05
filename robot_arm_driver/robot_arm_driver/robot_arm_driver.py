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

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from pyfeetech.servo import Servo

params = [
    'index',
    'id',
    'invert',
    'min_pos',
    'max_pos',
    'angle_resolution',
    'step_per_sec',
    'speed',
    'acceleration',
    'offset',
    'offset_gain'
]

class RobotArmDriver(Node):
    def __init__(self):
        super().__init__(
            'robot_arm_driver', 
            allow_undeclared_parameters=True
        )
        self.declare_parameters(
            namespace='',
            parameters=[
                ('serial_port', '/dev/ttyUSB0'),
                ('baud_rate', 1000000),
                ('joint_names', [])

        ])

        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        self._joints_config = {}
        self._joint_names = self.get_parameter('joint_names').value
        for joint_name in self._joint_names:
            self._joints_config[joint_name] = {}
            for param_name in params:
                self.declare_parameter(f'joints.{joint_name}.{param_name}')
                param = self.get_parameter(f'joints.{joint_name}.{param_name}').value
                self._joints_config[joint_name][param_name] = param

        self._servos = Servo(
            serial_port, 
            baud_rate, 
            self._joints_config
        )
        self._servos.init()
        
        self._joint_states = len(self._joint_names) * [0.0]

        self._joint_states_msg = JointState()
        self._joint_states_msg.header.frame_id = 'base_mount'
        self._joint_states_msg.name = self._joint_names 
        self._joint_states_msg.position = self._joint_states 

        self._joint_states_publisher = self.create_publisher(JointState, 'joint_states', 10)

        brake_timer = self.create_timer(0.2, self.control_callback)

    def control_callback(self):
        self._joint_states_msg.position = self._joint_states
        self._joint_states_msg.header.stamp = self.get_clock().now().to_msg()
        self._joint_states_publisher.publish(self._joint_states_msg)

def main(args=None):
    rclpy.init(args=args)

    robot_arm = RobotArmDriver()
    rclpy.spin(robot_arm)
    robot_arm.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()