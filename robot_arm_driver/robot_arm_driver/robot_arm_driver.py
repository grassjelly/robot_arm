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
from control_msgs.msg import JointTrajectoryControllerState
from std_msgs.msg import Bool
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile
from pyfeetech.servo import Revolute, Prismatic, Controller

revolute_params = [
    'index',
    'id',
    'invert',
    'min_pos',
    'max_pos',
    'angle_resolution',
    'step_per_sec',
    'speed',
    'acceleration',
    'torque',
    'max_torque',
    'offset',
    'offset_gain',
    'horn_radius'
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
                ('initial_pos', [0,0,0,0,0,0]),
                ('revolute_joint_names', []),
                ('prismatic_joint_names', []),
        ])

        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        self._joint_names = []
        rev_joint_names = self.get_parameter('revolute_joint_names').value
        pris_joint_names = self.get_parameter('prismatic_joint_names').value

        self._controller = Controller(serial_port, baud_rate)

        self._revolute_joints = Revolute(
            self._controller, 
            self._get_joints_config(rev_joint_names, 'revolute_joints')
        )
        revolute_joints_ok = self._revolute_joints.init()
        if not revolute_joints_ok:
            print("Failed to init revolute joints.")

        self._prismatic_joints = Prismatic(
            self._controller, 
            self._get_joints_config(pris_joint_names, 'prismatic_joints')
        )
        prismatic_joints_ok = self._prismatic_joints.init()
        if not prismatic_joints_ok:
            print("Failed to init prismatic joints.")
        
        self._joint_names = rev_joint_names + pris_joint_names
        self._joint_states = (len(rev_joint_names) + len(pris_joint_names)) * [0.0]
        self._joint_goals = self.get_parameter('initial_pos').value
        self._gripper_goal = 0        

        self._joint_states_msg = JointState()
        self._joint_states_msg.name = self._joint_names 
        self._joint_states_msg.position = self._joint_states 

        self._joint_states_publisher = self.create_publisher(JointState, 'joint_states', 10)

        brake_timer = self.create_timer(0.02, self.control_callback)

        pos_goal_qos = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=1
        )

        self._pos_goal_subscirber = self.create_subscription(
            JointTrajectoryControllerState,
            'robot_arm_controller/state',
            self._pos_goal_callback,
            pos_goal_qos
        )

        self._pos_goal_subscirber = self.create_subscription(
            Bool,
            'gripper',
            self._gripper_callback,
            pos_goal_qos
        )

    def _gripper_callback(self, msg):
        self._gripper_goal = msg.data

    def _pos_goal_callback(self, msg):
        self._joint_goals = msg.actual.positions

    def _get_joints_config(self, joint_names, joint_type):
        joints_config = {}
        for joint_name in joint_names:
            joints_config[joint_name] = {}
            for param_name in revolute_params:
                self.declare_parameter(f'{joint_type}.{joint_name}.{param_name}')
                param = self.get_parameter(f'{joint_type}.{joint_name}.{param_name}').value
                joints_config[joint_name][param_name] = param
        return joints_config

    def control_callback(self):
        rev_pos, rev_vel = self._revolute_joints.state
        pris_pos, pris_vel = self._prismatic_joints.state

        self._revolute_joints.go_to(self._joint_goals)
        if self._gripper_goal:
            self._prismatic_joints.go_to([0.0])
        else:
            self._prismatic_joints.go_to([0.025])


        self._joint_states_msg.position = rev_pos + pris_pos
        self._joint_states_msg.velocity = rev_vel + rev_vel

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