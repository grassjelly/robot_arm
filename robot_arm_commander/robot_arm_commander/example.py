# #!/usr/bin/env python3

# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import threading

import rclpy
from rclpy.node import Node
import rclpy.time
from tf2_ros import LookupException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class WaitsForTransform(Node):
    """
    Wait for a transform using callbacks.
    This class is an example of waiting for transforms.
    It avoids blocking the executor by registering a callback to be called when a transform becomes
    available.
    """

    def __init__(self):
        super().__init__('example_waits_for_transform')

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._output_timer = self.create_timer(1.0, self.on_timer)

        self._lock = threading.Lock()
        self._tf_future = None
        self._from_frame = None
        self._to_frame = None
        self._when = None

    def on_tf_ready(self, future):
        print(self._to_frame)
        print(self._from_frame)

        with self._lock:
            self._tf_future = None
            if future.result():
                try:
                    self._tf_buffer.lookup_transform(self._to_frame, self._from_frame, self._when)
                except LookupException:
                    self.get_logger().info('transform no longer available')
                else:
                    self.get_logger().info('Got transform')

    def on_timer(self):
        if self._tf_future:
            self.get_logger().warn('Still waiting for transform')
            return

        with self._lock:
            self._from_frame = 'base_mount'
            self._to_frame = 'tool_link'
            self._when = rclpy.time.Time()
            self._tf_future = self._tf_buffer.wait_for_transform_async(
                self._to_frame, self._from_frame, self._when)
            self._tf_future.add_done_callback(self.on_tf_ready)
            self.get_logger().info('Waiting for transform from {} to {}'.format(
                self._from_frame, self._to_frame))


def main():
    rclpy.init()
    node = WaitsForTransform()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

# import threading

# import rclpy
# from robot_arm_commander.move_group_interface import MoveGroupInterface
# from geometry_msgs.msg import Quaternion, PoseStamped, Pose
# import time
# from math import sin, cos, pi


# #refer https://index.ros.org/doc/ros2/Tutorials/URDF/Using-URDF-with-Robot-State-Publisher/
# def euler_to_quaternion(roll, pitch, yaw):
#   qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
#   qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
#   qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
#   qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
#   return Quaternion(x=qx, y=qy, z=qz, w=qw)

# def main(args=None):
#     rclpy.init(args=args)

#     # Initialise MoveIt2
#     moveit2 = MoveGroupInterface("robot_arm","base_mount")

#     # Spin MoveIt2 node in the background
#     executor = rclpy.executors.MultiThreadedExecutor(1)
#     executor.add_node(moveit2)
#     thread = threading.Thread(target=executor.spin)
#     thread.start()

#     # Set pose goal to reach
#     while(1): 
#         target_pose = Pose()
#         target_pose.orientation.x = -1.155227619165089e-05
#         target_pose.orientation.y = 0.46281740069389343
#         target_pose.orientation.z = -1.7791414848034037e-06
#         target_pose.orientation.w = 0.8864536285400391
#         target_pose.position.x = 0.23239827156066895
#         target_pose.position.y = 0.034903883934020996
#         target_pose.position.z = 0.5444427728652954


#         target_posestamped = PoseStamped()
#         # target_posestamped.header.stamp = self.get_clock().now().to_msg()
#         target_posestamped.header.frame_id = "base_mount"
#         target_posestamped.pose = target_pose
#         moveit2.moveToPose(target_posestamped, "gripper_link")
#         time.sleep(5)
   
#     rclpy.shutdown()


# if __name__ == "__main__":
#     main()
