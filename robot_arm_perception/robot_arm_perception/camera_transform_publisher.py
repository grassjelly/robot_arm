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
import copy
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose, TransformStamped
from tf2_ros import TransformBroadcaster
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from robot_arm_perception.numpify.image import image_to_numpy
from robot_arm_perception.geom.transforms import quaternion_from_euler
from robot_arm_perception.algorithms.algorithms import WeightedFilter, ArucoFinder, pixel_to_pose


class CameraTransformPublisher(Node):
    def __init__(self):
        super().__init__('camera_transform_publisher')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('dict_id', 'DICT_4X4_50'),
                ('image_topic', 'camera/color/image_raw'),
                ('depth_topic', 'camera/aligned_depth_to_color/image_raw'),
                ('depth_info_topic', 'camera/aligned_depth_to_color/camera_info'),
                ('base_to_cal_pos', [0.0, 0.0, 0.0]),
                ('base_to_cal_rot', [0.0, 0.0, 0.0]),
                ('aruco_marker_id', 1),
                ('base_frame', 'base_mount'),
                ('camera_frame', 'camera_aligned_depth_to_color_frame'),
                ('calibration_frame', 'calibration_link')
            ]
        )

        dict_id =  self.get_parameter('dict_id').value
        image_topic = self.get_parameter('image_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        depth_info_topic = self.get_parameter('depth_info_topic').value
        base_to_cal_pos = self.get_parameter('base_to_cal_pos').value
        base_to_cal_rot = self.get_parameter('base_to_cal_rot').value
        aruco_marker_id = self.get_parameter('aruco_marker_id').value
        base_frame = self.get_parameter('base_frame').value
        self._camera_frame =  self.get_parameter('camera_frame').value
        self._calibration_frame =  self.get_parameter('calibration_frame').value
        self._depth_frame = ''

        self._marker_id = aruco_marker_id
        self._depth_img_rec = False
        self._depth_info_rec = False

        self._weighted_filter = WeightedFilter(0.98)
        self._aruco = ArucoFinder(dict_id)

        self._depth_info_sub = self.create_subscription(
            CameraInfo,
            depth_info_topic,
            self._depth_info_cb,
            qos_profile_sensor_data
        )
        self._depth_info_sub

        self._depth_sub = self.create_subscription(
            Image,
            depth_topic,
            self._depth_cb,
            qos_profile_sensor_data
        )
        self._depth_sub

        self._image_sub = self.create_subscription(
            Image,
            image_topic,
            self._image_cb,
            qos_profile_sensor_data
        )
        self._image_sub

        self._tf_broadcaster = TransformBroadcaster(self)

        self._calibration_to_camera_recv = False
        self._calibration_to_camera = TransformStamped()
        self._depth_to_camera = TransformStamped()

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_timer = self.create_timer(0.01, self._tf_cb)

        quat = quaternion_from_euler(
            base_to_cal_rot[0], 
            base_to_cal_rot[1], 
            base_to_cal_rot[2]
        )
        self._base_to_calibration = self._fill_transform(
            base_frame,
            self._calibration_frame,
            base_to_cal_pos,
            quat
        )

        self._calibration_samples = 0
        self.get_logger().info('Calibrating Camera!')

    def _fill_transform(self, parent_frame, child_frame, pos, rot):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame

        t.transform.translation.x = pos[0]
        t.transform.translation.y = pos[1]
        t.transform.translation.z = pos[2]

        t.transform.rotation.x = rot[0]
        t.transform.rotation.y = rot[1]
        t.transform.rotation.z = rot[2]
        t.transform.rotation.w = rot[3]
        
        return t

    def _tf_cb(self):
        if not self._calibration_to_camera_recv:
            try:
                now = rclpy.time.Time()
                self._calibration_to_camera = self._tf_buffer.lookup_transform(
                    self._calibration_frame,
                    self._depth_frame,
                    now
                )
                self._calibration_samples += 1
  
            except TransformException as ex:
                return

            if self._calibration_samples > 100:
                self.get_logger().info('Camera Transform Ready!')

                self.destroy_subscription(self._depth_sub)
                self.destroy_subscription(self._image_sub)

                self._calibration_to_camera_recv = True

        else:
            now = self.get_clock().now().to_msg()
            self._base_to_calibration.header.stamp = now
            self._calibration_to_camera.header.stamp = now

            self._tf_broadcaster.sendTransform(self._base_to_calibration)
            self._tf_broadcaster.sendTransform(self._calibration_to_camera)
    
    def _depth_cb(self, msg):
        self._depth_img_rec = True
        self._depth_image = image_to_numpy(msg)

    def _depth_info_cb(self, msg):
        self._depth_info_rec = True
        self._depth_constant = 1.0 / msg.k[4]
        self._depth_frame = msg.header.frame_id
        self.get_logger().info(self._depth_frame)
        self.destroy_subscription(self._depth_info_sub)

    def _image_cb(self, msg):
        if not self._depth_img_rec or not self._depth_info_rec:
            return

        image = image_to_numpy(msg)

        object_center, object_size = self._aruco.find_marker(
            image,
            self._marker_id
        ) 

        if not object_center:
            return 

        pos, orientation = pixel_to_pose(
            self._depth_image, 
            object_center, 
            object_size,
            self._depth_constant,
            transform=False,
            same_depth=True
        )

        if pos is None or orientation is None:
            return

        f_pos, f_orientation = self._weighted_filter.filter(pos, orientation)
        marker_transform = self._fill_transform(
            self._depth_frame, 
            self._calibration_frame, 
            f_pos, 
            f_orientation
        )

        self._tf_broadcaster.sendTransform(marker_transform)


def main(args=None):
    rclpy.init(args=args)
    ctp = CameraTransformPublisher()
    rclpy.spin(ctp)
    calibrator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

