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
import math
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import Image, CameraInfo
from robot_arm_perception.numpify.image import image_to_numpy
from shapely.geometry import Polygon
import numpy as np
import cv2.aruco as ar
from scipy.spatial.transform import Rotation as R
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import PoseArray, Pose, TransformStamped
from sklearn import preprocessing
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


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
                ('base_frame_id', 'base_mount'),
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
        base_frame_id = self.get_parameter('base_frame_id').value
        self._camera_frame =  self.get_parameter('camera_frame').value
        self._calibration_frame =  self.get_parameter('calibration_frame').value

        self._marker_id = aruco_marker_id
        self._init_aruco(dict_id)
        self._depth_img_rec = False
        self._depth_info_rec = False

        self._weighted_filter = WeightedFilter(0.98)

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

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_timer = self.create_timer(0.01, self._tf_cb)

        quat = self._quaternion_from_euler(
            base_to_cal_rot[0], 
            base_to_cal_rot[1], 
            base_to_cal_rot[2]
        )
        self._base_to_calibration = self._fill_transform(
            base_frame_id,
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
                    self._camera_frame,
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
            
    def _init_aruco(self, dict_type):
        self._ar_dict = ar.Dictionary_get(ar.__getattribute__(dict_type))
        self._ar_params = ar.DetectorParameters_create()

    def _depth_cb(self, msg):
        self._depth_img_rec = True
        self._depth_image = image_to_numpy(msg)

    def _depth_info_cb(self, msg):
        self._depth_info_rec = True
        self._depth_constant = 1.0 / msg.k[4]
        self._depth_frame_id = msg.header.frame_id
        self.destroy_subscription(self._depth_info_sub)

    def _image_cb(self, msg):
        if not self._depth_img_rec or not self._depth_info_rec:
            return

        image = image_to_numpy(msg)
        corners, ar_ids, _ = ar.detectMarkers(
            image,
            self._ar_dict,
            parameters=self._ar_params
        )
        if not ar_ids:
            return

        img_center = None
        size_x = 0
        size_y = 0
        for i, markers in enumerate(ar_ids):
            for ii, m_id in enumerate(markers):
                if m_id == self._marker_id:
                    corners_x = []
                    corners_y = []
                    for point in corners[i][ii]:
                        corners_x.append(point[0])
                        corners_y.append(point[1])
                    size_x = max(corners_x) - min(corners_x)
                    size_y = max(corners_y) - min(corners_y)
                    img_center = Polygon(corners[i][ii]).centroid

        if not img_center:
            return 

        pos, orientation = self._pixel_to_pose(
            self._depth_image, 
            (int(img_center.x), int(img_center.y)), 
            (size_x, size_y),
            self._depth_constant
        )

        if pos is None or orientation is None:
            return

        if np.isnan(orientation).all():
            return

        if not pos.any():
            return

        f_pos, f_orientation = self._weighted_filter.filter(pos, orientation)

        marker_transform = self._fill_transform(
            self._camera_frame, 
            self._calibration_frame, 
            f_pos, 
            f_orientation
        )

        self._tf_broadcaster.sendTransform(marker_transform)

    def _pixel_to_pose(self, depth_image, pixel, sample_size, depth_constant):
        def normalize(v):
            return v / np.linalg.norm(v)

        def get_depth(depth_image, pixel, depth_constant):
            depth_img_h, depth_img_w = depth_image.shape
            unit_scaling = 0.001
            center_x = round(depth_img_w / 2.0) 
            center_y = round(depth_img_h / 2.0)

            fx = 1.0 / float(depth_constant)
            fy = 1.0 / float(depth_constant)
            c_x = unit_scaling / fx
            c_y = unit_scaling / fy

            obj_x, obj_y = pixel
            depth = depth_image[obj_y][obj_x]

            x = depth * unit_scaling
            y = (center_x - obj_x) * depth * c_x
            z = (center_y - obj_y) * depth * c_y

            return np.array([x, y, z])
        
        depth_img_h, depth_img_w = depth_image.shape
        sample_x, sample_y = sample_size
        sample_x = int((sample_x / 2.0) * 0.5)
        sample_y = int((sample_y / 2.0) * 0.5)

        obj_x, obj_y = pixel

        if obj_x > depth_img_w:
            return None, None
        if obj_y > depth_img_h:
            return None, None

        center = get_depth(
            depth_image, 
            pixel, 
            depth_constant
        )

        y_axis = get_depth(
            depth_image,
            (obj_x - sample_x, obj_y),
            depth_constant
        )

        z_axis = get_depth(
            depth_image,
            (obj_x, obj_y - sample_y),
            depth_constant
        )

        z_axis = normalize(z_axis - center)
        y_axis = normalize(y_axis - center)

        x_axis = np.cross(y_axis, z_axis)
        x_axis = normalize(x_axis)

        quat = R.from_matrix([
            [x_axis[0], y_axis[0], z_axis[0]],
            [x_axis[1], y_axis[1], z_axis[1]],
            [x_axis[2], y_axis[2], z_axis[2]]
        ]).as_quat()

        quat = normalize(quat)

        return center, quat

    def _quaternion_from_euler(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q[0] = cy * cp * sr - sy * sp * cr
        q[1] = sy * cp * sr + cy * sp * cr
        q[2] = sy * cp * cr - cy * sp * sr
        q[3] = cy * cp * cr + sy * sp * sr

        return q


class WeightedFilter:
    def __init__(self, beta):
        self._beta = beta
        self._pos = np.empty(3)
        self._orientation = np.empty(4)
        self._first_sample = True

    def filter(self, pos, orientation):
        if self._first_sample:
            self._pos = pos
            self._orientation = orientation
            self._first_sample = False

        else:
            self._pos = ((1 - self._beta) * pos) + (self._beta * self._pos)
            self._orientation = ((1 - self._beta) * orientation) + (self._beta * self._orientation)

        return self._pos, self._orientation


def main(args=None):
    rclpy.init(args=args)
    ctp = CameraTransformPublisher()
    rclpy.spin(ctp)
    calibrator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

