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
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from robot_arm_perception.numpify.image import image_to_numpy
from shapely.geometry import Polygon
import numpy as np
import cv2.aruco as ar
from scipy.spatial.transform import Rotation as R
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import PoseArray, Pose, TransformStamped
from sklearn import preprocessing


class CameraCalibrator(Node):
    def __init__(self):
        super().__init__('camera_calibrator')
        dict_id = 'DICT_4X4_50'
        image_topic = 'camera/color/image_raw'
        depth_topic = 'camera/aligned_depth_to_color/image_raw'
        depth_info_topic = 'camera/aligned_depth_to_color/camera_info'
        marker_id = 1

        self._marker_id = marker_id
        self._init_aruco(dict_id)
        self._depth_img_rec = False
        self._depth_info_rec = False

        self._pos = np.empty(3)
        self._orientation = np.empty(4)
        self._first_sample = True
        self._beta = 0.98

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
            (round(img_center.x), round(img_center.y)), 
            (size_x, size_y),
            self._depth_constant
        )
        
        if pos is None or orientation is None:
            return

        if np.isnan(orientation).all():
            return

        if not pos.any():
            return

        if self._first_sample:
            self._pos = pos
            self._orientation = orientation
            self._first_sample = False
        else:
            self._pos = ((1 - self._beta) * pos) + (self._beta * self._pos)
            self._orientation = ((1 - self._beta) * orientation) + (self._beta * self._orientation)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'camera_link'
        t.child_frame_id = 'test'

        t.transform.translation.x = self._pos[0]
        t.transform.translation.y = self._pos[1]
        t.transform.translation.z = self._pos[2]

        t.transform.rotation.x = self._orientation[0]
        t.transform.rotation.y = self._orientation[1]
        t.transform.rotation.z = self._orientation[2]
        t.transform.rotation.w = self._orientation[3]

        self._tf_broadcaster.sendTransform(t)

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

        # rot = R.from_rotvec(np.array([0, -np.pi/2,  -np.pi/2])).as_quat()

        # orientation = (quat * rot)
        # orientation = normalize(orientation)

        return center, quat

def main(args=None):
    rclpy.init(args=args)
    calibrator = CameraCalibrator()
    rclpy.spin(calibrator)
    calibrator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()