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
        for i, markers in enumerate(ar_ids):
            for ii, m_id in enumerate(markers):
                if m_id == self._marker_id:
                    img_center = Polygon(corners[i][ii]).centroid
        if not img_center:
            return 

        pos, orientation = self._pixel_to_pose(
            self._depth_image, 
            (round(img_center.x), round(img_center.y)), 
            self._depth_constant
        )

        if pos is None or orientation is None:
            return

        if np.isnan(orientation).all():
            return

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self._depth_frame_id
        t.child_frame_id = 'test'

        t.transform.translation.x = pos[0]
        t.transform.translation.y = pos[1]
        t.transform.translation.z = pos[2]

        t.transform.rotation.x = orientation[0]
        t.transform.rotation.y = orientation[1]
        t.transform.rotation.z = orientation[2]
        t.transform.rotation.w = orientation[3]

        self._tf_broadcaster.sendTransform(t)

    def _pixel_to_pose(self, depth_image, pixel, depth_constant):
        def normalize(v):
            norm = np.linalg.norm(v)
            return v / norm

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
            x = (obj_x - center_x) * depth * c_x
            y = (obj_y - center_y) * depth * c_y
            z = depth * unit_scaling

            return np.array([x, y, z])
        
        center = get_depth(
            depth_image, 
            pixel, 
            depth_constant
        )

        x_sample = get_depth(
            depth_image,
            (pixel[0] + 5, pixel[1]),
            depth_constant
        )

        y_sample = get_depth(
            depth_image,
            (pixel[0], pixel[1] - 5),
            depth_constant
        )

        x_vector = normalize(x_sample - center)
        y_vector = normalize(y_sample - center)
        z_vector = np.cross(x_vector, y_vector)
        z_vector = normalize(z_vector)
        quat = R.from_matrix([
            [x_vector[0], y_vector[0], z_vector[0]],
            [x_vector[1], y_vector[1], z_vector[1]],
            [x_vector[2], y_vector[2], z_vector[2]]
        ]).as_quat()
        rot = R.from_rotvec(np.array([np.pi/2, np.pi/2, 0])).as_quat()

        orientation = (quat * rot)
        orientation = normalize(orientation)
        
        return center, orientation


def main(args=None):
    rclpy.init(args=args)
    calibrator = CameraCalibrator()
    rclpy.spin(calibrator)
    calibrator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()