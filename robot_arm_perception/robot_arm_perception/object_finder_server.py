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
from robot_arm_perception.numpify.registry import msgify

import numpy as np
import cv2
import math
import copy
import matplotlib.pyplot as plt
from shapely.geometry import Polygon

class ObjectFinderServer(Node):
    def __init__(self):
        super().__init__('camera_transform_publisher')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('image_topic', 'camera/color/image_raw'),
                ('depth_topic', 'camera/aligned_depth_to_color/image_raw'),
                ('depth_info_topic', 'camera/aligned_depth_to_color/camera_info'),
                ('base_frame_id', 'base_mount'),
                ('camera_frame', 'camera_aligned_depth_to_color_frame'),
            ]
        )

        image_topic = self.get_parameter('image_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        depth_info_topic = self.get_parameter('depth_info_topic').value
  
        base_frame_id = self.get_parameter('base_frame_id').value
        self._camera_frame =  self.get_parameter('camera_frame').value


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

        self._img_debug_pub = self.create_publisher(Image, 'img_debug', 10)

    def _depth_cb(self, msg):
        self._depth_img_rec = True
        # self._depth_image = image_to_numpy(msg)

    def _depth_info_cb(self, msg):
        self._depth_info_rec = True
        self._depth_constant = 1.0 / msg.k[4]
        self._depth_frame_id = msg.header.frame_id

    def _image_cb(self, msg):
        if not self._depth_img_rec or not self._depth_info_rec:
            return

        top_left = (700, 300)
        bottom_right = (1130, 650)
        thresh = 130

        img = image_to_numpy(msg)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        min_x, min_y = top_left
        max_x, max_y = bottom_right

        cropped = np.full(gray.shape[:2], 255, dtype=np.uint8)
        cropped[min_y:max_y, min_x:max_x] = gray[min_y:max_y, min_x:max_x]

        thresholded = cv2.threshold(cropped.copy(), thresh, 255, cv2.THRESH_BINARY)[1]
        thresholded = cv2.erode(thresholded, None, iterations=2)
        thresholded = cv2.dilate(thresholded, None, iterations=2)

        canny = cv2.Canny(thresholded, 10, 250)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
        filled = cv2.morphologyEx(canny, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(filled.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        total = 0
        for c in contours:
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.02 * peri, True)

            points = []
            points_x = []
            points_y = []
            if len(approx) >= 4:
                total += 1
                for vertex in approx:
                    points_x.append(vertex[0][0])
                    points_y.append(vertex[0][1])
                    points.append(vertex[0])

            obj = Polygon(points)
            obj_c = obj.centroid

            if obj.is_empty:
                continue

            if obj.area < 500:
                continue
            
            m_x, m_y, mx_x, mx_y = obj.bounds

            cv2.circle(img, (int(m_x), int(m_y)),  8, (0, 255, 0), -1)
            cv2.circle(img, (int(mx_x), int(mx_y)),  8, (0, 255, 0), -1)
            cv2.circle(img, (int(obj_c.x), int(obj_c.y)), 10, (255, 255, 255), -1)
            cv2.circle(img, (int(obj_c.x), int(obj_c.y)),  8, (255,   0,   0), -1)

        cv2.rectangle(img, top_left, bottom_right, (0,255,0), 1)
        img_msg = msgify(Image, img, encoding='rgb8')

        img_msg.header.frame_id = msg.header.frame_id
        self._img_debug_pub.publish(img_msg)

def main(args=None):
    rclpy.init(args=args)
    ofs = ObjectFinderServer()
    rclpy.spin(ofs)
    calibrator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



