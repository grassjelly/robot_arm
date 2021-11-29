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
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from tf2_ros import TransformBroadcaster

from robot_arm_perception.numpify.image import image_to_numpy
from robot_arm_perception.numpify.registry import msgify
from robot_arm_perception.utils import get_depth, quaternion_from_euler

import numpy as np
import cv2
import math
import copy
import matplotlib.pyplot as plt
from shapely.geometry import Polygon, LineString, Point
from shapely.ops import nearest_points
from robot_arm_perception.geometry import transform_point

class ObjectFinderServer(Node):
    def __init__(self):
        super().__init__('object_finder_server')
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
  
        self._base_frame = self.get_parameter('base_frame_id').value
        self._camera_frame =  self.get_parameter('camera_frame').value

        self._depth_img_rec = False
        self._depth_info_rec = False

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = TransformBroadcaster(self)

        self._camera_transform = TransformStamped()
        self._camera_trans_rot = None
        self._wait_for_transform()

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

    def _wait_for_transform(self):
        t_received = False
        self.get_logger().info(f'Waiting for camera transform from \'{self._base_frame}\' to \'{self._camera_frame}\'')

        while not t_received:
            try:
                now = rclpy.time.Time()
                self._camera_transform = self._tf_buffer.lookup_transform(
                    self._base_frame,
                    self._camera_frame,
                    now
                )

                trans = (
                    self._camera_transform.transform.translation.x,
                    self._camera_transform.transform.translation.y,
                    self._camera_transform.transform.translation.z
                )
                quat = [
                    self._camera_transform.transform.rotation.x,
                    self._camera_transform.transform.rotation.y,
                    self._camera_transform.transform.rotation.z,
                    self._camera_transform.transform.rotation.w
                ]

                self._camera_trans_rot = (trans, quat)
                t_received = True
                self.get_logger().info('Camera Transform Received!')
                return

            except TransformException as ex:
                pass
     
            rclpy.spin_once(self, timeout_sec=1.0)

    def _depth_cb(self, msg):
        self._depth_img_rec = True
        self._depth_image = image_to_numpy(msg)

    def _depth_info_cb(self, msg):
        self._depth_info_rec = True
        self._depth_constant = 1.0 / msg.k[4]
        self._depth_frame_id = msg.header.frame_id

    def _image_cb(self, msg):
        if not self._depth_img_rec or not self._depth_info_rec:
            return

        top_left = (700, 190)
        bottom_right = (1010, 530)
        thresh = 130

        img = image_to_numpy(msg)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        min_x, min_y = top_left
        max_x, max_y = bottom_right

        cropped = np.full(gray.shape[:2], 255, dtype=np.uint8)
        cropped[min_y:max_y, min_x:max_x] = gray[min_y:max_y, min_x:max_x]

        thresholded = cv2.threshold(cropped, thresh, 255, cv2.THRESH_BINARY)[1]
        thresholded = cv2.erode(thresholded, None, iterations=3)
        thresholded = cv2.dilate(thresholded, None, iterations=3)

        canny = cv2.Canny(thresholded, 10, 250)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
        filled = cv2.morphologyEx(canny, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(filled, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        total = 0

        depth_image = copy.deepcopy(self._depth_image)
        for obj_id, c in enumerate(contours):

            #approximate the shape of the contours detected
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.02 * peri, True)
            
            #get the corners of the shape
            corners = []
            if len(approx) >= 4:
                total += 1
                for corner in approx:
                    corners.append(corner[0])

            if len(corners) < 3:
                continue

            #create a shapely object for easier geometric operations
            obj = Polygon(corners)
            obj_center = [int(obj.centroid.x), int(obj.centroid.y)]

            if obj.area < 500:
                continue

            #get the position of object in 3D using the RGBD image
            obj_depth = get_depth(
                depth_image, 
                obj_center, 
                self._depth_constant
            )

            if obj_depth is None:
                continue

            #the obj depth's reference frame is from the camera
            #now we transform it to the robot's base
            x, y, z = transform_point(self._camera_trans_rot, obj_depth)

            #just ignore noisy readings that says the object is below the base
            if z < 0:
                continue

            #from the detected corners find, the nearest point to the centroid
            nearest_points = self._find_nearest_point(corners, obj_center)

            #get the distance from centroid to the nearest point
            d_n_points_x = nearest_points[0][0] - obj_center[0]
            d_n_points_y = nearest_points[0][1] - obj_center[1]

            #using the delta we can calculate the angle of the object
            r = math.atan2(d_n_points_x, d_n_points_y)

            #using that angle we can get the gripping angle
            #just add half a pi so that the opening of the gripper is perpindicular
            #to the centroid->nearest_point vector
            r_quat = quaternion_from_euler(0, 0, r + 1.5708)

            object_transform = self._fill_transform(
                self._base_frame, 
                str(obj_id), 
                (x, y, z), 
                r_quat
            )

            self._tf_broadcaster.sendTransform(object_transform)

            cv2.circle(img, (int(nearest_points[0][0]), int(nearest_points[0][1])), 8, (0, 255, 0), -1)
            cv2.circle(img, (int(nearest_points[1][0]), int(nearest_points[1][1])), 8, (0, 255, 0), -1)
            cv2.circle(img, obj_center, 10, (255, 255, 255), -1)
            cv2.circle(img, obj_center,  8, (255,   0,   0), -1)

        # plt.imshow(img)
        # plt.show()
        cv2.rectangle(img, top_left, bottom_right, (0,255,0), 1)
        img_msg = msgify(Image, img, encoding='rgb8')
        # img_msg = msgify(Image, thresholded, encoding='mono8')

        img_msg.header.frame_id = msg.header.frame_id
        self._img_debug_pub.publish(img_msg)


    def _find_nearest_point(self, vertices, center):
        center[0] = int(center[0])
        center[1] = int(center[1])
        line_centers = []
        object_poly = vertices[:]
        object_poly.append(vertices[0])
        center_point = Point(center[0],center[1])
        distances = []

        #iterate every line segment in the polynomial
        for i in range(1, len(object_poly)):
            #create a line
            l = LineString([
                Point(object_poly[i-1][0], object_poly[i-1][1]),
                Point(object_poly[i][0], object_poly[i][1])
            ])
            #calculate shortest distance from the line to the centroid
            distances.append(center_point.distance(l))

            #get the point within the line that defines the shortest distance
            line_centers.append(nearest_points(l, center_point)[0])

        #pick the two shortest points
        shortest = np.argsort(np.array(distances))[:2]

        return [
            [line_centers[shortest[0]].x, line_centers[shortest[0]].y], 
            [line_centers[shortest[1]].x, line_centers[shortest[1]].y]
        ]

        
def main(args=None):
    rclpy.init(args=args)
    ofs = ObjectFinderServer()
    rclpy.spin(ofs)
    calibrator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



