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
from rclpy.action import ActionServer
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped, Pose, PoseArray
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from tf2_ros import TransformBroadcaster

from robot_arm_perception.numpify.image import image_to_numpy
from robot_arm_perception.numpify.registry import msgify
from robot_arm_perception.algorithms.algorithms import get_depth, find_objects, get_object_rotation
from robot_arm_perception.geom.geometry import transform_point
from robot_arm_perception.geom.transforms import quaternion_from_euler
from robot_arm_msgs.srv import GripperPose

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
            ]
        )

        image_topic = self.get_parameter('image_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        depth_info_topic = self.get_parameter('depth_info_topic').value
  
        self._base_frame = self.get_parameter('base_frame_id').value

        self._depth_img_rec = False
        self._depth_info_rec = False
        self._camera_tf_rec = False

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_broadcaster = TransformBroadcaster(self)

        self._camera_trans_rot = None

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
        self._wait_for_transform()

        self._gripper_pose_srv = self.create_service(
            GripperPose, 
            'gripper_pose', 
            self._gripper_pose_callback
        )

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

    def _fill_pose_msg(self, pos, orient):
        p = Pose()

        p.position.x = pos[0]
        p.position.y = pos[1]
        p.position.z = pos[2]

        p.orientation.x = orient[0]
        p.orientation.y = orient[1]
        p.orientation.z = orient[2]
        p.orientation.w = orient[3]

        return p

    def _wait_for_transform(self):
        while not self._camera_tf_rec:
            if not self._depth_info_rec:
                rclpy.spin_once(self, timeout_sec=1.0)
                continue
            
            self.get_logger().info(f'Waiting for camera transform from \'{self._base_frame}\' to \'{self._depth_frame}\'')

            try:
                now = rclpy.time.Time()
                camera_transform = self._tf_buffer.lookup_transform(
                    self._base_frame,
                    self._depth_frame,
                    now
                )

                trans = (
                    camera_transform.transform.translation.x,
                    camera_transform.transform.translation.y,
                    camera_transform.transform.translation.z
                )
                quat = [
                    camera_transform.transform.rotation.x,
                    camera_transform.transform.rotation.y,
                    camera_transform.transform.rotation.z,
                    camera_transform.transform.rotation.w
                ]

                self._camera_trans_rot = (trans, quat)
                self._camera_tf_rec = True
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
        self._depth_frame = msg.header.frame_id

    def _get_gripping_pose(self, image, depth_image):
        top_left = (700, 190)
        bottom_right = (1010, 530)
        thresh = 130

        objects, centroids = find_objects(
            image, 
            thresh=thresh, 
            limits=(top_left, bottom_right)
        )
        
        poses = []
        for vertices, center in zip(objects, centroids):
            r = get_object_rotation(vertices, center)
            r_quat = quaternion_from_euler(0, 0, r)

            obj_depth = get_depth(
                depth_image, 
                center, 
                self._depth_constant
            )

            if obj_depth is None:
                continue

            #the obj depth's reference frame is from the camera
            #now we transform it to the robot's base
            x, y, z = transform_point(self._camera_trans_rot, obj_depth)

            p = self._fill_pose_msg(
                (x,y,z),
                r_quat
            )
            poses.append(p)

        return poses

    def _gripper_pose_callback(self, request, response):
        gripping_poses = self._get_gripping_pose(self._image, self._depth_image)

        pose_array = PoseArray()
        pose_array.header.frame_id = self._base_frame
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.poses = gripping_poses

        response.poses = pose_array

        for pose_id, pose in enumerate(gripping_poses):
            object_transform = self._fill_transform(
                self._base_frame, 
                str(pose_id), 
                (pose.position.x, pose.position.y, pose.position.z), 
                (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w), 
            )

            self._tf_broadcaster.sendTransform(object_transform)
        
        return response

    def _image_cb(self, msg):
        self._image = image_to_numpy(msg)

        if not self._depth_img_rec or not self._depth_info_rec or not self._camera_tf_rec:
            return
        
def main(args=None):
    rclpy.init(args=args)
    ofs = ObjectFinderServer()
    rclpy.spin(ofs)
    calibrator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



