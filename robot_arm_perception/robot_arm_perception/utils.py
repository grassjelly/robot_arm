import math
import cv2.aruco as ar
from scipy.spatial.transform import Rotation as R
import numpy as np
from shapely.geometry import Polygon


def pixel_to_pose(depth_image, pixel, sample_size, depth_constant):
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
    
    if center is None or not center.any():     
        return None, None

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

    if np.isnan(quat).all():
        return None, None
    else:
        return center, quat

def quaternion_from_euler(roll, pitch, yaw):
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


class ArucoFinder:
    def __init__(self, dict_type):
        self._ar_dict = ar.Dictionary_get(ar.__getattribute__(dict_type))
        self._ar_params = ar.DetectorParameters_create()

    def find_marker(self, image, marker_id):
        corners, ar_ids, _ = ar.detectMarkers(
            image,
            self._ar_dict,
            parameters=self._ar_params
        )
        if not ar_ids:
            return None, None

        img_center = None
        size_x = 0
        size_y = 0
        for i, markers in enumerate(ar_ids):
            for ii, m_id in enumerate(markers):
                if m_id == marker_id:
                    corners_x = []
                    corners_y = []
                    for point in corners[i][ii]:
                        corners_x.append(point[0])
                        corners_y.append(point[1])
                    size_x = max(corners_x) - min(corners_x)
                    size_y = max(corners_y) - min(corners_y)
                    img_center = Polygon(corners[i][ii]).centroid
        
        return (int(img_center.x), int(img_center.y)), (size_x, size_y)