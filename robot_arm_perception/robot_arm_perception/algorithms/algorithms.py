import math
import cv2.aruco as ar
from scipy.spatial.transform import Rotation as R
import numpy as np
from shapely.geometry import Polygon, Point, LineString
from shapely.ops import nearest_points
import cv2


def find_objects(image, thresh=130, limits=None):
    if limits is None:
        top_left = (0,0)
        h, w, d = image.shape
        bottom_right = (h,w)
    else:
        top_left = limits[0]
        bottom_right = limits[1]

    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
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

    objects = []
    centroids = []

    poses = []
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
            
        objects.append(corners)
        centroids.append(obj_center)

    return objects, centroids


def find_min_max_points(vertices, center):
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
    shortest = np.argsort(np.array(distances))[0]
    longest = np.argsort(np.array(distances))[-1]

    return [
        [line_centers[shortest].x, line_centers[shortest].y], 
        [line_centers[longest].x, line_centers[longest].y]
    ]


def get_object_rotation(vertices, center):
    nearest, furthest = find_min_max_points(vertices, center)

    #get the distance from centroid to the nearest point
    d_n_points_x = nearest[0] - center[0]
    d_n_points_y = nearest[1] - center[1]

    #using the delta we can calculate the angle of the object
    return math.atan2(d_n_points_x, d_n_points_y) + 1.5708


def get_depth(depth_image, pixel, depth_constant, transform=False):
    depth_img_h, depth_img_w = depth_image.shape
    obj_x, obj_y = pixel

    if obj_x >= depth_img_w or obj_y >= depth_img_h:
        return None

    unit_scaling = 0.001
    center_x = round(depth_img_w / 2.0) 
    center_y = round(depth_img_h / 2.0)

    fx = 1.0 / float(depth_constant)
    fy = 1.0 / float(depth_constant)
    c_x = unit_scaling / fx
    c_y = unit_scaling / fy

    depth = depth_image[obj_y][obj_x]

    if transform:
        x = depth * unit_scaling
        y = (center_x - obj_x) * depth * c_x
        z = (center_y - obj_y) * depth * c_y

        return np.array([x, y ,z])

    else:
        x = (obj_x - center_x) * depth * c_x
        y = (obj_y - center_y) * depth * c_y
        z = depth * unit_scaling

        return np.array([x, y, z])


def pixel_to_pose(depth_image, pixel, sample_size, depth_constant, transform=False, same_depth=False):
    def normalize(v):
        return v / np.linalg.norm(v)
    
    depth_img_h, depth_img_w = depth_image.shape
    sample_x, sample_y = sample_size
    sample_x = int(sample_x * 0.3)
    sample_y = int(sample_y * 0.3)

    obj_x, obj_y = pixel

    if obj_x >= depth_img_w:
        return None, None
    if obj_y >= depth_img_h:
        return None, None

    center = get_depth(
        depth_image, 
        pixel, 
        depth_constant,
        transform=transform
    )
    if center is None or not center.any():     
        return None, None

    if transform:
        y_axis = get_depth(
            depth_image,
            (obj_x - sample_x , obj_y),
            depth_constant,
            transform=transform
        )
        if y_axis is None or not y_axis.any():     
            return None, None
        if same_depth:
            y_axis[0] = center[0]
        

        z_axis = get_depth(
            depth_image,
            (obj_x, obj_y - sample_y),
            depth_constant,
            transform=transform
        )
        if z_axis is None or not z_axis.any():     
            return None, None
        if same_depth:
            z_axis[0] = center[0]

        z_axis = normalize(z_axis - center)
        y_axis = normalize(y_axis - center )

        x_axis = np.cross(y_axis, z_axis)
        x_axis = normalize(x_axis)

    else:
        x_axis = get_depth(
            depth_image,
            (obj_x + sample_x, obj_y),
            depth_constant,
        )
        if x_axis is None or not x_axis.any():     
            return None, None
        if same_depth:
            x_axis[2] = center[2]


        y_axis = get_depth(
            depth_image,
            (obj_x, obj_y + sample_y),
            depth_constant,
        )
        if y_axis is None or not y_axis.any():     
            return None, None
        if same_depth:
            y_axis[2] = center[2]

        x_axis = normalize(x_axis - center)
        y_axis = normalize(y_axis - center)

        z_axis = np.cross(x_axis, y_axis)
        z_axis = normalize(z_axis)
    
    quat = R.from_dcm([
        [x_axis[0], y_axis[0], z_axis[0]],
        [x_axis[1], y_axis[1], z_axis[1]],
        [x_axis[2], y_axis[2], z_axis[2]]
    ]).as_quat()

    quat = normalize(quat)
    
    if np.isnan(quat).all():
        return None, None
    else:
        return center, quat


class WeightedFilter:
    def __init__(self, beta):
        self._beta = beta
        self._pos = np.empty(3)
        self._orientation = np.empty(4)
        self._first_sample = True

    def filter(self, pos, orientation):
        pos = np.array(pos)
        orientation = np.array(orientation)
        if self._first_sample:
            self._pos = pos
            self._orientation = orientation
            self._first_sample = False

        else:
            self._pos = ((1.0 - self._beta) * pos) + (self._beta * self._pos)
            self._orientation = ((1.0 - self._beta) * orientation) + (self._beta * self._orientation)

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