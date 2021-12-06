import cv2
import matplotlib.pyplot as plt
from robot_arm_perception.algorithms import find_objects, get_object_rotation
import math

img = cv2.imread('fruits.jpg', cv2.IMREAD_COLOR) 
objects, centroids = find_objects(img, thresh=160)
rotations = []

for vertices, center in zip(objects, centroids):
    r = get_object_rotation(vertices, center)
    rotations.append(r)

    a_x = center[0] + int(100 * math.sin(r))
    a_y = center[1] + int(100 * math.cos(r))

    cv2.circle(img, (int(center[0]), int(center[1])), 20, (255, 255, 255), -1)
    cv2.circle(img, (int(center[0]), int(center[1])), 15, (255, 0, 0), -1)
    cv2.line(img, (int(center[0]), int(center[1])), (a_x, a_y), (255, 0, 0), 15)

plt.imshow(img)
plt.show()