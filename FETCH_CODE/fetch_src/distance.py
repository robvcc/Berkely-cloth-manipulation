#!/usr/bin/env python
# -*- coding=utf-8 -*-
import math
import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2

from transformer import Transformer

dep = cv2.imread('../image/depth.png',
                 cv2.IMREAD_ANYCOLOR | cv2.IMREAD_ANYDEPTH)
                 
rgb = cv2.imread('../image/rgb.png', cv2.IMREAD_ANYCOLOR | cv2.IMREAD_ANYDEPTH)



# 0.75 0.08 0.69


def calc_3D_position(dep, u, v):
    '''
    Thanks to https://www.cnblogs.com/gaoxiang12/p/3695962.html
    and https://codeyarns.com/tag/primesense/
    and https://blog.csdn.net/lyl771857509/article/details/79633412
    '''
    x, y, z = 0, 0, 0
    # The defalt values of Fetch camera
    cx = 320.0
    cy = 240.0
    ax = 57.5
    ay = 45.0
    fx = cx / math.tan(ax * math.pi / 180.0 / 2.0)
    fy = cy / math.tan(ay * math.pi / 180.0 / 2.0)
    s = 50.0
    # s = 16.25
    # s = 5000.0
    
    x = dep[u, v] / s
    y = (u-cx)*x / fx
    z = (v-cy)*x / fy
    return [x, y, z]

def ndot(a, b):
    return np.dot(a, b)




rospy.init_node('tf_and_camera')

tf = Transformer()

cx = 320
cy = 240
ax = 57.5
ay = 45.0
fx = cx / math.tan(ax * math.pi / 180.0 / 2)
fy = cy / math.tan(ay * math.pi / 180.0 / 2)
s = 16.25

u = 309
v = 335
# value = 38


camera = np.zeros((3, 3))
camera[0, 0] = fx
camera[1, 1] = fy
camera[2, 2] = 1
camera[0, 2] = cx
camera[1, 2] = cy

scale = 1000


head = np.array([0.107, 0.0206, 1.038, 1])
cube = np.array([0.8, 0.1, 0.72, 1])
b2h = tf.transform_matrix_of_frames('base_link', 'head_tilt_link')
h2b = tf.transform_matrix_of_frames('head_tilt_link', 'base_link')

cube_h = np.dot(b2h, np.reshape(cube, (4, 1)))

trans = np.dot(camera, b2h[0:3])

p = calc_3D_position(dep, u, v)


# front_head = np.array([0.507, 0.0206, 1.038])
# 0.507 0.0206 1.038
# 241,376

# x： 1.22  y:  0.0590820231752 , z:  0.00106208605737
