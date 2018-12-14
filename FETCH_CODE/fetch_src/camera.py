#!/usr/bin/env python

import math
import time

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge, CvBridgeError
from PIL import Image as PIL_Image
from sensor_msgs.msg import Image, PointCloud2

from transformer import Transformer


class Camera(object):
    '''
    Use function take_photo() to save a rgb image and a depth image
    ---
    ```
    rospy.init_node('image_listener')
    cm = Camera()
    cm.take_photo()
    ```
    '''

    def __init__(self):

        self.cx = 320
        self.cy = 240
        self.ax = 57.5
        self.ay = 45
        self.fx = self.cx / math.tan(self.ax * math.pi / 180.0 / 2)
        self.fy = self.cy / math.tan(self.ay * math.pi / 180.0 / 2)
        self.scale_factor = 16.25

        self.head = np.array([0.107, 0.0206, 1.038, 1])
        self.cube = np.array([0.8, 0.1, 0.72, 1])
        tf = Transformer()
        self.b2h = tf.transform_matrix_of_frames('base_link', 'head_tilt_link')

        # The depth camera intrinsics
        self.cm = np.zeros((3, 3))
        self.cm[0, 0] = self.fx
        self.cm[1, 1] = self.fy
        self.cm[2, 2] = 1
        self.cm[0, 2] = self.cx
        self.cm[1, 2] = self.cy

        self.bridge = CvBridge()
        # self.pc = PointCloud2()
        self.rgb_received = False
        self.depth_received = False
        # Define your image topic
        rgb_topic = "/head_camera/rgb/image_raw"
        depth_topic = '/head_camera/depth_registered/image_raw'
        # Set up your subscriber and define its callback
        self.rbg_sub = rospy.Subscriber(rgb_topic, Image, self.rgb_callback)
        self.depth_sub = rospy.Subscriber(
            depth_topic, Image, self.depth_callback)
        rospy.sleep(1)
        # rospy.spin()

    def rgb_callback(self, msg):
        try:
            # Convert your ROS Image message to OpenCV2
            rgb_img = self.bridge.imgmsg_to_cv2(msg, "bgr16")
            # cv2_img = self.bridge.imgmsg_to_cv2(msg, '16UC1')
        except CvBridgeError, e:
            print(e)
        # rospy.sleep(1)
        self.rgb_received = True
        self.rgb_img = rgb_img

    def depth_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "32FC1")

        except CvBridgeError as e:
            print(e)
        self.depth_received = True
        self.depth_img = cv_img

    def take_photo(self, path='image/'):
        if self.rgb_received == True and self.depth_received == True:
            # Save your OpenCV2 image
            cv2.imwrite(path + 'rgb.png', self.rgb_img)

            max_value = np.nanmax(self.depth_img)
            self.depth_img = self.depth_img * 255.0 / max_value
            cv2.imwrite(path + 'depth.png', np.uint8(self.depth_img))

            print("Took image!")
            self.rgb_received = False
            self.depth_received = False
            return True
        else:
            print("404")
            return False

    def calc_3D_position(self, depth, u, v):
        z = depth[u, v]

        if z == 0:
            return None, None, None

        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy
        return x, y, z

    def generate_point_cloud(self, depth_file='./src/grasp_fetch/image/depth.png',
                             rgb_file='./src/grasp_fetch/image/rgb.png', 
                             ply_file='./src/grasp_fetch/image/points.ply'):
        """
        https://codereview.stackexchange.com/questions/79032/generating-a-3d-point-cloud

        Transform a depth image into a point cloud with one point for each
        pixel in the image, using the camera transform for a camera
        centred at cx, cy with field of view fx, fy.

        depth is a 2-D ndarray with shape (rows, cols) containing
        depths from 1 to 254 inclusive. The result is a 3-D array with
        shape (rows, cols, 3). Pixels with invalid depth in the input have
        NaN for the z-coordinate in the result.

        """
        if depth_file == None:
            depth_file = '../image/depth.png'
        if rgb_file == None:
            rgb_file = '../image/rgb.png'
        if ply_file == None:
            ply_file == './src/grasp_fetch/image/points.ply'

        rgb = PIL_Image.open(rgb_file)
        depth = PIL_Image.open(depth_file)

        if rgb.size != depth.size:
            raise Exception(
                "Color and depth image do not have the same resolution.")

        points = []
        for v in range(rgb.size[1]):
            for u in range(rgb.size[0]):
                color = rgb.getpixel((u, v))
                Z = depth.getpixel((u, v))
                if Z == 0:
                    continue
                X = (u - self.cx) * Z / self.fx
                Y = (v - self.cy) * Z / self.fy
                points.append("%f %f %f %d %d %d 0\n" %
                              (X, Y, Z, color[0], color[1], color[2]))
        file = open(ply_file, "w")
        file.write('''ply
    format ascii 1.0
    element vertex %d
    property float x
    property float y
    property float z
    property uchar red
    property uchar green
    property uchar blue
    property uchar alpha
    end_header
    %s
    ''' % (len(points), "".join(points)))
        file.close()


if __name__ == '__main__':
    rospy.init_node('take_photo')

    cm = Camera()
    # cm.take_photo(path="../image/")
    cm.take_photo()
    # cm.generate_point_cloud()
