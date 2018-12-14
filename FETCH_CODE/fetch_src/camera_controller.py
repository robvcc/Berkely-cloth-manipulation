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

from tf import transformations
import tf
from fetch_core import robot_interface
import moveit_commander
import pypcd.numpy_pc2


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
        '''https://codeyarns.com/2015/09/08/how-to-compute-intrinsic-camera-matrix-for-a-camera/'''
        self.cx = 320.0
        self.cy = 240.0
        self.cdiag = 400.0
        self.ax = 57.5
        self.ay = 45.0
        self.adiag = 69.0
        self.fx = self.cx / math.tan(self.ax * math.pi / 180.0 / 2.0)
        self.fy = self.cy / math.tan(self.ay * math.pi / 180.0 / 2.0)
        self.fdiag = self.cdiag / math.tan(self.adiag * math.pi / 180.0 / 2.0)
        self.scale_factor = 51.59

        self.head = np.array([0.107, 0.0206, 1.038, 1])
        self.cube = np.array([0.8, 0.1, 0.72, 1])
        # self.cube = np.array([0.757, 0.5, 0.8, 1])

        tf = Transformer()
        self.base_to_head = tf.transform_matrix_of_frames(
            'base_link', 'head_camera_depth_optical_frame')
        self.head_to_base = tf.transform_matrix_of_frames(
            'head_camera_depth_optical_frame', 'base_link')
            
        self.b2r = tf.transform_matrix_of_frames(
            'base_link', 'head_camera_rgb_optical_frame')
        self.r2b = tf.transform_matrix_of_frames(
            'head_camera_rgb_optical_frame', 'base_link')
        
        self.b2c = tf.transform_matrix_of_frames(
            'base_link', 'head_camera_link')
        self.c2b = tf.transform_matrix_of_frames(
            'head_camera_link', 'base_link')
        
        self.d2b = tf.transform_matrix_of_frames(
            'head_camera_depth_frame', 'base_link')
            
        self.cube_h = np.dot(self.base_to_head, np.reshape(self.cube, (4, 1)))

        # The depth camera intrinsics
        self.camera_matrix = np.zeros((3, 3))
        self.camera_matrix[0, 0] = self.fx
        self.camera_matrix[1, 1] = self.fy
        self.camera_matrix[2, 2] = 1
        self.camera_matrix[0, 2] = self.cx
        self.camera_matrix[1, 2] = self.cy

        self.bridge = CvBridge()
        self.rgb_received = False
        self.depth_received = False
        # Define your image topic
        rgb_topic = "/head_camera/rgb/image_raw"
        depth_topic = '/head_camera/depth_registered/image_raw'
        # Set up your subscriber and define its callback
        self.rbg_sub = rospy.Subscriber(rgb_topic, Image, self.rgb_callback)
        self.depth_sub = rospy.Subscriber(
            depth_topic, Image, self.depth_callback)
        rospy.sleep(0.5)

    def rgb_callback(self, msg):
        try:
            # Convert your ROS Image message to OpenCV2
            rgb_img = self.bridge.imgmsg_to_cv2(msg, "bgr16")
        except CvBridgeError, e:
            print(e)
        self.rgb_received = True
        self.rgb_img = rgb_img

    def depth_callback(self, msg):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        except CvBridgeError as e:
            print(e)
        self.depth_received = True
        self.depth_img = cv_img

    def take_photo(self, path='./src/grasp_fetch/image/'):
        '''https://stackoverflow.com/questions/47751323/get-depth-image-in-grayscale-in-ros-with-imgmsg-to-cv2-python'''

        desired_shape = (640, 480)
        u = 331
        v = 332
        
        if self.rgb_received == True and self.depth_received == True:

            rgb_array = np.array(self.rgb_img)
            depth_array = np.array(self.depth_img)

            self.rgb_img = cv2.resize(
                rgb_array, desired_shape, interpolation=cv2.INTER_CUBIC)
            self.depth_img = cv2.resize(
                depth_array, desired_shape, interpolation=cv2.INTER_CUBIC)
            rgb_img = self.rgb_img
            depth_img = self.depth_img

            print depth_img.shape
            print depth_img[u, v]
            print depth_img[v, u]
            
            # Save your OpenCV2 image
            cv2.imwrite(path + 'rgb.png', rgb_img)

            max_value = np.nanmax(depth_img)
            depth_img = depth_img * 255.0 / max_value

            cv2.imwrite(path + 'depth.png', np.uint8(depth_img))

            print("Took image!")
            self.rgb_received = False
            self.depth_received = False
            return True
        else:
            print("404")
            return False

    def calc_3D_position(self, depth, u, v):
        '''Return the position in base_link coordinate
        '''
        # Bacause the opencv coordinate is the opposite of image coordinate
        (u, v) = (v, u)
        z = depth[u, v] / self.scale_factor
        if z == 0:
            return None, None, None

        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy

        self.position_h = [x, y, z]
        print 'position_h => ', self.position_h
        self.position_c = [z, -1 * x, -1 * y]

        position = np.array((x, y, z))
        position = np.dot(
            self.head_to_base[0:3, 0:3],
            position
        ) + self.head_to_base[0:3, 3]
        return position

    def convert_head_to_base(self, array, is_point=True):
        ret = np.dot(self.head_to_base[0:3, 0:3], array)  # Rotate
        if is_point:
            ret = ret + self.head_to_base[0:3, 3]
        return ret

    def convert_base_to_head(self, array, is_point=True):
        ret = np.dot(self.base_to_head[0:3, 0:3], array)  # Rotate
        if is_point:
            ret = ret + self.base_to_head[0:3, 3]
        return ret

    def convert_point_to_base(self, array):
        ret = np.dot(self.r2b[0:3, 0:3], array)  # Rotate
        ret = ret + self.r2b[0:3, 3]
        return ret


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
        for u in range(rgb.size[1]):
            for v in range(rgb.size[0]):
                color = rgb.getpixel((u, v))
                X = depth.getpixel((u, v))
                if X == 0:
                    continue
                Y = (self.cx - u) * X / self.fx
                Z = (self.cy - v) * X / self.fy
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

    def broadcast_position(self, position, name):
        br = tf.TransformBroadcaster()
        while True:
            a = transformations.quaternion_from_euler(
                ai=-2.355, aj=-3.14, ak=0.0)
            b = transformations.quaternion_from_euler(
                ai=0.0, aj=0.0, ak=1.57)
            base_rot = transformations.quaternion_multiply(a, b)
            br.sendTransform(position,
                             base_rot,
                             rospy.Time.now(),
                             name,
                             'head_camera_rgb_optical_frame')


if __name__ == '__main__':
    # rospy.init_node('take_photo')
    print time.strftime('%H-%M-%S')
    print '============================'
    robot = robot_interface.Robot_Interface()
    # cam = camera.RGBD()
    # rospy.sleep(1)

    print time.strftime('%H-%M-%S')
    print '============================'
    cm = Camera()
    cm.take_photo(path="../image/")
    # cm.take_photo()
    # depth_file = './src/grasp_fetch/image/depth.png'
    depth_file = '../image/depth.png'
    dep = cv2.imread(depth_file, cv2.IMREAD_ANYCOLOR | cv2.IMREAD_ANYDEPTH)
    u = 331
    v = 332
    print time.strftime('%H-%M-%S')
    print '============================'
    

    tmp = cm.calc_3D_position(dep, u, v)
    print 'position_d => ', tmp
    # print cm.cube
    print 'position_D => ', np.dot(cm.d2b[0:3, 0:3], cm.position_c) + cm.d2b[0:3, 3]

    print 'position_c => ', np.dot(cm.c2b[0:3, 0:3], cm.position_c) + cm.c2b[0:3, 3]
    print 'position_r => ', np.dot(cm.r2b[0:3, 0:3], cm.position_h) + cm.r2b[0:3, 3]
    
    print time.strftime('%H-%M-%S')
    print '============================'

    point_cloud = robot.camera.read_point_cloud()
    point_cloud_array = pypcd.numpy_pc2.pointcloud2_to_array(
        point_cloud, split_rgb=True)
    points = pypcd.numpy_pc2.get_xyz_points(
        point_cloud_array, remove_nans=False)

    print time.strftime('%H-%M-%S')
    print '============================'

    # print po.shape
    # y, z, x = points[v, u]
    # print (x, y, z)

    point = points[v, u]
    print 'point_r => ', point

    point_b = cm.convert_point_to_base(point)
    print 'point_b => ', point_b

    
    # print p

    # point = [0.01769572, 0.11993767, 0.72651386]


    # cm.broadcast_position(points[v, u], 'POINT')
    # cm.broadcast_position( a , 'a')
    # cm.broadcast_position( b , 'b')

    # u = 66
    # v = 333
