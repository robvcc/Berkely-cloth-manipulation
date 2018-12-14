import tf
from image_geometry import PinholeCameraModel as PCM
import rospy
import thread
import time
import numpy as np
import image2corner
from fetch_core import robot_interface, camera, skeleton
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from moveit_python import MoveGroupInterface, PlanningSceneInterface
import cv2

DEG_TO_RAD = np.pi / 180
VEL = 0.4

class FetchClient(object):
    def __init__(self):
        self.bfp = True
        self.br = tf.TransformBroadcaster()
        self.robot = robot_interface.Robot_Interface()
        self.cam = camera.RGBD()
        while True:
            try:
                cam_info = self.cam.read_info_data()
                if cam_info is not None:
                    break
            except:
                rospy.logerr('info not recieved')
        self.pcm = PCM()
        self.pcm.fromCameraInfo(cam_info)

    def broadcast_from_pix(self, point, name):
        while self.bfp:
            dep_data = self.robot.get_img_data()[1]
            dep = self.robot.get_depth(point, dep_data)
            while dep == 0:
                print("depth info error")
                dep = self.robot.get_depth(point, dep_data)
            td_points = self.pcm.projectPixelTo3dRay(point)
            norm_pose = np.array(td_points)
            norm_pose = norm_pose / norm_pose[2]
            norm_pose = norm_pose * (dep)
            a = tf.transformations.quaternion_from_euler(
                ai=-2.355, aj=-3.14, ak=0.0)
            b = tf.transformations.quaternion_from_euler(
                ai=0.0, aj=0.0, ak=1.57)
            base_rot = tf.transformations.quaternion_multiply(a, b)

            self.br.sendTransform(norm_pose,
                             base_rot,
                             rospy.Time.now(),
                             name,
                             'head_camera_rgb_optical_frame')

    def get_position_of_baselink(self, name):
        listener = tf.TransformListener()
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                pose, rot = listener.lookupTransform('base_link', name, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            rate.sleep()
            if pose:
                rot = tf.transformations.quaternion_from_euler(0*DEG_TO_RAD, 90*DEG_TO_RAD, 0*DEG_TO_RAD)
                return pose, rot

    def create_tf_of_base_link(self, name, position, rot):
        quat = tf.transformations.quaternion_from_euler(ai=rot[0], aj=rot[1], ak=rot[2])
        while True:
            self.br.sendTransform(position,
                                  quat,
                                  rospy.Time.now(),
                                  name,
                                  'base_link')




if __name__ == '__main__':
    fetch = FetchClient()
    detect = image2corner.Img2corner()
    img = fetch.robot.get_img_data()[0]
    # print img
    result = detect.image2cor(img)
    print result

    # point = (254+(359-254)/2, 287+(446-287)/2)
    #(254.421356,287.304230) (359.038650,258.225371) (242.024372,446.534959) (373.631185,546.785382)
    # name = 'tree'

    # thread.start_new_thread(fetch.broadcast_from_pix, (point, name))
    # position, quat = fetch.get_position_of_baselink(name)
    # if position:
    #     fetch.bfp=False
    # print(position, quat)
    # x, y, z = position
    # rot_x, rot_y, rot_z = (0.0, 90.0, 0.0)
    # move = skeleton.Robot_Skeleton()
    # pose0 = move.create_grasp_pose(x, y, z, rot_x * DEG_TO_RAD, rot_y * DEG_TO_RAD, rot_z * DEG_TO_RAD)
    # rospy.sleep(1)
    # move.move_to_pose(pose0, velocity_factor=VEL)
    # print("move_to_pose pose0")
