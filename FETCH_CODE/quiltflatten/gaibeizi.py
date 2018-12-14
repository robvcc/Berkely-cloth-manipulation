import cv2
from tf import TransformBroadcaster, transformations, TransformListener
import requests
from image_geometry import PinholeCameraModel as PCM
import rospy
import thread
from quilt_flatten.transformer import Transformer
import time
from matplotlib import pyplot as plt
import pylab
from keras.models import load_model
import numpy as np
from fetch_core import robot_interface, camera, skeleton
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from moveit_msgs.msg import MoveItErrorCodes
import math
import pypcd.numpy_pc2
import moveit_commander


class FetchClient(object):
    def __init__(self):
        self.bfp = True
        self.robot = robot_interface.Robot_Interface()
        self.url = 'http://172.31.76.30:80/ThinkingQ/'
        self.joint_names = ["shoulder_pan_joint",
                       "shoulder_lift_joint", "upperarm_roll_joint",
                       "elbow_flex_joint", "forearm_roll_joint",
                       "wrist_flex_joint", "wrist_roll_joint"]
        self.tf_listener = TransformListener()
        trfm = Transformer()
        self.r2b = trfm.transform_matrix_of_frames(
            'head_camera_rgb_optical_frame', 'base_link')
        self.model = load_model("./model/0.988.h5")
        self.br = TransformBroadcaster()
        self.move_group = MoveGroupInterface("arm", "base_link")
        # self.pose_group = moveit_commander.MoveGroupCommander("arm")
        self.cam = camera.RGBD()
        self.position_cloud = None
        while True:
            try:
                cam_info = self.cam.read_info_data()
                if cam_info is not None:
                    break
            except:
                rospy.logerr('camera info not recieved')
        self.pcm = PCM()
        self.pcm.fromCameraInfo(cam_info)

    def transform_matrix_of_frames(self, source_frame, target_frame):
        if self.tf_listener.frameExists(source_frame) and self.tf_listener.frameExists(target_frame):
            t = self.tf_listener.getLatestCommonTime(
                source_frame, target_frame)
            # Compute the transform from source to target
            position, quaternion = self.tf_listener.lookupTransform(
                target_frame, source_frame, t)

            transfrom_matrix = transformations.quaternion_matrix(quaternion)
            transfrom_matrix[0:3, 3] = position

            return transfrom_matrix
        else:
            return None

    def get_corner(self):
        print "get_corner"
        html = ""
        while html=="":
            image_time = time.time()
            img = self.cam.read_color_data()
            while img is None:
                img = self.cam.read_color_data()
            cv2.imwrite("image/fetch.jpg", img)
            files = {'IMAGE': (str(image_time) + '.jpg', open('image/fetch.jpg', 'rb'), 'image/png', {})}
            response = requests.request("POST", url=self.url, files=files, timeout=15000)
            html = response.text
        print html
        exit(0)
        a = html.split("\n")[1:-1]
        # print a
        pl = np.array([int(i) for i in a])
        # print pl
        re = []
        i = 0
        while i < len(a):
            point = []
            # print "pl i", pl[i]
            point.append(pl[i])
            point.append(pl[i+1])
            re.append(point)
            i = i+2
        # print re
        return re

    def convert_camera_position_to_base_position(self, array):
        print 'r2b', self.r2b
        ret = np.dot(self.r2b[0:3, 0:3], array)  # Rotate
        ret = ret + self.r2b[0:3, 3]
        return ret

    def update_position_cloud(self):
        pc = self.robot.camera.read_point_cloud()
        arr = pypcd.numpy_pc2.pointcloud2_to_array(pc, split_rgb=True)
        self.position_cloud = pypcd.numpy_pc2.get_xyz_points(arr, remove_nans=False)

    def get_position_by_pix(self, x, y):
        pc = self.robot.camera.read_point_cloud()
        arr = pypcd.numpy_pc2.pointcloud2_to_array(pc, split_rgb=True)
        po = pypcd.numpy_pc2.get_xyz_points(arr, remove_nans=False)
        return po[x, y]

    def broadcast_position(self, position, name):
        # while self.bfp:
        # print "111"
        a = transformations.quaternion_from_euler(
            ai=-2.355, aj=-3.14, ak=0.0)
        b = transformations.quaternion_from_euler(
            ai=0.0, aj=0.0, ak=1.57)
        base_rot = transformations.quaternion_multiply(a, b)
        self.br.sendTransform(position,
                              base_rot,
                              rospy.Time.now(),
                              name,
                              'head_camera_rgb_optical_frame')

    def get_position_of_baselink(self, name):
        listener = TransformListener()
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                fetch_client.broadcast_position(cam_position, name)
                pose, rot = listener.lookupTransform('base_link', name, rospy.Time(0))
            except Exception as e:
                print e
                continue
            rate.sleep()
            if pose:
                return pose

    def show_image(self, im):
        plt.imshow(im)
        pylab.show()

    def move_to_position(self, position):
        position[2] -= 0.24
        x = np.round(np.array([position]), 4) * 100
        print self.model.predict(x)
        joints = np.round(self.model.predict(x)[0], 4) / 100
        print joints
        joint_3 = 0 - joints[1] - joints[2] + 1.5

        final_p = [position[0], position[1] - 0.4, position[2]]
        f_x = np.round(np.array([final_p]), 4) * 100
        f_joints = np.round(self.model.predict(f_x)[0], 4) / 100
        f_joint_3 = 0 - f_joints[1] - f_joints[2] + 1.5

        disco_poses = [
            [joints[0], joints[1], 0.0, joints[2] - 0.5, 0.0, joint_3 + 0.5, 1.5+joints[0]], #up
            [joints[0], joints[1], 0.0, joints[2], 0.0, joint_3, 1.5+joints[0]],# 1 close gripper

            [joints[0], joints[1], 0.0, joints[2] - 0.5, 0.0, joint_3 + 0.5, 1.5+joints[0]], #up

            [f_joints[0], f_joints[1], 0.0, f_joints[2] - 0.5, 0.0, f_joint_3 + 0.5, 1.5+f_joints[0]],#final up
            # [f_joints[0], f_joints[1], 0.0, f_joints[2], 0.0, f_joint_3, 1.5],# 4 open final
            # [f_joint0s[0], f_joints[1], 0.0, f_joints[2] - 0.5, 0.0, f_joint_3 + 0.5, 1.5],  # final up
            [-0.0, -0.7, 0.0, 0.5, 0.0, 1.5, 1.3],
            [1.32, 0.7, 0.0, -2.0, 0.0, -0.57, 0.0]#stow
        ]

        for pose in disco_poses:
            if rospy.is_shutdown():
                break
            print pose
            # Plans the joints in joint_names to angles in pose
            self.move_group.moveToJointPosition(self.joint_names, pose, wait=False)

            # Since we passed in wait=False above we need to wait here
            self.move_group.get_move_action().wait_for_result()
            result = self.move_group.get_move_action().get_result()

            if result:
                # Checking the MoveItErrorCode
                if result.error_code.val == MoveItErrorCodes.SUCCESS:
                    rospy.loginfo("Reach success!")
                    if disco_poses.index(pose) == 1:
                        self.robot.close_gripper()
                        time.sleep(1)
                    if disco_poses.index(pose) == 3:
                        time.sleep(1)
                        self.robot.open_gripper()
                    # position = self.pose_group.get_current_pose("wrist_roll_link").pose.position
                    # print(position)
                else:
                    # If you get to this point please search for:
                    # moveit_msgs/MoveItErrorCodes.msg
                    rospy.logerr("Reach fail")
            else:
                rospy.logerr("MoveIt! failure no result returned.")

        # This stops all arm movement goals
        # It should be called when a program is exiting so movement stops
        # self.move_group.get_move_action().cancel_all_goals()


if __name__ == '__main__':
    fetch_client = FetchClient()
    time.sleep(10)
    position = [0.7788437979162758, 0.3, 0.7733053490563912]
    fetch_client.move_to_position(position)

