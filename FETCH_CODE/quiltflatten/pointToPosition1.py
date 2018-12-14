import tf
import cv2
import requests
from image_geometry import PinholeCameraModel as PCM
import rospy
import thread
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
import moveit_commander

class FetchClient(object):
    def __init__(self):
        self.bfp = True
        self.cx = 320
        self.cy = 240
        self.ax = 57.5
        self.ay = 45
        self.fx = self.cx / math.tan(self.ax * math.pi / 180.0 / 2)
        self.fy = self.cy / math.tan(self.ay * math.pi / 180.0 / 2)
        self.robot = robot_interface.Robot_Interface()
        self.br = tf.TransformBroadcaster()
        self.move_group = MoveGroupInterface("arm", "base_link")
        self.pose_group = moveit_commander.MoveGroupCommander("arm")
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

    def broadcast_position(self, position, name):
        pass

    def calc_3D_position(self, u, v):
        dep_data = self.robot.get_img_data()[1]
        z = self.robot.get_depth((u, v), dep_data)

        if z == 0:
            return None, None, None

        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy
        return x, y, z

    def broadcast_from_pix(self, point, name):
        while self.bfp:
            # dep = self.robot.get_depth(point, dep_data)
            # while dep == 0:
            #     print("depth info error")
            #     dep = self.robot.get_depth(point, dep_data)
            #
            # td_points = self.pcm.projectPixelTo3dRay(point)
            # norm_pose = np.array(td_points)
            # norm_pose = norm_pose / norm_pose[2]
            # norm_pose = norm_pose * (dep)
            norm_pose = self.calc_3D_position(point[0], point[1])
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
                # rot = tf.transformations.quaternion_from_euler(0*DEG_TO_RAD, 90*DEG_TO_RAD, 0*DEG_TO_RAD)
                return pose

    def create_tf_of_base_link(self, name, position, rot):
        quat = tf.transformations.quaternion_from_euler(ai=rot[0], aj=rot[1], ak=rot[2])
        while True:
            self.br.sendTransform(position,
                                  quat,
                                  rospy.Time.now(),
                                  name,
                                  'base_link')

    def show_image_with_point(self, im):
        plt.imshow(im)
        pylab.show()

    def reach(self, point_xyz):
        pose = Pose(Point(point_xyz[0], point_xyz[1], point_xyz[2]), Quaternion(0, 0.707106781187, 0, 0.707106781187))
        # pose = Pose(Point(0.57503179279, 0.170263536187, 0.97855338914), Quaternion(0, 0.707106781187, 0, 0.707106781187))
        print(pose)
        gripper_pose_stamped = PoseStamped()
        gripper_pose_stamped.header.frame_id = 'base_link'
        gripper_pose_stamped.header.stamp = rospy.Time.now()
        gripper_pose_stamped.pose = pose
        gripper_frame = 'wrist_roll_link'
        # gripper_frame = 'l_gripper_finger_link'
        self.move_group.moveToPose(gripper_pose_stamped, gripper_frame, plan_only=True)
        result = self.move_group.get_move_action().get_result()
        if result:
            # Checking the MoveItErrorCode
            print("plan0 success!")
            height = result.planned_trajectory.joint_trajectory.points[0]
            gripper_pose_stamped.pose.position.z += height
            self.move_group.moveToPose(gripper_pose_stamped, gripper_frame, plan_only=True)
            result1 = self.move_group.get_move_action().get_result()
            if result1:
                if result1.error_code.val == MoveItErrorCodes.SUCCESS:
                    print("plan1 success!")
                    joint_names = ["shoulder_pan_joint", "shoulder_lift_joint",
                                   "upperarm_roll_joint", "elbow_flex_joint",
                                   "forearm_roll_joint",
                                   "wrist_flex_joint", "wrist_roll_joint"]
                    joint_pose = result1.planned_trajectory.joint_trajectory[0].position[6:13]
                    print joint_pose
                    self.move_group.moveToJointPosition(joint_names, joint_pose, wait=False)
                    result2 = self.move_group.get_move_action().get_result()
                    if result2:
                        # Checking the MoveItErrorCode
                        if result2.error_code.val == MoveItErrorCodes.SUCCESS:
                            rospy.loginfo("reach!")
                        else:
                            # If you get to this point please search for:
                            # moveit_msgs/MoveItErrorCodes.msg
                            rospy.logerr("reach fail")
                    else:
                        rospy.logerr("MoveIt! failure no result returned.")
                else:
                    rospy.logerr("plan1 fail")
            else:
                rospy.logerr("MoveIt! failure no result returned.")
        else:
            rospy.logerr("MoveIt! failure no result returned.")


if __name__ == '__main__':
    fetch = FetchClient()
    img = fetch.robot.get_img_data()[0]
    cv2.imwrite("fetch.jpg", img)
    t = time.time()
    files = {'IMAGE': (str(t)+'.jpg', open('fetch.jpg', 'rb'), 'image/png', {})}
    url = 'http://172.31.76.30:80/ThinkingQ/'
    response = requests.request("POST", url=url, files=files)
    html1 = response.text
    print html1
    a = html1.split("\n")[1:-1]
    p = np.array([float(i) for i in a])
    # print p
    point = (p[1], p[0])
    point = (462, 353)
    name = 'tree'
    thread.start_new_thread(fetch.broadcast_from_pix, (point, name))
    position = fetch.get_position_of_baselink(name)
    # fetch.show_image_with_point(img)
    if position:
        fetch.bfp=False
    print(position)
    # position[2] += 0.2
    model = load_model("./model/0.98.h5")
    joint_names = ["shoulder_pan_joint",
                   "shoulder_lift_joint", "upperarm_roll_joint",
                   "elbow_flex_joint", "forearm_roll_joint",
                   "wrist_flex_joint", "wrist_roll_joint"]

    x = np.round(np.array([position]), 4) * 100
    print model.predict(x)
    # x.reshape(1,3)
    # print(x.shape)
    joints = np.round(model.predict(x)[0], 4) / 100
    print joints
    joint_3 = 0 - joints[1] - joints[2] + 1.5
    disco_poses = [
        [joints[0], joints[1], 0.0, joints[2]-1.5, 0.0, joint_3+1.5, 0.0],
        [joints[0], joints[1], 0.0, joints[2], 0.0, joint_3, 0.0],
    ]

    for pose in disco_poses:
        if rospy.is_shutdown():
            break
        # Plans the joints in joint_names to angles in pose
        fetch.move_group.moveToJointPosition(joint_names, pose, wait=False)

        # Since we passed in wait=False above we need to wait here
        fetch.move_group.get_move_action().wait_for_result()
        result = fetch.move_group.get_move_action().get_result()

        if result:
            # Checking the MoveItErrorCode
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                rospy.loginfo("Reach!")
                position = fetch.pose_group.get_current_pose("wrist_roll_link").pose.position
                print(position)
            else:
                # If you get to this point please search for:
                # moveit_msgs/MoveItErrorCodes.msg
                rospy.logerr("Error")
        else:
            rospy.logerr("MoveIt! failure no result returned.")

    # This stops all arm movement goals
    # It should be called when a program is exiting so movement stops
    fetch.move_group.get_move_action().cancel_all_goals()



    # fetch.reach(position)