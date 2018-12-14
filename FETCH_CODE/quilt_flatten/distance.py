from fetch_core import robot_interface, camera
import time
import cv2
import numpy as np
import requests
import rospy
import socket
from moveit_python import MoveGroupInterface, PlanningSceneInterface
import os
from matplotlib import pyplot as plt
from cal_position import CalPosition
from manage_fetch_robot import FetchPose, GripperClient
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from moveit_msgs.msg import MoveItErrorCodes


class Detect:
    def __init__(self):
        self.detect_ip = '172.31.76.30'
        self.detect_port = 7777
        self.cam = camera.RGBD()
        self.cal_position = CalPosition()
        self.fetch_pose = FetchPose()

    def get_detect_info(self):
        print "get_detect_info"
        file_size = 0
        while file_size == 0:
            time.sleep(0.05)
            img = self.cam.read_color_data()
            cv2.imwrite("image/fetch.png", img)

            path = "image/fetch.png"
            # path = "../fetch_core/image/26.png"
            file_size = os.stat(path).st_size
            time.sleep(0.2)
        print 'get img'

    def move_corner(self, x, y):
        position = self.cal_position.get_base_position_from_pix(x, y)
        position[0] = position[0] - 0.20
        move_group = MoveGroupInterface("arm_with_torso", "base_link")
        planning_scene = PlanningSceneInterface("base_link")
        planning_scene.removeCollisionObject("my_front_ground")
        planning_scene.removeCollisionObject("my_back_ground")
        planning_scene.removeCollisionObject("my_right_ground")
        planning_scene.removeCollisionObject("my_left_ground")
        planning_scene.addCube("my_front_ground", 2, 1.1, 0.0, -1.0)
        planning_scene.addCube("my_back_ground", 2, -1.2, 0.0, -1.0)
        planning_scene.addCube("my_left_ground", 2, 0.0, 1.2, -1.0)
        planning_scene.addCube("my_right_ground", 2, 0.0, -1.2, -1.0)

        # This is the wrist link not the gripper itself
        gripper_frame = 'wrist_roll_link'
        pose = Pose(Point(position[0], position[1], position[2]),
                              Quaternion(0, 0, 0, 1))


        # Construct a "pose_stamped" message as required by moveToPose
        gripper_pose_stamped = PoseStamped()
        gripper_pose_stamped.header.frame_id = 'base_link'

        # Finish building the Pose_stamped message
        # If the message stamp is not current it could be ignored
        gripper_pose_stamped.header.stamp = rospy.Time.now()
        # Set the message pose
        gripper_pose_stamped.pose = pose

        # Move gripper frame to the pose specified
        move_group.moveToPose(gripper_pose_stamped, gripper_frame)
        result = move_group.get_move_action().get_result()

        if result:
            # Checking the MoveItErrorCode
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                rospy.loginfo("Hello there!")
            else:
                # If you get to this point please search for:
                # moveit_msgs/MoveItErrorCodes.msg
                rospy.logerr("Arm goal in state: %s",
                             move_group.get_move_action().get_state())
        else:
            rospy.logerr("MoveIt! failure no result returned.")
        time.sleep(1)
        joint_names = ["torso_lift_joint", "shoulder_pan_joint",
                       "shoulder_lift_joint", "upperarm_roll_joint",
                       "elbow_flex_joint", "forearm_roll_joint",
                       "wrist_flex_joint", "wrist_roll_joint"]
        joints_value = [0.3, 1.32, 0.7, 0.0, -2.0, 0.0, -0.57, 0.0]
        move_group.moveToJointPosition(joint_names, joints_value, wait=False)

        # Since we passed in wait=False above we need to wait here
        move_group.get_move_action().wait_for_result()
        result = move_group.get_move_action().get_result()

        if result:
            # Checking the MoveItErrorCode
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                rospy.loginfo("pose Success!")
            else:
                # If you get to this point please search for:
                # moveit_msgs/MoveItErrorCodes.msg
                rospy.logerr("Arm goal in state: %s",
                             self.move_group.get_move_action().get_state())
        else:
            rospy.logerr("MoveIt! failure no result returned.")


def on_press(event):
    if event.inaxes is None:
        print 'None'
        return
    print event.xdata, event.ydata
    detect.move_corner(int(event.ydata), int(event.xdata))


if __name__ == '__main__':
    rospy.init_node("test_connect")
    detect = Detect()
    while True:
        detect.get_detect_info()
        plt.imshow(plt.imread('image/fetch.png'))
        plt.connect("button_press_event", on_press)
        plt.show()
