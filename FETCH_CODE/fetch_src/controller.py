#!/usr/bin/env python

import copy
import actionlib
import rospy
import time
from math import sin, cos
from moveit_python import (MoveGroupInterface,
                           PlanningSceneInterface,
                           PickPlaceInterface)
from moveit_python.geometry import rotate_pose_msg_by_euler_angles

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import PointHeadAction, PointHeadGoal, GripperCommandAction, GripperCommandGoal
from grasping_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal
from geometry_msgs.msg import PoseStamped, Quaternion, Point, Pose, Vector3
from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from transformer import Transformer
from tf import transformations
import numpy as np


class PlanningSceneClient(object):
    '''Define ground plane
    This creates objects in the planning scene that mimic the ground
    If these were not in place gripper could hit the ground'''

    def __init__(self):
        self.planning_scene = PlanningSceneInterface("base_link")
        self.planning_scene.removeCollisionObject("my_front_ground")
        self.planning_scene.removeCollisionObject("my_back_ground")
        self.planning_scene.removeCollisionObject("my_right_ground")
        self.planning_scene.removeCollisionObject("my_left_ground")
        self.planning_scene.addCube("my_front_ground", 2, 1.1, 0.0, -1.0)
        self.planning_scene.addCube("my_back_ground", 2, -1.2, 0.0, -1.0)
        self.planning_scene.addCube("my_left_ground", 2, 0.0, 1.2, -1.0)
        self.planning_scene.addCube("my_right_ground", 2, 0.0, -1.2, -1.0)

    def close(self):
        self.planning_scene.removeCollisionObject("my_front_ground")
        self.planning_scene.removeCollisionObject("my_back_ground")
        self.planning_scene.removeCollisionObject("my_right_ground")
        self.planning_scene.removeCollisionObject("my_left_ground")


class GripperController(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient(
            "gripper_controller/gripper_action", GripperCommandAction)
        rospy.loginfo("Waiting for gripper_action...")
        self.move_group = MoveGroupInterface("arm_with_torso", "base_link")
        self.goal = GripperCommandGoal()

        self.client.wait_for_server()

    def close(self):
        '''Close the gripper'''
        self.goal.command.position = 0
        self.goal.command.max_effort = 100
        self.client.send_goal(self.goal)
        self.client.wait_for_result()

    def release(self):
        '''Release the gripper'''
        self.goal.command.position = 1
        self.goal.command.max_effort = 100
        self.client.send_goal(self.goal)
        self.client.wait_for_result()

    def tuck(self):
        '''Tuck the arm of fetch'''
        position = (0.0555, -0.138, 0.571)
        quaternion = (0.45848, -0.50438, 0.5078, 0.5268)
        self.move_to_pose(position, quaternion)

    def move_to_pose(self, postion, quaternion, frame='base_link'):
        target_frame = 'wrist_roll_link'

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = frame
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.pose = Pose(
            Point(postion[0], postion[1], postion[2]),
            Quaternion(quaternion[0], quaternion[1],
                       quaternion[2], quaternion[3])
        )
        if not rospy.is_shutdown():
            self.move_group.moveToPose(pose_stamped, target_frame)
            result = self.move_group.get_move_action().get_result()
            if result:
                if result.error_code.val == MoveItErrorCodes.SUCCESS:
                    rospy.loginfo("Move success!")
                else:
                    rospy.logerr("in state: %s",
                                 self.move_group.get_move_action().get_state())
            else:
                rospy.logerr("MoveIt! failure no result returned.")
        # self.move_group.get_move_action().cancel_all_goals()

    def move_to_some_pose(self, postion, direction, frame='base_link'):
        target_frame = 'wrist_roll_link'
        quatrenion = Transformer().quaternion_from_direction((1, 0, 0), direction)

        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = frame
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.pose = Pose(
            Point(postion[0], postion[1], postion[2]),
            quatrenion
        )
        if not rospy.is_shutdown():
            self.move_group.moveToPose(pose_stamped, target_frame)
            result = self.move_group.get_move_action().get_result()
            if result:
                if result.error_code.val == MoveItErrorCodes.SUCCESS:
                    rospy.loginfo("See!")
                else:
                    rospy.logerr("in state: %s",
                                 self.move_group.get_move_action().get_state())
            else:
                rospy.logerr("MoveIt! failure no result returned.")
        self.move_group.get_move_action().cancel_all_goals()

    def wave(self):
        '''Just for fun'''
        gripper_poses = [((0.347, 0.045, 1.022), (-0.174, -0.301, 0.273, 0.635)),
                         ((0.347, 0.245, 1.022), (-0.274, -0.701, 0.173, 0.635))
                         ]
        for i in range(5):
            for pose in gripper_poses:
                self.move_to_pose(pose[0], pose[1])
        self.stop()

    def stop(self):
        """This stops all arm movement goals
        It should be called when a program is exiting so movement stops
        """
        self.move_group.get_move_action().cancel_all_goals()


class HeadController(object):
    def __init__(self):
        self.client = actionlib.SimpleActionClient(
            "head_controller/point_head", PointHeadAction)
        rospy.loginfo("Waiting for head_controller...")
        self.client.wait_for_server()

    def look_at(self, x, y, z, frame, duration=1.0):
        goal = PointHeadGoal()
        goal.target.header.stamp = rospy.Time.now()
        goal.target.header.frame_id = frame
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z
        goal.min_duration = rospy.Duration(duration)
        goal.pointing_frame = "head_tilt_link"
        self.client.send_goal(goal)
        self.client.wait_for_result()


if __name__ == "__main__":

    rospy.init_node("demo")

    # tf = Transformer()
    p = HeadController()
    p.look_at(0.8, 0.1, 0.72, 'base_link')
    # g = GripperController()
    # g.wave()
    # g.tuck()
    # g.stop()
    # g.move_to_some_pose((0.5, 0, 1), (1, 0, 0))
    # p.look_at(0.8, -0.1, 0.72, 'base_link')

    # while True:
    # transform_matrix = tf.transformMatrix('base_link', 'head_tilt_link')
    # print transform_matrix
    # p = raw_input("Input position: ")
    # try:
    #     point = [ float(i) for i in p.split() ]
    # except:
    #     break

    # ac.look_at(point[0], point[1], point[2], "base_link")
