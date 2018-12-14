import rospy
from moveit_msgs.msg import MoveItErrorCodes
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from tf.transformations import quaternion_multiply
from transformer import Transformer
import moveit_commander


class GripperClient:
    def __init__(self):
        self.move_group = MoveGroupInterface("gripper", "base_link")
        self.joint_names = ['l_gripper_finger_joint', 'r_gripper_finger_joint']

    def open_gripper(self):
        value = [0.1, 0.1]
        self.__execu(value)
        # Plans the joints in joint_names to angles in pose

    def close_gripper(self):

        value = [0.0, 0.0]
        self.__execu(value)
        # Plans the joints in joint_names to angles in pose

    def __execu(self, value):
        self.move_group.moveToJointPosition(self.joint_names, value, wait=False)
        # Since we passed in wait=False above we need to wait here
        self.move_group.get_move_action().wait_for_result()
        result = self.move_group.get_move_action().get_result()
        if result:
            # Checking the MoveItErrorCode
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                print 'gripper'
            else:
                # If you get to this point please search for:
                rospy.logerr("Arm goal in state: %s",
                             self.move_group.get_move_action().get_joint_values())
        else:
            rospy.logerr("MoveIt! failure no result returned.")


class FetchPose:
    def __init__(self):
        self.move_group = MoveGroupInterface("arm_with_torso", "base_link")
        self.pose_group = moveit_commander.MoveGroupCommander("arm_with_torso")
        self.joint_names = ["torso_lift_joint", "shoulder_pan_joint",
                   "shoulder_lift_joint", "upperarm_roll_joint",
                   "elbow_flex_joint", "forearm_roll_joint",
                   "wrist_flex_joint", "wrist_roll_joint"]
        # planning_scene = PlanningSceneInterface("base_link")
        # planning_scene.removeCollisionObject("my_front_ground")
        # planning_scene.removeCollisionObject("my_back_ground")
        # planning_scene.removeCollisionObject("my_right_ground")
        # planning_scene.removeCollisionObject("my_left_ground")
        # planning_scene.addCube("my_front_ground", 2, 1.1, 0.0, -1.0)
        # planning_scene.addCube("my_back_ground", 2, -1.2, 0.0, -1.0)
        # planning_scene.addCube("my_left_ground", 2, 0.0, 1.2, -1.0)
        # planning_scene.addCube("my_right_ground", 2, 0.0, -1.2, -1.0)
        self.tuck_the_arm_joints_value = [0, 1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0]
        self.stow_joints_value = [0.3, 1.32, 0.7, 0.0, -2.0, 0.0, -0.57, 0.0]
        # self.high_pose_value = [0.38380688428878784, -0.12386894226074219, -0.31408262252807617, 2.1759514808654785, 0.0061359405517578125, 0.9556700587272644, -1.921694278717041, -1.6908303499221802]
        self.high_pose_value = [0.4946120488643647, -0.19136428833007812, -0.4793691635131836, 0.009587380103766918, 0.1629854440689087, 0.2983585298061371, 1.8430781364440918, -1.6992675065994263]

        self.trans = Transformer()

    def excute(self, joints_value):
        if rospy.is_shutdown():
            return False

        # Plans the joints in joint_names to angles in pose
        self.move_group.moveToJointPosition(self.joint_names, joints_value, wait=False)

        # Since we passed in wait=False above we need to wait here
        self.move_group.get_move_action().wait_for_result()
        result = self.move_group.get_move_action().get_result()

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

    def tuck_the_arm(self):
        self.excute(self.tuck_the_arm_joints_value)

    def stow(self):
        self.excute(self.stow_joints_value)

    def hige_pose(self):
        self.excute(self.high_pose_value)

    def control_torso(self, offset=0.1):
        joints_values = self.pose_group.get_current_joint_values()
        joints_values[0] += offset
        self.excute(joints_values)

    def grasp_by_move_group(self, point_xyz):
        wrist_roll_link_gripper_offset = 0.20
        q1 = self.trans.quaternion_from_direction((1, 0, 0), (0, 0, -1))
        q2 = self.trans.quaternion_from_axis(90, (1, 0, 0))
        quaternion = quaternion_multiply(q1, q2)
        pose = Pose(
            Point(point_xyz[0], point_xyz[1], point_xyz[2] + wrist_roll_link_gripper_offset),  # offset
            Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]))
        gripper_pose_stamped = PoseStamped()
        gripper_pose_stamped.header.frame_id = 'base_link'
        gripper_pose_stamped.header.stamp = rospy.Time.now()
        gripper_pose_stamped.pose = pose
        gripper_frame = 'wrist_roll_link'
        self.move_group.moveToPose(gripper_pose_stamped, gripper_frame)
        result = self.move_group.get_move_action().get_result()
        if result:
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                print("reach success!")
                return 'Success'
            else:
                rospy.logerr("reach fail")
                return 'Fail'
        else:
            rospy.logerr("MoveIt! failure no result returned.")
            return 'Fail'


if __name__ == '__main__':
    rospy.init_node("manage_fetch_robot")
    pose = FetchPose()
    # start_position = 0.888132710190764, -0.2603273599502236, 1.052433410024562224
    # pose.grasp_by_move_group(start_position)
    # pose.hige_pose()
    # pose.stow()

    # print pose.pose_group.get_current_joint_values()
    pose.tuck_the_arm()