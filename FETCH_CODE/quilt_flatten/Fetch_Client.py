from fetch_core import robot_interface
from connect_to_mrcnn import Detect
from cal_position import CalPosition
from manage_fetch_robot import FetchPose, GripperClient
from fetch_core.skeleton import Robot_Skeleton
import rospy


class FetchClient(object):
    def __init__(self):
        rospy.init_node('fetch_client')
        self.detect = Detect()
        self.cal_position = CalPosition()
        self.fetch_pose = FetchPose()
        self.gripper = GripperClient()

    def analysis_corner(self, corner_point):
        for point in corner_point:
            start_point = self.cal_position.get_base_position_from_pix(point[0], point[1])
            end_point = self.cal_position.get_base_position_from_pix(point[0], point[1])
    
    def move_corner(self, point_list):  # point_list
        position = self.cal_position.get_base_position_from_pix(point_list[0], point_list[1])
        position_end = self.cal_position.get_base_position_from_pix(point_list[2], point_list[3])
        position[0] = position[0] + 0.05
        position[2] = position[2] + 0.25
        rospy.sleep(1)
        self.fetch_pose.grasp_by_move_group(position)
        rospy.sleep(1)
        self.fetch_pose.control_torso(-0.21)
        self.gripper.close_gripper()
        rospy.sleep(1)
        self.fetch_pose.control_torso(0.21)

        position_end[0] = position_end[0] + 0.05
        position_end[2] = position_end[2] + 0.25
        self.fetch_pose.grasp_by_move_group(position_end)
        rospy.sleep(1)
        self.fetch_pose.control_torso(-0.21)
        self.gripper.open_gripper()
        rospy.sleep(1)
        self.fetch_pose.control_torso(0.21)

        self.fetch_pose.hige_pose()
        self.fetch_pose.stow()

    def gai_bei_zi(self, start_p, end_p):  # point_list
        position = self.cal_position.get_base_position_from_pix(start_p[0], start_p[1])
        position_end = self.cal_position.get_base_position_from_pix(end_p[0], end_p[1])
        position[0] = position[0] + 0.05
        position_end[1] = position_end[1] - 0.08
        position[2] = position[2] + 0.25
        rospy.sleep(1)
        self.fetch_pose.grasp_by_move_group(position)
        rospy.sleep(1)
        self.fetch_pose.control_torso(-0.22)
        self.gripper.close_gripper()
        rospy.sleep(1)
        self.fetch_pose.control_torso(0.21)

        position_end[0] = position_end[0] + 0.05
        position_end[1] = position_end[1] - 0.08
        position_end[2] = position_end[2] + 0.25
        self.fetch_pose.grasp_by_move_group(position_end)
        rospy.sleep(1)
        self.fetch_pose.control_torso(-0.10)
        self.gripper.open_gripper()
        rospy.sleep(1)
        self.fetch_pose.control_torso(0.10)

        self.fetch_pose.hige_pose()
        self.fetch_pose.stow()


if __name__ == '__main__':
    fetch_client = FetchClient()
    detect_info = fetch_client.detect.get_detect_info()
    print detect_info
    if len(detect_info['edge_point']) == 0:
        point_s = (detect_info['corner_point'][0][0] + detect_info['corner_point'][1][0]) / 2
        point_e = (detect_info['corner_point'][0][1] + detect_info['corner_point'][1][1]) / 2
        s_p = [point_s, point_e]
    else:
        s_p = detect_info['edge_point'][0]
    e_p = detect_info['neck_point'][0]
    fetch_client.gai_bei_zi(s_p, e_p)
