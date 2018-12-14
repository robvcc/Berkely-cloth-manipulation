#!/usr/bin/env python

from __future__ import print_function

import roslib;
import moveit_commander

roslib.load_manifest('teleop_twist_keyboard')
import rospy
from moveit_python import MoveGroupInterface, PlanningSceneInterface
from geometry_msgs.msg import Twist
from moveit_msgs.msg import MoveItErrorCodes

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

* : fudu (+0.01)
/ : fudu (-0.01)

anything else : stop
'0':"torso_lift_joint"
'1': "shoulder_pan_joint"
'2': "shoulder_lift_joint"
'3': "upperarm_roll_joint"
'4': "elbow_flex_joint"
'5': "forearm_roll_joint"
'6': "wrist_flex_joint"
'7': "wrist_roll_joint"

CTRL-C to quit
"""

moveBindings = {
    'i': (1, 0, 0, 0),
    'o': (1, 0, 0, -1),
    'j': (0, 0, 0, 1),
    'l': (0, 0, 0, -1),
    'u': (1, 0, 0, 1),
    ',': (-1, 0, 0, 0),
    '.': (-1, 0, 0, 1),
    'm': (-1, 0, 0, -1),
    'O': (1, -1, 0, 0),
    'I': (1, 0, 0, 0),
    'J': (0, 1, 0, 0),
    'L': (0, -1, 0, 0),
    'U': (1, 1, 0, 0),
    '<': (-1, 0, 0, 0),
    '>': (-1, -1, 0, 0),
    'M': (-1, 1, 0, 0),
    't': (0, 0, 1, 0),
    'b': (0, 0, -1, 0),
}

jointsAddBindings = {
    '0': "torso_lift_joint",
    '1': "shoulder_pan_joint",
    '2': "shoulder_lift_joint",
    '3': "upperarm_roll_joint",
    '4': "elbow_flex_joint",
    '5': "forearm_roll_joint",
    '6': "wrist_flex_joint",
    '7': "wrist_roll_joint",
}


def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)


if __name__ == "__main__":
    # '''run in terminal'''
    settings = termios.tcgetattr(sys.stdin)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.init_node('teleop_twist_keyboard')
    pose_group = moveit_commander.MoveGroupCommander("arm_with_torso")
    move_group = MoveGroupInterface("arm_with_torso", "base_link")
    joint_names = ["torso_lift_joint","shoulder_pan_joint",
                   "shoulder_lift_joint", "upperarm_roll_joint",
                   "elbow_flex_joint", "forearm_roll_joint",
                   "wrist_flex_joint", "wrist_roll_joint"]
    fd = 0.05
    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    x = 0
    y = 0
    z = 0
    th = 0
    status = 0
    add = False
    try:
        print(msg)
        print(vels(speed, turn))
        while True:
            key = getKey()

            if key == '*':
                fd += 0.01
            elif key == '/':
                fd -= 0.01
            elif key == '+':
                add = True
            elif key == '-':
                add = False
            elif key in moveBindings.keys():
                x = moveBindings[key][0]
                y = moveBindings[key][1]
                z = moveBindings[key][2]
                th = moveBindings[key][3]
                twist = Twist()
                twist.linear.x = x * speed
                twist.linear.y = y * speed
                twist.linear.z = z * speed
                twist.angular.x = 0
                twist.angular.y = 0
                twist.angular.z = th * turn
                pub.publish(twist)
            elif key in jointsAddBindings.keys():
                disco_poses = pose_group.get_current_joint_values()
                if add is True:
                    disco_poses[int(key)] += fd
                elif add is False:
                    disco_poses[int(key)] -= fd

                if rospy.is_shutdown():
                    break
                move_group.moveToJointPosition(joint_names, disco_poses, wait=False)
                move_group.get_move_action().wait_for_result()
                result = move_group.get_move_action().get_result()
                if result:
                    # Checking the MoveItErrorCode
                    if result.error_code.val == MoveItErrorCodes.SUCCESS:
                        rospy.loginfo("Disco!")
                        print(disco_poses)
                    else:
                        # If you get to this point please search for:
                        # moveit_msgs/MoveItErrorCodes.msg
                        rospy.logerr("Arm goal in state: %s",
                                     move_group.get_move_action().get_state())
                else:
                    rospy.logerr("MoveIt! failure no result returned.")
            else:
                break
            print("key : ", key)
            print("add : ", add)
            print("fudu : ", fd)
            print()

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        pub.publish(twist)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

