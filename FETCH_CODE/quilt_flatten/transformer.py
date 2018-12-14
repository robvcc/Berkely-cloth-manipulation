#!/usr/bin/env python

import rospy
import math
import numpy as np
from tf import transformations, TransformListener
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped, Vector3, Quaternion, Transform


class Transformer(object):
    def __init__(self):
        self.tf_listener = TransformListener()
        # Enough time for listener to connect to the roscore
        # self.tf_sub = rospy.Subscriber('/tf', TFMessage, self.transform_callback)
        # self.tf2_msgs = TFMessage()
        rospy.sleep(1)

    def transform_callback(self, msg):
        self.tf2_msgs = msg

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

    def transform_of_frames(self, source_frame, target_frame):
        if self.tf_listener.frameExists(source_frame) and self.tf_listener.frameExists(target_frame):
            t = self.tf_listener.getLatestCommonTime(
                source_frame, target_frame)
            # Compute the transform from source to target
            position, quaternion = self.tf_listener.lookupTransform(
                target_frame, source_frame, t)
            return position, quaternion
        else:
            return None, None
    
    def quaternion_from_direction(self, source_direction, target_direction):
        """
        """
        # calc the rotate angle
        rotate_angle = self.angle_between_vectors(
            source_direction, target_direction)
        
        # calc the rotate axis
        rotate_axis = np.cross(source_direction, target_direction)
        quaternion = transformations.quaternion_about_axis(rotate_angle, rotate_axis)
        return quaternion
        # return Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3])

    def quaternion_from_axis(self, angle, direction):
        radian = math.pi * angle / 180
        q = transformations.quaternion_about_axis(radian, direction)
        return q

    def angle_between_vectors(self, a, b):
        '''Return the arc between two vectors
        '''
        vA = np.array(a)
        vB = np.array(b)
        lenA = np.sqrt(vA.dot(vA))
        lenB = np.sqrt(vB.dot(vB))

        if (lenA == 0 or lenB == 0):
            return 0

        cos = vA.dot(vB) / (lenA*lenB)
        return np.arccos(cos)

    def arc_to_euler(self, arc):
        '''Convert the arc to euler angle
        '''
        return float(arc)*180.0/np.pi


    # def direction_of_frame(self, source_frame, target_frame):
    #     position, quaternion = self.transform_of_frames(source_frame, target_frame)
    #     return transformations.euler_from_quaternion(quaternion)

if __name__ == '__main__':
    rospy.init_node('transformer')

    tf = Transformer()
    print tf.tf_listener.frameExists('head_camera_rgb_optical_frame')
    print tf.tf_listener.frameExists('base_link')
    r2b = tf.transform_matrix_of_frames(
        'head_camera_rgb_optical_frame', 'base_link')
    print r2b