import tf
from image_geometry import PinholeCameraModel as PCM
import rospy
from tf2_msgs.msg import TFMessage

class cor2:

    def cor2point(self, from_, to_):

        listener = tf.TransformListener()
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                (trans, rot) = listener.lookupTransform(to_, from_, rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            rate.sleep()
            if trans:
                return trans


def broadcast_poses():
    robot = robot_interface.Robot_Interface()
    cam = camera.RGBD()

    not_read = True
    while not_read:
        try:
            cam_info = cam.read_info_data()
            if (not cam_info == None):
                not_read = False
        except:
            rospy.logerr('info not recieved')

    pcm = PCM()
    pcm.fromCameraInfo(cam_info)
    point = (208+(323-208)/2, 247+(424-247)/2)
    print(point)
    #(208.076538,247.264099) (323.411957,242.667325) (204.806457,424.053619) (324.232857,434.011618)
    dep_data = robot.get_img_data()[1]
    print(dep_data)
    dep = robot.get_depth(point, dep_data)
    print(dep)
    br = tf.TransformBroadcaster()
    td_points = pcm.projectPixelTo3dRay(point)
    # pose = np.array([td_points[0], td_points[1], 0.001 * dep])
    norm_pose = np.array(td_points)
    norm_pose = norm_pose / norm_pose[2]
    norm_pose = norm_pose * (dep)
    a = tf.transformations.quaternion_from_euler(
        ai=-2.355, aj=-3.14, ak=0.0)
    b = tf.transformations.quaternion_from_euler(
        ai=0.0, aj=0.0, ak=1.57)
    base_rot = tf.transformations.quaternion_multiply(a, b)
    br.sendTransform(norm_pose,
                     base_rot,
                     rospy.Time.now(),
                     'tree',
                     'head_camera_rgb_optical_frame')

if __name__ == '__main__':
    rospy.init_node("get_p")
    cor = cor2()
    point = cor.cor2point('tree', 'base_link')
    print(point)