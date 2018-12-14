import numpy as np
import pypcd
import transformer
from fetch_core.camera import RGBD
import rospy


class CalPosition:
    def __init__(self):
        trans = transformer.Transformer()
        self.r2b = trans.transform_matrix_of_frames(
            'head_camera_rgb_optical_frame', 'base_link')
        self.camera = RGBD()
        self.camera.read_point_cloud()
        self.point_cloud = None

    def convert_camera_position_to_base_position(self, array):
        # print 'r2b', self.r2b
        ret = np.dot(self.r2b[0:3, 0:3], array)  # Rotate
        ret = ret + self.r2b[0:3, 3]
        return ret

    def update_point_cloud(self):
        self.point_cloud = self.camera.read_point_cloud()
        while self.point_cloud is None:
            self.point_cloud = self.camera.read_point_cloud()
        print "point_cloud update"

    def get_camera_position_by_pix(self, x, y):

        arr = pypcd.numpy_pc2.pointcloud2_to_array(self.point_cloud, split_rgb=True)
        po = pypcd.numpy_pc2.get_xyz_points(arr, remove_nans=False)
        return po[x, y]

    def get_base_position_from_pix(self, x, y):
        self.update_point_cloud()
        camera_p = self.get_camera_position_by_pix(x, y)
        base_p = self.convert_camera_position_to_base_position(camera_p)
        base_position = [base_p[0], base_p[1], base_p[2]]
        print "base_position", base_position
        return base_position

    def get_base_average_position_from_pix(self, x, y):
        size = 25
        step = 5
        self.update_point_cloud()
        base_position_all = np.zeros(3)
        for i in range(x-size, x+size, step):
            for j in range(y-size, y+size, step):
                camera_p = self.get_camera_position_by_pix(i, j)
                base_p = self.convert_camera_position_to_base_position(camera_p)
                base_position_all += base_p
        base_position = base_position_all/((size/step*2)**2)
        print "base_average_position", base_position
        return base_position


if __name__ == '__main__':
    rospy.init_node("test_CalPosition")
    cal = CalPosition()
    print "a",cal.get_base_position_from_pix(240, 320)
    print 'b',cal.get_base_average_position_from_pix(240, 320)
