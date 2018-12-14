from fetch_core import robot_interface, camera
import time
import cv2
import numpy as np
import requests
import rospy
import socket
import os


class Detect:

    def __init__(self):
        self.detect_ip = '172.31.76.30'
        self.detect_port = 7777
        self.cam = camera.RGBD()

    def get_detect_info(self):
        print "get_detect_info"
        file_size = 0
        while file_size == 0:
            time.sleep(0.05)
            img = self.cam.read_color_data()
            cv2.imwrite("image/fetch.png", img)

            path = "image/fetch.png"
            # path = "../fetch_core/image/26.png"
            filename = os.path.basename(path)
            file_size = os.stat(path).st_size
            file_info = 'post|%s|%s' % (filename, file_size)
            time.sleep(0.2)
        print "file info", file_info
        sk = socket.socket()
        address = (self.detect_ip, self.detect_port)
        sk.connect(address)
        sk.sendall(bytes(file_info))
        f = open(path, 'rb')
        has_sent = 0
        while has_sent != file_size:
            data = f.read(1024)
            sk.sendall(data)
            has_sent += len(data)

        data = sk.recv(1024)
        sk.close()
        f.flush()
        f.close()
        rs = str(data)
        print rs

        detect_info = eval(rs)
        edge_point = []
        for s in detect_info['edge']:
            if s == 'xxxx':
                break
            edge_point.append([int(s.split(',')[0]), int(s.split(',')[1])])

        quilt_point = []
        for s in detect_info['quilt']:
            if s == 'xxxx':
                break
            quilt_point.append([int(s.split(',')[0]), int(s.split(',')[1])])

        neck_point = []
        for s in detect_info['head']:
            if s == 'xxxx':
                break
            neck_point.append([int(s.split(',')[0]), int(s.split(',')[1])])

        corner_point = []
        for s in detect_info['corner']:
            if s == 'xxxx':
                break
            corner_point.append(
                [int(s.split(',')[0]), int(s.split(',')[1]), int(s.split(',')[2]), int(s.split(',')[3])])

        cover = []
        for s in detect_info['cover']:
            if s == 'xxxx':
                break
            cover.append([int(s)])
        return {'edge_point':edge_point, 'quilt_point':quilt_point,
                'neck_point':neck_point, 'corner_point':corner_point, 'cover':cover}


if __name__ == '__main__':
    rospy.init_node("test_connect")
    detect = Detect()
    info = detect.get_detect_info()
    print info