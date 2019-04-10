import socket
import MaskRCNN
import os
import struct
import time
from mrcnn import visualize
import matplotlib.pyplot as plt
import threading

HOST, PORT = '', 7777

listen_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
listen_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
listen_socket.bind((HOST, PORT))
listen_socket.listen(1)
print ('Serving HTTP on port %s ...' % PORT)
model=MaskRCNN.load_model()
# BASE_DIR = os.path.dirname(os.path.abspath(__file__)) 
BASE_DIR = "c:/wamp64/www/ThinkingQ"
def showImage(image):
    plt.ion()
    plt.imshow(image)
    plt.pause(3)
    print("1 second")
    plt.close()
while True:
    try:
        client_connection, client_address = listen_socket.accept()
        request = client_connection.recv(1024)
        print (request)
        cmd,filename,filesize=str(request,'utf8').split('|')   
        path = os.path.join(BASE_DIR,"fetch_camera/",str(time.time())+'.png')
        filesize=int(filesize)
        f = open(path, 'ab')
        has_receive = 0
        while has_receive != filesize:
            data = client_connection.recv(1024)
            f.write(data)
            has_receive += len(data)
        f.close()

        r, image = MaskRCNN.detect(path, model)

        a = MaskRCNN._drop(r)
        r=str(a)
        client_connection.sendall(r.encode())
        # threading._start_new_thread(showImage,(image))
        plt.ion()
        plt.imshow(image)
        plt.pause(3)
        print("1 second")
        plt.close()

        client_connection.close()
    except Exception:
        continue