import socket  
import os  
import time
sk = socket.socket()  
# print(sk)  
  
address=('172.31.233.133',8787)
sk.connect(address)  
BASE_DIR = os.path.dirname(os.path.abspath(__file__))  
while True:
    cmd='post';path="fetch_camera/164.png"
    path=os.path.join(BASE_DIR,path)  
  
    filename = os.path.basename(path)
  
    file_size = os.stat(path).st_size
  
    file_info='post|%s|%s'%(filename,file_size)
   # print(file_info)
    sk.sendall(bytes(file_info,'utf8'))
  
    f = open(path,'rb')  
    has_sent=0  
    while has_sent!=file_size:  
        data = f.read(1024)  
        sk.sendall(data)                            
        has_sent+=len(data)  
    
    data=sk.recv(1024)   
    print(str(data))
    f.close()  
    print('上传成功') 
    break