import os
from mrcnn.config import Config
from mrcnn import model as modellib
from mrcnn import visualize
import skimage
import time
import cv2
import numpy as np
import matplotlib.pyplot as plt
class_names=['BG', 'edge','quilt','corner','head']
class ShapesConfig(Config):
    NAME = "shape"
    GPU_COUNT = 1
    IMAGES_PER_GPU = 2
    NUM_CLASSES = 1 + 4
    IMAGE_MIN_DIM = 320
    IMAGE_MAX_DIM = 384
    RPN_ANCHOR_SCALES = (8 * 6, 16 * 6, 32 * 6, 64 * 6, 128 * 6)
    TRAIN_ROIS_PER_IMAGE = 100
    STEPS_PER_EPOCH = 100
    VALIDATION_STEPS = 50
class InferenceConfig(ShapesConfig):
    GPU_COUNT = 1
    IMAGES_PER_GPU = 1

class inference_corner():
    def __init__(self):
        self.ROOT_DIR = os.path.abspath("../..")
        print(self.ROOT_DIR)
        # Directory to save logs and trained model
        MODEL_DIR = os.path.join(self.ROOT_DIR, "logs")
        model_path=self.ROOT_DIR+"/logs/quilt and baby.h5"  # without it
        model_path = self.ROOT_DIR + "/logs/corner.h5"  # without it
        model_path = self.ROOT_DIR + "/logs/2018.12.2.h5"

        print("Loading weights from ", model_path)
        start=time.clock()
        inference_config = InferenceConfig()
        self.model = modellib.MaskRCNN(mode="inference",
                                  config=inference_config,
                                  model_dir=MODEL_DIR)
        self.model.load_weights(model_path, by_name=True)
        end=time.clock()
        print(end-start)

    def _drop(self,r):
        aa = {}; quilt = 0
        left = 0; right = 0
        up = 0; down = 0
        for i in range(1, 5):
            if i in r['class_ids']:
                index = np.argwhere(r['class_ids'] == i)  # 索引值，下标值
                print(class_names[i], ':')
                # a.append(str(class_names[i]))
                if i == 1:
                    b = []
                    for k in index:
                        rois = r['rois'][k[0]]
                        x = (int)((rois[0] + rois[2]) / 2);
                        y = (int)((rois[1] + rois[3]) / 2)
                        print("edge:", x, ",", y)  # 坐标
                        b.append(str(x) + "," + str(y))
                    aa['edge'] = b
                if i == 2:  # four corner in bbox of quilt
                    rois = r['rois'][index[0][0]]
                    quilt = index[0][0]
                    left = rois[1];
                    right = rois[3];
                    up = rois[0];
                    down = rois[2]
                    print(rois[0], ",", rois[1])  # leftup
                    print(rois[0], ",", rois[3])  # rightup
                    print(rois[2], ",", rois[1])  # leftdown
                    print(rois[2], ",", rois[3])  # rightdown
                    a = []
                    a.append(str(int(rois[0])) + "," + str(int(rois[1])))
                    a.append(str(int(rois[0])) + "," + str(int(rois[3])))
                    a.append(str(int(rois[2])) + "," + str(int(rois[1])))
                    a.append(str(int(rois[2])) + "," + str(int(rois[3])))
                    aa['quilt'] = a

                if i == 3:  # four center bbox point of corner
                    b = []
                    for k in index:
                        rois = r['rois'][k[0]]
                        x = (int)((rois[0] + rois[2]) / 2);
                        y = (int)((rois[1] + rois[3]) / 2)
                        detect_x = (x - up) > (down - x) and down or up
                        detect_y = (y - left) > (right - y) and right or left
                        if detect_y == right:
                            print("move:", x, ",", y, ",", detect_x, ",", detect_y)  # 坐标
                            b.append(str(x) + "," + str(y) + "," + str(detect_x) + "," + str(detect_y))
                    c = []
                    print("b:",b)
                    if len(b)>1:
                        if int(b[0].split(",")[2]) > int(b[1].split(",")[2]):
                            c.append(b[1]);
                            c.append(b[0]);
                            b = c
                    aa['corner'] = b

                if i == 4:
                    neck = []
                    c = [];
                    rois = r['rois'][index[0][0]]
                    print(index[0][0])
                    middle = (left + right) / 2
                    body_up = int((3 * rois[0] - rois[2]) / 2)
                    body_down = int((3 * rois[2] - rois[0]) / 2)
                    if middle < (rois[1] + rois[3]) / 2:  # 左边
                        body_left = int(4 * rois[1] - 3 * rois[3])
                        body_right = int(rois[1])
                        neck.append(str(int((rois[0] + rois[2]) / 2)) + ',' + str(int(body_right)))
                    else:  # 右边
                        body_right = int(4 * rois[3] - 3 * rois[1])
                        body_left = int(rois[3])
                        neck.append(str(int((rois[0] + rois[2]) / 2)) + ',' + str(int(body_left)))
                    # print(body_down,body_left,body_right,body_up)
                    aa['head'] = neck
                    count = 0
                    mask = np.zeros((r['masks'].shape[0], r['masks'].shape[1]))
                    for i in range(body_up, body_down):
                        for j in range(body_left, body_right):
                            if r['masks'][i][j][quilt]:
                                count += 1
                    # print(count)
                    b = (body_up - body_down) * (body_left - body_right)
                    print(count / b)
                    c.append(str(int(1000 * count / b)))
                    aa['cover'] = c
            else:
                if i == 4:
                    aa['cover'] = ['xxxx']
                print(class_names[i], ":xxxx");
                a = []
                a.append('xxxx')
                aa[str(class_names[i])] = a
        return aa
    def train(self):
        start=time.clock()
        file_dir="C:/wamp64/www/ThinkingQ/fetch_camera/"
        list=os.listdir(file_dir)
        list.sort(key=lambda fn: os.path.getmtime(file_dir+fn) if not os.path.isdir(file_dir+fn) else 0)
        IMG_PATH=self.ROOT_DIR+"/new_data/pic/29.png"
        IMG_PATH="C:/wamp64/www/ThinkingQ/fetch_camera/"+list[-1]
        # IMG_PATH="C:/wamp64/www/ThinkingQ/fetch_camera/5.png"
        # IMG_PATH="C:/Users/VCC/Desktop/ftp sharing/new_data/pic/281.png"
        # IMG_PATH="C:/Users/VCC/Desktop/5.jpg"
        print(IMG_PATH)

        original_image = skimage.io.imread(IMG_PATH)# --- 绝对路径选择图片

        results = self.model.detect([original_image], verbose=1)
        class_names=['BG', 'corner']
        class_names=['BG', 'baby','quilt']
        class_names=['BG', 'edge','quilt','corner','head']

        r = results[0]
        print(type(r))
        self._drop(r)
        end=time.clock()
        def get_ax(rows=1, cols=1, size=8):
            _, ax = plt.subplots(rows, cols, figsize=(size * cols, size * rows))
            return ax
        print(end-start)
        r = results[0]

        from samples.shapes.result2mask import result2mask as r2m
        r2m1=r2m()
        count=0
        mask=np.zeros((r['masks'].shape[0],r['masks'].shape[1]))
        for i in range(r['masks'].shape[0]):
            for j in range(r['masks'].shape[1]):
                if (r['masks'][i][j]+0).sum()>0:
                    count+=1
                    mask[i][j]+=1
        # print("the bounding box of leftup and rightdowm:",r['rois'])
        # print('scores:')
        # for i in r['rois']:
        #     print((i[0] + i[2]) / 2)
        #     print((i[1] + i[3]) / 2)

        #plt.imshow(mask)
        # print((r['masks'][0][0]+0).sum())
        print('gripper point:')
        for i in r['rois']:
            print((i[0] + i[2]) / 2 , "," ,
                  (i[1] + i[3]) / 2)
        print(r['scores'])
        print(r['class_ids'])
        plt.imshow(mask)
        body_up = 0
        body_down = 640
        body_left = 0
        body_right = 320
        a=1.0 * (mask[body_up:body_down][body_left:body_right] + 0).sum()
        b= (body_up - body_down) * (body_left - body_right)
        print(a/b)
        # print((mask[body_up:body_down][body_left:body_right] + 0).sum())

        plt.imshow(visualize.display_instances(original_image, r['rois'], r['masks'], r['class_ids'],
                            class_names, r['scores'], ax=get_ax()))
        plt.show()

if __name__ == '__main__':
    infer=inference_corner()
    infer.train()