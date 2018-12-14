import os
import matplotlib.pyplot as plt
from mrcnn.config import Config
from mrcnn import model as modellib
from mrcnn import utils_temp as utils
from mrcnn import visualize
from mrcnn.model import log
# print(ROOT_DIR)
class ShapesConfig(Config):
    NAME = "shape"
    GPU_COUNT = 1
    IMAGES_PER_GPU = 2
    NUM_CLASSES = 1 + 1
    IMAGE_MIN_DIM = 320
    IMAGE_MAX_DIM = 384
    RPN_ANCHOR_SCALES = (8 * 6, 16 * 6, 32 * 6, 64 * 6, 128 * 6)
    TRAIN_ROIS_PER_IMAGE = 100
    STEPS_PER_EPOCH = 100
    VALIDATION_STEPS = 50


class InferenceConfig(ShapesConfig):
    GPU_COUNT = 1
    IMAGES_PER_GPU = 1


class Img2corner(object):
    def importweight(self):
        ROOT_DIR = "/home/vcc/Mask_RCNN"
        MODEL_DIR = "/home/vcc/Mask_RCNN/logs"

        inference_config = InferenceConfig()

        model = modellib.MaskRCNN(mode="inference",
                                  config=inference_config,
                                  model_dir=MODEL_DIR)

        model_path = ROOT_DIR + "/logs/corner.h5"

        print("Loading weights from ", model_path)
        model.load_weights(model_path, by_name=True)
        return model

    def image2cor(self, img):
        model = self.importweight()
        # IMG_PATH=ROOT_DIR+"/corner_data/pic/37.png"
        original_image = img
        results = model.detect([original_image], verbose=1)
        # class_names=['BG', 'corner']
        r = results[0]
        # def get_ax(rows=1, cols=1, size=8):
        #     _, ax = plt.subplots(rows, cols, figsize=(size * cols, size * rows))
        #     return ax
        # visualize.display_instances(original_image, r['rois'], r['masks'], r['class_ids'],
        #                             class_names, r['scores'], ax=get_ax())
        return r


if __name__ == '__main__':
    import time
    IMG_PATH="/home/vcc/Mask_RCNN/corner_data/pic/37.png"
    o=time.clock()
    i=Img2corner()
    a=time.clock()
    import skimage
    original_image = skimage.io.imread(IMG_PATH)
    i.image2cor(original_image)

    b=time.clock()
    print("load model time:",a-o,"s")
    print("detect time:",b-a,"s")
