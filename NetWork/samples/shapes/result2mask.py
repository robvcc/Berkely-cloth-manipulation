import matplotlib.pyplot as plt
import numpy as np
class result2mask:
    def r2mask(r):
        count=0
        mask=np.zeros((r['masks'].shape[0],r['masks'].shape[1]))
        for i in range(r['masks'].shape[0]):
            for j in range(r['masks'].shape[1]):
                if (r['masks'][i][j]+0).sum()>0:
                    count+=1
                    mask[i][j]+=1
        print("the bounding box of leftup and rightdowm:",r['rois'])
        print('scores:')
        for i in r['rois']:
            print((i[0] + i[2]) / 2)
            print((i[1] + i[3]) / 2)

        plt.imshow(mask)
        print((r['masks'][0][0]+0).sum())

    def save_image(self,SAVE_DIR,image):
        plt.imsave(SAVE_DIR,image,cmap='gray')