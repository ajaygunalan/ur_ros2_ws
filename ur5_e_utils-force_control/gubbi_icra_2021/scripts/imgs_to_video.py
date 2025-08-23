import cv2
import numpy as np
import glob

img_array = []
for filename in glob.glob('/home/jsu30/Documents/ur5_imgs/*.png'):
    img = cv2.imread(filename)
    height, width, layers = img.shape
    width = int(width / 4 * 2)
    height = int(height / 2)
    size = (width,height)
    img_array.append(img[:height, :width])

out = cv2.VideoWriter('su_visual_servoing_1.avi',cv2.VideoWriter_fourcc(*'DIVX'), 15, size)

for i in range(len(img_array)):
    out.write(img_array[i])

out.release()
