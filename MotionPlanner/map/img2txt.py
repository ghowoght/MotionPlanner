# -*- coding: utf-8 -*-
"""
Created on Sun Dec 20 18:10:21 2020

@author: 25503
"""

import matplotlib.pyplot as plt # plt 用于显示图片
import numpy as np
import cv2


img = cv2.imread("./map.bmp",cv2.IMREAD_UNCHANGED)

h,w= (img.shape)
print(h, w)
# img = 
plt.imshow(img) # 显示图片
plt.axis('on') # 不显示坐标轴
plt.show()

f = open("./map.txt", "wb")
for i in range(h):
    for j in range(w):
        if img[i, j] == 0:
            f.write((str(1) + " " + str(i) + " " + str(j) + "\n").encode("utf-8"))
            
f.close()