import numpy as np
from vedo import *
import cv2
import vtk

point = Lines([[1,2,3]],[[1,2,5]],c=(255,255,0))


#show(point)

path = 'G:/学习/2-博士/申请文件/IMG_65522min2.jpg'
print(cv2.imdecode(np.fromfile(path,dtype=np.uint8),-1).shape)