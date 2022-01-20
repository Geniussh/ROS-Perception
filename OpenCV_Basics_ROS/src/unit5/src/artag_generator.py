#!/usr/bin/env python

import cv2 
import numpy as np 
from cv2 import aruco

#Initialize the dictionary
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
for i in range (1, 5):

    size = 700
    img = aruco.drawMarker(aruco_dict, i, size)
    
    cv2.imwrite('/home/user/catkin_ws/src/unit5/Tags/image_'+str(i)+".jpg",img)
    
    cv2.imshow('artag',img)
    cv2.waitKey(0)
    cv2.destroyAllWindows