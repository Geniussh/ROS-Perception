#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np


class Transformations(object):

    def __init__(self):
    
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
        self.bridge_object = CvBridge()

    def camera_callback(self,data):
        try:
            # We select bgr8 because its the OpenCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        example_path = '/home/user/catkin_ws/src/opencv_for_robotics_images/Unit_2/Course_images/world.png'
            
        img = cv2.imread(example_path,0)
        img = cv2.resize(img,(250,250))

        #Define a kernel for the erosion 
        kernel_a = np.ones((5,5),np.uint8)
        erosion = cv2.erode(img,kernel_a,iterations = 1)

        #Define a kernel for the dilation
        kernel_b = np.ones((3,3),np.uint8)
        dilation = cv2.dilate(img,kernel_b,iterations = 1)

        #Define a kernel for the opening
        kernel_c = np.ones((7,7),np.uint8)
        opening = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel_c)

        #Define a kernel for the closing
        kernel_d = np.ones((7,7),np.uint8)
        closing = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel_d)

        cv2.imshow('Original',img)
        cv2.imshow('Erosion',erosion)
        cv2.imshow('Dilation',dilation)
        cv2.imshow('Opening',opening)
        cv2.imshow('Closing',closing)

        cv2.waitKey(1)



def main():
    transformations_object = Transformations()
    rospy.init_node('transformations_node', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()