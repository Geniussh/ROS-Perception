#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from skimage import exposure 
from skimage import feature


class LoadPeople(object):

    def __init__(self):
    
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
        self.bridge_object = CvBridge()

    def camera_callback(self,data):
        try:
            # We select bgr8 because its the OpenCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        
        img = cv2.imread('/home/user/catkin_ws/src/opencv_for_robotics_images/Unit_3/Course_images/test_e.jpg')
        imX = 720
        imY = 1080
        img = cv2.resize(img,(imX,imY))
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        (H, hogImage) = feature.hog(gray, orientations=9, pixels_per_cell=(8, 8),
            cells_per_block=(2, 2),
            visualize=True)

        hogImage = exposure.rescale_intensity(hogImage, out_range=(0, 255))
        hogImage = hogImage.astype("uint8")

        cv2.imshow('features',hogImage)
        cv2.waitKey(1)

def main():
    load_people_object = LoadPeople()
    rospy.init_node('load_people_node', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()