#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Range
import cv2
import numpy as np


class LoadPeople(object):

    def __init__(self):
    
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
        self.bridge_object = CvBridge()
        self.hector_sub = rospy.Subscriber("/sonar_height", Range, self.sonar_callback) 
        self.a = 0.0  # store the height to the ground
        self.b = 0.0  # store the last right/left detection

    def sonar_callback(self, msg):
        self.a = msg.range

    def camera_callback(self,data):
        try:
            # We select bgr8 because its the OpenCV encoding by default
            img = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        
        hog = cv2.HOGDescriptor()

        #We set the hog descriptor as a People detector
        hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        #The image is pretty big so we will gibve it a resize
        imX = 700
        imY = 500
        img = cv2.resize(img, (imX, imY))

        #We will define de 8x8 blocks in the winStride
        boxes, weights = hog.detectMultiScale(img, winStride=(8,6))
        boxes = np.array([[x, y, x + w, y + h] for (x, y, w, h) in boxes])

        for (xA, yA, xB, yB) in boxes:
            
            #Center in X 
            medX = xB - xA 
            xC = int(xA+(medX/2)) 

            #Center in Y
            medY = yB - yA 
            yC = int(yA+(medY/2)) 

            #Draw a circle in the center of the box 
            cv2.circle(img, (xC,yC), 1, (0,255,255), -1)

            # display the detected boxes in the original picture
            cv2.rectangle(img, (xA, yA), (xB, yB),
                                (255, 255, 0), 2)    

            if self.a > 1.96:
                if xC < int(imX/2)-50 and self.b != 1:
                    print("The person is on the left")
                    self.b = 1
                elif xC > int(imX/2)+50 and self.b != 2:
                    print("The person is on the right")
                    self.b = 2
                elif int(imX/2)-50 <= xC <= int(imX/2)+50 and self.b != 3:
                    print("The person is in the center")
                    self.b = 3
                
                cv2.imshow('human_detect', img)

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