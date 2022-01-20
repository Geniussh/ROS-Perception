#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from cv2 import aruco


class LoadPolygon2(object):

    def __init__(self):
    
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
        self.bridge_object = CvBridge()

    def camera_callback(self,data):
        try:
            # We select bgr8 because its the OpenCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        def order_coordinates(pts, var):
            coordinates = np.zeros((4,2),dtype="int")

            if(var):
                #Parameters sort model 1 
                s = pts.sum(axis=1)
                coordinates[0] = pts[np.argmin(s)]
                coordinates[3] = pts[np.argmax(s)] 

                diff = np.diff(pts, axis=1)
                coordinates[1] = pts[np.argmin(diff)]
                coordinates[2] = pts[np.argmax(diff)]
            
            else:
                #Parameters sort model 2 
                s = pts.sum(axis=1)
                coordinates[0] = pts[np.argmin(s)]
                coordinates[2] = pts[np.argmax(s)] 

                diff = np.diff(pts, axis=1)
                coordinates[1] = pts[np.argmin(diff)]
                coordinates[3] = pts[np.argmax(diff)]
            
            return coordinates

        image = cv_image
        h, w = image.shape[:2]

        image = cv2.resize(image,(int(w*0.7), int(h*0.7)))
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        #Initialize the aruco Dictionary and its parameters 
        aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        parameters =  aruco.DetectorParameters_create()

        #Detect the corners and ids in the images 
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        #Initialize an empty list for the coordinates 
        params = []
        try:

            for i in range(len(ids)):

                #Catch the corners of each tag
                c = corners[i][0]

                #Draw a circle in the center of each detection
                cv2.circle(image,(int(c[:, 0].mean()), int(c[:, 1].mean())), 3, (255,255,0), -1)
                
                #Save the coordinates of the center of each tag
                params.append((int(c[:, 0].mean()), int(c[:, 1].mean())))

            #Transfom the coordinates list to an array
            params = np.array(params)
            if(len(params)>=4):
                #Sort model 1 
                params = order_coordinates(params,False)
                
                #Sort Model 2
                params_2 = order_coordinates(params,True)

            #Her we are going to read the image we want to overlap
            paint = cv2.imread('/home/simulations/public_sim_ws/src/all/opencv_course_sims/project/images/wanted.png')
            height, width = paint.shape[:2]

            #We extract the coordinates of this new image which are basically the full sized image
            coordinates = np.array([[0,0],[width,0],[0,height],[width,height]])

            #Just like in chapter 3 we will find a perspective between the planes
            #Homography will help us with the image transformations
            hom, status = cv2.findHomography(coordinates, params_2)
            
            #We will save the warped image in a dark space same with the same size as the main image
            warped_image = cv2.warpPerspective(paint, hom, (int(w*0.7), int(h*0.7)))

            #We create a black mask to do the image operations 
            mask = np.zeros([int(h*0.7), int(w*0.7),3], dtype=np.uint8)

            #To the black mask we will replace the area described by the ar tags with white 
            cv2.fillConvexPoly(mask, np.int32([params]), (255, 255, 255), cv2.LINE_AA)
            #cv2.imshow('black mask',mask)
            substraction = cv2.subtract(image,mask)
            #cv2.imshow('substraction',substraction)
            addition = cv2.add(warped_image,substraction)
            cv2.imshow('detection',addition) 
            

        except:
            pass          
        cv2.imshow('image',cv_image) 
        cv2.waitKey(1)



def main():
    load_polygon2_object = LoadPolygon2()
    rospy.init_node('load_polygon2_node', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()