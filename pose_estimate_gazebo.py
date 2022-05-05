'''
Class: 16-681, MRSD Project 1
Team I: AIce
Team Members: Rathin Shah, Nick Carcione, Yilin Cai, Jiayi Qiu, Kelvin Shen
Description: This script subscribes to RealSense D435i RGB stream, estimates the pose 
             of the ArUco marker board in the image, and publishes the leader vehicle
             pose with respect to either camera that detects it.
Date of first revision: 02/10/2022
'''

#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import message_filters
import tf
from cv2 import aruco
from utils import Hx, Hz, tf2T

from gazebo_msgs.msg import LinkStates
from pose_estimate.msg import LeaderPose
from pose_estimate.msg import GazeboGT


class LoadIds(object):

    def __init__(self):
        # Read calibration matrix only once
        camera1_info_msg = rospy.wait_for_message('/camera1/color/camera_info', CameraInfo)
        self.camera1_matrix_rgb = np.array(camera1_info_msg.K).reshape(3, 3)
        camera2_info_msg = rospy.wait_for_message('/camera2/color/camera_info', CameraInfo)
        self.camera2_matrix_rgb = np.array(camera2_info_msg.K).reshape(3, 3)
        self.dist_coeffs_rgb = np.zeros(5)  # github.com/ros-planning/movein_calibration/issues/77
        
        # Optional depth camera matrix to read
        # camera_info_msg = rospy.wait_for_message('/camera/depth/camera_info', CameraInfo)
        # self.camera_matrix_depth = np.array(camera_info_msg.K).reshape(3, 3)
        # self.dist_coeffs_depth = np.zeros(5)  

        self.bridge_object = CvBridge()

        # To get ground truth in Gazebo
        # self.gt_sub = message_filters.Subscriber("/gt_publisher", GazeboGT)

        # RGB-D image topic
        self.image_sub1 = message_filters.Subscriber("/camera1/color/image_raw", Image)
        self.depth_sub1 = message_filters.Subscriber("/camera1/depth/image_raw", Image)
        self.image_sub2 = message_filters.Subscriber("/camera2/color/image_raw", Image)
        self.depth_sub2 = message_filters.Subscriber("/camera2/depth/image_raw", Image)
        
        # Message filter to synchronize two RGB streams from two cameras
        self.ts = message_filters.ApproximateTimeSynchronizer([self.image_sub1, self.image_sub2, ], queue_size=10, slop=0.5)
                                                            #    self.depth_sub1, self.depth_sub2], queue_size=10, slop=0.5)
        self.ts.registerCallback(self.camera_callback)
        
        # ArUco utilities
        self.rear_dict = aruco.Dictionary_get(aruco.DICT_6X6_1000)
        self.left_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
        self.right_dict = aruco.Dictionary_get(aruco.DICT_5X5_100)
        self.parameters =  aruco.DetectorParameters_create()
        markerLength = 60  # in px
        markerSeparation = 15  # in px
        self.board = aruco.GridBoard_create(5, 5, markerLength, markerSeparation, self.rear_dict)
        self.left_board = aruco.GridBoard_create(5, 5, markerLength, markerSeparation, self.rear_dict)
        self.right_board = aruco.GridBoard_create(5, 5, markerLength, markerSeparation, self.right_dict)
        self.brdImgSize = 390  # in px
        
        # ROS tf listener and broadcaster
        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        
        # Store in advance the transform from leader base to three different ArUco marker boards
        self.board2leader_base = {}
        while True:
            try:
                self.board2leader_base['left'] = tf2T(self.listener.lookupTransform('leader_zamboni/left_aruco_link', 'leader_zamboni/base_footprint', rospy.Time(0)))
                self.board2leader_base['rear'] = tf2T(self.listener.lookupTransform('leader_zamboni/rear_aruco_link', 'leader_zamboni/base_footprint', rospy.Time(0)))
                self.board2leader_base['right'] = tf2T(self.listener.lookupTransform('leader_zamboni/right_aruco_link', 'leader_zamboni/base_footprint', rospy.Time(0)))
                if len(self.board2leader_base.keys()) == 3:
                    break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print("Board to Leader Base Tranform not found")
                continue

                
    # Broadcast the transform from the camera to leader base
    def send_camera_leader_base(self, rvec, tvec, camera_link, board2leader):
        rmat, _ = cv2.Rodrigues(rvec)
        T = np.eye(4)
        T[:3, :3] = rmat
        T[:3, -1] = tvec.flatten()
        cam2board = Hz(-1.5707963).dot(Hx(-1.5707963)).dot(T)  # from tf conventions to OpenCV conventions
        boardOffset = np.array([[1,0,0,0.5 - 15.0/390],
                                [0,1,0,0.5 - 15.0/390],
                                [0,0,1,0],
                                [0,0,0,1]])
        cam2leader_base = cam2board.dot(boardOffset).dot(Hx(-1.5707963)).dot(Hz(1.5707963)).dot(board2leader)  # from OpenCV conventions to tf conventions 
                                                                                                               # plus board offset
        x, y, z = cam2leader_base[:3, -1]
        i, j, k, w = tf.transformations.quaternion_from_matrix(cam2leader_base)
        self.br.sendTransform((x,y,z), (i,j,k,w), rospy.Time.now(), "leader_base", camera_link)

    
    # Detect any marker in the image based on which side of the vehicle the board is on
    def perceive_marker(self, side, gray, image, camera_K, camera_dist):
        # Different side has a different ArUco dictionary
        if side == 'left':
            aruco_dict, aruco_board = self.left_dict, self.left_board
        elif side == 'right':
            aruco_dict, aruco_board = self.right_dict, self.right_board
        else:
            aruco_dict, aruco_board = self.rear_dict, self.board

        # Detect the corners and id's (TODO: refining doesn't seem to improve the results)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=self.parameters)

        T = np.eye(4)  # transform from board frame to the camera frame, i.e., board's position in camera frame
        if ids is not None and len(ids) > 0: # if there is at least one marker detected
            im_with_aruco_board = aruco.drawDetectedMarkers(image.copy(), corners, ids, (0,255,0))
            retval, rvec, tvec = aruco.estimatePoseBoard(corners, ids, aruco_board, camera_K, camera_dist)  # pose estimation from a diamond
            return retval, rvec, tvec, im_with_aruco_board, corners
        
        else:
            return 0, None, None, image, corners  # fail to detect any marker

    
    # Main callback of two RGB streams from two cameras
    def camera_callback(self, rgb_data1, rgb_data2): #, depth_data1, depth_data2):
        try:
            # We select bgr8 because it's the OpenCV encoding by default
            cv_image_rgb1 = self.bridge_object.imgmsg_to_cv2(rgb_data1, desired_encoding="bgr8")
            # cv_image_depth1 = self.bridge_object.imgmsg_to_cv2(depth_data1, desired_encoding="passthrough")
            cv_image_rgb2 = self.bridge_object.imgmsg_to_cv2(rgb_data2, desired_encoding="bgr8")
            # cv_image_depth2 = self.bridge_object.imgmsg_to_cv2(depth_data2, desired_encoding="passthrough")
        except CvBridgeError as e:
            print(e)

        # cv2.imwrite('/home/didi/AIce/zamboni_ws/src/pose_estimate/depth.png', cv_image_depth)

        # Need to normalize before visualizing uint16 image
        # depth_norm1 = cv2.normalize(cv_image_depth1, None, 0, 65535, cv2.NORM_MINMAX)
        # depth_norm2 = cv2.normalize(cv_image_depth2, None, 0, 65535, cv2.NORM_MINMAX)
        # depth_norm = np.concatenate((depth_norm1, depth_norm2), axis=1)
        # cv2.namedWindow('depth', cv2.WINDOW_KEEPRATIO)
        # cv2.imshow('depth', depth_norm)
        # cv2.resizeWindow('depth', 1280, 360)

        limage = cv_image_rgb1
        rimage = cv_image_rgb2

        # cv2.imwrite('/home/didi/AIce/zamboni_ws/src/pose_estimate/raw.jpg', image)
        # cv2.imshow('raw', image)

        # h,w = limage.shape[:2]
        # image = cv2.resize(image,(int(w*0.7), int(h*0.7)))
        lgray = cv2.cvtColor(limage, cv2.COLOR_BGR2GRAY)
        rgray = cv2.cvtColor(rimage, cv2.COLOR_BGR2GRAY)
        
        # Iterate over each marker board and use the one with the most markers detected
        # to estimate the leader's pose
        max_markers_detected = -1
        tf_to_send = []
        for side in ['rear', 'right', 'left']:
            lres = self.perceive_marker(side, lgray, limage, self.camera1_matrix_rgb, self.dist_coeffs_rgb)
            rres = self.perceive_marker(side, rgray, rimage, self.camera2_matrix_rgb, self.dist_coeffs_rgb)

            if max_markers_detected < max(lres[0], rres[0]):
                max_markers_detected = max(lres[0], rres[0])
            else:
                print("Skipping %s Board due to less markers detected" % side)
                continue

            if (lres[0] != 0 and rres[0] == 0) or (lres[0] != 0 and rres[0] != 0 and len(lres[-1]) > len(rres[-1])):
                # Use left camera detections
                rvec, tvec, im_with_aruco_board, corners = lres[1:]
                
                im_with_aruco_board = aruco.drawAxis(im_with_aruco_board, self.camera1_matrix_rgb, self.dist_coeffs_rgb, rvec, tvec, 100)
                tvec = tvec / self.brdImgSize  # from pixel to meter
                tf_to_send = [rvec, tvec, 'follower_zamboni/camera1_link', self.board2leader_base[side]]
                # self.send_camera_leader_base(rvec, tvec, 'follower_zamboni/camera1_link', self.board2leader_base[side])

                print("Translation Norm of %s Board: %.2f meters" % (side, np.linalg.norm(tvec)))
                # TODO: Get ROI and average depth

                cv2.namedWindow('pose', cv2.WINDOW_KEEPRATIO)
                im_to_show = np.concatenate((im_with_aruco_board, rimage), axis=1)
                cv2.imshow('pose', im_to_show)
                cv2.resizeWindow('pose', 1280, 360)
                cv2.waitKey(1)
            
            elif (rres[0] != 0 and lres[0] == 0) or (lres[0] != 0 and rres[0] != 0 and len(rres[-1]) > len(lres[-1])):
                # Use right camera detections
                rvec, tvec, im_with_aruco_board, corners = rres[1:]
                im_with_aruco_board = aruco.drawAxis(im_with_aruco_board, self.camera2_matrix_rgb, self.dist_coeffs_rgb, rvec, tvec, 100)
                tvec = tvec / self.brdImgSize  # from pixel to meter
                tf_to_send = [rvec, tvec, 'follower_zamboni/camera2_link', self.board2leader_base[side]]
                # self.send_camera_leader_base(rvec, tvec, 'follower_zamboni/camera2_link', self.board2leader_base[side])

                print("Translation Norm of %s Board: %.2f meters" % (side, np.linalg.norm(tvec)))
                # TODO: Get ROI and average depth

                cv2.namedWindow('pose', cv2.WINDOW_KEEPRATIO)
                im_to_show = np.concatenate((limage, im_with_aruco_board), axis=1)
                cv2.imshow('pose', im_to_show)
                cv2.resizeWindow('pose', 1280, 360)
                cv2.waitKey(1)
            
            else:
                # both cameras don't see any marker on this board
                print("Translation Norm of %s Board: N/A" % (side))

                cv2.namedWindow('pose', cv2.WINDOW_KEEPRATIO)
                im_to_show = np.concatenate((limage, rimage), axis=1)
                cv2.imshow('pose', im_to_show)
                cv2.resizeWindow('pose', 1280, 360)
                cv2.waitKey(1)
                continue
        
        if len(tf_to_send) != 0:
            self.send_camera_leader_base(*tf_to_send)
        
        print("="*20)
        print("")

    
    # Optional: using depth stream to calculate the translation between the camera and the board
    # This will only be used when the purely marker-based solution does not provide accurate translation. 
    def get_depth_in_ROI(self, corners, ids, cv_image_depth, image):
        # Get the center of each corner in the list of markers detected
        center_of_corners = np.array([[int(c[0][:, 0].mean()), int(c[0][:, 1].mean())] for c in corners])
        if len(center_of_corners) >= 4:
            # Contour
            coordinates = np.zeros((4,2), dtype="int")
            s = center_of_corners.sum(axis = 1)
            coordinates[0] = center_of_corners[np.argmin(s)]
            coordinates[2] = center_of_corners[np.argmax(s)]
            d = np.diff(center_of_corners, axis = 1)
            coordinates[1] = center_of_corners[np.argmin(d)]
            coordinates[3] = center_of_corners[np.argmax(d)]
            
            # Get the mask that will cover the largest area of all detected markers
            mask = np.zeros_like(cv_image_depth)
            cv2.drawContours(mask, [coordinates], -1, 255, -1)
            
            # Only use the depth values inside the mask
            out = np.zeros_like(cv_image_depth)
            out[mask == 255] = cv_image_depth[mask == 255]
            avg_depth = np.sum(out) / cv2.countNonZero(out) * 0.001 # default depth scale of RealSense 400 Series
            print("Interpolated Average Depth: %.2f meters" % (avg_depth))
            
            # Visualize
            frame_markers = aruco.drawDetectedMarkers(image.copy(), corners, ids)
            cv2.drawContours(frame_markers, [coordinates], -1, (255, 0, 150), -1)
            # cv2.namedWindow('markers', cv2.WINDOW_KEEPRATIO)
            # cv2.imshow('markers', frame_markers)
            # cv2.resizeWindow('markers', 640, 360)

            return avg_depth, frame_markers
        else:
            return -1, None
    

def main():
    rospy.init_node('pose_estimate', anonymous=True)
    load_ids_object = LoadIds()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
