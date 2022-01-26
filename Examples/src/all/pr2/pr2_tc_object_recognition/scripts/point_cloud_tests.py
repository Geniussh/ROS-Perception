#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs import point_cloud2 as pc2
from sensor_msgs.msg import Image, PointCloud2


class Detector:

   def __init__(self):
      
      image_topic = "/camera/rgb/image_raw"
      point_cloud_topic = "/camera/depth_registered/points"

      self._global_frame = "camera_rgb_optical_frame"
      
      # create detector
      self._bridge = CvBridge()

      # image and point cloud subscribers
      # and variables that will hold their values
      rospy.Subscriber(image_topic, Image, self.image_callback)

      rospy.Subscriber(point_cloud_topic, PointCloud2, self.pc_callback)

      self._current_image = None
      self._current_pc = None

      
      rospy.loginfo('Ready to detect!')

   def image_callback(self, image):
      """Image callback"""
      # Store value on a private attribute
      self._current_image = image

   def pc_callback(self, pc):
      """Point cloud callback"""
      # Store value on a private attribute
      self._current_pc = pc

   def run(self):
      # run while ROS runs
      while not rospy.is_shutdown():
         # only run if there's an image present
         if self._current_image is not None and self._current_pc is not None:
            try:

                # convert image from the subscriber into an OpenCV image
                scene = self._bridge.imgmsg_to_cv2(self._current_image, 'rgb8')

                ymin = 200
                xmin = 200
                ymax = 300
                xmax = 300
                y_center = int(ymax - ((ymax - ymin) / 2))
                x_center = int(xmax - ((xmax - xmin) / 2))
                print(type(y_center))
                print(type(x_center))

                pc_list = list(
                    pc2.read_points(self._current_pc,
                                    skip_nans=True,
                                    field_names=('x', 'y', 'z'),
                                    uvs=[(x_center, y_center)]))
                # pc_list =  pc2.read_points(self._current_pc,
                #                     skip_nans=True,
                #                     field_names=('x', 'y', 'z'),
                #                     uvs=[(x_center, y_center)])
                print(pc_list)

                if len(pc_list) > 0:
                    point_x, point_y, point_z = pc_list[0]

                
            except CvBridgeError as e:
               print(e)


if __name__ == '__main__':
   rospy.init_node('point_cloud_test_node', log_level=rospy.DEBUG)

   try:
      Detector().run()
   except KeyboardInterrupt:
      rospy.loginfo('Shutting down')
