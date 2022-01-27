#!/usr/bin/env python

import rospy
from people_msgs.msg import PositionMeasurementArray
import numpy
from move_aibo import AiboJointMover
import time
# Move base using navigation stack

"""
# PositionMeasurementArray
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
people_msgs/PositionMeasurement[] people
  std_msgs/Header header
    uint32 seq
    time stamp
    string frame_id
  string name
  string object_id
  geometry_msgs/Point pos
    float64 x
    float64 y
    float64 z
  float64 reliability
  float64[9] covariance
  byte initialization
float32[] cooccurrence
"""

class FaceDetectClient(object):

    def __init__(self):
        self.face_detect_subs = rospy.Subscriber(   "/face_detector/people_tracker_measurements_array",
                                                    PositionMeasurementArray,
                                                    self.face_detect_subs_callback)
                                                    
        self.min_wigle_tail_distance = 1.0

    
        self.pos_mesurement_array = PositionMeasurementArray()
        
        self.aibo_mover = AiboJointMover()
        
        # We set the head to a serahc faces tilt
        self.aibo_mover.pub_headTilt_position.publish(-0.3)
    
    def face_detect_subs_callback(self,msg):
        
        pos_mesurement_array = msg
        for people_data in pos_mesurement_array.people:
            person_name = people_data.name
            person_id = people_data.object_id
            distance = self.get_vector_magnitude(people_data.pos)
            
            rospy.loginfo("#################")
            rospy.loginfo("Name="+str(person_name))
            rospy.loginfo("person_id="+str(person_id))
            rospy.loginfo("distance="+str(distance))
            rospy.loginfo("####### END #######")
            
            if distance < self.min_wigle_tail_distance:
                self.wiggle_tail()
                
                
    def wiggle_tail(self):
        """
        Move the Tail for a Second
        """
        rospy.loginfo("Wiggle Tail!")
        
        tailPan_min = -0.7
        tailPan_max = 0.7
        sleep_time = 0.2
        
        for i in range(3):
            self.aibo_mover.pub_tailPan_position.publish(tailPan_min)
            time.sleep(sleep_time)
            self.aibo_mover.pub_tailPan_position.publish(tailPan_max)
            time.sleep(sleep_time)
            
    
    def get_vector_magnitude(self, point_object):
        """
        Returns the magnitude of the point vector
        """
        numpy_array_vector = numpy.array((point_object.x, point_object.y, point_object.z))
    
        magnitude = numpy.linalg.norm(numpy_array_vector)
    
        return magnitude

    
def Face_DetectionClient_Start():
    # Create a node
    rospy.init_node("face_detection_client_start_node", log_level=rospy.INFO)

    # Make sure sim time is working
    while not rospy.Time.now():
        pass

    face_detector_client = FaceDetectClient()

    rospy.spin()

if __name__ == "__main__":
    Face_DetectionClient_Start()

