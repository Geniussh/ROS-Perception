#!/usr/bin/env python
import sys
import rospy
import math
import tf
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from move_aibo import AiboJointMover
import time
import numpy

class AiboPeopleFollower(object):
    
    def __init__(self):
        
        # Minimum distance from subject, closer will give serious probles to see the adult.
        self.min_distance = 3.0

        
        
        self.aibo_mover = AiboJointMover()
        
        self.follower_model_name = "aiboERS7"
        self.aibo_vel = rospy.Publisher(self.follower_model_name+'/cmd_vel', Twist,queue_size=1)
        
        self.check_publishers()
        
        # We set the head to search for adults.
        rospy.loginfo("Moving Head to Adult search position...")
        self.aibo_mover.pub_headTilt_position.publish(-0.7)
        rospy.loginfo("DONE Moving Head to Adult search position...")
        
        self.ctrl_c = False
        rospy.on_shutdown(self.shutdownhook)
       
        self.people_pose = self.check_people_tracking_ready()
        rospy.Subscriber("/people_tracker/pose", PoseStamped, self.people_tracker_pose_callback)
        
            
    def check_people_tracking_ready(self):
        
        people_tracker_data = None
        rospy.logdebug("Waiting for /camera/depth/points to be READY...")
        while people_tracker_data is None and not rospy.is_shutdown():
            
            # Start Searching person
            rospy.loginfo("Searching Person...")
            self.move_aibo(linear=0.0, angular=0.2)
            
            try:
                people_tracker_data = rospy.wait_for_message("/people_tracker/pose", PoseStamped, timeout=2.0)
                rospy.logdebug("Current /people_tracker/pose READY=>")
    
            except:
                rospy.logerr("Current /people_tracker/pose not ready yet, retrying for getting people_tracker_data")
                
        return people_tracker_data.pose
        
    
    def check_pub_headTilt_position_publishers_connection(self):
        """
        Checks that all the publishers are working
        :return:
        """
        rate = rospy.Rate(10)  # 10hz
        while self.aibo_mover.pub_headTilt_position.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.logdebug("No susbribers to self.aibo_mover.pub_headTilt_position yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("self.aibo_mover.pub_headTilt_position Publisher Connected")
        
        
    def check_aibo_vel_publishers_connection(self):
        """
        Checks that all the publishers are working
        :return:
        """
        rate = rospy.Rate(10)  # 10hz
        while self.aibo_vel.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.logdebug("No susbribers to self.aibo_vel yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug("self.aibo_vel Publisher Connected")
        
        
    def check_publishers(self):
        """
        We check that all publishers work
        """
        self.check_pub_headTilt_position_publishers_connection()
        self.check_aibo_vel_publishers_connection()

        rospy.loginfo("All Publishers READY")
                
    def shutdownhook(self):
            print "shutdown time! Stop the robot"
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.aibo_vel.publish(cmd)
            self.ctrl_c = True
            
            
    def people_tracker_pose_callback(self,data):
        """
        std_msgs/Header header
          uint32 seq
          time stamp
          string frame_id
        geometry_msgs/Pose pose
          geometry_msgs/Point position
            float64 x
            float64 y
            float64 z
          geometry_msgs/Quaternion orientation
            float64 x
            float64 y
            float64 z
            float64 w
        """
        self.people_pose = data.pose
            
    def start_follow_people_loop(self):
        
        rate = rospy.Rate(10.0)
        
        while not self.ctrl_c:
            
            if self.people_pose:
                trans_0 = self.people_pose.position.x
                trans_1 = self.people_pose.position.y
                
                rospy.loginfo("Translation PeoplePosition = ["+str(trans_0)+","+str(trans_1)+"]")
                
                point_object = Point()
                point_object.x = trans_0
                point_object.y = trans_1
                distance = self.get_vector_magnitude(point_object)
                rospy.loginfo("distance from person = "+str(distance))
                
                if distance > self.min_distance:
                    angular = 4 * math.atan2(trans_1, trans_0)
                    linear = 0.1 * math.sqrt(trans_0 ** 2 + trans_1 ** 2)
                else:
                    angular = 0.0
                    linear = 0.0
                    self.wiggle_tail()
        
                # We now clear the self.people_pose so that only when we detect we move
                self.people_pose = None
    
            else:
                
                # We set the searching mode
                rospy.loginfo("Searching Person...")
                angular = 0.1
                linear = 0.0
            
            
            self.move_aibo(linear, angular)   
    
            rate.sleep()
            
            
    def move_aibo(self, linear, angular):
        
        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        rospy.loginfo("Move="+str(cmd))
        self.aibo_vel.publish(cmd)
        
        
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
            


if __name__ == '__main__':
    rospy.init_node('people_listener_aibo')
    aibofollower = AiboPeopleFollower()
    aibofollower.start_follow_people_loop()
    
