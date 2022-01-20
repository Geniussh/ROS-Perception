#!/usr/bin/env python
import time
import rospy
from math import pi, sin, cos, acos
import random
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
"""
Topics To Write on:
type: std_msgs/Float64
/mira/pitch_joint_position_controller/command
/mira/roll_joint_position_controller/command
/mira/yaw_joint_position_controller/command
"""

class MiraBlobTracker(object):

    def __init__(self):
        
        rospy.loginfo("Mira JointMover Initialising...")

        self.YAW_DELTA_MAX = 0.05
        self.PITCH_DELTA_MAX = 0.05
        self.ROLL_DELTA_MAX = 0.05

        self.pub_mira_roll_joint_position = rospy.Publisher('/mira/roll_joint_position_controller/command',
                                                            Float64,
                                                            queue_size=1)
        self.pub_mira_pitch_joint_position = rospy.Publisher('/mira/pitch_joint_position_controller/command',
                                                             Float64,
                                                             queue_size=1)
        self.pub_mira_yaw_joint_position = rospy.Publisher('/mira/yaw_joint_position_controller/command',
                                                           Float64,
                                                           queue_size=1)
        
        self._check_all_publishers()
        
        self.joint_limits = {"roll_joint":[-0.2,0.7],
                            "pitch_joint":[0.0, 0.44],
                            "yaw_joint":[-1.2,1.2]}

        # We initialise Mira
        current_roll = 0.0
        current_pitch = 0.0
        current_yaw = 0.0
        self.current_pose = [current_roll, current_pitch, current_yaw]
        self.move_mira_all_joints(self.current_pose[0], self.current_pose[1] , self.current_pose[2])

        rospy.Subscriber('/mira/commands/velocity',  Twist, self.cmd_vel_callback)
    

    def _check_publishers_connection(self, publisher_obj, name):
        """
        Checks that all the publishers are working
        :return:
        """
        rate = rospy.Rate(10)  # 10hz
        while publisher_obj.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.logdebug("No susbribers to "+name+" yet so we wait and try again")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # This is to avoid error when world is rested, time when backwards.
                pass
        rospy.logdebug(""+name+" Publisher Connected")


    def _check_all_publishers(self):
        self._check_publishers_connection(self.pub_mira_roll_joint_position, "pub_mira_roll_joint_position")
        self._check_publishers_connection(self.pub_mira_pitch_joint_position, "pub_mira_pitch_joint_position")
        self._check_publishers_connection(self.pub_mira_yaw_joint_position, "pub_mira_yaw_joint_position")
        rospy.logdebug("All Publishers READY")

        
    def cmd_vel_callback(self, msg):
        rospy.loginfo("Blob info Detected==>"+str(msg.angular.z))
        roll_delta = 0.0
        pitch_delta = 0.0

        # Yaw
        turn_value = msg.angular.z
        if turn_value > 0.0:
            yaw_delta = self.YAW_DELTA_MAX
        elif turn_value < 0.0:
            yaw_delta = -1 * self.YAW_DELTA_MAX
        else:
            yaw_delta = 0.0

        # Pitch
        pitch_value = msg.angular.y
        if pitch_value > 0.0:
            pitch_delta = self.PITCH_DELTA_MAX
        elif pitch_value < 0.0:
            pitch_delta = -1 * self.PITCH_DELTA_MAX
        else:
            pitch_delta = 0.0
        

        # Roll
        pitch_value = msg.angular.x
        if pitch_value > 0.0:
            roll_delta = self.ROLL_DELTA_MAX
        elif pitch_value < 0.0:
            roll_delta = -1 * self.ROLL_DELTA_MAX
        else:
            roll_delta = 0.0


        self.current_pose[0] +=  roll_delta
        self.current_pose[1] +=  pitch_delta
        self.current_pose[2] +=  yaw_delta

        # We check if some value exceded the limits
        if self.current_pose[0] > self.joint_limits["roll_joint"][1]:
            self.current_pose[0] = self.joint_limits["roll_joint"][1]
        elif self.current_pose[0] < self.joint_limits["roll_joint"][0]:
            self.current_pose[0] = self.joint_limits["roll_joint"][0]
        else:
            rospy.loginfo("ROLL Inside Range")

        if self.current_pose[1] > self.joint_limits["pitch_joint"][1]:
            self.current_pose[1] = self.joint_limits["pitch_joint"][1]
        elif self.current_pose[1] < self.joint_limits["pitch_joint"][0]:
            self.current_pose[1] = self.joint_limits["pitch_joint"][0]
        else:
            rospy.loginfo("PITCH Inside Range")

        if self.current_pose[2] > self.joint_limits["yaw_joint"][1]:
            self.current_pose[2] = self.joint_limits["yaw_joint"][1]
        elif self.current_pose[2] < self.joint_limits["yaw_joint"][0]:
            self.current_pose[2] = self.joint_limits["yaw_joint"][0]
        else:
            rospy.loginfo("YAW Inside Range")


        rospy.loginfo("Move Head to Blob==>"+str(self.current_pose))
        self.move_mira_all_joints(self.current_pose[0], self.current_pose[1] , self.current_pose[2])

    def move_mira_all_joints(self, roll, pitch, yaw):
        angle_roll = Float64()
        angle_roll.data = roll
        angle_pitch = Float64()
        angle_pitch.data = pitch
        angle_yaw = Float64()
        angle_yaw.data = yaw
        self.pub_mira_roll_joint_position.publish(angle_roll)
        self.pub_mira_pitch_joint_position.publish(angle_pitch)
        self.pub_mira_yaw_joint_position.publish(angle_yaw)
    
    def loop(self):

        rospy.spin()



if __name__ == "__main__":
    rospy.init_node('mira_move_head_node', anonymous=True, log_level=rospy.DEBUG)
    mira_jointmover_object = MiraBlobTracker()
    mira_jointmover_object.loop()

