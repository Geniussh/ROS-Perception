#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

class Flying():

    def __init__(self):
        self.hector_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.hector_sub = rospy.Subscriber("/sonar_height", Range, self.scan_callback)
        
        self.cmd = Twist()
        self.ctrl_c = False
        self.rate = rospy.Rate(10) # 10hz
        self.a = 0.0
        rospy.on_shutdown(self.shutdownhook)

    def scan_callback(self, msg):
        self.a = msg.range 

    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.ctrl_c = True

    def move_drone(self):
        while not self.ctrl_c:

            if self.a < 1.8:
                self.cmd.linear.z = 0.12
                
            if self.a > 1.93:
                self.cmd.linear.z = 0.0

            if self.a < 0.39:
                self.cmd.linear.x = -1.3
            if self.a > 0.40 and self.a < 0.50:
                self.cmd.linear.x = 0.7
            if self.a > 0.51 and self.a < 0.60:
                self.cmd.linear.x = -0.2
            if self.a > 0.61 and self.a < 0.70:
                self.cmd.linear.x = 0.7
            if self.a > 0.71 and self.a < 0.80:
                self.cmd.linear.x = -0.2
            if self.a > 0.81 and self.a < 0.95:
                self.cmd.linear.x = 0.9
            if self.a > 0.95 and self.a < 1.00:
                self.cmd.linear.x = -0.2
            if self.a > 1.01 and self.a < 1.10:
                self.cmd.linear.x = 0.5
            if self.a > 1.11 and self.a < 1.20:
                self.cmd.linear.x = -0.2
            if self.a > 1.21 and self.a < 1.30:
                self.cmd.linear.x = 0.5
            if self.a > 1.31 and self.a < 1.40:
                self.cmd.linear.x = -0.34
            if self.a > 1.41 and self.a < 1.50:
                self.cmd.linear.x = 0.72
            if self.a > 1.51 and self.a < 1.60:
                self.cmd.linear.x = -0.29
            if self.a > 1.61 and self.a < 1.78:
                self.cmd.linear.x = 0.71
            if self.a > 1.78 and self.a < 1.98:
                self.cmd.linear.x = 0.08
            if self.a > 1.98 :
                self.cmd.linear.x = 0.0
            
                
            self.hector_vel_publisher.publish(self.cmd)


            

if __name__ == '__main__':
    rospy.init_node('hector_test', anonymous=True)
    hector_object = Flying()
    try:
        hector_object.move_drone()
    except rospy.ROSInterruptException:
        pass